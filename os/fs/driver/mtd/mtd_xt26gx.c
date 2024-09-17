/*************************** Included Files *****************************/
#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <tinyara/kmalloc.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/spi/spi.h>
#include <tinyara/fs/mtd.h>

/*************************** Pre-processor Definitions *****************************/
#ifndef CONFIG_XT26GX_SPIMODE
#define CONFIG_XT26GX_SPIMODE	SPIDEV_MODE0
#endif

#ifndef CONFIG_XT26GX_SPI_FREQUENCY
#define CONFIG_XT26GX_SPI_FREQUENCY	(20000000)
#endif

/* XT26GX specific commands */ 
#define XT26GX_CMD_READID 0x9F 
#define XT26GX_CMD_READ 0x13
#define XT26GX_CMD_READ_CACHE 0x03 
#define XT26GX_CMD_PROG_PAGE 0x02 
#define XT26GX_CMD_PROG_EXE 0x10
#define XT26GX_CMD_ERASE_BLOCK 0xD8 
#define XT26GX_CMD_READ_STATUS 0x0F
#define XT26GX_CMD_SET_FEATURES 0x1F
#define XT26GX_REG_BLOCK_LOCK 0xA0


#define XT26GX_STATUS_REG 0xC0
#define XT26GX_CMD_WRITE_ENABLE 0x06 
#define XT26GX_CMD_WRITE_DISABLE 0x04 
/* XT26GX status register bits */ 
#define XT26GX_STATUS_BUSY (1 << 0) 
#define XT26GX_STATUS_WEL (1 << 1) 
/* Other constants */ 
#define XT26GX_BLOCK_SIZE 131072 /* Block size is 128KB */ 
#define XT26GX_PAGE_SIZE 2048 /* Page size is 2KB */ 
#define XT26GX_OOB_SIZE 64 /* Out-of-band size per page */ 
#define XT26GX_MAX_PAGES (128 * 1024) /* 128K pages */
#define XT26GX_PAGES_PER_BLOCK 64
#define XT26GX_MAX_BLOCKS 2048


#define XT26GX_DUMMY		(0xa5)

/************************************************************************************
 * Private Types
 ************************************************************************************/
struct spi_nand_dev_s {
    struct mtd_dev_s mtd; 
    FAR struct spi_dev_s *spi; /* SPI interface */ 
    uint32_t size; /* Total size in bytes */ 
    uint16_t page_size; /* Page size */ 
    uint32_t block_size; /* Block size */ 
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************//* Low-level SPI NAND operations */

/* Helpers */
static void nand_lock(FAR struct spi_dev_s *dev);
static inline void nand_unlock(FAR struct spi_dev_s *dev);
static void nand_writeenable(struct spi_nand_dev_s *priv);
static void nand_waitwritecomplete(struct spi_nand_dev_s *priv);
static inline int xt26gx_readid(FAR struct spi_nand_dev_s *priv);
static inline void xt26gx_sectorerase(struct spi_nand_dev_s *priv, off_t sector);
static inline void xt26gx_pagewrite(struct spi_nand_dev_s *priv, FAR const uint8_t *buffer, off_t page);
static int xt26gx_unlockprotected(FAR struct spi_nand_dev_s *priv);

/* MTD driver methods */
static int xt26gx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t xt26gx_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes, FAR uint8_t *buffer);
static ssize_t xt26gx_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buffer); 
static ssize_t xt26gx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buffer);
static int xt26gx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/*************************** Private Data *****************************/  
/* Define a table of supported XT26GX SPI NAND devices, including page size 
 * and block size for each device. 
*/

static const struct spi_nand_dev_s g_spi_nand_dev = { 
    .page_size = XT26GX_PAGE_SIZE, 
    .block_size = XT26GX_BLOCK_SIZE, 
    .size = XT26GX_MAX_PAGES * XT26GX_PAGE_SIZE, 
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: nand_lock
 ************************************************************************************/
static void nand_lock(FAR struct spi_dev_s *dev)
{
	/* On SPI busses where there are multiple devices, it will be necessary to
	 * lock SPI to have exclusive access to the busses for a sequence of
	 * transfers.  The bus should be locked before the chip is selected.
	 *
	 * This is a blocking call and will not return until we have exclusiv access to
	 * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
	 */

	(void)SPI_LOCK(dev, true);

	/* After locking the SPI bus, the we also need call the setfrequency, setbits, and
	 * setmode methods to make sure that the SPI is properly configured for the device.
	 * If the SPI buss is being shared, then it may have been left in an incompatible
	 * state.
	 */

	SPI_SETMODE(dev, CONFIG_XT26GX_SPIMODE);
	SPI_SETBITS(dev, 8);
	(void)SPI_SETFREQUENCY(dev, CONFIG_XT26GX_SPI_FREQUENCY);
}

/************************************************************************************
 * Name: nand_unlock
 ************************************************************************************/

static inline void nand_unlock(FAR struct spi_dev_s *dev)
{
	(void)SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name:  nand_writeenable
 ************************************************************************************/

static void nand_writeenable(struct spi_nand_dev_s *priv)
{
	/* Select this FLASH part */

	SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

	/* Send "Write Enable (WREN)" command */

	(void)SPI_SEND(priv->spi, XT26GX_CMD_WRITE_ENABLE);

	/* Deselect the FLASH */

	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
	lldbg("Enabled\n");
}

/************************************************************************************
 * Name: nand_waitwritecomplete
 ************************************************************************************/

static void nand_waitwritecomplete(struct spi_nand_dev_s *priv)
{
	uint8_t status;

	/* Loop as long as the memory is busy with a write cycle */

	do {
		/* Select this FLASH part */

		SPI_SELECT(priv->spi , SPIDEV_FLASH, true);

		/* Send "Read Status Register (RDSR)" command */

		(void)SPI_SEND(priv->spi, XT26GX_CMD_READ_STATUS);

        (void)SPI_SEND(priv->spi, XT26GX_STATUS_REG);

		/* Send a dummy byte to generate the clock needed to shift out the status */

		status = SPI_SEND(priv->spi, XT26GX_DUMMY);

		/* Deselect the FLASH */

		SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

		/* Given that writing could take up to few tens of milliseconds, and erasing
		 * could take more.  The following short delay in the "busy" case will allow
		 * other peripherals to access the SPI bus.
		 */

		if ((status & XT26GX_STATUS_BUSY) != 0) {
			nand_unlock(priv->spi);
			usleep(1000);
			nand_lock(priv->spi);
		}
	} while ((status & XT26GX_STATUS_BUSY) != 0);

	lldbg("Complete\n");
}


// /* Function to select the SPI device */ 
// static void spi_nand_select(FAR struct spi_dev_s *spi, bool selected) { 
//     SPI_SELECT(spi, SPIDEV_FLASH(0), selected); 
//     }

// /* Function to send SPI commands and receive responses */ 
// static uint8_t spi_nand_send_cmd(FAR struct spi_nand_dev_s *priv, uint8_t cmd) { 
//     spi_nand_select(priv->spi, true); 
//     SPI_SEND(priv->spi, cmd); 
//     uint8_t response = SPI_RECV(priv->spi); 
//     spi_nand_select(priv->spi, false); 
//     return response; 
// } 

/* Function to read the NAND ID */ 
static int xt26gx_readid(FAR struct spi_nand_dev_s *priv)
{
   // uint8_t cmd =  XT26GX_CMD_READID;
    uint8_t id[3];

    nand_lock(priv->spi);
	//nand_waitwritecomplete(priv);

    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    (void)SPI_SEND(priv->spi, XT26GX_CMD_READID); 

    (void)SPI_SEND(priv->spi, 0x00); /* Address*/ 

    /* Read 2 bytes of ID */
     id[0] = SPI_SEND(priv->spi, 0x00);
     id[1] = SPI_SEND(priv->spi, 0x00);
    // for (int i = 0; i < 2; i++) { 
    //         id[i] = SPI_RECV(priv->spi); 
    //     } 
    
	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
	nand_unlock(priv->spi);

	lldbg("manufacturer: %02x device ID: %02x\n", id[0], id[1]);

    /* Validate the manufacturer ID */ 
    if (id[0] != 0x0B) { /* XT26GX Manufacturer ID */
        lldbg("ERROR: Unsupported NAND flash ID: %02x\n", id[0]); 
        return -ENODEV; 
    } 
    return OK; 
}

/* Function to read a page of data from NAND */ 
static ssize_t xt26gx_read(FAR struct mtd_dev_s *dev, off_t block, size_t page, FAR uint8_t *buffer)
{ 
    FAR struct spi_nand_dev_s *priv = (FAR struct spi_nand_dev_s *)dev;
    //off_t offset = offset * priv->page_size; 
    
    // uint32_t page_addr = offset /  priv->page_size;
    uint16_t column_addr = 0;

    uint8_t page_offset_num = 0;
    uint32_t block_num = 0;
    uint32_t addr = 0;


    addr =  page;
    addr +=  ((block << 6) & 0x01FFC0);

    lldbg("block: %02x page: %02x spi addr %08lx\n ", block, page, (long)addr);

    nand_lock(priv->spi);
    nand_waitwritecomplete(priv);

    /* Select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    //spi_nand_select(priv->spi, true); 
    SPI_SEND(priv->spi, XT26GX_CMD_READ); 
    SPI_SEND(priv->spi, (addr >> 16) & 0xFF); 
    SPI_SEND(priv->spi, (addr >> 8) & 0xFF); 
    SPI_SEND(priv->spi, addr & 0xFF); 
    /* De-select this FLASH part */
    SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
    /* Wait for completion */
    nand_waitwritecomplete(priv);

    /* select this FLASH part */
	SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    (void)SPI_SEND(priv->spi, XT26GX_CMD_READ_CACHE); 
    //while (spi_nand_send_cmd(priv, XT26GX_CMD_READ_STATUS) & XT26GX_STATUS_BUSY);
     (void)SPI_SEND(priv->spi, 0x00); 
     (void)SPI_SEND(priv->spi, 0x00); 

     SPI_SEND(priv->spi, 0x00);

     /* Wait for completion */
    //nand_waitwritecomplete(priv);

    /* Then read all of the requested bytes */

	SPI_RECVBLOCK(priv->spi, buffer, 2048);

    /* Deselect the FLASH and unlock the SPI bus */

	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
    nand_unlock(priv->spi);

	lldbg("return nbytes: %d\n", 2048);
	return 2048;

    // for (int i = 0; i < priv->page_size; i++) { 
    //         buffer[i] = SPI_RECV(priv->spi); 
    //     } 
    //     spi_nand_select(priv->spi, false); 
    //     buffer += priv->page_size; 
    //     offset += priv->page_size; 
    // }
} 

static ssize_t xt26gx_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR uint8_t *buffer)
{
	FAR struct spi_nand_dev_s *priv = (FAR struct spi_nand_dev_s *)dev;
	ssize_t nbytes;
    size_t blocksleft;
    uint32_t block_num = 0;
    off_t maxblock;
    unsigned int page;

    block_num = startblock / priv->page_size;
    maxblock = XT26GX_MAX_BLOCKS;
    page = startblock % priv->page_size;


	lldbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	/* On this device, we can handle the block read just like the byte-oriented read */
    for(blocksleft = nblocks; blocksleft> 0; blocksleft--)
    {
        if(block_num > maxblock)
        {
            lldbg("Error: Read beyond end of FLASH, block=%1d\n", (long)startblock, block_num);
            return ERROR;
        }

        /* Read the page*/
        nbytes = xt26gx_read(dev, block_num, page, buffer);
        if (nbytes > 0) {
            if(++page >= XT26GX_PAGES_PER_BLOCK)
            {
                page = 0;
                block_num++;
            }
        }
        buffer += XT26GX_PAGE_SIZE;
    }
	return (int)nblocks;
}

/* Function to page write data to NAND */ 
static inline void xt26gx_pagewrite(struct spi_nand_dev_s *priv, FAR const uint8_t *buffer, off_t page)
{
	off_t offset = page << 11;
    uint8_t page_offset_num = 0;
    uint32_t block_num = 0;
    uint32_t addr = 0;

    block_num = offset/priv->block_size;
    page_offset_num  = ((offset/priv->page_size)%XT26GX_PAGES_PER_BLOCK);
    addr = page_offset_num & 0x3F;

    addr += ((block_num << 6) & 0x01FFC0);

	lldbg("page: %08lx addr: %08lx block no: %08lx\n spi addr: %08lx", (long)page, (long)offset, (long)block_num, addr);

	/* Wait for any preceding write to complete.  We could simplify things by
	 * perform this wait at the end of each write operation (rather than at
	 * the beginning of ALL operations), but have the wait first will slightly
	 * improve performance.
	 */

	nand_waitwritecomplete(priv);

    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    (void)SPI_SEND(priv->spi, XT26GX_CMD_PROG_PAGE);

    (void)SPI_SEND(priv->spi, 0x00);
    (void)SPI_SEND(priv->spi, 0x00);

    /* Then write the specified number of bytes */

	SPI_SNDBLOCK(priv->spi, buffer, 2048);

    /* Deselect this FLASH part */

	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

    nand_writeenable(priv);

    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    // (void)SPI_SEND(priv->spi, XT26GX_CMD_WRITE_ENABLE);

    (void)SPI_SEND(priv->spi, XT26GX_CMD_PROG_EXE);

    //SPI_SNDBLOCK(priv->spi, 0x00, 3);

    (void)SPI_SEND(priv->spi, (addr >> 16) & 0xff);
	(void)SPI_SEND(priv->spi, (addr >> 8) & 0xff);
	(void)SPI_SEND(priv->spi, addr & 0xff);

    /* Deselect the FLASH: Chip Select high */

	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
    nand_waitwritecomplete(priv);

    lldbg("Written\n");
}

/* Function to write a page of data to NAND */ 

static ssize_t xt26gx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks, FAR const uint8_t *buffer)
{ 
    FAR struct spi_nand_dev_s *priv = (FAR struct spi_nand_dev_s *)dev; 
    size_t blocksleft = nblocks;
    //size_t blocksleft = 1;
    size_t pagesize = priv->page_size;
    if(xt26gx_unlockprotected(priv) != OK) {
        lldbg("ERROR: failed to unloack the block protect byte\n");
    }

    lldbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

    /* Lock the SPI bus and write each page to FLASH */
    nand_lock(priv->spi);
    //off_t offset = startblock * priv->page_size; 
    while (blocksleft--) { /* Enable write operation */ 
        xt26gx_pagewrite(priv, buffer, startblock);
        buffer += pagesize;
        startblock++;
    }

    nand_unlock(priv->spi);
    return nblocks;
}

static inline void xt26gx_sectorerase(struct spi_nand_dev_s *priv, off_t sector)
{
    off_t offset;
    offset = sector << 17;

    lldbg("sector: %08lx\n", (long)sector);
    nand_waitwritecomplete(priv);

    /* Send write enable instruction */
	nand_writeenable(priv);

    /* Select this FLASH part */
    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    /* Send the "Sector Erase (SE)" or Sub-Sector Erase (SSE) instruction
	 * that was passed in as the erase type.
	 */
    (void)SPI_SEND(priv->spi, XT26GX_CMD_ERASE_BLOCK);
    (void)SPI_SEND(priv->spi, (offset >> 16) & 0xff);
	(void)SPI_SEND(priv->spi, (offset >> 8) & 0xff);
	(void)SPI_SEND(priv->spi, offset & 0xff);

    /* Deselect the FLASH */
	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
	lldbg("Erased\n");
}

/* Function to erase a block */ 
static int xt26gx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{ 
    FAR struct spi_nand_dev_s *priv = (FAR struct spi_nand_dev_s *)dev; 
    size_t blocksleft = nblocks;
    // off_t offset = block * priv->block_size;

    lldbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

    nand_lock(priv->spi);

    while(blocksleft > 0) {

        xt26gx_sectorerase(priv, startblock);
        startblock++;
        blocksleft--;
    }
    nand_unlock(priv->spi);
    return (int)nblocks;
}

static int xt26gx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
    FAR struct spi_nand_dev_s *priv = (FAR struct spi_nand_dev_s *)dev;
    int ret = -ENOTTY;

    switch (cmd) {
        case MTDIOC_GEOMETRY: {
            FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;
            if(geo) {
                geo->blocksize = priv->page_size;
                geo->erasesize = priv->block_size;
                geo->neraseblocks = priv->size / priv->block_size;
                ret = OK;
            }
        }
        break;

        default:
        ret = -ENOTTY;
        break;
    }
    return ret;
}

/* Function to read the NAND ID */ 
static int xt26gx_unlockprotected(FAR struct spi_nand_dev_s *priv)
{

    nand_lock(priv->spi);
	//nand_waitwritecomplete(priv);

    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    (void)SPI_SEND(priv->spi, XT26GX_CMD_SET_FEATURES); 

    (void)SPI_SEND(priv->spi, XT26GX_REG_BLOCK_LOCK); /* Address*/ 

    (void)SPI_SEND(priv->spi, 0x00); /* data*/ 

    
	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
	nand_unlock(priv->spi);

    return OK; 
}

/* Function to read the NAND ID */ 
static int xt26gx_Readprotected(FAR struct spi_nand_dev_s *priv)
{
   // uint8_t cmd =  XT26GX_CMD_READID;
    uint8_t id[1];

    nand_lock(priv->spi);
	//nand_waitwritecomplete(priv);

    SPI_SELECT(priv->spi, SPIDEV_FLASH, true);

    (void)SPI_SEND(priv->spi, XT26GX_CMD_READ_STATUS); 

    (void)SPI_SEND(priv->spi, XT26GX_REG_BLOCK_LOCK); /* Address*/ 

    id[0] = SPI_SEND(priv->spi, 0x00);

	SPI_SELECT(priv->spi, SPIDEV_FLASH, false);
	nand_unlock(priv->spi);

	lldbg("Block Reg: %02x \n", id[0]);
    return OK; 
}

static void nand_status_read(struct spi_nand_dev_s *priv, uint8_t reg)
{
	uint8_t status;

	/* Loop as long as the memory is busy with a write cycle */

		/* Select this FLASH part */

		SPI_SELECT(priv->spi , SPIDEV_FLASH, true);

		/* Send "Read Status Register (RDSR)" command */

		(void)SPI_SEND(priv->spi, XT26GX_CMD_READ_STATUS);

        (void)SPI_SEND(priv->spi, reg);

		/* Send a dummy byte to generate the clock needed to shift out the status */

		status = SPI_SEND(priv->spi, XT26GX_DUMMY);

		/* Deselect the FLASH */

		SPI_SELECT(priv->spi, SPIDEV_FLASH, false);

	    lldbg("Status Reg : %02x  Value : %02x\n", reg, status);
}
/*************************** Public Functions *****************************/

void Read_status_reg(struct spi_nand_dev_s *priv)
{
    nand_status_read(priv, 0xA0);
    nand_status_read(priv, 0xB0);
    nand_status_read(priv, 0xC0);
    nand_status_read(priv, 0xD0);
}


/* Function to initialize the SPI NAND device */ 
FAR struct mtd_dev_s *xt26gx_initialize(FAR struct spi_dev_s *spi)
{
    lldbg("xt26gx_initialize start \n"); 
    FAR struct spi_nand_dev_s *priv; 
    /* Allocate memory for the SPI NAND device */ 
    priv = (FAR struct spi_nand_dev_s *)kmm_zalloc(sizeof(struct spi_nand_dev_s)); 
    if (!priv) { 
            lldbg("ERROR: Failed to allocate memory for SPI NAND device\n"); 
            return NULL; 
        } 

        /* setup MTD interface callbacks */
        priv->mtd.erase = xt26gx_erase;
        priv->mtd.bread = xt26gx_bread;
        priv->mtd.bwrite = xt26gx_bwrite;
        priv->mtd.ioctl = xt26gx_ioctl;
        priv->mtd.read = xt26gx_read;

        /* Initialize the SPI NAND device */ 
        priv->spi = spi;
        priv->page_size = XT26GX_PAGE_SIZE;
        priv->block_size = XT26GX_BLOCK_SIZE;
        priv->size = XT26GX_MAX_PAGES*XT26GX_PAGE_SIZE;

        /* Deselect the FLASH */
		SPI_SELECT(spi, SPIDEV_FLASH, false);

        /*Read and validate the NAND ID */
        if(xt26gx_readid(priv) != OK) {
            lldbg("ERROR: Failed to read NAND ID\n");
            kmm_free(priv);
            return NULL;
        }
        /*Read block protect byte */
        if(xt26gx_Readprotected(priv) != OK) {
            lldbg("ERROR: failed to Read block protect byte\n");
        }
        Read_status_reg(priv);
        /*unloack the block protect byte */
        if(xt26gx_unlockprotected(priv) != OK) {
            lldbg("ERROR: failed to unloack the block protect byte\n");
        }
        nand_status_read(priv, 0xC0);
        /*Read block protect byte */
        if(xt26gx_Readprotected(priv) != OK) {
            lldbg("ERROR: failed to Read block protect byte\n");
        }
        nand_status_read(priv,0xC0);
    return (FAR struct mtd_dev_s *)priv;
}
