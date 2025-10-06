# TizenRT NAND Flash with LittleFS - Part 4: Operations and Error Handling
## For RTL8730E Chipset - Beginner's Guide

---

## Table of Contents
1. [Complete Operation Flows](#complete-operation-flows)
2. [Error Handling Strategy](#error-handling-strategy)
3. [Bad Block Management](#bad-block-management)
4. [ECC and Data Integrity](#ecc-and-data-integrity)
5. [Manufacturer-Specific Handling](#manufacturer-specific-handling)
6. [Performance Optimization](#performance-optimization)
7. [Debugging and Troubleshooting](#debugging-and-troubleshooting)
8. [Best Practices](#best-practices)

---

## 1. Complete Operation Flows

### 1.1 Complete Read Flow

**Scenario**: Application reads 512 bytes from a file

```c
// Application code
int fd = open("/mnt/data.bin", O_RDONLY);
char buffer[512];
read(fd, buffer, 512);
close(fd);
```

**Detailed execution trace**:

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 0: Application                                        │
└───────────────────┬─────────────────────────────────────────┘
                    │ read(fd, buffer, 512)
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 1: VFS (Virtual File System)                          │
│ Function: file_read()                                       │
│ Actions:                                                    │
│   - Validate file descriptor                                │
│   - Check permissions (O_RDONLY)                            │
│   - Route to LittleFS operations                            │
└───────────────────┬─────────────────────────────────────────┘
                    │ fs_ops->read(file, buffer, 512)
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 2: LittleFS Core                                      │
│ Function: lfs_file_read()                                   │
│ Actions:                                                    │
│   - Get file handle metadata                                │
│   - Current position: offset=1024                           │
│   - File size: 8192 bytes                                   │
│   - Calculate: block 0, page offset 1024                    │
│   - Call lfs_file_rawread()                                 │
│     └─ lfs_bd_read(block=0, off=1024, buf, 512)             │
│        └─ lfs_config->read(0, 1024, buffer, 512)            │
└───────────────────┬─────────────────────────────────────────┘
                    │ lfs_config->read callback
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 3: NAND Adapter                                       │
│ Function: lfs_nand_read()                                   │
│ Actions:                                                    │
│   1. Calculate NAND address:                                │
│      NandAddr = 0x0A000000 + (0 × 131072) + 1024            │
│               = 0x0A000400                                  │
│   2. Convert to page address:                               │
│      PageAddr = 0x0A000400 >> 12 = 0xA000                   │
│      (This is page 0 in the partition)                      │
│   3. Lock hardware: device_mutex_lock(RT_DEV_LOCK_FLASH)    │
│   4. Call FTL: NAND_FTL_ReadPage(0xA000, buffer)            │
└───────────────────┬─────────────────────────────────────────┘
                    │ NAND_FTL_ReadPage(0xA000, buffer)
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 4: NAND FTL                                           │
│ Function: NAND_FTL_ReadPage()                               │
│ Actions:                                                    │
│   1. Check initialization: nand->Initialized == 1 ✓         │
│   2. Select target/die: NF_SelectTarget(nand, 0xA000)       │
│      - Single-die chip, no action needed                    │
│   3. Check bad block: NF_IsBad(nand, 0xA000, &is_bad)       │
│      a. Get block address: 0xA000 & 0xFFC0 = 0xA000         │
│      b. Read OOB bytes: NAND_Page_Read(0xA000, 2048, 2, bb) │
│      c. Check marker: [0xFF, 0xFF] = good block ✓           │
│   4. Read page: status = NAND_Page_Read(0xA000, 0, 2048, buffer) │
│   5. Check ECC: ecc = ops->GetEccStatus(nand, status)       │
│      - Status byte = 0x00 (no errors)                       │
│      - ECC bits [6:4] = 0b000 = no bitflips                 │
│      - Return HAL_OK                                        │
└───────────────────┬─────────────────────────────────────────┘
                    │ NAND_Page_Read(0xA000, 0, 2048, buffer)
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 5: SPI NAND Driver                                    │
│ Function: NAND_Page_Read()                                  │
│ Actions:                                                    │
│   Stage 1: Array → Cache                                    │
│   1. NAND_Page_Read_ArrayToCache(0xA000)                    │
│      a. Format address: [0x00][0xA0][0x00]                  │
│      b. NAND_TxCmd(0x13, 3, addr, 0, NULL)                  │
│         - Send PAGE READ command                            │
│      c. NAND_WaitBusy(WAIT_FLASH_BUSY)                      │
│         - Poll status register OIP bit                      │
│         - Takes ~35 µs                                      │
│      d. Return status = 0x00                                │
│                                                             │
│   Stage 2: Cache → Host                                     │
│   2. NAND_RxData(0x0B, 0, 2048, buffer)                     │
│      - Send FAST READ command (0x0B)                        │
│      - Column address: [0x00][0x00]                         │
│      - Dummy byte: 0x00                                     │
│      - Read 2048 bytes via SPI                              │
│      - Takes ~200 µs at 100 MHz                             │
└───────────────────┬─────────────────────────────────────────┘
                    │ NAND_RxData() → SPIC register access
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 6: SPIC Hardware                                      │
│ Function: NAND_RxData()                                     │
│ Actions:                                                    │
│   1. NAND_UserModeEn(ENABLE)                                │
│      - Switch SPIC to user control                          │
│   2. Configure RX mode:                                     │
│      spi_flash->CTRLR0 |= TMOD(3)  // RX-only               │
│      spi_flash->RX_NDF = 2048      // Expect 2048 bytes     │
│   3. Push to TX FIFO:                                       │
│      spi_flash->DR[0] = 0x0B       // FAST READ command     │
│      spi_flash->DR[0] = 0x00       // Dummy                 │
│      spi_flash->DR[0] = 0x00       // Column addr high      │
│      spi_flash->DR[0] = 0x00       // Column addr low       │
│   4. Enable SPI: spi_flash->SSIENR = BIT_SPIC_EN            │
│   5. Read from RX FIFO:                                     │
│      while (rx_num < 2048) {                                │
│        if (spi_flash->SR & BIT_RFNE) {  // FIFO not empty   │
│          buffer[rx_num++] = spi_flash->DR[0];               │
│        }                                                    │
│      }                                                      │
│   6. Wait for completion: NAND_WaitBusy(WAIT_TRANS_COMPLETE)│
└───────────────────┬─────────────────────────────────────────┘
                    │ SPI transaction on hardware
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 7: W25N NAND Flash Chip                               │
│ Internal operations:                                        │
│   Stage 1 (PAGE READ 0x13):                                 │
│     1. Receive page address 0xA000                          │
│     2. Read page from NAND array to internal cache          │
│     3. Perform ECC correction                               │
│     4. Set ECC status bits in status register               │
│   Stage 2 (FAST READ 0x0B):                                 │
│     1. Receive column address 0x0000                        │
│     2. Stream data from cache starting at offset 0          │
│     3. Clock out 2048 bytes on MISO line                    │
└─────────────────────────────────────────────────────────────┘
                    ↓ Return path (unwinding)
┌─────────────────────────────────────────────────────────────┐
│ SPIC Hardware → Driver: Return (data in buffer)             │
│ Driver → FTL: Return status=0x00, HAL_OK                    │
│ FTL → Adapter: Unlock hardware, return HAL_OK               │
│ Adapter → LittleFS: Return LFS_ERR_OK                       │
│ LittleFS → VFS: Return 512 (bytes read)                     │
│ VFS → Application: Return 512                               │
└─────────────────────────────────────────────────────────────┘

Total time: ~240 µs
  - Array→Cache: 35 µs
  - Cache→Host: 200 µs
  - Overhead: ~5 µs
```

**Only first 512 bytes needed**:
```
LittleFS reads entire page (2048 bytes) because:
  - NAND page granularity = 2048 bytes
  - Can't read partial pages

But only copies 512 bytes to application buffer:
  memcpy(app_buffer, nand_buffer + offset, 512);

Remaining 1536 bytes are cached by LittleFS for future reads.
```

### 1.2 Complete Write Flow

**Scenario**: Application writes 100 bytes to a new file

```c
int fd = open("/mnt/newfile.txt", O_WRONLY | O_CREAT);
write(fd, "Hello World", 11);
close(fd);
```

**Detailed execution trace**:

```
┌─────────────────────────────────────────────────────────────┐
│ Layer 0: Application                                        │
└───────────────────┬─────────────────────────────────────────┘
                    │ write(fd, "Hello World", 11)
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 1: VFS                                                │
│ Function: file_write()                                      │
│ Actions:                                                    │
│   - Validate fd, check O_WRONLY permission                  │
│   - Route to LittleFS                                       │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 2: LittleFS Core                                      │
│ Function: lfs_file_write()                                  │
│ Actions:                                                    │
│   1. Check if file is new: yes, allocate file ID = 0x0010   │
│   2. Allocate block for data:                               │
│      - Search free block list                               │
│      - Find block 42 is free                                │
│      - Mark as allocated                                    │
│   3. Prepare write:                                         │
│      - Block 42 needs erase before write                    │
│      - Offset 0 (first page)                                │
│      - Need to write 11 bytes                               │
│      - Round up to page size: 2048 bytes                    │
│      - Pad with 0xFF: "Hello World" + [0xFF × 2037]         │
│   4. Call lfs_bd_prog(block=42, off=0, buffer, 2048)        │
│      └─ lfs_config->prog(42, 0, buffer, 2048)               │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 3: NAND Adapter                                       │
│ Function: lfs_nand_prog()                                   │
│ Actions:                                                    │
│   1. Offset = 0, so do_erase = 1                            │
│   2. Calculate address:                                     │
│      NandAddr = 0x0A000000 + (42 × 131072) + 0              │
│               = 0x0A000000 + 5505024                        │
│               = 0x0A540000                                  │
│      PageAddr = 0x0A540000 >> 12 = 0xA540                   │
│   3. Lock: device_mutex_lock(RT_DEV_LOCK_FLASH)             │
│   4. Call: NAND_FTL_WritePage(0xA540, buffer, do_erase=1)   │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 4: NAND FTL                                           │
│ Function: NAND_FTL_WritePage()                              │
│ Actions:                                                    │
│   1. Check init: ✓                                          │
│   2. Select target: (single-die, no-op)                     │
│   3. Check bad block: NF_IsBad(nand, 0xA540, &is_bad)       │
│      - Block addr: 0xA540                                   │
│      - Read OOB: [0xFF, 0xFF] = good ✓                      │
│   4. Erase block (do_erase=1):                              │
│      Block addr: 0xA540                                     │
│      LastErasedBlockAddr: 0xFFFFFFFF (none cached)          │
│      Block not in cache, perform erase:                     │
│        ret = NF_EraseBlock(nand, 0xA540)                    │
│          └─ NAND_Erase(0xA540)                              │
│             - NAND_WriteEn()                                │
│             - NAND_TxCmd(0xD8, [0x00][0xA5][0x40])          │
│             - Wait ~5 ms                                    │
│             - Check E_FAIL bit = 0 (success)                │
│             - Return 0                                      │
│      Cache result: LastErasedBlockAddr = 0xA540             │
│   5. Write page: NAND_Page_Write(0xA540, 0, 2048, buffer)   │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 5: SPI NAND Driver                                    │
│ Function: NAND_Page_Write()                                 │
│ Actions:                                                    │
│   Stage 1: Write Enable                                     │
│   1. NAND_WriteEn()                                         │
│      - NAND_TxCmd(0x06)  // WRITE ENABLE                    │
│      - Wait for WEL bit = 1                                 │
│                                                             │
│   Stage 2: Host → Cache                                     │
│   2. Select command:                                        │
│      - Quad enabled? Yes (Winbond default)                  │
│      - cmd = 0x32 (QUAD PROGRAM LOAD)                       │
│   3. NAND_TxData(0x32, 0, 2048, buffer)                     │
│      - Send command 0x32                                    │
│      - Column address [0x00][0x00]                          │
│      - Stream 2048 bytes of data (4-bit mode)               │
│      - Takes ~50 µs                                         │
│                                                             │
│   Stage 3: Cache → Array                                    │
│   4. NAND_Page_Write_Program_Execute(0xA540)                │
│      - Format address: [0x00][0xA5][0x40]                   │
│      - NAND_TxCmd(0x10, addr)  // PROGRAM EXECUTE           │
│      - Wait for completion (~400 µs)                        │
│      - Check P_FAIL bit = 0 (success)                       │
│      - Return 0                                             │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 6: SPIC Hardware                                      │
│ Function: NAND_TxData() for QUAD PROGRAM LOAD               │
│ Actions:                                                    │
│   1. Switch to user mode                                    │
│   2. Configure TX mode with quad data:                      │
│      spi_flash->CTRLR0 &= ~TMOD(3);                         │
│      spi_flash->CTRLR0 |= TMOD(1) | DATA_CH(2);             │
│   3. Set TX count: spi_flash->TX_NDF = 2048                 │
│   4. Push command and address:                              │
│      spi_flash->DR[0] = 0x32  // QPP command                │
│      spi_flash->DR[0] = 0x00  // CA high                    │
│      spi_flash->DR[0] = 0x00  // CA low                     │
│   5. Pre-load FIFO (prevent underrun):                      │
│      for (i=0; i<28; i++) {  // Fill 32-byte FIFO           │
│        spi_flash->DR[0] = buffer[i];                        │
│      }                                                      │
│   6. Enable SPI: spi_flash->SSIENR = BIT_SPIC_EN            │
│   7. Continue pushing data:                                 │
│      while (tx_num < 2048) {                                │
│        if (spi_flash->SR & BIT_TFNF) {  // TX FIFO not full │
│          spi_flash->DR[0] = buffer[tx_num++];               │
│        }                                                    │
│      }                                                      │
│   8. Wait for completion                                    │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Layer 7: W25N NAND Flash                                    │
│ Operations:                                                 │
│   Erase (0xD8):                                             │
│     1. Receive block address                                │
│     2. Erase all 64 pages in block                          │
│     3. Set all bytes to 0xFF                                │
│     4. Update block erase count                             │
│     5. Takes ~5000 µs                                       │
│                                                             │
│   Program Load (0x32):                                      │
│     1. Receive column address 0x0000                        │
│     2. Receive 2048 bytes via 4-bit interface               │
│     3. Store in internal page cache                         │
│                                                             │
│   Program Execute (0x10):                                   │
│     1. Receive page address                                 │
│     2. Transfer cache data to NAND array                    │
│     3. Verify programming                                   │
│     4. Takes ~400 µs                                        │
└─────────────────────────────────────────────────────────────┘
                    ↓ Return path
┌─────────────────────────────────────────────────────────────┐
│ Driver → FTL: Return HAL_OK                                 │
│ FTL → Adapter: Unlock, return HAL_OK                        │
│ Adapter → LittleFS: Return LFS_ERR_OK                       │
│ LittleFS:                                                   │
│   - Update file metadata (size = 11 bytes)                  │
│   - Update directory entry                                  │
│   - Write metadata pair to blocks 2-3                       │
│ LittleFS → VFS: Return 11 (bytes written)                   │
│ VFS → Application: Return 11                                │
└─────────────────────────────────────────────────────────────┘

Total time: ~5500 µs (5.5 ms)
  - Block erase: 5000 µs
  - Program load: 50 µs
  - Program execute: 400 µs
  - Overhead: ~50 µs
```

### 1.3 Block Erase Flow

**Scenario**: Erasing a block before writing

```
┌─────────────────────────────────────────────────────────────┐
│ Trigger: LittleFS needs to allocate new block               │
└───────────────────┬─────────────────────────────────────────┘
                    │ lfs_nand_erase(block=100)
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Adapter: lfs_nand_erase()                                   │
│   1. Verify block size = 0x20000 (128 KB) ✓                 │
│   2. Calculate address:                                     │
│      NandAddr = 0x0A000000 + (100 × 131072)                 │
│               = 0x0A000000 + 13107200                       │
│               = 0x0AC80000                                  │
│      PageAddr = 0x0AC80000 >> 12 = 0xAC80                   │
│   3. Lock hardware                                          │
│   4. Call NAND_FTL_EraseBlock(0xAC80, force=0)              │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ FTL: NAND_FTL_EraseBlock()                                  │
│   1. Select target: (single-die)                            │
│   2. Check bad block: NF_IsBad(nand, 0xAC80, &is_bad)       │
│      Result: is_bad = 0 (good block) ✓                      │
│   3. Allow erase (force=0, block is good)                   │
│   4. Call NF_EraseBlock(nand, 0xAC80)                       │
│        └─ NAND_Erase(0xAC80)                                │
│           a. NAND_WriteEn()                                 │
│              - Send 0x06 (WRITE ENABLE)                     │
│              - Wait for WEL=1                               │
│           b. Format address: [0x00][0xAC][0x80]             │
│           c. NAND_TxCmd(0xD8, addr)  // BLOCK ERASE         │
│           d. NAND_WaitBusy(WAIT_FLASH_BUSY)                 │
│              Poll loop:                                     │
│                i=0: status=0x01 (OIP=1, busy)               │
│                i=1: status=0x01 (still busy)                │
│                ...                                          │
│                i=20000: status=0x00 (OIP=0, ready)          │
│              Returns status=0x00                            │
│           e. Check E_FAIL bit (bit 2):                      │
│              status & 0x04 = 0 (no failure) ✓               │
│           f. Return 0 (success)                             │
│   5. Return HAL_OK                                          │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ Adapter: lfs_nand_erase()                                   │
│   5. Unlock hardware                                        │
│   6. Return LFS_ERR_OK                                      │
└───────────────────┬─────────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────────┐
│ LittleFS:                                                   │
│   - Mark block 100 as erased                                │
│   - Update allocation bitmap                                │
│   - Block ready for programming                             │
└─────────────────────────────────────────────────────────────┘

Time: ~5000 µs (5 ms) for erase operation
```

---

## 2. Error Handling Strategy

### 2.1 Error Code Hierarchy

**Error Code Flow**:

```
Hardware Error          FTL Translation         Adapter Translation    LittleFS Handling
──────────────          ───────────────         ───────────────────    ─────────────────
E_FAIL bit set    →     UERR_NAND_WORN_BLOCK → LFS_ERR_CORRUPT    →   Skip block
P_FAIL bit set    →     UERR_NAND_WORN_BLOCK → LFS_ERR_CORRUPT    →   Skip block
OOB = 0x00        →     UERR_NAND_BAD_BLOCK  → LFS_ERR_CORRUPT    →   Skip block
ECC uncorrectable →     UERR_NAND_BITFLIP_FATAL → LFS_ERR_CORRUPT →   Try redundant copy
Timeout           →     HAL_TIMEOUT          → LFS_ERR_IO         →   Retry or fail
```

**Complete Error Code List**:

```c
// HAL Layer (lfs_nand_ftl.c)
#define HAL_OK                    0x00
#define HAL_TIMEOUT               0xFF
#define HAL_ERR_PARA              0x01
#define HAL_ERR_HW                0x02
#define HAL_ERR_UNKNOWN           0x03

// FTL Errors
#define UERR_INIT                 0x20  // Not initialized
#define UERR_PERM                 0x21  // Permission denied
#define UERR_PROTO                0x22  // Protocol error
#define UERR_CHK                  0x23  // Checksum error
#define UERR_OVERRANGE            0x24  // Out of range
#define UERR_NAND_BAD_BLOCK       0x25  // Factory/existing bad block
#define UERR_NAND_WORN_BLOCK      0x26  // Block just failed (runtime bad)
#define UERR_NAND_BITFLIP_WARN    0x27  // Correctable bitflips (low count)
#define UERR_NAND_BITFLIP_ERROR   0x28  // Correctable bitflips (high count)
#define UERR_NAND_BITFLIP_FATAL   0x29  // Uncorrectable bitflips

// LittleFS Errors (lfs.h)
#define LFS_ERR_OK                0     // No error
#define LFS_ERR_IO                -5    // Error during device operation
#define LFS_ERR_CORRUPT           -84   // Corrupted
#define LFS_ERR_NOENT             -2    // No directory entry
#define LFS_ERR_EXIST             -17   // Entry already exists
#define LFS_ERR_NOTDIR            -20   // Entry is not a dir
#define LFS_ERR_ISDIR             -21   // Entry is a dir
#define LFS_ERR_NOTEMPTY          -39   // Dir is not empty
#define LFS_ERR_BADF              -9    // Bad file number
#define LFS_ERR_FBIG              -27   // File too large
#define LFS_ERR_INVAL             -22   // Invalid parameter
#define LFS_ERR_NOSPC             -28   // No space left on device
#define LFS_ERR_NOMEM             -12   // No more memory available
#define LFS_ERR_NOATTR            -61   // No data/attr available
#define LFS_ERR_NAMETOOLONG       -36   // File name too long
```

### 2.2 Error Handling by Layer

**Application Layer**:
```c
int fd = open("/mnt/file.txt", O_RDONLY);
if (fd < 0) {
    switch (errno) {
        case ENOENT:   // File doesn't exist
            printf("File not found\n");
            break;
        case EIO:      // I/O error (flash problem)
            printf("Flash error, check hardware\n");
            break;
        case ENOSPC:   // No space (all blocks used/bad)
            printf("Disk full\n");
            break;
        default:
            printf("Error: %d\n", errno);
    }
    return -1;
}

ssize_t ret = read(fd, buffer, size);
if (ret < 0) {
    // Read error occurred
    perror("read");
    close(fd);
    return -1;
} else if (ret < size) {
    // Partial read (reached EOF)
    printf("Read %d bytes (expected %d)\n", ret, size);
}
```

**LittleFS Layer**:
```c
// Internal LittleFS error handling
int lfs_file_write(lfs_t *lfs, lfs_file_t *file,
                   const void *buffer, lfs_size_t size)
{
    // Try to write to block
    int err = lfs_bd_prog(lfs, block, off, buffer, size);

    if (err == LFS_ERR_CORRUPT) {
        // Block is bad, try different block
        lfs_block_t new_block = lfs_alloc(lfs);
        if (new_block == 0xFFFFFFFF) {
            return LFS_ERR_NOSPC;  // No free blocks
        }

        // Retry with new block
        err = lfs_bd_prog(lfs, new_block, off, buffer, size);
    }

    return err;
}
```

**Adapter Layer**:
```c
int lfs_nand_prog(const struct lfs_config *c, lfs_block_t block,
                  lfs_off_t off, const void *buffer, lfs_size_t size)
{
    // ... address calculation ...

    ret = NAND_FTL_WritePage(PageAddr, (uint8_t *)buffer, do_erase);

    // Translate FTL errors to LittleFS errors
    switch (ret) {
        case HAL_OK:
            return LFS_ERR_OK;

        case UERR_NAND_BAD_BLOCK:
        case UERR_NAND_WORN_BLOCK:
            // Block is bad, tell LittleFS to skip it
            return LFS_ERR_CORRUPT;

        case HAL_TIMEOUT:
            return LFS_ERR_IO;

        default:
            return LFS_ERR_IO;
    }
}
```

**FTL Layer**:
```c
u8 NAND_FTL_WritePage(u32 addr, u8 *buf, u8 do_erase)
{
    // ... initialization checks ...

    // Try to erase block
    if (do_erase) {
        ret = NF_EraseBlock(nand, block_addr);
        if (ret != HAL_OK && ret != UERR_NAND_WORN_BLOCK) {
            return ret;  // Erase failed, propagate error
        }
    }

    // Try to write page
    ret = NAND_Page_Write(addr, 0, info->PageSize, buf);

    if (ret == 0) {
        return HAL_OK;  // Success
    } else {
        // Write failed, mark block as bad
        FS_DBG(FS_ERROR, "Write failed at 0x%08X: 0x%02X", addr, ret);
        NF_MarkBad(nand, addr);
        return UERR_NAND_WORN_BLOCK;  // Tell upper layer block is now bad
    }
}
```

**Driver Layer**:
```c
u8 NAND_Page_Write(u32 PageAddr, u32 ByteAddr, u32 ByteLen, u8 *pData)
{
    // ... write enable ...
    // ... program load ...

    // Program execute
    status = NAND_Page_Write_Program_Execute(PageAddr);

    // Check for program failure
    if (status & flash_init_para.FLASH_PFail_bit) {
        // P_FAIL bit set in status register
        return status;  // Return error status to FTL
    } else {
        return 0;  // Success
    }
}
```

### 2.3 Retry Strategy

**No Automatic Retries at Lower Layers**:
```c
// FTL does NOT retry failed operations
u8 NF_EraseBlock(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    u8 ret = NAND_Erase(addr);

    if (ret != 0) {
        // Erase failed - mark bad immediately, don't retry
        NF_MarkBad(nand, addr);
        return UERR_NAND_WORN_BLOCK;
    }

    return HAL_OK;
}
```

**Why no retries?**
- NAND block failures are usually permanent
- Retrying wastes time (erase takes 5ms)
- Better to mark bad and move on
- LittleFS has redundancy at higher level

**LittleFS Handles Retries**:
```c
// LittleFS internally retries with different blocks
// Application doesn't see the retry

Write attempt 1: Block 42 → Returns LFS_ERR_CORRUPT
  └─ LittleFS: "Block 42 is bad, try block 43"

Write attempt 2: Block 43 → Returns LFS_ERR_OK
  └─ LittleFS: "Success, update metadata"

Application sees: write() returns 11 (success)
```

### 2.4 Error Recovery Examples

**Example 1: Read Error from Bad Block**

```
Application: read(fd, buf, 2048)
   ↓
LittleFS: Read from block 50
   ↓
Adapter: lfs_nand_read(block=50, ...)
   ↓
FTL: NAND_FTL_ReadPage(page=0xC80)
   ├─ NF_IsBad(0xC80) → is_bad=1 (bad block marker found)
   └─ Return UERR_NAND_BAD_BLOCK
   ↓
Adapter: Translate to LFS_ERR_CORRUPT
   ↓
LittleFS: Check metadata redundancy
   ├─ File has backup in block 51
   └─ Read from block 51 instead
   ↓
Success: Application receives data from block 51
```

**Example 2: Write Failure During Program**

```
Application: write(fd, buf, 2048)
   ↓
LittleFS: Allocate block 100
   ↓
Adapter: lfs_nand_prog(block=100, off=0, ...)
   ↓
FTL: NAND_FTL_WritePage(page=0x1900, do_erase=1)
   ├─ NF_EraseBlock(0x1900) → HAL_OK ✓
   ├─ NAND_Page_Write(0x1900, ...) → returns 0x04 (P_FAIL)
   ├─ NF_MarkBad(0x1900) → Writes 0x00 to OOB
   └─ Return UERR_NAND_WORN_BLOCK
   ↓
Adapter: Return LFS_ERR_CORRUPT
   ↓
LittleFS: Block 100 is bad
   ├─ Mark block 100 as unusable in metadata
   ├─ Allocate block 101
   ├─ Retry write to block 101
   └─ Success
   ↓
Application: write() returns 2048 (success, didn't notice retry)
```

**Example 3: Uncorrectable ECC Error**

```
Application: read(fd, buf, 2048)
   ↓
LittleFS: Read from block 75
   ↓
FTL: NAND_FTL_ReadPage(page=0x1770)
   ├─ NAND_Page_Read() → status=0x20 (ECC bits=0b010)
   ├─ ops->GetEccStatus(0x20) → UERR_NAND_BITFLIP_FATAL
   └─ Return UERR_NAND_BITFLIP_FATAL
   ↓
Adapter: Return LFS_ERR_CORRUPT
   ↓
LittleFS: Data corrupted, check redundancy
   ├─ This file has metadata pair
   ├─ Try reading backup metadata
   ├─ Backup says data is also in block 76
   ├─ Read from block 76 → Success
   └─ Schedule block 75 for relocation
   ↓
Application: Receives valid data from block 76
```

---

## 3. Bad Block Management

### 3.1 Bad Block Detection

**Factory Bad Blocks**:
```c
// Detected at first use
static u8 NF_IsBad(NAND_FTL_DeviceTypeDef *nand, u32 addr, u8 *value)
{
    u8 data[2];
    u32 block_addr = NF_GetBlockAddr(nand, addr);

    // Read OOB bytes 0-1 from first page of block
    NAND_Page_Read(block_addr, info->PageSize, 2, data);

    // Check bad block marker
    if ((data[0] != 0xFF) || (data[1] != 0xFF)) {
        *value = 1;  // Bad block
        FS_DBG(FS_WARNING, "Factory bad block at 0x%08X", addr);
    } else {
        *value = 0;  // Good block
    }

    return HAL_OK;
}
```

**Runtime Bad Blocks**:
```c
// Detected when operation fails
u8 NF_EraseBlock(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    u8 ret = NAND_Erase(addr);

    if (ret != 0) {
        // Block failed during erase
        FS_DBG(FS_ERROR, "Block 0x%08X failed erase, marking bad", addr);

        // Mark as bad for future
        ret = NF_MarkBad(nand, addr);

        if (ret == HAL_OK) {
            return UERR_NAND_WORN_BLOCK;  // Newly bad block
        }
    }

    return HAL_OK;
}
```

### 3.2 Bad Block Marking

**Marking Strategy**:
```c
static u8 NF_MarkBad(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    u8 data[2] = {NF_BAD_BLOCK, NF_BAD_BLOCK};  // [0x00, 0x00]
    u32 block_addr = NF_GetBlockAddr(nand, addr);

    // Write bad block marker to OOB bytes 0-1 of first page
    ret = NAND_Page_Write(block_addr, info->PageSize, 2, data);

    if (ret == 0) {
        FS_DBG(FS_WARNING, "Marked block 0x%08X as bad", addr);
        return HAL_OK;
    } else {
        // Couldn't even mark it bad - very bad!
        FS_DBG(FS_ERROR, "Failed to mark block 0x%08X as bad!", addr);
        return HAL_ERR_HW;
    }
}
```

**Bad Block Marker Format**:
```
Good Block:
┌──────────────┬──────────────┬─────────┐
│ Main Area    │ OOB Byte 0   │ OOB ... │
│ 2048 bytes   │ 0xFF         │ 0xFF... │
└──────────────┴──────────────┴─────────┘

Factory Bad Block:
┌──────────────┬──────────────┬─────────┐
│ Main Area    │ OOB Byte 0   │ OOB ... │
│ (undefined)  │ 0x00         │ 0x**    │
└──────────────┴──────────────┴─────────┘

Runtime Bad Block:
┌──────────────┬──────────────┬─────────┐
│ Main Area    │ OOB Byte 0   │ OOB ... │
│ (old data)   │ 0x00         │ 0x00    │
└──────────────┴──────────────┴─────────┘
```

### 3.3 Bad Block Statistics

**Typical Bad Block Distribution**:
```
New NAND Flash (256 MB, 2048 blocks):
  Factory bad blocks: 20-40 blocks (1-2%)
  Good blocks: 2008-2028 blocks
  Usable capacity: ~250 MB

After 1 year of heavy use:
  Factory bad: 20 blocks
  Runtime bad: 5 blocks
  Good blocks: 2023 blocks
  Usable capacity: ~252 MB

After 5 years (reaching erase limit):
  Factory bad: 20 blocks
  Runtime bad: 200 blocks (10%)
  Good blocks: 1828 blocks
  Usable capacity: ~228 MB
  (Still usable, but should consider replacement)
```

**Bad Block Growth Rate**:
```c
// Theoretical calculation
// NAND endurance: 100,000 erase cycles per block
// Wear leveling distributes erases evenly

Total writes over lifetime:
  = 2048 blocks × 100,000 erases
  = 204,800,000 block erases
  = 204.8 million × 128 KB
  = ~25.6 TB total write endurance

Realistic usage:
  10 GB/day writes
  = ~25,000 days until wear-out
  = ~68 years

Conclusion: Bad blocks from wear-out are rare in typical IoT use
```

### 3.4 Bad Block Avoidance

**All Operations Check Bad Blocks First**:

```c
// Read operation
u8 NAND_FTL_ReadPage(u32 addr, u8 *buf)
{
    u8 is_bad_block;

    // CHECK BEFORE READING
    NF_IsBad(nand, addr, &is_bad_block);
    if (is_bad_block) {
        return UERR_NAND_BAD_BLOCK;  // Don't even try to read
    }

    // Only read if block is good
    status = NAND_Page_Read(addr, 0, info->PageSize, buf);
    // ...
}

// Write operation
u8 NAND_FTL_WritePage(u32 addr, u8 *buf, u8 do_erase)
{
    u8 is_bad_block;

    // CHECK BEFORE WRITING
    NF_IsBad(nand, addr, &is_bad_block);
    if (is_bad_block) {
        return UERR_NAND_BAD_BLOCK;  // Don't waste time writing
    }

    // Only write if block is good
    // ...
}

// Erase operation
u8 NAND_FTL_EraseBlock(u32 addr, u8 force)
{
    u8 is_bad_block;

    NF_IsBad(nand, addr, &is_bad_block);

    // CHECK UNLESS FORCED
    if (is_bad_block && !force) {
        FS_DBG(FS_WARNING, "Refusing to erase bad block 0x%08X", addr);
        return UERR_NAND_BAD_BLOCK;
    } else if (is_bad_block && force) {
        FS_DBG(FS_WARNING, "Force erasing bad block 0x%08X", addr);
        // Continue with erase
    }

    // ...
}
```

**Force Erase Use Case**:
```c
// Rare: Only for factory reset or recovery
void factory_reset_nand(void)
{
    u32 page;

    // Erase ALL blocks, even bad ones
    for (u32 block = 0; block < 2048; block++) {
        page = block << 6;  // Convert block to page address

        // force=1 to erase even bad blocks
        NAND_FTL_EraseBlock(page, 1);
    }

    // Note: Bad block markers persist because OOB area
    // might not fully erase on bad blocks
}
```

---

## 4. ECC and Data Integrity

### 4.1 ECC Status Checking

**ECC Status Bits** (Status Register [6:4]):
```
0b000 (0x00): No bitflips detected
              → HAL_OK
              → Data is perfect

0b001 (0x10): Bitflips corrected (low count)
              → UERR_NAND_BITFLIP_WARN
              → Data OK, but monitor block health

0b011 (0x30): Bitflips corrected (high count, near ECC limit)
              → UERR_NAND_BITFLIP_ERROR
              → Data OK, but should refresh (read+rewrite)

0b010 (0x20): Bitflips uncorrectable
              → UERR_NAND_BITFLIP_FATAL
              → Data corrupted, use backup if available
```

### 4.2 Manufacturer-Specific ECC Parsing

**Winbond ECC Status** (`lfs_nand_ftl_mfg.c:247-269`):
```c
static u8 NAND_FTL_Winbond_GetEccStatus(NAND_FTL_DeviceTypeDef *nand, u8 status)
{
    u8 ecc_bits = (status & NF_STS_ECC_MASK) >> 4;  // Extract bits [6:4]

    switch (ecc_bits) {
        case 0b000:  // No errors
            return HAL_OK;

        case 0b001:  // 1-4 bitflips corrected
            // Conservative: assume maximum bitflips
            return UERR_NAND_BITFLIP_ERROR;

        case 0b010:  // Uncorrectable
            return UERR_NAND_BITFLIP_FATAL;

        default:     // Reserved
            return HAL_OK;
    }
}
```

**Micron ECC Status** (`lfs_nand_ftl_mfg.c:397-423`):
```c
static u8 NAND_FTL_Micron_GetEccStatus(NAND_FTL_DeviceTypeDef *nand, u8 status)
{
    u8 ecc_bits = (status & NF_STS_ECC_MASK) >> 4;

    switch (ecc_bits) {
        case 0b00:  // No bitflips
            return HAL_OK;

        case 0b01:  // 1-3 bitflips OR 7-8 bitflips
            // Check additional bit for distinction
            if (status & NF_STS_MICRON_ECCS2) {  // Bit 6
                // 7-8 bitflips (near limit)
                return UERR_NAND_BITFLIP_ERROR;
            } else {
                // 1-3 bitflips (low count)
                return UERR_NAND_BITFLIP_WARN;
            }

        case 0b10:  // Uncorrectable
            return UERR_NAND_BITFLIP_FATAL;

        case 0b11:  // 4-6 bitflips
            return UERR_NAND_BITFLIP_WARN;
    }
}
```

**GigaDevice ECC Status** (`lfs_nand_ftl_mfg.c:320-342`):
```c
static u8 NAND_FTL_GigaDevice_GetEccStatus(NAND_FTL_DeviceTypeDef *nand, u8 status)
{
    u8 ecc_bits = (status & NF_STS_ECC_MASK) >> 4;

    switch (ecc_bits) {
        case 0b00:  // No errors
            return HAL_OK;

        case 0b01:  // 1-7 bitflips (corrected, under threshold)
            return UERR_NAND_BITFLIP_WARN;

        case 0b10:  // Uncorrectable
            return UERR_NAND_BITFLIP_FATAL;

        case 0b11:  // Exactly 8 bitflips (at ECC limit)
            return UERR_NAND_BITFLIP_ERROR;
    }
}
```

**Macronix Extended ECC Status** (`lfs_nand_ftl_mfg.c:356-383`):
```c
static u8 NAND_FTL_Macronix_GetEccStatus(NAND_FTL_DeviceTypeDef *nand, u8 status)
{
    u8 ecc_bits = (status & NF_STS_ECC_MASK) >> 4;
    u8 eccsr;

    switch (ecc_bits) {
        case 0b00:  // No errors
            return HAL_OK;

        case 0b01:  // Bitflips corrected
            // Read extended ECC status register
            eccsr = NAND_FTL_GetMfgStatus(NF_ECCSR_MACRONIX_CMD,
                                         NF_ECCSR_MACRONIX_ADDR);
            // Command 0x7C, Address 0x10

            // Extract bitflip count from ECCSR
            u8 bitflip_count = eccsr & NF_ECCSR_MACRONIX_MASK;

            if (bitflip_count == 4) {
                // Exactly 4 bitflips (at threshold for 4-bit ECC)
                return UERR_NAND_BITFLIP_ERROR;
            } else {
                // 1-3 bitflips
                return UERR_NAND_BITFLIP_WARN;
            }

        case 0b10:  // Uncorrectable
            return UERR_NAND_BITFLIP_FATAL;

        default:
            return HAL_OK;
    }
}
```

### 4.3 ECC-Based Block Health Monitoring

**Function**: `NAND_FTL_ReadBlockStatus()` (`lfs_nand_ftl.c:400-486`):

```c
u8 NAND_FTL_ReadBlockStatus(u32 addr, u8 *buf, u8 *block_status, u32 *page_status)
{
    u32 i, page_addr;
    u8 ret, ecc_status, worn_status;
    u8 has_page_warn = 0, has_page_error = 0, has_page_fatal = 0;

    // Read all pages in block
    for (i = 0; i < info->PagesPerBlock; i++) {
        page_addr = NF_GetBlockAddr(nand, addr) + i;

        // Read page
        ret = NAND_Page_Read(page_addr, 0, info->PageSize, buf);

        // Get ECC status
        ecc_status = ops->GetEccStatus(nand, ret);

        // Convert to wear status (2 bits per page)
        switch (ecc_status) {
            case HAL_OK:
                worn_status = 0b00;  // OK
                break;
            case UERR_NAND_BITFLIP_WARN:
                worn_status = 0b01;  // WARN
                has_page_warn = 1;
                break;
            case UERR_NAND_BITFLIP_ERROR:
                worn_status = 0b10;  // ERROR
                has_page_error = 1;
                break;
            case UERR_NAND_BITFLIP_FATAL:
                worn_status = 0b11;  // FATAL
                has_page_fatal = 1;
                break;
            default:
                worn_status = 0b00;
                break;
        }

        // Store in page_status array (2 bits per page)
        // page_status[4] can hold 64 pages × 2 bits = 128 bits
        u32 idx = i / 16;  // Which u32 element
        u32 shift = (i % 16) * 2;  // Bit position within u32
        page_status[idx] |= (worn_status << shift);
    }

    // Determine overall block health
    if (has_page_fatal) {
        *block_status = UERR_NAND_BITFLIP_FATAL;
    } else if (has_page_error) {
        *block_status = UERR_NAND_BITFLIP_ERROR;
    } else if (has_page_warn) {
        *block_status = UERR_NAND_BITFLIP_WARN;
    } else {
        *block_status = HAL_OK;
    }

    return HAL_OK;
}
```

**Usage Example**:
```c
// Check health of block 100
u8 block_health;
u32 page_health[4] = {0};  // 2 bits × 64 pages = 128 bits = 4 × u32
u8 buffer[2048];

NAND_FTL_ReadBlockStatus(0x1900, buffer, &block_health, page_health);

if (block_health == UERR_NAND_BITFLIP_ERROR) {
    // Block has pages with high bitflip counts
    // Should relocate data to fresh block
    printf("Block 100 needs refresh\n");

    // Decode page-level health
    for (u32 page = 0; page < 64; page++) {
        u32 idx = page / 16;
        u32 shift = (page % 16) * 2;
        u8 page_stat = (page_health[idx] >> shift) & 0x03;

        if (page_stat == 0b10) {  // ERROR
            printf("  Page %d: high bitflips\n", page);
        }
    }
}
```

### 4.4 Data Refresh Strategy

**When to Refresh**:
```c
u8 NAND_FTL_ReadPage(u32 addr, u8 *buf)
{
    // ... read and ECC check ...

    if (ecc_status == UERR_NAND_BITFLIP_ERROR) {
        // High bitflip count, data should be refreshed
        FS_DBG(FS_WARNING, "Page 0x%08X needs refresh (high bitflips)", addr);

        // Return data (it's corrected), but warn
        // Higher layer can schedule refresh
        return UERR_NAND_BITFLIP_ERROR;
    }

    return HAL_OK;
}
```

**Refresh Operation** (Application-level):
```c
void refresh_block(u32 block_num)
{
    u8 temp_buf[2048];
    u32 page_addr;

    printf("Refreshing block %d...\n", block_num);

    // Read all pages from old block
    for (u32 page = 0; page < 64; page++) {
        page_addr = (block_num << 6) + page;

        // Read page (ECC will correct bitflips)
        NAND_FTL_ReadPage(page_addr, temp_buf);

        // Write back to same location
        // This refreshes the charge in NAND cells
        NAND_FTL_WritePage(page_addr, temp_buf, (page == 0) ? 1 : 0);
    }

    printf("Block %d refreshed\n", block_num);
}
```

**Proactive Refresh** (Background Task):
```c
void background_health_monitor(void)
{
    static u32 current_block = 0;
    u8 block_health;
    u32 page_health[4];
    u8 buffer[2048];

    while (1) {
        // Check one block per iteration
        u32 page_addr = current_block << 6;

        NAND_FTL_ReadBlockStatus(page_addr, buffer, &block_health, page_health);

        if (block_health == UERR_NAND_BITFLIP_ERROR) {
            // Schedule refresh
            refresh_block(current_block);
        } else if (block_health == UERR_NAND_BITFLIP_FATAL) {
            // Mark block as bad
            FS_DBG(FS_ERROR, "Block %d has uncorrectable errors, marking bad",
                   current_block);
            NF_MarkBad(&nand_device, page_addr);
        }

        // Move to next block
        current_block = (current_block + 1) % 2048;

        // Sleep to avoid hogging CPU
        sleep(60);  // Check 1 block per minute
    }
}
```

---

## 5. Manufacturer-Specific Handling

### 5.1 Manufacturer Detection

**Initialization Flow** (`lfs_nand_ftl.c:223-279`):

```c
u8 NAND_FTL_Init(NAND_FTL_DeviceTypeDef *nand)
{
    u8 flash_ID[3];

    // Step 1: Reset NAND
    NAND_TxCmd(0xFF, 0, NULL, 0, NULL);

    // Step 2: Read ID
    NAND_RxCmd(FLASH_cmd_rd_id, 0, NULL, 2, flash_ID);

    // Step 3: Check for Micron (uses 3-byte ID)
    if (flash_ID[0] == NAND_MFG_MICRON) {
        NAND_RxCmd(FLASH_cmd_rd_id, 0, NULL, 3, flash_ID);
    }

    // Step 4: Store IDs
    nand->MemInfo.MID = flash_ID[0];
    nand->MemInfo.DID = flash_ID[1];
    nand->MemInfo.ExtDID = flash_ID[2];

    // Step 5: Enable ECC
    u8 reg = NAND_GetStatus(NAND_REG_CFG);
    reg |= NAND_CFG_ECC_ENABLE;
    NAND_SetStatus(NAND_REG_CFG, reg);

    // Step 6: Call manufacturer-specific init
    ret = NAND_FTL_MfgInit(nand);

    return ret;
}
```

**Manufacturer Selection** (`lfs_nand_ftl_mfg.c:717-797`):

```c
u8 NAND_FTL_MfgInit(NAND_FTL_DeviceTypeDef *nand)
{
    // Default to single die
    nand->MemInfo.Targets = 1;

    // Assign manufacturer operations
    switch (nand->MemInfo.MID) {
        case NAND_MFG_DOSILICON:  // 0xE5
            nand->MfgOps = &DosiliconOps;
            break;

        case NAND_MFG_GIGADEVICE:  // 0xC8
            nand->MfgOps = &GigaDeviceOps;
            break;

        case NAND_MFG_MACRONIX:  // 0xC2
            nand->MfgOps = &MacronixOps;
            break;

        case NAND_MFG_MICRON:  // 0x2C
            nand->MfgOps = &MicronOps;
            break;

        case NAND_MFG_WINBOND:  // 0xEF
            nand->MfgOps = &WinbondOps;

            // Check for dual-die variant
            if (nand->MemInfo.DID == 0xAB) {
                nand->MemInfo.Targets = 2;  // W25N02KV dual-die
            }
            break;

        default:
            // Unknown manufacturer, use default ops
            nand->MfgOps = &NandDefaultOps;
            FS_DBG(FS_WARNING, "Unknown NAND manufacturer: 0x%02X",
                   nand->MemInfo.MID);
            break;
    }

    // Call manufacturer init function
    NAND_FTL_MfgOpsTypeDef *ops = (NAND_FTL_MfgOpsTypeDef *)nand->MfgOps;
    if (ops->Init) {
        ret = ops->Init(nand);
    }

    // Read ONFI parameter page (if supported)
    if (ops->ReadParameterPage) {
        ret = ops->ReadParameterPage(nand, buffer);
        // Parse ONFI data to get geometry...
    }

    return ret;
}
```

### 5.2 Winbond-Specific Handling

**Dual-Die Support** (`lfs_nand_ftl_mfg.c:198-234`):

```c
// Winbond W25N02KV has 2 dies (128 MB each)

static u8 NAND_FTL_Winbond_Init(NAND_FTL_DeviceTypeDef *nand)
{
    // Configure buffer read mode on all dies
    if (nand->MemInfo.Targets > 1) {
        // Multi-die chip
        for (u8 i = 0; i < nand->MemInfo.Targets; i++) {
            NAND_FTL_Winbond_SelectTarget(nand, i);

            // Set buffer read mode (bit 3 of CFG)
            NAND_SetStatusBits(NAND_REG_CFG,
                              NF_CFG_WINBOND_BUF_READ,
                              ENABLE);
        }
    } else {
        // Single-die chip
        NAND_SetStatusBits(NAND_REG_CFG,
                          NF_CFG_WINBOND_BUF_READ,
                          ENABLE);
    }

    return HAL_OK;
}

static u8 NAND_FTL_Winbond_SelectTarget(NAND_FTL_DeviceTypeDef *nand, u8 target)
{
    if (nand->MemInfo.Targets > 1) {
        // Send die select command (0xC2)
        // target: 0=die0 (blocks 0-1023), 1=die1 (blocks 1024-2047)
        NAND_TxCmd(NF_WINBOND_DIE_SEL_CMD, 0, NULL, 1, &target);

        nand->CurTarget = target;
    }

    return HAL_OK;
}

// Called before every operation
static u8 NF_SelectTarget(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    if (nand->MemInfo.Targets > 1) {
        // Determine which die contains this address
        u32 pages_per_target = (nand->MemInfo.BlocksPerLun *
                               nand->MemInfo.PagesPerBlock);
        u8 target = (addr >> 12) / pages_per_target;

        if (target != nand->CurTarget) {
            // Switch to different die
            NAND_FTL_MfgOpsTypeDef *ops = nand->MfgOps;
            if (ops->SelectTarget) {
                return ops->SelectTarget(nand, target);
            }
        }
    }

    return HAL_OK;
}
```

**Example Die Selection**:
```
W25N02KV (256 MB, 2 dies):
  Die 0: Pages 0-65535 (blocks 0-1023)
  Die 1: Pages 65536-131071 (blocks 1024-2047)

Access page 70000:
  pages_per_target = 1024 × 64 = 65536
  target = 70000 / 65536 = 1 (die 1)

  if (current_target != 1):
    Send command 0xC2 with data 0x01
    current_target = 1

  Proceed with operation on die 1
```

### 5.3 Micron-Specific Handling

**Quad Mode Always On** (`lfs_spinand.c:584-586`):

```c
// Micron QSPI is enabled by default and cannot be disabled
if (flash_init_para.FLASH_id[0] == NAND_MFG_MICRON) {
    FLASH_InitStruct->FLASH_QuadEn_bit = 0;  // Don't try to enable
}
```

**3-Byte Device ID** (`lfs_nand_ftl.c:248-251`):

```c
NAND_RxCmd(FLASH_cmd_rd_id, 0, NULL, 2, flash_ID);

if (flash_ID[0] == NAND_MFG_MICRON) {
    // Micron uses extended 3-byte ID
    NAND_RxCmd(FLASH_cmd_rd_id, 0, NULL, 3, flash_ID);
}
```

**Parameter Page Access** (`lfs_nand_ftl_mfg.c:603-647`):

```c
static u8 NAND_FTL_Micron_ReadParameterPage(NAND_FTL_DeviceTypeDef *nand, u8 *data)
{
    u8 reg;

    // Special CFG register setup for parameter page access
    reg = NAND_GetStatus(NAND_REG_CFG);

    reg &= ~NF_CFG_MICRON_CFG0;  // Clear bit 1
    reg |= NF_CFG_MICRON_CFG1;   // Set bit 6
    reg &= ~NF_CFG_MICRON_CFG2;  // Clear bit 7

    NAND_SetStatus(NAND_REG_CFG, reg);

    // Read parameter page at address 0x01
    NAND_Page_Read(NF_PARAMETER_PAGE_ADDR, 0, 768, data);

    // Restore CFG register
    NAND_SetStatus(NAND_REG_CFG, 0x00);

    return HAL_OK;
}
```

### 5.4 GigaDevice-Specific Handling

**Dummy Byte in Address** (`ameba_rom_patch.c:814-817`):

```c
// GigaDevice requires dummy byte after command
if (flash_init_para.FLASH_addr_phase_len == NAND_COL_ADDR_3_BYTE) {
    spi_flash->DR[0].BYTE = 0x00;  // Dummy byte
}
```

**Variable Parameter Page Address** (`lfs_nand_ftl_mfg.c:575-580`):

```c
// Different GigaDevice models use different parameter page addresses
if ((nand->MemInfo.DID == 0x81) || (nand->MemInfo.DID == 0x91) ||
    (nand->MemInfo.DID == 0x82) || (nand->MemInfo.DID == 0x92)) {
    addr = NF_PARAMETER_PAGE_ADDR;             // 0x01 for GD5F1GM7/GD5F2GM7
} else {
    addr = NF_PARAMETER_PAGE_GIGADEVICE_ADDR;  // 0x04 for other models
}
```

### 5.5 Macronix-Specific Handling

**Extended ECC Status Register** (`lfs_nand_ftl_mfg.c:368-373`):

```c
// Macronix has additional ECC status register at 0x7C
if (ecc_bits == 0b01) {  // Bitflips corrected
    // Read extended ECC status
    u8 eccsr = NAND_FTL_GetMfgStatus(NF_ECCSR_MACRONIX_CMD,  // 0x7C
                                     NF_ECCSR_MACRONIX_ADDR); // 0x10

    // Lower 4 bits contain exact bitflip count
    u8 bitflip_count = eccsr & 0x0F;

    if (bitflip_count == 4) {
        // At 4-bit ECC threshold
        return UERR_NAND_BITFLIP_ERROR;
    } else {
        // 1-3 bitflips
        return UERR_NAND_BITFLIP_WARN;
    }
}
```

**Parameter Page Access** (`lfs_nand_ftl_mfg.c:657-693`):

```c
static u8 NAND_FTL_Macronix_ReadParameterPage(NAND_FTL_DeviceTypeDef *nand, u8 *data)
{
    u8 reg = NAND_GetStatus(NAND_REG_CFG);

    // Set CFG to 0x40 to access parameter page
    NAND_SetStatus(NAND_REG_CFG, 0x40);

    // Read parameter page
    NAND_Page_Read(NF_PARAMETER_PAGE_ADDR, 0, 768, data);

    // Restore CFG with sequence
    NAND_SetStatus(NAND_REG_CFG, 0x10);  // or 0x00
    NAND_SetStatus(NAND_REG_CFG, reg);    // Original value

    return HAL_OK;
}
```

---

## 6. Performance Optimization

### 6.1 Block Erase Caching

**Problem**: Sequential writes to same block cause redundant erases

**Solution**: Cache last erased block address

```c
// Without caching:
Page 0 write: Erase block (5 ms) + Write (0.7 ms) = 5.7 ms
Page 1 write: Erase block (5 ms) + Write (0.7 ms) = 5.7 ms
...
Page 63 write: Erase block (5 ms) + Write (0.7 ms) = 5.7 ms
Total: 64 × 5.7 ms = 364.8 ms

// With caching:
Page 0 write: Erase block (5 ms) + Write (0.7 ms) = 5.7 ms
             Cache: LastErasedBlockAddr = 0x1000
Page 1 write: Check cache (hit!) + Write (0.7 ms) = 0.7 ms
...
Page 63 write: Check cache (hit!) + Write (0.7 ms) = 0.7 ms
Total: 5.7 + (63 × 0.7) = 49.8 ms

Speedup: 7.3x faster!
```

**Implementation** (`lfs_nand_ftl.c:559-568`):

```c
if (do_erase) {
    block_addr = NF_GetBlockAddr(nand, addr);

    if (block_addr != nand->LastErasedBlockAddr) {
        // Block not in cache, perform erase
        ret = NF_EraseBlock(nand, block_addr);

        if (ret == HAL_OK || ret == UERR_NAND_WORN_BLOCK) {
            // Update cache
            nand->LastErasedBlockAddr = block_addr;
        }
    } else {
        // Block already erased, skip erase
        FS_DBG(FS_DEBUG, "Block 0x%08X already erased (cached)", block_addr);
    }
}
```

### 6.2 Word-Aligned Transfers

**Problem**: Byte-by-byte FIFO access is slow

**Solution**: Use 32-bit word access when buffer is aligned

```c
// Byte-by-byte (unaligned buffer):
while (rx_num < 2048) {
    if (spi_flash->SR & BIT_RFNE) {
        buffer[rx_num++] = spi_flash->DR[0].BYTE;  // 1 byte per iteration
    }
}
// Time: ~200 µs

// Word-by-word (aligned buffer):
u32 *buf32 = (u32 *)buffer;
while (rx_num < 2048 - 3) {
    if (spi_flash->SR & BIT_RFNE) {
        buf32[rx_num >> 2] = spi_flash->DR[0].WORD;  // 4 bytes per iteration
        rx_num += 4;
    }
}
// Remaining bytes byte-by-byte
while (rx_num < 2048) {
    buffer[rx_num++] = spi_flash->DR[0].BYTE;
}
// Time: ~50 µs

Speedup: 4x faster for aligned buffers!
```

**Implementation** (`ameba_rom_patch.c:838-854`):

```c
if (UNALIGNED32(read_data)) {
    // Buffer not 32-bit aligned, use byte access
    while (rx_num < read_len) {
        if (spi_flash->SR & BIT_RFNE) {
            read_data[rx_num] = spi_flash->DR[0].BYTE;
            rx_num++;
        }
    }
} else {
    // Buffer is 32-bit aligned, use word access
    u32 *aligned_buf = (u32 *)read_data;

    // Read full words
    while (rx_num < (read_len - 3)) {
        if (spi_flash->SR & BIT_RFNE) {
            aligned_buf[rx_num >> 2] = spi_flash->DR[0].WORD;
            rx_num += 4;
        }
    }

    // Read remaining 0-3 bytes
    while (rx_num < read_len) {
        if (spi_flash->SR & BIT_RFNE) {
            read_data[rx_num] = spi_flash->DR[0].BYTE;
            rx_num++;
        }
    }
}
```

### 6.3 FIFO Pre-loading

**Problem**: TX FIFO underrun causes wait states

**Solution**: Pre-load FIFO before enabling SPI

```c
// Without pre-loading:
spi_flash->SSIENR = BIT_SPIC_EN;  // Start SPI immediately
while (tx_num < 2048) {
    if (spi_flash->SR & BIT_TFNF) {  // Wait for FIFO space
        spi_flash->DR[0] = data[tx_num++];
    }
}
// Many wait cycles due to FIFO underruns

// With pre-loading:
// Pre-load FIFO (32 bytes) before starting
for (i = 0; i < 32; i++) {
    spi_flash->DR[0] = data[tx_num++];
}
spi_flash->SSIENR = BIT_SPIC_EN;  // Now start SPI
while (tx_num < 2048) {
    if (spi_flash->SR & BIT_TFNF) {
        spi_flash->DR[0] = data[tx_num++];
    }
}
// Fewer wait cycles, continuous SPI stream

Speedup: ~10-20% faster for writes
```

**Implementation** (`ameba_rom_patch.c:924-935`):

```c
// Pre-load TX FIFO before enabling SPI
u32 temp = sizeof(spi_flash->DR[0]);  // 4 bytes
while (tx_num < ByteLen) {
    spi_flash->DR[0].BYTE = pData[tx_num];
    tx_num++;
    if (tx_num == (U32BLOCKSIZE - temp)) break;  // FIFO almost full
}

// Now enable SPI (starts transmission)
spi_flash->SSIENR = BIT_SPIC_EN;

// Continue pushing data
while (tx_num < ByteLen) {
    // ...
}
```

### 6.4 Quad SPI Mode

**Performance Comparison**:

```
Standard SPI (1-bit data):
  Clock: 100 MHz
  Data rate: 100 Mbps = 12.5 MB/s
  Page read (2048 bytes): 2048 × 8 / 100 MHz = 164 µs

Quad SPI (4-bit data):
  Clock: 100 MHz
  Data rate: 400 Mbps = 50 MB/s
  Page read (2048 bytes): 2048 × 8 / (4 × 100 MHz) = 41 µs

Speedup: 4x faster!
```

**Enabled by Default for Winbond**:

```c
// Winbond W25N comes with quad mode enabled
// No configuration needed

// GigaDevice/Macronix need QE bit set:
u8 reg = NAND_GetStatus(NAND_REG_CFG);
reg |= NAND_CFG_QE;  // Set QE bit (bit 0)
NAND_SetStatus(NAND_REG_CFG, reg);
```

### 6.5 Bad Block Check Optimization

**Naive Approach** (check on every operation):
```c
// Read 100 pages from same block
for (i = 0; i < 100; i++) {
    NF_IsBad(block_addr);  // Reads OOB every time
    NAND_Page_Read(page_addr + i, buf);
}
// Total: 100 bad block checks = 100 × 50 µs = 5 ms wasted
```

**Optimized Approach** (check once per block):
```c
// Check bad block once
NF_IsBad(block_addr);  // 50 µs

// Read all pages
for (i = 0; i < 100; i++) {
    NAND_Page_Read(page_addr + i, buf);
}
// Total: 1 bad block check = 50 µs

// Note: Current implementation checks every time for safety
// Could be optimized for sequential access
```

### 6.6 Performance Summary

**Operation Timings** (typical):

| Operation | Time | Notes |
|-----------|------|-------|
| Page Read (Array→Cache) | 35 µs | NAND internal |
| Page Read (Cache→Host, Quad) | 41 µs | SPI transfer |
| **Total Page Read** | **~80 µs** | |
| Page Write (Host→Cache, Quad) | 50 µs | SPI transfer |
| Page Write (Cache→Array) | 400 µs | NAND internal |
| **Total Page Write** | **~450 µs** | |
| **Block Erase** | **5000 µs** | **5 ms** |
| Bad Block Check | 50 µs | Reads OOB |
| Status Register Read | 0.24 µs | SPI command |

**Full Block Write** (64 pages):
```
Without optimization:
  64 × (Erase + Write) = 64 × 5450 µs = 348.8 ms

With erase caching:
  1 × Erase + 64 × Write = 5000 + (64 × 450) = 33.8 ms

Speedup: 10.3x faster!
```

---

## 7. Debugging and Troubleshooting

### 7.1 Debug Logging

**Enable Debug Output**:

```c
// In lfs_nand_ftl.c
#define FS_DBG(level, fmt, ...) \
    printf("[%s] " fmt "\n", level, ##__VA_ARGS__)

// Log levels
#define FS_ERROR    "ERROR"
#define FS_WARNING  "WARN"
#define FS_INFO     "INFO"
#define FS_DEBUG    "DEBUG"
```

**Typical Debug Output**:

```
[INFO] NAND_FTL_Init: Detected Winbond W25N02KV (0xEF 0xAA 0x22)
[INFO] NAND_FTL_Init: Page=2048, Block=64, Capacity=256MB
[INFO] lfs_mount: Mounting LittleFS on /mnt
[INFO] lfs_mount: Superblock found, version 2.0
[WARN] NF_IsBad: Factory bad block at 0x00080000 (block 4)
[WARN] NF_IsBad: Factory bad block at 0x001C0000 (block 14)
[INFO] lfs_mount: Filesystem mounted, 250MB available
[DEBUG] lfs_nand_read: Read block=0, off=0, size=2048
[DEBUG] NAND_FTL_ReadPage: Page 0xA000, ECC status=0x00 (OK)
[WARN] NAND_FTL_ReadPage: Page 0xA540, ECC status=0x10 (bitflips corrected)
[ERROR] NAND_Page_Write: Write failed at 0xB200, status=0x04 (P_FAIL)
[ERROR] NF_MarkBad: Marking block 0xB200 as bad
```

### 7.2 Common Error Scenarios

**Scenario 1: Mount Failure**

```
Symptom:
  lfs_mount() returns LFS_ERR_CORRUPT

Possible causes:
  1. NAND not properly initialized
  2. Filesystem not formatted
  3. Too many bad blocks
  4. Incorrect configuration parameters

Debug steps:
  1. Check NAND_FTL_Init() return value
  2. Verify NAND ID detection:
     printf("MID=0x%02X, DID=0x%02X\n", nand->MemInfo.MID, nand->MemInfo.DID);
  3. Try formatting:
     lfs_format(&lfs, &config);
     lfs_mount(&lfs, &config);
  4. Check defconfig block counts match:
     CONFIG_MTD_NAND_MAXNUMBLOCKS=2048
```

**Scenario 2: Write Failures**

```
Symptom:
  write() returns -1, errno=EIO

Possible causes:
  1. Flash is full (all blocks used or bad)
  2. Block wear-out
  3. Hardware connection issue

Debug steps:
  1. Check disk usage:
     struct statvfs stat;
     statvfs("/mnt", &stat);
     printf("Free: %lu KB\n", (stat.f_bfree * stat.f_bsize) / 1024);

  2. Check bad block count:
     // Count bad blocks by checking all OOB areas

  3. Verify SPI connection:
     // Read status register
     u8 status = NAND_GetStatus(NAND_REG_STATUS);
     printf("Status: 0x%02X\n", status);
```

**Scenario 3: Data Corruption**

```
Symptom:
  Read returns different data than written

Possible causes:
  1. ECC failures (uncorrectable bitflips)
  2. Block wear-out
  3. Power loss during write

Debug steps:
  1. Check ECC status:
     u8 block_health;
     NAND_FTL_ReadBlockStatus(addr, buf, &block_health, NULL);
     printf("Block health: 0x%02X\n", block_health);

  2. Enable LittleFS integrity checking:
     lfs_fs_traverse(&lfs, check_callback);

  3. Run full filesystem check:
     lfs_fs_check(&lfs);
```

**Scenario 4: Performance Issues**

```
Symptom:
  Writes are very slow (>10 ms per page)

Possible causes:
  1. Erase cache not working
  2. Too many bad blocks (lots of remapping)
  3. Frequent wear leveling

Debug steps:
  1. Check erase cache hits:
     // Add counters in NAND_FTL_WritePage()
     static u32 erase_cache_hits = 0;
     static u32 total_writes = 0;

     if (block_addr == nand->LastErasedBlockAddr) {
         erase_cache_hits++;
     }
     total_writes++;

     printf("Cache hit rate: %d%%\n", (erase_cache_hits * 100) / total_writes);

  2. Profile operations:
     u32 start = get_time_us();
     NAND_FTL_WritePage(...);
     u32 end = get_time_us();
     printf("Write took %d us\n", end - start);
```

### 7.3 Hardware Verification

**Check SPI Connection**:

```c
void test_spi_connection(void)
{
    u8 id[3];

    // Reset NAND
    NAND_TxCmd(0xFF, 0, NULL, 0, NULL);
    usleep(1000);

    // Read ID
    NAND_RxCmd(0x9F, 0, NULL, 3, id);

    printf("NAND ID: %02X %02X %02X\n", id[0], id[1], id[2]);

    if (id[0] == 0xFF || id[0] == 0x00) {
        printf("ERROR: NAND not responding (check connections)\n");
    } else if (id[0] == 0xEF) {
        printf("OK: Winbond NAND detected\n");
    } else {
        printf("WARNING: Unknown manufacturer: 0x%02X\n", id[0]);
    }
}
```

**Check Bad Blocks**:

```c
void scan_bad_blocks(void)
{
    u32 bad_count = 0;
    u8 is_bad;

    printf("Scanning for bad blocks...\n");

    for (u32 block = 0; block < 2048; block++) {
        u32 page_addr = block << 6;  // First page of block

        NF_IsBad(&nand_device, page_addr, &is_bad);

        if (is_bad) {
            printf("  Bad block: %d (0x%08X)\n", block, page_addr);
            bad_count++;
        }
    }

    printf("Total bad blocks: %d (%.1f%%)\n",
           bad_count, (bad_count * 100.0) / 2048);
    printf("Usable capacity: %d MB\n",
           (2048 - bad_count) * 128 / 1024);
}
```

**Erase/Write/Read Test**:

```c
void test_block(u32 block_num)
{
    u32 page_addr = block_num << 6;
    u8 write_buf[2048];
    u8 read_buf[2048];
    u8 is_bad;

    printf("Testing block %d...\n", block_num);

    // Check if bad
    NF_IsBad(&nand_device, page_addr, &is_bad);
    if (is_bad) {
        printf("  Block is marked bad, skipping\n");
        return;
    }

    // Fill with test pattern
    for (u32 i = 0; i < 2048; i++) {
        write_buf[i] = i & 0xFF;
    }

    // Erase
    printf("  Erasing...\n");
    u8 ret = NAND_FTL_EraseBlock(page_addr, 0);
    if (ret != HAL_OK) {
        printf("  ERROR: Erase failed (0x%02X)\n", ret);
        return;
    }

    // Write
    printf("  Writing...\n");
    ret = NAND_FTL_WritePage(page_addr, write_buf, 0);
    if (ret != HAL_OK) {
        printf("  ERROR: Write failed (0x%02X)\n", ret);
        return;
    }

    // Read
    printf("  Reading...\n");
    ret = NAND_FTL_ReadPage(page_addr, read_buf);
    if (ret != HAL_OK) {
        printf("  ERROR: Read failed (0x%02X)\n", ret);
        return;
    }

    // Verify
    printf("  Verifying...\n");
    for (u32 i = 0; i < 2048; i++) {
        if (read_buf[i] != write_buf[i]) {
            printf("  ERROR: Mismatch at offset %d: wrote 0x%02X, read 0x%02X\n",
                   i, write_buf[i], read_buf[i]);
            return;
        }
    }

    printf("  PASS: Block %d is good\n", block_num);
}
```

### 7.4 Diagnostic Commands

**Implement Debug Shell Commands**:

```c
// Shell command: nand_info
void cmd_nand_info(void)
{
    NAND_FTL_DeviceTypeDef *nand = NAND_FTL_GetDevice();
    Flash_InfoTypeDef *info = &nand->MemInfo;

    printf("NAND Flash Information:\n");
    printf("  Manufacturer: 0x%02X (%s)\n", info->MID,
           (info->MID == 0xEF) ? "Winbond" : "Unknown");
    printf("  Device ID: 0x%02X\n", info->DID);
    printf("  Page Size: %d bytes\n", info->PageSize);
    printf("  OOB Size: %d bytes\n", info->OobSize);
    printf("  Pages/Block: %d\n", info->PagesPerBlock);
    printf("  Blocks/LUN: %d\n", info->BlocksPerLun);
    printf("  LUNs: %d\n", info->LunsPerTarget);
    printf("  Dies: %d\n", info->Targets);
    printf("  Capacity: %d MB\n", info->Capacity / (1024 * 1024));
    printf("  Max Bad Blocks: %d\n", info->MaxBadBlocksPerLun);
}

// Shell command: nand_status
void cmd_nand_status(void)
{
    u8 status = NAND_GetStatus(NAND_REG_STATUS);
    u8 config = NAND_GetStatus(NAND_REG_CFG);

    printf("NAND Status:\n");
    printf("  Status Register (0xC0): 0x%02X\n", status);
    printf("    OIP (Busy): %d\n", (status & 0x01) ? 1 : 0);
    printf("    WEL: %d\n", (status & 0x02) ? 1 : 0);
    printf("    E_FAIL: %d\n", (status & 0x04) ? 1 : 0);
    printf("    P_FAIL: %d\n", (status & 0x08) ? 1 : 0);
    printf("    ECC: 0x%X\n", (status >> 4) & 0x07);
    printf("  Config Register (0xB0): 0x%02X\n", config);
    printf("    QE: %d\n", (config & 0x01) ? 1 : 0);
    printf("    ECC-E: %d\n", (config & 0x02) ? 1 : 0);
}

// Shell command: lfs_stat
void cmd_lfs_stat(void)
{
    struct statvfs stat;
    int ret = statvfs("/mnt", &stat);

    if (ret == 0) {
        u64 total = (u64)stat.f_blocks * stat.f_bsize;
        u64 free = (u64)stat.f_bfree * stat.f_bsize;
        u64 used = total - free;

        printf("LittleFS Statistics:\n");
        printf("  Total: %llu MB\n", total / (1024 * 1024));
        printf("  Used: %llu MB\n", used / (1024 * 1024));
        printf("  Free: %llu MB\n", free / (1024 * 1024));
        printf("  Usage: %d%%\n", (int)((used * 100) / total));
    } else {
        printf("Error getting filesystem stats\n");
    }
}
```

---

## 8. Best Practices

### 8.1 Application Guidelines

**DO**:
- ✓ Always check return values from file operations
- ✓ Close files after use
- ✓ Use fsync() for critical data
- ✓ Handle out-of-space errors gracefully
- ✓ Implement wear-aware write patterns (if applicable)

**DON'T**:
- ✗ Assume infinite flash lifetime
- ✗ Write unnecessarily (cache in RAM when possible)
- ✗ Ignore write errors
- ✗ Exceed filesystem limits (file count, size)
- ✗ Perform frequent small writes (buffer them)

**Example Good Practice**:

```c
int save_data(const char *filename, const void *data, size_t size)
{
    int fd = -1;
    ssize_t written;
    int ret = -1;

    // Open with error checking
    fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", filename, strerror(errno));
        goto cleanup;
    }

    // Write with error checking
    written = write(fd, data, size);
    if (written != size) {
        fprintf(stderr, "Write failed: wrote %zd of %zu bytes\n", written, size);
        goto cleanup;
    }

    // Sync to ensure data is committed
    if (fsync(fd) != 0) {
        fprintf(stderr, "Sync failed: %s\n", strerror(errno));
        goto cleanup;
    }

    ret = 0;  // Success

cleanup:
    if (fd >= 0) {
        close(fd);
    }
    return ret;
}
```

### 8.2 Filesystem Maintenance

**Periodic Health Check**:

```c
// Run weekly or monthly
void maintenance_check(void)
{
    printf("Running filesystem maintenance...\n");

    // 1. Check filesystem integrity
    lfs_t *lfs = get_lfs_instance();
    int ret = lfs_fs_traverse(lfs, check_callback);
    if (ret < 0) {
        printf("WARNING: Filesystem inconsistency detected\n");
    }

    // 2. Scan for bad blocks
    scan_bad_blocks();

    // 3. Check fragmentation (if supported)
    // ...

    printf("Maintenance complete\n");
}
```

**Defragmentation** (if needed):

```c
// LittleFS auto-defragments during wear leveling
// No manual defrag needed for normal use

// For extreme cases:
void force_defragment(void)
{
    // Copy all files to temp location
    // Reformat filesystem
    // Copy files back
    // (Rarely needed)
}
```

### 8.3 Power-Loss Protection

**Already Handled by LittleFS**:
- ✓ Copy-on-write ensures atomic updates
- ✓ Metadata redundancy provides recovery
- ✓ No need for application-level journaling

**Application Responsibility**:
- ✓ Use fsync() for critical writes
- ✓ Close files properly
- ✓ Don't rely on unflushed data

**Example**:

```c
// BAD: Data may be lost on power failure
fd = open("/mnt/config.txt", O_WRONLY);
write(fd, data, size);
// Power failure here → data lost
close(fd);

// GOOD: Data is committed to flash
fd = open("/mnt/config.txt", O_WRONLY);
write(fd, data, size);
fsync(fd);  // Force commit to flash
// Power failure here → data safe
close(fd);
```

### 8.4 Wear Leveling Awareness

**LittleFS Handles Automatically**:
- Dynamic wear leveling built-in
- No application changes needed

**For Very Long Lifetime**:

```c
// Avoid writing to same files repeatedly
// Instead, rotate files:

void log_event(const char *msg)
{
    static int log_num = 0;
    char filename[32];

    // Rotate log files (0-9)
    sprintf(filename, "/mnt/log%d.txt", log_num);

    // Write to current log
    append_to_file(filename, msg);

    // Check file size
    struct stat st;
    stat(filename, &st);
    if (st.st_size > 100000) {  // 100 KB
        log_num = (log_num + 1) % 10;  // Rotate to next file
    }
}
```

### 8.5 Error Recovery

**Implement Graceful Degradation**:

```c
int write_with_retry(const char *path, const void *data, size_t size)
{
    int attempts = 3;

    for (int i = 0; i < attempts; i++) {
        int ret = save_data(path, data, size);
        if (ret == 0) {
            return 0;  // Success
        }

        fprintf(stderr, "Write attempt %d failed, retrying...\n", i + 1);
        usleep(100000);  // Wait 100 ms
    }

    // All attempts failed
    fprintf(stderr, "ERROR: Failed to write after %d attempts\n", attempts);

    // Try alternate location
    char alt_path[256];
    snprintf(alt_path, sizeof(alt_path), "%s.backup", path);
    int ret = save_data(alt_path, data, size);
    if (ret == 0) {
        fprintf(stderr, "Saved to backup location: %s\n", alt_path);
        return 0;
    }

    return -1;  // Complete failure
}
```

---

## Summary

This part covered complete operation flows and error handling:

**Key Takeaways**:

1. **Operation Flows**: Complete traces from application to hardware
2. **Error Handling**: Multi-layer error translation and recovery
3. **Bad Block Management**: Detection, marking, and avoidance
4. **ECC**: Manufacturer-specific parsing and health monitoring
5. **Performance**: Erase caching, word-aligned transfers, FIFO pre-loading
6. **Debugging**: Logging, diagnostics, hardware verification
7. **Best Practices**: Application guidelines, maintenance, power-loss protection

**Final Notes**:
- System is highly resilient with multiple layers of protection
- Bad blocks are handled transparently
- ECC provides data integrity
- Performance optimizations reduce latency by 7-10x
- Comprehensive debugging tools available

---

**Document Version**: 1.0
**Last Updated**: 2025
**Target Platform**: RTL8730E with W25N NAND Flash
**Complete Documentation Series**: Parts 1-4

**For More Information**:
- Part 1: Overview and Architecture
- Part 2: Hardware and Configuration
- Part 3: Software Layers and Integration
- Part 4: Operations and Error Handling (this document)
