# TizenRT NAND Flash with LittleFS - Part 3: Software Layers and Integration
## For RTL8730E Chipset - Beginner's Guide

---

## Table of Contents
1. [Software Stack Overview](#software-stack-overview)
2. [LittleFS Layer](#littlefs-layer)
3. [NAND Adapter Layer](#nand-adapter-layer)
4. [NAND FTL Layer](#nand-ftl-layer)
5. [SPI NAND Driver Layer](#spi-nand-driver-layer)
6. [Hardware Abstraction Layer](#hardware-abstraction-layer)
7. [Data Structures and APIs](#data-structures-and-apis)
8. [Threading and Synchronization](#threading-and-synchronization)

---

## 1. Software Stack Overview

### 1.1 Layer Responsibilities

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 1: VFS (Virtual File System)                         │
│  ────────────────────────────────────────────────────────── │
│  Location: os/fs/vfs/                                       │
│  Responsibility: POSIX API (open, read, write, close)       │
│  Input:  File paths, file descriptors                       │
│  Output: Routes to appropriate filesystem                   │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│  Layer 2: LittleFS Core                                     │
│  ────────────────────────────────────────────────────────── │
│  Location: os/fs/littlefs/, os/board/.../r2.50/             │
│  Responsibility: Filesystem logic, metadata management      │
│  Input:  VFS requests                                       │
│  Output: Block device operations (read/prog/erase)          │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│  Layer 3: LittleFS NAND Adapter                             │
│  ────────────────────────────────────────────────────────── │
│  Location: littlefs_adapter.c                               │
│  Responsibility: Translate LFS calls to NAND operations     │
│  Input:  lfs_config callbacks (read/prog/erase)             │
│  Output: NAND FTL calls with page addresses                 │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│  Layer 4: NAND FTL (Flash Translation Layer)                │
│  ────────────────────────────────────────────────────────── │
│  Location: lfs_nand_ftl.c, lfs_nand_ftl_mfg.c               │
│  Responsibility: Bad block mgmt, ECC checking, multi-die    │
│  Input:  Page addresses, buffers                            │
│  Output: SPI NAND commands                                  │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│  Layer 5: SPI NAND Driver                                   │
│  ────────────────────────────────────────────────────────── │
│  Location: lfs_spinand.c                                    │
│  Responsibility: NAND commands (read/write/erase)           │
│  Input:  Command codes, addresses, data                     │
│  Output: SPIC register operations                           │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│  Layer 6: SPIC Hardware Abstraction                         │
│  ────────────────────────────────────────────────────────── │
│  Location: ameba_rom_patch.c                                │
│  Responsibility: SPI controller register access             │
│  Input:  Commands, addresses, data buffers                  │
│  Output: Hardware register writes, SPI transactions         │
└────────────────────────┬────────────────────────────────────┘
                         │
                    ┌────▼────┐
                    │ Hardware │
                    └─────────┘
```

### 1.2 Function Call Flow Example

**Scenario**: Application writes 100 bytes to a file

```c
// Application code
int fd = open("/mnt/test.txt", O_WRONLY | O_CREAT);
write(fd, "Hello World", 11);
close(fd);
```

**Complete call stack**:
```
write()                                    [VFS Layer - os/fs/vfs/]
  └─ file_write()
      └─ lfs_file_write()                  [LittleFS - lfs.c]
          └─ lfs_file_rawwrite()
              └─ lfs_bd_prog()
                  └─ lfs_config->prog()    [Callback to adapter]
                      └─ lfs_nand_prog()   [Adapter - littlefs_adapter.c:51]
                          ├─ device_mutex_lock(RT_DEV_LOCK_FLASH)
                          ├─ Address calculation
                          ├─ NAND_FTL_WritePage()  [FTL - lfs_nand_ftl.c:534]
                          │   ├─ NF_SelectTarget()
                          │   ├─ NF_IsBad()        [Bad block check]
                          │   ├─ NF_EraseBlock()   [If needed]
                          │   └─ NAND_Page_Write() [Driver - lfs_spinand.c]
                          │       ├─ NAND_WriteEn()
                          │       ├─ NAND_TxData() [Program Load]
                          │       │   └─ SPIC register writes
                          │       └─ NAND_Page_Write_Program_Execute()
                          │           └─ SPIC register writes
                          └─ device_mutex_unlock(RT_DEV_LOCK_FLASH)
```

**Time breakdown** (approximate):
```
Layer                    Operation               Time
─────────────────────    ──────────────────      ─────────
Application/VFS          Function overhead       <1 µs
LittleFS                 Metadata updates        10-50 µs
Adapter                  Address translation     <1 µs
FTL                      Bad block check         <1 µs
SPI Driver               Command transmission    5 µs
Hardware                 Page program            200-700 µs
                         ─────────────────────
                         Total                   ~220-760 µs
```

---

## 2. LittleFS Layer

### 2.1 Core Concepts

**What is LittleFS?**
- Power-loss resilient filesystem for embedded systems
- Designed for NOR/NAND flash with limited RAM
- Uses copy-on-write and metadata redundancy

**Key Features**:
1. **Dynamic wear leveling**: Automatically distributes writes
2. **Power-loss resilience**: Data remains consistent after power failure
3. **Bounded RAM**: Uses fixed amount of RAM regardless of filesystem size
4. **Bad block handling**: Gracefully handles block failures

### 2.2 On-Disk Structure

**LittleFS Metadata Blocks**:
```
Block 0-1: Superblock Pair
┌─────────────────────────────────────────────────┐
│ Magic: "littlefs"                               │
│ Version: 2.0                                    │
│ Block Size: 131072 (128 KB)                     │
│ Block Count: 1024                               │
│ Name Max: 255                                   │
│ File Max: 2147483647                            │
│ Attr Max: 1022                                  │
└─────────────────────────────────────────────────┘

Block 2-3: Root Directory Pair
┌─────────────────────────────────────────────────┐
│ Entry: "test.txt"                               │
│   └─ Type: REG (regular file)                   │
│   └─ ID: 0x0004                                 │
│   └─ Size: 11 bytes                             │
│   └─ CRC: 0x1234ABCD                            │
│                                                 │
│ Entry: "logs/"                                  │
│   └─ Type: DIR (directory)                      │
│   └─ ID: 0x0005                                 │
└─────────────────────────────────────────────────┘

Block 4+: File Data
┌─────────────────────────────────────────────────┐
│ File ID: 0x0004 (test.txt)                      │
│ Data: "Hello World"                             │
└─────────────────────────────────────────────────┘
```

**Metadata Pair Structure**:
```
LittleFS uses "metadata pairs" for redundancy:

Block A                      Block B
┌──────────────────┐        ┌──────────────────┐
│ Revision: 5      │        │ Revision: 4      │
│ Data: [...]      │        │ Data: [...]      │
│ CRC: 0xABCD      │        │ CRC: 0x1234      │
└──────────────────┘        └──────────────────┘
     ▲                              │
     │                              │
     └─ Newer (used for reads)      └─ Older (backup)

On write:
  1. Write new version to Block B with Revision 6
  2. Verify CRC
  3. Now Block B is newer, used for future reads
  4. Block A becomes the backup

This ensures at least one valid copy always exists.
```

### 2.3 File Operations

**File Write Flow**:
```c
// User code
lfs_file_t file;
lfs_file_open(&lfs, &file, "test.txt", LFS_O_WRONLY | LFS_O_CREAT);
lfs_file_write(&lfs, &file, "Hello", 5);
lfs_file_close(&lfs, &file);
```

**Internal LittleFS operations**:
```
1. lfs_file_open():
   ├─ Allocate file handle
   ├─ Look up file in directory
   ├─ If creating: allocate new file ID
   └─ Read file metadata

2. lfs_file_write():
   ├─ Check if data fits in current block
   ├─ If not: allocate new block
   │   ├─ Search free block list
   │   ├─ Check block is not bad (via lfs_config->erase)
   │   └─ Update allocation bitmap
   ├─ Calculate page offset
   ├─ Call lfs_config->prog(block, offset, buffer, size)
   │   └─ This calls lfs_nand_prog() in adapter
   ├─ Update file size in metadata
   └─ Update directory entry

3. lfs_file_close():
   ├─ Flush any cached data
   ├─ Write metadata pair (with new revision)
   ├─ Verify CRC
   └─ Release file handle
```

**File Read Flow**:
```c
lfs_file_t file;
lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDONLY);
char buf[10];
lfs_file_read(&lfs, &file, buf, 10);
lfs_file_close(&lfs, &file);
```

**Internal operations**:
```
1. lfs_file_open():
   ├─ Look up file in directory tree
   ├─ Read file metadata (size, block list)
   └─ Initialize file cursor to offset 0

2. lfs_file_read():
   ├─ Calculate which block contains data at current offset
   ├─ Calculate offset within block
   ├─ Call lfs_config->read(block, offset, buffer, size)
   │   └─ This calls lfs_nand_read() in adapter
   ├─ Verify CRC if enabled
   ├─ Advance file cursor
   └─ Return bytes read

3. lfs_file_close():
   └─ Release file handle (no writes needed for read-only)
```

### 2.4 Wear Leveling Algorithm

**LittleFS Dynamic Wear Leveling**:

```
Configuration:
  .block_cycles = 100

Algorithm:
  1. Track erase count per block in metadata
  2. When allocating blocks, prefer low-erase-count blocks
  3. Every 100 erases (block_cycles), trigger wear leveling:
     a. Find block with highest erase count (hot block)
     b. Find block with lowest erase count (cold block)
     c. If difference > threshold:
        - Copy data from cold block to hot block
        - Update metadata to point to new location
        - Erase cold block (now it's hot)

Example:
  Block 0: 150 erases (static data: config file, rarely changed)
  Block 10: 145 erases (dynamic data: logs, frequently changed)

  Trigger: Block 10 reaches 150 erases
  Action: Swap Block 0 ↔ Block 10 data
  Result:
    Block 0: 150 erases, now has logs (will get more erases)
    Block 10: 145 erases, now has config (fewer erases)

  Over time: All blocks converge to similar erase counts
```

**Benefits**:
- Extends flash lifetime by 10-100x
- Prevents premature wear-out of heavily-used blocks
- Automatic, no application intervention needed

### 2.5 LittleFS Configuration for NAND

**Key Configuration** (`littlefs_adapter.c:11-28`):
```c
struct lfs_config g_nand_lfs_cfg = {
    // Block device operations
    .context = NULL,
    .read  = lfs_nand_read,      // Read from NAND
    .prog  = lfs_nand_prog,      // Write to NAND
    .erase = lfs_nand_erase,     // Erase NAND block
    .sync  = lfs_diskio_sync,    // Flush to flash

    // Thread safety
    .lock   = lfs_diskio_lock,   // Acquire mutex
    .unlock = lfs_diskio_unlock, // Release mutex

    // Block device configuration
    .read_size      = 2048,      // Must read in 2KB chunks (1 page)
    .prog_size      = 2048,      // Must write in 2KB chunks (1 page)
    .block_size     = 131072,    // Block = 128 KB (64 pages)
    .block_count    = 1024,      // 1024 blocks = 128 MB partition
    .cache_size     = 2048,      // Cache 1 page
    .lookahead_size = 8,         // Look ahead 8 blocks for allocation

    // Wear leveling
    .block_cycles   = 100,       // Trigger wear leveling every 100 erases

    // Advanced
    .metadata_max   = 256,       // Max metadata size
};
```

**Why these values?**

| Parameter | Value | Reason |
|-----------|-------|--------|
| `read_size=2048` | 2 KB | NAND page size - can't read smaller |
| `prog_size=2048` | 2 KB | NAND page size - can't write smaller |
| `block_size=131072` | 128 KB | NAND block size - erase granularity |
| `cache_size=2048` | 2 KB | Minimize RAM, cache 1 page |
| `lookahead_size=8` | 8 blocks | Balance allocation speed vs RAM |
| `block_cycles=100` | 100 | Good balance for 100K erase cycles |

---

## 3. NAND Adapter Layer

### 3.1 Purpose and Responsibilities

**What does the adapter do?**
1. **Address Translation**: LittleFS block numbers → NAND page addresses
2. **Thread Safety**: Lock/unlock flash hardware during operations
3. **Error Translation**: NAND error codes → LittleFS error codes
4. **Device Abstraction**: Hide NAND-specific details from LittleFS

**Location**: `os/board/rtl8730e/src/component/file_system/littlefs/littlefs_adapter.c`

### 3.2 Read Operation

**Function**: `lfs_nand_read()` (lines 30-49)

```c
int lfs_nand_read(const struct lfs_config *c, lfs_block_t block,
                  lfs_off_t off, void *buffer, lfs_size_t size)
{
    int ret;
    u32 NandAddr, PageAddr;

    // Step 1: Calculate NAND address
    // VFS1_FLASH_BASE_ADDR = 0x0A000000 (NAND start address)
    NandAddr = VFS1_FLASH_BASE_ADDR + block * c->block_size + off;

    // Step 2: Convert to page address
    // NAND_ADDR_TO_PAGE_ADDR() shifts right by 12 bits
    // (divides by 4096 to get page number)
    PageAddr = NAND_ADDR_TO_PAGE_ADDR(NandAddr);

    // Step 3: Lock flash hardware (prevent concurrent access)
    device_mutex_lock(RT_DEV_LOCK_FLASH);

    // Step 4: Read page from NAND
    ret = NAND_FTL_ReadPage(PageAddr, (uint8_t *)buffer);

    // Step 5: Unlock flash hardware
    device_mutex_unlock(RT_DEV_LOCK_FLASH);

    // Step 6: Translate error codes
    if (ret == HAL_OK) {
        return LFS_ERR_OK;
    } else {
        return LFS_ERR_IO;
    }
}
```

**Address Calculation Example**:
```
LittleFS wants to read:
  block = 5
  off = 4096
  size = 2048

Step 1: Calculate NAND address
  NandAddr = 0x0A000000 + (5 × 131072) + 4096
           = 0x0A000000 + 655360 + 4096
           = 0x0A0A1000

Step 2: Convert to page address
  PageAddr = 0x0A0A1000 >> 12
           = 0xA0A1

Result: Read NAND page 0xA0A1

Verification:
  Block 5 = pages 320-383 (5 × 64 = 320 start page)
  Offset 4096 = 2 pages into block
  Page 0xA0A1 = decimal 41121 = 320 + 2 ✓ Correct!
```

### 3.3 Write Operation

**Function**: `lfs_nand_prog()` (lines 51-77)

```c
int lfs_nand_prog(const struct lfs_config *c, lfs_block_t block,
                  lfs_off_t off, const void *buffer, lfs_size_t size)
{
    int ret;
    u32 NandAddr, PageAddr;
    int do_erase = 0;

    // Step 1: Determine if erase is needed
    // LittleFS calls erase() separately, but we optimize by
    // checking if this is the first write to a block
    if (off == 0) {
        do_erase = 1;  // First page of block, needs erase
    }

    // Step 2: Calculate NAND address
    NandAddr = VFS1_FLASH_BASE_ADDR + block * c->block_size + off;
    PageAddr = NAND_ADDR_TO_PAGE_ADDR(NandAddr);

    // Step 3: Lock flash hardware
    device_mutex_lock(RT_DEV_LOCK_FLASH);

    // Step 4: Write page to NAND (includes optional erase)
    ret = NAND_FTL_WritePage(PageAddr, (uint8_t *)buffer, do_erase);

    // Step 5: Unlock flash hardware
    device_mutex_unlock(RT_DEV_LOCK_FLASH);

    // Step 6: Translate error codes
    if (ret == HAL_OK) {
        return LFS_ERR_OK;
    } else if (ret == UERR_NAND_BAD_BLOCK) {
        return LFS_ERR_CORRUPT;  // Block is bad, LFS will skip it
    } else {
        return LFS_ERR_IO;
    }
}
```

**Write Optimization**:
```
Traditional approach:
  1. LittleFS calls erase(block)
  2. Wait for erase (3-10 ms)
  3. LittleFS calls prog(block, 0, data)
  4. Wait for program (200-700 µs)
  5. LittleFS calls prog(block, 2048, data)
  6. Wait for program (200-700 µs)
  ... continue for all 64 pages

Optimized approach:
  1. LittleFS calls prog(block, 0, data) with do_erase=1
     - Erase block (3-10 ms)
     - Write page 0 (200-700 µs)
  2. LittleFS calls prog(block, 2048, data) with do_erase=0
     - Skip erase (already done)
     - Write page 1 (200-700 µs)
  ... continue

Savings: Erase once per block instead of multiple times
```

### 3.4 Erase Operation

**Function**: `lfs_nand_erase()` (lines 79-100)

```c
int lfs_nand_erase(const struct lfs_config *c, lfs_block_t block)
{
    int ret;
    u32 NandAddr, PageAddr;

    // Step 1: Verify block size
    if (c->block_size != 0x20000) {  // 0x20000 = 128 KB
        return LFS_ERR_IO;
    }

    // Step 2: Calculate NAND address (first page of block)
    NandAddr = VFS1_FLASH_BASE_ADDR + block * c->block_size;
    PageAddr = NAND_ADDR_TO_PAGE_ADDR(NandAddr);

    // Step 3: Lock flash hardware
    device_mutex_lock(RT_DEV_LOCK_FLASH);

    // Step 4: Erase NAND block
    ret = NAND_FTL_EraseBlock(PageAddr, 0);  // 0 = don't force erase bad blocks

    // Step 5: Unlock flash hardware
    device_mutex_unlock(RT_DEV_LOCK_FLASH);

    // Step 6: Translate error codes
    if (ret == HAL_OK) {
        return LFS_ERR_OK;
    } else if (ret == UERR_NAND_BAD_BLOCK) {
        // Block is bad, tell LittleFS to skip it
        return LFS_ERR_CORRUPT;
    } else if (ret == UERR_NAND_WORN_BLOCK) {
        // Block just became bad, tell LittleFS
        return LFS_ERR_CORRUPT;
    } else {
        return LFS_ERR_IO;
    }
}
```

**Bad Block Handling**:
```
Scenario 1: Erasing a factory-bad block
  1. lfs_nand_erase(block=10)
  2. NAND_FTL_EraseBlock() checks bad block marker
  3. OOB bytes [0-1] = [0x00, 0x00] → bad block
  4. Return UERR_NAND_BAD_BLOCK
  5. Adapter returns LFS_ERR_CORRUPT
  6. LittleFS marks block 10 as bad in its metadata
  7. LittleFS allocates block 11 instead

Scenario 2: Block fails during erase
  1. lfs_nand_erase(block=20)
  2. NAND_FTL_EraseBlock() sends ERASE command
  3. NAND chip returns E_FAIL bit = 1
  4. FTL marks block as bad (writes 0x00 to OOB)
  5. Return UERR_NAND_WORN_BLOCK
  6. Adapter returns LFS_ERR_CORRUPT
  7. LittleFS marks block 20 as bad
  8. User data is safe (LittleFS has redundancy)
```

### 3.5 Sync Operation

**Function**: `lfs_diskio_sync()` (lines 102-106)

```c
int lfs_diskio_sync(const struct lfs_config *c)
{
    // For NAND flash, writes are immediately committed
    // No cache to flush at this layer
    return LFS_ERR_OK;
}
```

**Why no-op?**
- NAND writes are synchronous (blocking)
- When `NAND_Page_Write()` returns, data is in flash
- No write-back cache at adapter/FTL/driver level
- LittleFS has its own cache, which it manages

---

## 4. NAND FTL Layer

### 4.1 Core Responsibilities

**FTL (Flash Translation Layer) Functions**:
1. **Bad Block Management**: Detect and skip bad blocks
2. **ECC Status Checking**: Verify data integrity after reads
3. **Multi-Die Support**: Handle NAND chips with multiple dies
4. **Manufacturer Abstraction**: Handle vendor-specific quirks
5. **Optimization**: Cache last erased block to avoid redundant erases

**Location**: `os/board/rtl8730e/src/component/file_system/littlefs/lfs_nand_ftl.c`

### 4.2 Key Data Structures

**NAND FTL Device Structure** (`lfs_nand_ftl.h:87-93`):
```c
typedef struct {
    Flash_InfoTypeDef MemInfo;     // Flash geometry
    void *MfgOps;                  // Manufacturer operations
    __IO u8 CurTarget;             // Current die selection
    __IO u8 Initialized;           // Init flag
    __IO u32 LastErasedBlockAddr;  // Optimization cache
} NAND_FTL_DeviceTypeDef;
```

**Flash Information Structure** (`lfs_nand_ftl.h:64-85`):
```c
typedef struct {
    // Basic identification
    u8  MID;                       // Manufacturer ID (0xEF for Winbond)
    u8  DID;                       // Device ID (0xAA/0xAB)
    u8  ExtDID;                    // Extended ID

    // ONFI parameters (from parameter page)
    u8  MFG[12];                   // Manufacturer name
    u8  Model[20];                 // Device model
    u32 PageSize;                  // 2048
    u16 OobSize;                   // 64
    u32 PagesPerBlock;             // 64
    u32 BlocksPerLun;              // 1024
    u8  LunsPerTarget;             // 1
    u16 MaxBadBlocksPerLun;        // 40
    u8  ReqHostEccLevel;           // 1 (1-bit per 512 bytes)
    u8  Targets;                   // 1 or 2 (number of dies)
    u32 Capacity;                  // 134217728 or 268435456
} _PACKED_ Flash_InfoTypeDef;
```

**Manufacturer Operations** (`lfs_nand_ftl.h:95-100`):
```c
typedef struct {
    u8 (*Init)(NAND_FTL_DeviceTypeDef *nand);
    u8 (*SelectTarget)(NAND_FTL_DeviceTypeDef *nand, u8 target);
    u8 (*GetEccStatus)(NAND_FTL_DeviceTypeDef *nand, u8 status);
    u8 (*ReadParameterPage)(NAND_FTL_DeviceTypeDef *nand, u8 *data);
} NAND_FTL_MfgOpsTypeDef;
```

### 4.3 Bad Block Management

**Bad Block Detection** (`lfs_nand_ftl.c:103-124`):
```c
static u8 NF_IsBad(NAND_FTL_DeviceTypeDef *nand, u32 addr, u8 *value)
{
    Flash_InfoTypeDef *info = &nand->MemInfo;
    u8 data[2];
    u32 block_addr;
    u8 ret;

    // Step 1: Get block address (first page of block)
    block_addr = NF_GetBlockAddr(nand, addr);

    // Step 2: Read OOB bytes 0-1 from first page of block
    // PageSize = 2048, so reading at offset 2048 reads OOB
    ret = NAND_Page_Read(block_addr, info->PageSize, 2, data);

    // Step 3: Check bad block marker
    // Good block: [0xFF, 0xFF]
    // Bad block:  [0x00, 0x**] or [0x**, 0x00]
    if ((data[0] != NF_GOOD_BLOCK) || (data[1] != NF_GOOD_BLOCK)) {
        *value = 1;  // Bad block
    } else {
        *value = 0;  // Good block
    }

    return ret;
}
```

**Bad Block Marking** (`lfs_nand_ftl.c:134-158`):
```c
static u8 NF_MarkBad(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    Flash_InfoTypeDef *info = &nand->MemInfo;
    u8 data[2] = {NF_BAD_BLOCK, NF_BAD_BLOCK};  // [0x00, 0x00]
    u32 block_addr;
    u8 ret;

    // Step 1: Get block address
    block_addr = NF_GetBlockAddr(nand, addr);

    // Step 2: Write bad block marker to OOB bytes 0-1
    ret = NAND_Page_Write(block_addr, info->PageSize, 2, data);

    if (ret == 0) {
        FS_DBG(FS_WARNING, "Mark block 0x%08X as bad", addr);
        ret = HAL_OK;
    } else {
        FS_DBG(FS_ERROR, "Failed to mark block 0x%08X as bad", addr);
        ret = HAL_ERR_HW;
    }

    return ret;
}
```

**Block Address Calculation** (`lfs_nand_ftl.c:86-92`):
```c
static u32 NF_GetBlockAddr(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    Flash_InfoTypeDef *info = &nand->MemInfo;

    // Align address to block boundary
    // For 64 pages/block: mask = ~(64-1) << 12 = 0xFFFF0000
    u32 mask = ~(info->PagesPerBlock - 1) << NAND_PAGE_ADDR_SHIFT;

    return addr & mask;
}
```

**Example**:
```
Input address: 0x0A0A5678

Step 1: Convert to page number
  Page = 0x0A0A5678 >> 12 = 0xA0A5 (decimal 41125)

Step 2: Calculate block number
  Block = 41125 / 64 = 642.89 → Block 642

Step 3: Get block start address
  Block start page = 642 × 64 = 41088 = 0xA080

Step 4: Convert back to address
  Block address = 0xA080 << 12 = 0x0A080000

Verification:
  Original page: 41125 (0xA0A5)
  Block start page: 41088 (0xA080)
  Pages in block: 41088-41151 (0xA080-0xA0BF)
  41125 is in this range ✓
```

### 4.4 Read Page with ECC Check

**Function**: `NAND_FTL_ReadPage()` (`lfs_nand_ftl.c:307-345`):
```c
u8 NAND_FTL_ReadPage(u32 addr, u8 *buf)
{
    NAND_FTL_DeviceTypeDef *nand = NAND_FTL_GetDevice();
    Flash_InfoTypeDef *info = &nand->MemInfo;
    NAND_FTL_MfgOpsTypeDef *ops = (NAND_FTL_MfgOpsTypeDef *)nand->MfgOps;
    u8 is_bad_block = 0;
    u8 status, ecc_status;
    u8 ret;

    // Step 1: Check if initialized
    if (!nand->Initialized) {
        return UERR_INIT;
    }

    // Step 2: Select target/die (for multi-die chips)
    ret = NF_SelectTarget(nand, addr);
    if (ret != HAL_OK) {
        return ret;
    }

    // Step 3: Check if block is bad
    ret = NF_IsBad(nand, addr, &is_bad_block);
    if (is_bad_block) {
        FS_DBG(FS_WARNING, "Read from bad block 0x%08X", addr);
        return UERR_NAND_BAD_BLOCK;
    }

    // Step 4: Read page from NAND
    status = NAND_Page_Read(addr, 0, info->PageSize, buf);

    // Step 5: Check ECC status
    if (ops->GetEccStatus) {
        ecc_status = ops->GetEccStatus(nand, status);

        switch (ecc_status) {
            case HAL_OK:
                // No bitflips, perfect read
                ret = HAL_OK;
                break;

            case UERR_NAND_BITFLIP_WARN:
                // Bitflips corrected, data OK, just warn
                FS_DBG(FS_INFO, "Page 0x%08X: correctable bitflips (low)", addr);
                ret = HAL_OK;
                break;

            case UERR_NAND_BITFLIP_ERROR:
                // Bitflips corrected but high count, should refresh
                FS_DBG(FS_WARNING, "Page 0x%08X: correctable bitflips (high)", addr);
                ret = UERR_NAND_BITFLIP_ERROR;
                break;

            case UERR_NAND_BITFLIP_FATAL:
                // Uncorrectable bitflips, data corrupted
                FS_DBG(FS_ERROR, "Page 0x%08X: uncorrectable bitflips", addr);
                ret = UERR_NAND_BITFLIP_FATAL;
                break;

            default:
                ret = HAL_OK;
                break;
        }
    } else {
        // No ECC status available, assume OK
        ret = (status == 0) ? HAL_OK : HAL_ERR_HW;
    }

    return ret;
}
```

**ECC Status Flow**:
```
Read Page
    ↓
NAND returns status byte: 0x01
    ↓
Extract ECC bits [6:4] = 0b001
    ↓
Call manufacturer-specific GetEccStatus()
    ↓
Winbond interpretation:
  0b001 = "corrected bitflips"
  Return UERR_NAND_BITFLIP_ERROR
    ↓
FTL logs warning
Data returned to application (corrected)
Application doesn't know about bitflips
```

### 4.5 Write Page with Erase Optimization

**Function**: `NAND_FTL_WritePage()` (`lfs_nand_ftl.c:534-584`):
```c
u8 NAND_FTL_WritePage(u32 addr, u8 *buf, u8 do_erase)
{
    NAND_FTL_DeviceTypeDef *nand = NAND_FTL_GetDevice();
    Flash_InfoTypeDef *info = &nand->MemInfo;
    u8 is_bad_block = 0;
    u32 block_addr;
    u8 ret;

    // Step 1: Check initialization
    if (!nand->Initialized) {
        return UERR_INIT;
    }

    // Step 2: Select target/die
    ret = NF_SelectTarget(nand, addr);
    if (ret != HAL_OK) {
        return ret;
    }

    // Step 3: Check bad block
    ret = NF_IsBad(nand, addr, &is_bad_block);
    if (is_bad_block) {
        FS_DBG(FS_WARNING, "Write to bad block 0x%08X", addr);
        return UERR_NAND_BAD_BLOCK;
    }

    // Step 4: Erase block if needed (optimization)
    if (do_erase) {
        block_addr = NF_GetBlockAddr(nand, addr);

        // Check if this block was already erased
        if (block_addr != nand->LastErasedBlockAddr) {
            // Erase the block
            ret = NF_EraseBlock(nand, block_addr);

            if (ret == HAL_OK || ret == UERR_NAND_WORN_BLOCK) {
                // Update cache
                nand->LastErasedBlockAddr = block_addr;
            } else {
                // Erase failed
                return ret;
            }
        }
        // else: block already erased, skip erase
    }

    // Step 5: Write page
    ret = NAND_Page_Write(addr, 0, info->PageSize, buf);

    if (ret == 0) {
        ret = HAL_OK;
    } else if (ret == 0xFF) {
        ret = HAL_TIMEOUT;
    } else {
        // Write failed, mark block as bad
        FS_DBG(FS_ERROR, "Write failed at 0x%08X: 0x%02X", addr, ret);
        NF_MarkBad(nand, addr);
        ret = UERR_NAND_WORN_BLOCK;
    }

    return ret;
}
```

**Erase Optimization Example**:
```
Scenario: Writing 64 pages to a block sequentially

Without optimization:
  Page 0: do_erase=1 → Erase block (3 ms), write page (700 µs)
  Page 1: do_erase=1 → Erase block (3 ms), write page (700 µs)
  ...
  Page 63: do_erase=1 → Erase block (3 ms), write page (700 µs)
  Total: 64 × (3000 + 700) µs = 236.8 ms

With optimization:
  Page 0: do_erase=1 → Erase block (3 ms), write page (700 µs)
          Cache: LastErasedBlockAddr = block 10
  Page 1: do_erase=1 → Check cache (block 10 = last erased)
          Skip erase, write page (700 µs)
  ...
  Page 63: do_erase=1 → Skip erase, write page (700 µs)
  Total: 3000 + (64 × 700) µs = 47.8 ms

Savings: 189 ms (80% faster!)
```

### 4.6 Erase Block

**Function**: `NF_EraseBlock()` (`lfs_nand_ftl.c:167-185`):
```c
static u8 NF_EraseBlock(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    u8 ret;

    // Step 1: Send erase command to NAND
    ret = NAND_Erase(addr);

    // Step 2: Check result
    if (ret == 0) {
        // Success
        ret = HAL_OK;
    } else if (ret == 0xFF) {
        // Timeout
        ret = HAL_TIMEOUT;
    } else {
        // Erase failure - mark block as bad
        FS_DBG(FS_ERROR, "Failed to erase block 0x%08X: 0x%02X", addr, ret);

        ret = NF_MarkBad(nand, addr);

        if (ret == HAL_OK) {
            // Successfully marked as bad
            ret = UERR_NAND_WORN_BLOCK;
        }
    }

    return ret;
}
```

**Erase Failure Handling**:
```
Normal flow:
  1. NAND_Erase(block_addr)
  2. Wait for E_FAIL bit = 0
  3. Return 0 (success)

Failure flow:
  1. NAND_Erase(block_addr)
  2. Wait for completion
  3. E_FAIL bit = 1 (erase failed)
  4. Return status with E_FAIL set
  5. NF_EraseBlock() detects failure
  6. Call NF_MarkBad() to write 0x00 to OOB
  7. Return UERR_NAND_WORN_BLOCK
  8. NAND_FTL_WritePage() returns error to adapter
  9. Adapter returns LFS_ERR_CORRUPT to LittleFS
  10. LittleFS marks block as bad in metadata
  11. LittleFS allocates different block
  12. User write succeeds (transparently)
```

---

## 5. SPI NAND Driver Layer

### 5.1 Command Interface

**Location**: `os/board/rtl8730e/src/component/file_system/littlefs/lfs_spinand.c`

**Key Functions**:
- `NAND_Page_Read()`: Two-stage page read
- `NAND_Page_Write()`: Three-stage page write
- `NAND_Erase()`: Block erase
- `NAND_GetStatus()`: Read status register
- `NAND_SetStatus()`: Write configuration register

### 5.2 Page Read Implementation

**Two-Stage Read Process**:

**Stage 1: Array → Cache** (`lfs_spinand.c:363-377`):
```c
u8 NAND_Page_Read_ArrayToCache(u32 PageAddr)
{
    u8 Addr[3];
    u8 status;

    // Step 1: Format page address (24-bit)
    Addr[0] = (u8)((PageAddr >> 16) & 0xFF);  // PA23-PA16
    Addr[1] = (u8)((PageAddr >> 8) & 0xFF);   // PA15-PA8
    Addr[2] = (u8)(PageAddr & 0xFF);          // PA7-PA0

    // Step 2: Send PAGE READ command (0x13)
    // Command format: [0x13][PA23-16][PA15-8][PA7-0]
    NAND_TxCmd(flash_init_para.FLASH_cmd_page_read, 3, Addr, 0, NULL);

    // Step 3: Wait for transfer to complete
    // NAND internally:
    //   1. Reads page from storage array
    //   2. Performs ECC correction
    //   3. Loads corrected data into page cache
    //   4. Sets ECC status bits in status register
    // This takes 25-60 microseconds
    status = NAND_WaitBusy(WAIT_FLASH_BUSY);

    // Status byte contains ECC result in bits [6:4]
    return status;
}
```

**Stage 2: Cache → Host** (called from `NAND_Page_Read`):
```c
// Wrapper function
u8 NAND_Page_Read(u32 PageAddr, u32 ByteAddr, u32 ByteLen, u8 *pData)
{
    u8 status;

    // Stage 1: Move page from array to NAND internal cache
    status = NAND_Page_Read_ArrayToCache(PageAddr);

    // Stage 2: Read data from cache to host
    // ByteAddr: column address (0-2111)
    // ByteLen: number of bytes to read
    NAND_RxData(flash_init_para.FLASH_cur_cmd, ByteAddr, ByteLen, pData);

    // Return status (contains ECC bits)
    return status;
}
```

**NAND_RxData() - SPI Transfer** (`ameba_rom_patch.c:781-863`):
```c
void NAND_RxData(u8 cmd, u32 StartAddr, u32 read_len, u8 *read_data)
{
    u32 rx_num = 0;
    u8 rd_cmd = cmd;  // e.g., 0x0B (FAST READ), 0x3B (DUAL), 0x6B (QUAD)

    // Step 1: Switch SPIC to user mode
    NAND_UserModeEn(ENABLE);

    // Step 2: Configure SPIC for RX mode
    spi_flash->CTRLR0 &= ~(TMOD(3));
    spi_flash->CTRLR0 |= TMOD(3);  // RX-only mode

    // Step 3: Set number of data frames to receive
    spi_flash->RX_NDF = RX_NDF(read_len);
    spi_flash->TX_NDF = 0;

    // Step 4: Push command and address to TX FIFO
    spi_flash->DR[0].BYTE = rd_cmd;  // READ command

    // GigaDevice quirk: needs dummy byte
    if (flash_init_para.FLASH_addr_phase_len == NAND_COL_ADDR_3_BYTE) {
        spi_flash->DR[0].BYTE = 0x00;  // Dummy
    }

    // Column address (2 bytes)
    spi_flash->DR[0].BYTE = (u8)((StartAddr & 0xFF00) >> 8);  // CA15-CA8
    spi_flash->DR[0].BYTE = (u8)(StartAddr & 0xFF);           // CA7-CA0

    // Step 5: Enable SPI (starts the transaction)
    spi_flash->SSIENR = BIT_SPIC_EN;

    // Step 6: Read data from RX FIFO
    if (UNALIGNED32(read_data)) {
        // Buffer is not 32-bit aligned, read byte-by-byte
        while (rx_num < read_len) {
            if (spi_flash->SR & BIT_RFNE) {  // RX FIFO not empty
                read_data[rx_num] = spi_flash->DR[0].BYTE;
                rx_num++;
            }
        }
    } else {
        // Buffer is 32-bit aligned, read word-by-word (faster)
        u32 *aligned_buf = (u32 *)read_data;

        // Read full words
        while (rx_num < (read_len - 3)) {
            if (spi_flash->SR & BIT_RFNE) {
                aligned_buf[rx_num >> 2] = spi_flash->DR[0].WORD;
                rx_num += 4;
            }
        }

        // Read remaining bytes
        while (rx_num < read_len) {
            if (spi_flash->SR & BIT_RFNE) {
                read_data[rx_num] = spi_flash->DR[0].BYTE;
                rx_num++;
            }
        }
    }

    // Step 7: Wait for transfer to complete
    NAND_WaitBusy(WAIT_TRANS_COMPLETE);
}
```

**Timing Diagram for Quad Read**:
```
Command Phase (1-bit):
CLK:   __┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_┌─┐_____
CS#:   ────┐                             ┌───
           └─────────────────────────────┘
IO0:   ────<0><0><0><0><1><0><1><1>─────  0x6B (QUAD READ)

Address Phase (1-bit):
IO0:   <CA15><CA14><CA13>...<CA1><CA0>────

Data Phase (4-bit):
IO0:   <D0><D4><D8>...
IO1:   <D1><D5><D9>...
IO2:   <D2><D6><D10>...
IO3:   <D3><D7><D11>...
        ▲
        4 bits per clock = 4x faster
```

### 5.3 Page Write Implementation

**Three-Stage Write Process**:

**Stage 1: Write Enable** (`lfs_spinand.c:188-198`):
```c
void NAND_WriteEn(void)
{
    // Step 1: Wait for NAND to be ready
    NAND_WaitBusy(WAIT_FLASH_BUSY);

    // Step 2: Send WRITE ENABLE command (0x06)
    NAND_TxCmd(flash_init_para.FLASH_cmd_wr_en, 0, NULL, 0, NULL);

    // Step 3: Wait for WEL bit to be set
    // Status register bit 1 (WEL) must be 1 before write/erase
    NAND_WaitBusy(WAIT_WRITE_EN);
}
```

**Stage 2: Host → Cache** (called from `NAND_Page_Write`):
```c
u8 NAND_Page_Write(u32 PageAddr, u32 ByteAddr, u32 ByteLen, u8 *pData)
{
    u8 cmd, status;

    // Stage 1: Enable write operations
    NAND_WriteEn();

    // Stage 2: Select command based on mode
    if (flash_init_para.FLASH_QuadEn_bit) {
        cmd = NAND_CMD_QPP;  // 0x32: Quad Program Load
    } else {
        cmd = NAND_CMD_PP;   // 0x02: Program Load
    }

    // Transfer data from host to NAND cache
    NAND_TxData(cmd, ByteAddr, ByteLen, pData);

    // Stage 3: Program cache to array
    status = NAND_Page_Write_Program_Execute(PageAddr);

    return status;
}
```

**NAND_TxData() - SPI Transfer** (`ameba_rom_patch.c:888-970`):
```c
void NAND_TxData(u8 cmd, u32 StartAddr, u32 ByteLen, u8 *pData)
{
    u32 tx_num = 0;

    // Step 1: Switch to user mode
    NAND_UserModeEn(ENABLE);

    // Step 2: Configure for TX mode
    spi_flash->CTRLR0 &= ~(TMOD(3) | CMD_CH(3) | ADDR_CH(3) | DATA_CH(3));

    // Enable quad data channel for QPP
    if ((cmd == NAND_CMD_QPP) || (cmd == NAND_CMD_QPP_RANDOM)) {
        spi_flash->CTRLR0 |= DATA_CH(2);  // 4-bit data mode
    }

    // Step 3: Set TX length
    spi_flash->RX_NDF = 0;
    spi_flash->TX_NDF = TX_NDF(ByteLen);

    // Step 4: Push command and address to FIFO
    spi_flash->DR[0].BYTE = cmd;                          // Command
    spi_flash->DR[0].BYTE = (u8)((StartAddr >> 8) & 0xFF); // CA15-8
    spi_flash->DR[0].BYTE = (u8)(StartAddr & 0xFF);        // CA7-0

    // Step 5: Pre-load FIFO with data (avoid underrun)
    u32 temp = sizeof(spi_flash->DR[0]);  // 4 bytes
    while (tx_num < ByteLen) {
        spi_flash->DR[0].BYTE = pData[tx_num];
        tx_num++;
        if (tx_num == (U32BLOCKSIZE - temp)) break;  // FIFO almost full
    }

    // Step 6: Enable SPI (starts transaction)
    spi_flash->SSIENR = BIT_SPIC_EN;

    // Step 7: Continue pushing data to FIFO
    if (UNALIGNED32(pData)) {
        // Byte-by-byte for unaligned buffers
        while (tx_num < ByteLen) {
            if (spi_flash->SR & BIT_TFNF) {  // TX FIFO not full
                spi_flash->DR[0].BYTE = pData[tx_num];
                tx_num++;
            }
        }
    } else {
        // Word-by-word for aligned buffers
        u32 *aligned_buf = (u32 *)pData;

        while (tx_num < (ByteLen - 3)) {
            if (spi_flash->SR & BIT_TFNF) {
                spi_flash->DR[0].WORD = aligned_buf[tx_num >> 2];
                tx_num += 4;
            }
        }

        // Remaining bytes
        while (tx_num < ByteLen) {
            if (spi_flash->SR & BIT_TFNF) {
                spi_flash->DR[0].BYTE = pData[tx_num];
                tx_num++;
            }
        }
    }

    // Step 8: Wait for transfer complete
    NAND_WaitBusy(WAIT_TRANS_COMPLETE);
}
```

**Stage 3: Cache → Array** (`lfs_spinand.c:417-436`):
```c
u8 NAND_Page_Write_Program_Execute(u32 PageAddr)
{
    u8 Addr[3];
    u8 status;

    // Step 1: Format page address
    Addr[0] = (u8)((PageAddr >> 16) & 0xFF);
    Addr[1] = (u8)((PageAddr >> 8) & 0xFF);
    Addr[2] = (u8)(PageAddr & 0xFF);

    // Step 2: Send PROGRAM EXECUTE command (0x10)
    NAND_TxCmd(flash_init_para.FLASH_cmd_page_write, 3, Addr, 0, NULL);

    // Step 3: Wait for programming to complete (200-700 µs)
    status = NAND_WaitBusy(WAIT_FLASH_BUSY);

    // Step 4: Check for program failure
    if (status & flash_init_para.FLASH_PFail_bit) {
        // P_FAIL bit is set, program failed
        return status;
    } else {
        return 0;  // Success
    }
}
```

### 5.4 Block Erase Implementation

**Function**: `NAND_Erase()` (`lfs_spinand.c:332-353`):
```c
u8 NAND_Erase(u32 PageAddr)
{
    u8 Addr[3];
    u8 status;

    // Step 1: Format page address (any page in block)
    Addr[0] = (u8)((PageAddr >> 16) & 0xFF);
    Addr[1] = (u8)((PageAddr >> 8) & 0xFF);
    Addr[2] = (u8)(PageAddr & 0xFF);

    // Step 2: Enable write operations
    NAND_WriteEn();

    // Step 3: Send BLOCK ERASE command (0xD8)
    // NAND automatically aligns to block boundary
    NAND_TxCmd(flash_init_para.FLASH_cmd_block_e, 3, Addr, 0, NULL);

    // Step 4: Wait for erase to complete (3-10 ms)
    status = NAND_WaitBusy(WAIT_FLASH_BUSY);

    // Step 5: Check for erase failure
    if (status & flash_init_para.FLASH_EFail_bit) {
        // E_FAIL bit is set, erase failed
        return status;
    } else {
        return 0;  // Success
    }
}
```

**Erase Timing**:
```
Total erase time breakdown:

WRITE ENABLE command:        ~1 µs
BLOCK ERASE command:         ~1 µs
NAND internal erase:         3000-10000 µs
Status polling:              ~0.24 µs per poll × N iterations
                             ────────────────────
Total:                       ~3002-10002 µs (3-10 ms)

Timeout protection:          500 ms (0x200000 iterations)
```

### 5.5 Status Register Operations

**Read Status** (`lfs_spinand.c:201-207`):
```c
u8 NAND_GetStatus(u8 FeatureID)
{
    u8 status;

    // Send GET FEATURES command (0x0F) with feature address
    // FeatureID = 0xC0 for status register
    NAND_RxCmd(flash_init_para.FLASH_cmd_rd_status, 1, &FeatureID, 1, &status);

    return status;
}
```

**Write Configuration** (`lfs_spinand.c:220-226`):
```c
void NAND_SetStatus(u8 FeatureID, u8 Value)
{
    // Send SET FEATURES command (0x1F)
    // FeatureID = 0xB0 for configuration register
    NAND_TxCmd(flash_init_para.FLASH_cmd_wr_status, 1, &FeatureID, 1, &Value);
}
```

**Wait Busy** (`lfs_spinand.c:141-179`):
```c
u8 NAND_WaitBusy(u32 WaitType)
{
    u32 BusyCheck = 0;
    u8 status = 0;
    u32 i = 0;

    do {
        if (WaitType == WAIT_SPIC_BUSY) {
            // Wait for SPIC controller
            BusyCheck = (spi_flash->SR & BIT_BUSY);

        } else if (WaitType == WAIT_TRANS_COMPLETE) {
            // Wait for SPI transfer complete
            BusyCheck = (spi_flash->SSIENR & BIT_SPIC_EN);

        } else if (WaitType == WAIT_FLASH_BUSY) {
            i++;
            // Read NAND status register
            status = NAND_GetStatus(NAND_REG_STATUS);
            BusyCheck = (status & flash_init_para.FLASH_Busy_bit);  // OIP bit

        } else if (WaitType == WAIT_WRITE_EN) {
            i++;
            status = NAND_GetStatus(NAND_REG_STATUS);
            // Check WEL bit is set AND device not busy
            BusyCheck = (!(status & flash_init_para.FLASH_WLE_bit))
                     || (status & flash_init_para.FLASH_Busy_bit);
        }

        if (!BusyCheck) break;  // No longer busy

        // Timeout protection (500 ms)
        if (i > 0x200000) {
            status = 0xFF;  // Timeout indicator
            break;
        }
    } while (1);

    return status;
}
```

---

## 6. Hardware Abstraction Layer

### 6.1 SPIC Register Access

**Location**: `os/board/rtl8730e/src/component/soc/amebad2/fwlib/ameba_rom_patch.c`

**User Mode Control** (`ameba_rom_patch.c`):
```c
void NAND_UserModeEn(u8 NewState)
{
    if (NewState == ENABLE) {
        // Disable auto mode, enable user mode
        spi_flash->SSIENR = 0;         // Disable SPI
        spi_flash->CTRLR0 |= BIT_USER_MODE;
    } else {
        // Enable auto mode (for NOR flash)
        spi_flash->CTRLR0 &= ~BIT_USER_MODE;
    }
}
```

**Command Transmission**:
```c
void NAND_TxCmd(u8 cmd, u32 addr_len, u8 *paddr, u32 data_len, u8 *pdata)
{
    u32 i;

    // Disable SPI
    spi_flash->SSIENR = 0;

    // Set TX-only mode
    spi_flash->CTRLR0 &= ~TMOD(3);
    spi_flash->CTRLR0 |= TMOD(1);

    // Set data frame count
    spi_flash->TX_NDF = TX_NDF(data_len);
    spi_flash->RX_NDF = 0;

    // Push command
    spi_flash->DR[0].BYTE = cmd;

    // Push address bytes
    for (i = 0; i < addr_len; i++) {
        spi_flash->DR[0].BYTE = paddr[i];
    }

    // Push data bytes
    for (i = 0; i < data_len; i++) {
        spi_flash->DR[0].BYTE = pdata[i];
    }

    // Enable SPI (starts transaction)
    spi_flash->SSIENR = BIT_SPIC_EN;

    // Wait for completion
    NAND_WaitBusy(WAIT_TRANS_COMPLETE);
}
```

### 6.2 SPIC Registers

**Key Register Addresses**:
```c
#define SPIC_BASE            0x50000000

struct SPIC_TypeDef {
    __IO u32 CTRLR0;           // +0x00: Control Register 0
    __IO u32 CTRLR1;           // +0x04: Control Register 1
    __IO u32 SSIENR;           // +0x08: SSI Enable
    __I  u32 RSVD0;            // +0x0C: Reserved
    __IO u32 SER;              // +0x10: Slave Enable
    __IO u32 BAUDR;            // +0x14: Baud Rate Select
    __IO u32 TXFTLR;           // +0x18: TX FIFO Threshold
    __IO u32 RXFTLR;           // +0x1C: RX FIFO Threshold
    __I  u32 TXFLR;            // +0x20: TX FIFO Level
    __I  u32 RXFLR;            // +0x24: RX FIFO Level
    __I  u32 SR;               // +0x28: Status Register
    __IO u32 IMR;              // +0x2C: Interrupt Mask
    __I  u32 ISR;              // +0x30: Interrupt Status
    __I  u32 RISR;             // +0x34: Raw Interrupt Status
    __I  u32 TXOICR;           // +0x38: TX FIFO Overflow Int Clear
    __I  u32 RXOICR;           // +0x3C: RX FIFO Overflow Int Clear
    __I  u32 RXUICR;           // +0x40: RX FIFO Underflow Int Clear
    // ... more registers ...
    __IO u32 DR[32];           // +0x60-0xDC: Data Register (FIFO)
    __IO u32 RX_NDF;           // +0xA0: RX Number of Data Frames
    __IO u32 TX_NDF;           // +0xA4: TX Number of Data Frames
    // ... more registers ...
};

#define spi_flash ((struct SPIC_TypeDef *)SPIC_BASE)
```

**Status Register Bits**:
```c
#define BIT_BUSY    BIT(0)   // SPI busy
#define BIT_TFNF    BIT(1)   // TX FIFO not full
#define BIT_TFE     BIT(2)   // TX FIFO empty
#define BIT_RFNE    BIT(3)   // RX FIFO not empty
#define BIT_RFF     BIT(4)   // RX FIFO full
```

---

## 7. Data Structures and APIs

### 7.1 Key Structures Summary

**LittleFS Configuration**:
```c
struct lfs_config {
    void *context;
    int (*read)(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, void *buffer, lfs_size_t size);
    int (*prog)(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, const void *buffer, lfs_size_t size);
    int (*erase)(const struct lfs_config *c, lfs_block_t block);
    int (*sync)(const struct lfs_config *c);
    lfs_size_t read_size;
    lfs_size_t prog_size;
    lfs_size_t block_size;
    lfs_size_t block_count;
    int32_t block_cycles;
    lfs_size_t cache_size;
    lfs_size_t lookahead_size;
};
```

**NAND FTL Device**:
```c
typedef struct {
    Flash_InfoTypeDef MemInfo;
    void *MfgOps;
    u8 CurTarget;
    u8 Initialized;
    u32 LastErasedBlockAddr;
} NAND_FTL_DeviceTypeDef;
```

**Flash Information**:
```c
typedef struct {
    u8  MID, DID, ExtDID;
    u8  MFG[12], Model[20];
    u32 PageSize, PagesPerBlock;
    u16 OobSize;
    u32 BlocksPerLun;
    u8  LunsPerTarget, Targets;
    u16 MaxBadBlocksPerLun;
    u8  ReqHostEccLevel;
    u32 Capacity;
} Flash_InfoTypeDef;
```

### 7.2 API Summary

**Application Level (POSIX)**:
```c
int fd = open("/mnt/file.txt", O_RDWR | O_CREAT);
ssize_t bytes = read(fd, buffer, size);
ssize_t bytes = write(fd, buffer, size);
off_t pos = lseek(fd, offset, SEEK_SET);
int ret = close(fd);
```

**LittleFS Level**:
```c
int lfs_mount(lfs_t *lfs, const struct lfs_config *config);
int lfs_file_open(lfs_t *lfs, lfs_file_t *file, const char *path, int flags);
lfs_ssize_t lfs_file_read(lfs_t *lfs, lfs_file_t *file, void *buffer, lfs_size_t size);
lfs_ssize_t lfs_file_write(lfs_t *lfs, lfs_file_t *file, const void *buffer, lfs_size_t size);
int lfs_file_close(lfs_t *lfs, lfs_file_t *file);
```

**Adapter Level**:
```c
int lfs_nand_read(const struct lfs_config *c, lfs_block_t block,
                  lfs_off_t off, void *buffer, lfs_size_t size);
int lfs_nand_prog(const struct lfs_config *c, lfs_block_t block,
                  lfs_off_t off, const void *buffer, lfs_size_t size);
int lfs_nand_erase(const struct lfs_config *c, lfs_block_t block);
```

**FTL Level**:
```c
u8 NAND_FTL_Init(NAND_FTL_DeviceTypeDef *nand);
u8 NAND_FTL_ReadPage(u32 addr, u8 *buf);
u8 NAND_FTL_WritePage(u32 addr, u8 *buf, u8 do_erase);
u8 NAND_FTL_EraseBlock(u32 addr, u8 force);
```

**Driver Level**:
```c
u8 NAND_Page_Read(u32 PageAddr, u32 ByteAddr, u32 ByteLen, u8 *pData);
u8 NAND_Page_Write(u32 PageAddr, u32 ByteAddr, u32 ByteLen, u8 *pData);
u8 NAND_Erase(u32 PageAddr);
u8 NAND_GetStatus(u8 FeatureID);
void NAND_SetStatus(u8 FeatureID, u8 Value);
```

---

## 8. Threading and Synchronization

### 8.1 Locking Strategy

**Two-Level Locking**:

1. **LittleFS Metadata Lock** (`lfs_op_mux`):
   - Protects LittleFS internal structures
   - Prevents concurrent filesystem operations
   - Optional (via LFS_THREADSAFE flag)

2. **Flash Hardware Lock** (`RT_DEV_LOCK_FLASH`):
   - Protects SPIC hardware registers
   - Prevents concurrent flash access
   - Always enabled

**Lock Hierarchy**:
```
Thread A                Thread B
────────                ────────
lfs_file_write()       lfs_file_read()
  ├─ lfs_op_mux.lock()    ├─ lfs_op_mux.lock() ← Blocks until A releases
  │  (acquired)           │  (waiting...)
  ├─ lfs_nand_prog()
  │  ├─ RT_DEV_LOCK_FLASH.lock()
  │  │  (acquired)
  │  ├─ NAND_FTL_WritePage()
  │  ├─ NAND_Page_Write()
  │  └─ RT_DEV_LOCK_FLASH.unlock()
  └─ lfs_op_mux.unlock()
                          ├─ lfs_op_mux.lock() ← Now acquires
                          │  (acquired)
                          ├─ lfs_nand_read()
                          │  ├─ RT_DEV_LOCK_FLASH.lock()
                          │  │  (acquired)
                          │  ├─ NAND_FTL_ReadPage()
                          │  └─ RT_DEV_LOCK_FLASH.unlock()
                          └─ lfs_op_mux.unlock()
```

### 8.2 LittleFS Lock Implementation

**Code** (`littlefs_adapter.c:180-201`):
```c
#ifdef LFS_THREADSAFE
_mutex lfs_op_mux = NULL;

int lfs_diskio_lock(const struct lfs_config *c)
{
    if (lfs_op_mux == NULL) {
        rtw_mutex_init(&lfs_op_mux);
    }
    rtw_mutex_get(&lfs_op_mux);
    return LFS_ERR_OK;
}

int lfs_diskio_unlock(const struct lfs_config *c)
{
    if (lfs_op_mux == NULL) {
        rtw_mutex_init(&lfs_op_mux);
    }
    rtw_mutex_put(&lfs_op_mux);
    return LFS_ERR_OK;
}
#endif
```

### 8.3 Hardware Lock Implementation

**Usage** (`littlefs_adapter.c:37-46`):
```c
int lfs_nand_read(const struct lfs_config *c, lfs_block_t block,
                  lfs_off_t off, void *buffer, lfs_size_t size)
{
    device_mutex_lock(RT_DEV_LOCK_FLASH);

    // ... NAND operations ...

    device_mutex_unlock(RT_DEV_LOCK_FLASH);
}
```

**Why Hardware Lock is Critical**:
```
Without lock:
  Thread A: NAND_Page_Read(page=100)
    → spi_flash->CTRLR0 = RX_MODE
    → spi_flash->DR[0] = CMD
  Thread B: NAND_Page_Write(page=200) ← Context switch
    → spi_flash->CTRLR0 = TX_MODE  ← Corrupts A's operation!
    → spi_flash->DR[0] = CMD
  Result: Both operations fail, possible hardware lockup

With lock:
  Thread A: device_mutex_lock()
    → NAND_Page_Read(page=100)
  Thread B: device_mutex_lock() ← Blocks
    (waits for A to finish)
  Thread A: device_mutex_unlock()
  Thread B: (now acquires lock)
    → NAND_Page_Write(page=200)
  Result: Both operations succeed
```

---

## Summary

This part covered the detailed software implementation:

**Key Takeaways**:

1. **LittleFS**: Power-loss resilient with copy-on-write and metadata pairs
2. **Adapter Layer**: Translates LFS blocks to NAND pages, provides thread safety
3. **FTL Layer**: Manages bad blocks, checks ECC, optimizes erases
4. **Driver Layer**: Two-stage read, three-stage write, handles SPI protocol
5. **Hardware Layer**: Direct SPIC register access for SPI transactions
6. **Threading**: Two-level locking (filesystem metadata + hardware)

**Next Part**: Detailed operation flows, error handling, debugging techniques

---

**Document Version**: 1.0
**Last Updated**: 2025
**Target Platform**: RTL8730E with W25N NAND Flash
