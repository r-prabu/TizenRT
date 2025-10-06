# TizenRT NAND Flash with LittleFS - Part 5: Dhara FTL Integration
## For RTL8730E Chipset - Beginner's Guide

---

## Table of Contents
1. [Dhara FTL Overview](#dhara-ftl-overview)
2. [Architecture and Design](#architecture-and-design)
3. [Dhara vs Custom FTL Comparison](#dhara-vs-custom-ftl-comparison)
4. [Integration with TizenRT](#integration-with-tizenrt)
5. [Configuration and Setup](#configuration-and-setup)
6. [Operation Details](#operation-details)
7. [Garbage Collection](#garbage-collection)
8. [Performance Characteristics](#performance-characteristics)

---

## 1. Dhara FTL Overview

### 1.1 What is Dhara?

**Dhara** is an open-source Flash Translation Layer (FTL) designed specifically for NAND flash memory. It was created by Daniel Beer and is licensed under a permissive ISC license.

**Key Features**:
- **Wear Leveling**: Automatically distributes erases across all blocks
- **Bad Block Management**: Handles factory and runtime bad blocks transparently
- **Journaling**: Provides power-loss protection through journaled metadata
- **Garbage Collection**: Automatic reclamation of stale data
- **Sector Mapping**: Maps logical sectors to physical NAND pages
- **ECC Handling**: Works with hardware or software ECC

**Purpose in TizenRT**:
Dhara sits between the MTD (Memory Technology Device) NAND driver and the filesystem (LittleFS), providing a reliable block device interface with wear leveling and bad block management.

### 1.2 Why Use Dhara?

**Advantages**:
1. **Proven Code**: Well-tested, open-source implementation
2. **Power-Loss Safe**: Journaling ensures consistency
3. **Automatic Wear Leveling**: Extends flash lifetime
4. **Bad Block Transparent**: Hides bad blocks from upper layers
5. **Configurable**: Tunable garbage collection ratio
6. **Lightweight**: Small memory footprint

**Compared to Direct NAND Access**:
```
Without Dhara:
  Application → LittleFS → NAND Driver
  - LittleFS must handle bad blocks
  - No wear leveling below filesystem level
  - Risk of uneven wear

With Dhara:
  Application → LittleFS → Dhara FTL → NAND Driver
  - Dhara handles bad blocks transparently
  - Dhara provides wear leveling
  - LittleFS sees perfect block device
  - Better flash lifetime
```

### 1.3 Dhara in TizenRT Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    User Application                          │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                  VFS (Virtual File System)                   │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                   LittleFS Filesystem                        │
│  • File/directory management                                │
│  • Power-loss protection                                    │
│  • Metadata redundancy                                      │
└────────────────────────┬────────────────────────────────────┘
                         │ Block Device Interface
┌────────────────────────▼────────────────────────────────────┐
│                    Dhara FTL Layer                           │
│  • Sector → Page mapping                                    │
│  • Wear leveling                                            │
│  • Garbage collection                                       │
│  • Bad block management                                     │
│  • Journal for power-loss protection                        │
└────────────────────────┬────────────────────────────────────┘
                         │ MTD Interface
┌────────────────────────▼────────────────────────────────────┐
│                MTD NAND Driver Layer                         │
│  • Bad block detection                                      │
│  • ECC status checking                                      │
│  • Page read/write/erase                                    │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│              NAND Flash Hardware (W25N)                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture and Design

### 2.1 Core Components

**Dhara consists of three main components**:

1. **NAND Interface** (`dhara/nand.h`)
   - Abstracts physical NAND operations
   - Provides callbacks for read/write/erase
   - Handles bad block detection

2. **Journal** (`dhara/journal.h`, `dhara/journal.c`)
   - Maintains page allocation metadata
   - Provides power-loss protection
   - Tracks free/used pages

3. **Map** (`dhara/map.h`, `dhara/map.c`)
   - Maps logical sectors to physical pages
   - Implements garbage collection
   - Manages sector allocation

### 2.2 Data Structures

**Dhara Device Structure** (`os/fs/driver/mtd/dhara.c:73-96`):
```c
struct dhara_dev_s {
    struct dhara_nand nand;      // NAND interface callbacks
    struct dhara_map map;         // Sector mapping
    sem_t lock;                   // Thread synchronization

    FAR struct mtd_dev_s *mtd;    // Underlying MTD device
    struct mtd_geometry_s geo;    // Device geometry
    uint16_t blkper;              // R/W blocks per erase block
    uint16_t refs;                // Reference count
    bool unlinked;                // Unlink flag

    FAR uint8_t *pagebuf;         // Working buffer (2 × page_size)

    // Read cache for metadata acceleration
    struct dq_queue_s readcache;
    dhara_pagecache_t readpage[CONFIG_DHARA_READ_NCACHES];
};
```

**Dhara Map Structure** (`dhara/map.h:48-53`):
```c
struct dhara_map {
    struct dhara_journal journal; // Journal for metadata
    uint8_t gc_ratio;             // Garbage collection ratio
    dhara_sector_t count;         // Number of allocated sectors
};
```

**Dhara Journal Structure** (`dhara/journal.h`):
```c
struct dhara_journal {
    const struct dhara_nand *nand;  // NAND interface
    uint8_t *page_buf;              // Metadata buffer

    dhara_page_t root;              // Root page of journal
    uint32_t epoch;                 // Current epoch number

    dhara_block_t bb_current;       // Current bad block
    dhara_block_t bb_last;          // Last bad block
};
```

### 2.3 Sector and Page Mapping

**Logical to Physical Mapping**:

```
Logical View (LittleFS sees):
  Sector 0, Sector 1, Sector 2, ..., Sector N
  (Contiguous logical address space)
       ↓
Dhara Map Translation:
  Sector 0 → Page 100
  Sector 1 → Page 45
  Sector 2 → Page 200
  ...
       ↓
Physical NAND:
  Page 0:   (free)
  Page 45:  Sector 1 data
  Page 100: Sector 0 data
  Page 150: (old Sector 0 data - stale)
  Page 200: Sector 2 data
  ...
```

**Key Concepts**:
- **Sector**: Logical unit (e.g., 2048 bytes) that LittleFS reads/writes
- **Page**: Physical NAND page (also 2048 bytes in this case)
- **Block**: Erase unit containing multiple pages (64 pages = 128 KB)

**Mapping Table**:
Dhara maintains a mapping table in the journal:
```c
typedef uint32_t dhara_sector_t;  // Logical sector number
typedef uint32_t dhara_page_t;    // Physical page number

// Mapping stored in journal metadata
// Sector S → Page P
```

### 2.4 Journaling for Power-Loss Protection

**Journal Structure**:

```
NAND Flash Layout with Journal:

Block 0 (Journal Root):
┌─────────────────────────────────────┐
│ Page 0: Journal Header              │
│   - Magic number: "DHARA"           │
│   - Epoch: 12345                    │
│   - Root page pointer               │
├─────────────────────────────────────┤
│ Page 1: Sector Map (part 1)        │
│   - Sector 0 → Page 100             │
│   - Sector 1 → Page 45              │
│   - ...                             │
├─────────────────────────────────────┤
│ Page 2: Sector Map (part 2)        │
│   - More mappings...                │
└─────────────────────────────────────┘

Block 1-N: Data Pages
┌─────────────────────────────────────┐
│ Page 45: Sector 1 data              │
│ Page 100: Sector 0 data             │
│ Page 200: Sector 2 data             │
│ ...                                 │
└─────────────────────────────────────┘
```

**Journal Update Process**:
```
1. Before Write:
   Journal: Sector 5 → Page 300 (old mapping)

2. Write New Data:
   Write sector 5 data to Page 450 (new page)

3. Update Journal:
   Write new journal entry: Sector 5 → Page 450
   Increment epoch number
   Commit journal

4. After Commit:
   Journal: Sector 5 → Page 450 (new mapping)
   Page 300 becomes stale (marked for GC)

Power failure during step 2-3:
   → Journal still has old mapping (Page 300)
   → Data remains consistent
   → New write (Page 450) is orphaned, will be GC'd
```

**Epoch Number**:
```c
// Epoch increments on each journal update
// Used to determine which journal is newest after power loss

Journal A: Epoch 100
Journal B: Epoch 101  ← Newer, use this one

// Prevents journal corruption confusion
```

---

## 3. Dhara vs Custom FTL Comparison

### 3.1 Implementation Comparison

**Custom FTL** (in `lfs_nand_ftl.c`):
```
Purpose: Lightweight abstraction over NAND hardware
Features:
  ✓ Bad block detection and marking
  ✓ ECC status checking
  ✓ Multi-die support
  ✓ Manufacturer-specific quirks
  ✗ No wear leveling
  ✗ No garbage collection
  ✗ Direct page mapping (no translation)

Use Case: When LittleFS provides wear leveling
Location: os/board/rtl8730e/src/component/file_system/littlefs/
```

**Dhara FTL** (in `dhara.c`):
```
Purpose: Full-featured FTL with wear leveling
Features:
  ✓ Bad block management
  ✓ Sector-to-page mapping
  ✓ Automatic wear leveling
  ✓ Garbage collection
  ✓ Journal for power-loss protection
  ✓ Read caching for metadata

Use Case: When additional wear leveling is desired
Location: os/fs/driver/mtd/dhara.c
```

### 3.2 Architectural Differences

**With Custom FTL**:
```
LittleFS
   ↓ (reads/writes blocks directly)
Custom FTL
   ↓ (checks bad blocks, ECC)
NAND Driver
   ↓
Hardware

Wear Leveling: LittleFS handles it
Bad Blocks: Custom FTL + LittleFS both aware
```

**With Dhara FTL**:
```
LittleFS
   ↓ (reads/writes sectors)
Dhara FTL
   ↓ (maps sectors, wear levels, GC)
MTD Driver
   ↓ (bad blocks, ECC)
Hardware

Wear Leveling: Both Dhara + LittleFS
Bad Blocks: Dhara hides from LittleFS
```

### 3.3 Configuration Choice

**In `defconfig` for rtl8730e/flat_dev_ddr_nand**:
```bash
# Dhara FTL is enabled
CONFIG_MTD_DHARA=y
CONFIG_DHARA_GC_RATIO=4
CONFIG_DHARA_READ_NCACHES=4

# This means Dhara FTL is used instead of custom FTL
# LittleFS sits on top of Dhara block device
```

**Stack Selection**:
```
Current Configuration (with Dhara):
  LittleFS → Dhara FTL → MTD NAND → Hardware

Alternative (without Dhara):
  LittleFS → Custom FTL → SPI NAND Driver → Hardware

Decision factors:
  - Use Dhara for maximum flash lifetime
  - Use Custom FTL for simpler, lighter stack
  - RTL8730E uses Dhara by default
```

---

## 4. Integration with TizenRT

### 4.1 Device Registration

**Initialization Flow** (`dhara.c:550-650`):

```c
FAR struct mtd_dev_s *dhara_initialize(FAR struct mtd_dev_s *mtd)
{
    FAR dhara_dev_t *dev;
    struct mtd_geometry_s geo;
    dhara_error_t err;

    // Step 1: Get MTD geometry
    ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);

    // Step 2: Allocate Dhara device
    dev = (FAR dhara_dev_t *)kmm_zalloc(sizeof(dhara_dev_t));

    // Step 3: Setup NAND interface callbacks
    dev->nand.log2_page_size = ilog2(geo.blocksize);   // e.g., 11 for 2048
    dev->nand.log2_ppb = ilog2(geo.erasesize / geo.blocksize); // e.g., 6 for 64
    dev->nand.num_blocks = geo.neraseblocks;           // e.g., 2048

    // Callbacks
    dev->nand.read = dhara_nand_read;
    dev->nand.prog = dhara_nand_prog;
    dev->nand.erase = dhara_nand_erase;
    dev->nand.is_bad = dhara_nand_is_bad;
    dev->nand.mark_bad = dhara_nand_mark_bad;

    // Step 4: Allocate page buffer (2 × page_size for journal)
    dev->pagebuf = (FAR uint8_t *)kmm_malloc(geo.blocksize * 2);

    // Step 5: Initialize read cache
    dhara_init_readcache(dev);

    // Step 6: Initialize Dhara map
    dhara_map_init(&dev->map, &dev->nand, dev->pagebuf, CONFIG_DHARA_GC_RATIO);

    // Step 7: Resume from existing state or format
    if (dhara_map_resume(&dev->map, &err) < 0) {
        // No valid journal found, clear map (format)
        dhara_map_clear(&dev->map);
    }

    // Step 8: Initialize semaphore
    sem_init(&dev->lock, 0, 1);

    return (FAR struct mtd_dev_s *)dev;
}
```

**Registration in Boot** (`rtl8730e_boot.c`):
```c
void rtl8730e_flash_initialize(void)
{
    struct mtd_dev_s *mtd_nand;
    struct mtd_dev_s *mtd_dhara;

    // Initialize NAND MTD driver
    mtd_nand = w25n_initialize(spi);

    // Wrap with Dhara FTL
    mtd_dhara = dhara_initialize(mtd_nand);

    // Register as block device
    register_mtddriver("/dev/mtdblock1", mtd_dhara, 0, NULL);
}
```

### 4.2 NAND Interface Callbacks

**Read Callback** (`dhara.c:102-145`):
```c
static int dhara_nand_read(const struct dhara_nand *n,
                           dhara_page_t p, size_t offset,
                           size_t length, uint8_t *data)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
    off_t block_offset;
    ssize_t ret;

    // Check cache first
    FAR uint8_t *buf = dhara_find_readcache(dev, p);
    if (buf) {
        // Cache hit
        memcpy(data, buf + offset, length);
        return 0;
    }

    // Cache miss - read from NAND
    buf = dhara_grab_readcache(dev);
    block_offset = p * dev->geo.blocksize;

    ret = MTD_READ(dev->mtd, block_offset, dev->geo.blocksize, buf);

    if (ret == dev->geo.blocksize) {
        // Success - update cache
        buf->page = p;
        dhara_insert_readcache(dev, buf);
        memcpy(data, buf + offset, length);
        return 0;
    }

    return -EIO;
}
```

**Write Callback** (`dhara.c:147-175`):
```c
static int dhara_nand_prog(const struct dhara_nand *n,
                           dhara_page_t p, const uint8_t *data)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
    off_t block_offset;
    ssize_t ret;

    // Calculate offset
    block_offset = p * dev->geo.blocksize;

    // Write to NAND
    ret = MTD_WRITE(dev->mtd, block_offset, dev->geo.blocksize, data);

    if (ret == dev->geo.blocksize) {
        // Update cache with new data
        dhara_update_readcache(dev, p, data);
        return 0;
    }

    return -EIO;
}
```

**Erase Callback** (`dhara.c:177-199`):
```c
static int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
    off_t erase_offset;
    int ret;

    // Calculate block offset
    erase_offset = b * dev->geo.erasesize;

    // Erase block
    ret = MTD_ERASE(dev->mtd, erase_offset / dev->geo.erasesize, 1);

    if (ret == 0) {
        // Discard cache entries for all pages in erased block
        for (dhara_page_t p = b * dev->blkper;
             p < (b + 1) * dev->blkper; p++) {
            dhara_discard_readcache(dev, p);
        }
        return 0;
    }

    return -EIO;
}
```

**Bad Block Callbacks**:
```c
static int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
    // Ask MTD driver if block is bad
    return MTD_ISBAD(dev->mtd, b);
}

static int dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
    // Mark block as bad in MTD
    return MTD_MARKBAD(dev->mtd, b);
}
```

### 4.3 Block Device Interface

**Read Operation** (`dhara.c:330-380`):
```c
static ssize_t dhara_read(FAR struct inode *inode, FAR unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
    FAR dhara_dev_t *dev = inode->i_private;
    dhara_error_t err;
    dhara_sector_t s;

    sem_wait(&dev->lock);

    for (s = 0; s < nsectors; s++) {
        // Read logical sector via Dhara map
        if (dhara_map_read(&dev->map, start_sector + s,
                          buffer + s * dev->geo.blocksize, &err) < 0) {
            // Error reading sector
            sem_post(&dev->lock);
            return -dhara_error_to_errno(err);
        }
    }

    sem_post(&dev->lock);
    return nsectors;
}
```

**Write Operation** (`dhara.c:382-435`):
```c
static ssize_t dhara_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           size_t start_sector, unsigned int nsectors)
{
    FAR dhara_dev_t *dev = inode->i_private;
    dhara_error_t err;
    dhara_sector_t s;

    sem_wait(&dev->lock);

    for (s = 0; s < nsectors; s++) {
        // Write logical sector via Dhara map
        if (dhara_map_write(&dev->map, start_sector + s,
                           buffer + s * dev->geo.blocksize, &err) < 0) {
            sem_post(&dev->lock);
            return -dhara_error_to_errno(err);
        }
    }

    sem_post(&dev->lock);
    return nsectors;
}
```

**IOCTL Operations** (`dhara.c:437-510`):
```c
static int dhara_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
    FAR dhara_dev_t *dev = inode->i_private;
    dhara_error_t err;

    switch (cmd) {
        case BIOC_GETFORMAT:
            // Report sector size and count
            FAR struct geometry *geo = (FAR struct geometry *)arg;
            geo->geo_available = true;
            geo->geo_mediachanged = false;
            geo->geo_writeenabled = true;
            geo->geo_nsectors = dhara_map_capacity(&dev->map);
            geo->geo_sectorsize = dev->geo.blocksize;
            return OK;

        case BIOC_SYNC:
            // Sync journal to flash
            sem_wait(&dev->lock);
            ret = dhara_map_sync(&dev->map, &err);
            sem_post(&dev->lock);
            return (ret < 0) ? -dhara_error_to_errno(err) : OK;

        case MTDIOC_BULKERASE:
            // Erase all sectors
            sem_wait(&dev->lock);
            dhara_map_clear(&dev->map);
            sem_post(&dev->lock);
            return OK;

        default:
            return -ENOTTY;
    }
}
```

---

## 5. Configuration and Setup

### 5.1 Kernel Configuration

**Enable Dhara** (`defconfig`):
```bash
# Enable MTD NAND support
CONFIG_MTD=y
CONFIG_MTD_NAND=y
CONFIG_MTD_W25N=y

# Enable Dhara FTL
CONFIG_MTD_DHARA=y

# Dhara parameters
CONFIG_DHARA_GC_RATIO=4          # GC ratio (1-255)
CONFIG_DHARA_READ_NCACHES=4      # Number of read cache entries
```

**Parameter Explanation**:

| Parameter | Range | Description |
|-----------|-------|-------------|
| `CONFIG_DHARA_GC_RATIO` | 1-255 | GC operations per write (lower = more GC) |
| `CONFIG_DHARA_READ_NCACHES` | 1-16 | Metadata page cache count |

### 5.2 GC Ratio Configuration

**What is GC Ratio?**
```c
// GC Ratio = Number of writes per GC operation
// Example: GC_RATIO=4

Write sector 0  ← No GC
Write sector 1  ← No GC
Write sector 2  ← No GC
Write sector 3  ← No GC
Write sector 4  ← Trigger GC (every 4th write)
Write sector 5  ← No GC
...
Write sector 8  ← Trigger GC
```

**GC Ratio Trade-offs**:

```
Low GC Ratio (e.g., 1-2):
  Pros:
    ✓ More frequent GC
    ✓ Less stale data accumulated
    ✓ More predictable write latency
  Cons:
    ✗ Lower usable capacity
    ✗ More erase cycles (faster wear)
    ✗ Lower write throughput

High GC Ratio (e.g., 8-16):
  Pros:
    ✓ Higher usable capacity
    ✓ Fewer erase cycles (longer lifetime)
    ✓ Higher write throughput
  Cons:
    ✗ More stale data accumulates
    ✗ Occasional long GC pauses
    ✗ Less predictable latency

Recommended: 4-8 for balanced performance
```

### 5.3 Read Cache Configuration

**Purpose of Read Cache**:
```c
// Dhara metadata (journal, map) is stored in NAND pages
// Reading metadata on every sector access is slow

Without cache:
  Read sector 0 → Read journal page → Read data page (2 NAND reads)
  Read sector 1 → Read journal page → Read data page (2 NAND reads)
  ...

With cache (4 entries):
  Read sector 0 → Read journal (cached) → Read data (1 NAND read)
  Read sector 1 → Use cached journal → Read data (1 NAND read)
  ...
  50% reduction in NAND reads for metadata
```

**Cache Entry Structure**:
```c
struct dhara_pagecache_s {
    dq_entry_t node;          // Doubly-linked list node
    dhara_page_t page;        // Page number
    FAR uint8_t *buffer;      // Cached page data (2048 bytes)
};

// Total cache memory = NCACHES × page_size
// Example: 4 caches × 2048 bytes = 8 KB
```

**Cache Replacement Policy**:
```
LRU (Least Recently Used):

  Cache state:
    [Most recent] → [Page 5] → [Page 12] → [Page 3] → [Page 9] → [Least recent]

  Access page 12:
    [Page 12] → [Page 5] → [Page 3] → [Page 9]  (moved to front)

  Access new page 20 (cache full):
    [Page 20] → [Page 12] → [Page 5] → [Page 3]  (evict Page 9)
```

### 5.4 Memory Usage

**Dhara Memory Footprint**:
```c
// Per-device allocation
sizeof(dhara_dev_t) = ~100 bytes (structs)
  + (2 × page_size) for pagebuf = 4096 bytes
  + (NCACHES × page_size) for cache = 4 × 2048 = 8192 bytes
  + (NCACHES × sizeof(dhara_pagecache_t)) = 4 × 24 = 96 bytes
  ────────────────────────────────────────────────
  Total ≈ 12.4 KB per Dhara device

Journal metadata (in NAND):
  ~1-2 blocks reserved for journal
  = 128-256 KB of NAND space
```

---

## 6. Operation Details

### 6.1 Sector Read Flow

**Complete Read Path**:

```
Application: read(fd, buffer, 2048)
   ↓
LittleFS: Read block 0
   ↓
dhara_read(sector=0)
   ├─ sem_wait(&dev->lock)
   ├─ dhara_map_read(&dev->map, 0, buffer, &err)
   │    ↓
   │  dhara_map_find(map, sector=0, &page, &err)
   │    ├─ Search journal for sector 0 mapping
   │    ├─ Check read cache for journal page
   │    ├─ Cache hit! Journal says: Sector 0 → Page 100
   │    └─ Return page=100
   │    ↓
   │  dhara_nand_read(nand, page=100, 0, 2048, buffer)
   │    ├─ Check read cache for page 100
   │    ├─ Cache miss
   │    ├─ MTD_READ(mtd, offset=100×2048, 2048, buffer)
   │    │    ↓
   │    │  w25n_read() → NAND_FTL_ReadPage() → NAND_Page_Read()
   │    │    (See Part 4 for details)
   │    │    ↓
   │    ├─ Update read cache with page 100 data
   │    └─ Return success
   │    ↓
   └─ sem_post(&dev->lock)
   ↓
LittleFS: Got 2048 bytes
   ↓
Application: Success
```

**Journal Lookup Optimization**:
```c
// Without cache:
Every sector read → Journal page read → Data page read
= 2 NAND reads per sector

// With 4-entry cache:
First 4 sector reads → Journal cached
Subsequent reads → Only data page read
= 1 NAND read per sector (after cache warm-up)

Speedup: 2x for repeated metadata access
```

### 6.2 Sector Write Flow

**Complete Write Path with GC**:

```
Application: write(fd, buffer, 2048)
   ↓
LittleFS: Write block 0
   ↓
dhara_write(sector=0, buffer)
   ├─ sem_wait(&dev->lock)
   ├─ dhara_map_write(&dev->map, 0, buffer, &err)
   │    ↓
   │  // Step 1: Find free page
   │  dhara_journal_next_page(&map->journal, &new_page, &err)
   │    ├─ Search journal for free page
   │    ├─ Find: Page 450 is free
   │    └─ Return new_page=450
   │    ↓
   │  // Step 2: Write data to new page
   │  dhara_nand_prog(nand, page=450, buffer)
   │    ├─ MTD_WRITE(mtd, offset=450×2048, 2048, buffer)
   │    │    ↓
   │    │  w25n_write() → NAND_FTL_WritePage() → NAND_Page_Write()
   │    │    ↓
   │    └─ Update cache
   │    ↓
   │  // Step 3: Update journal
   │  dhara_journal_commit(&map->journal, sector=0, page=450, &err)
   │    ├─ Write journal entry: Sector 0 → Page 450
   │    ├─ Increment epoch
   │    ├─ Mark old page (100) as stale
   │    └─ Flush journal to NAND
   │    ↓
   │  // Step 4: Check if GC needed
   │  if (write_count % gc_ratio == 0) {
   │      dhara_map_gc(&map, &err);
   │        ↓
   │      // Garbage collection (see section 7)
   │  }
   │    ↓
   └─ sem_post(&dev->lock)
   ↓
LittleFS: Write complete
   ↓
Application: Success
```

**Write Amplification**:
```
User write: 2048 bytes
Dhara writes:
  - Data page: 2048 bytes
  - Journal update: ~256 bytes (amortized)
  - GC overhead: ~512 bytes (amortized with GC_RATIO=4)
  ────────────────────────────
  Total: ~2816 bytes written

Write amplification: 2816 / 2048 = 1.375x

This is excellent! (typical NAND FTL: 2-5x)
```

### 6.3 Sector Trim Flow

**Trim Operation**:
```c
// Application hints that sector is no longer needed
dhara_map_trim(&map, sector=5, &err)
   ↓
1. Find mapping: Sector 5 → Page 200
2. Mark page 200 as stale (available for GC)
3. Remove sector 5 from map
4. Update journal
   ↓
Result: Page 200 will be reclaimed by next GC
```

**Benefit**:
```
Without trim:
  Delete file → Sectors still mapped → GC must read/copy them
  Wasted: Time + erase cycles

With trim:
  Delete file → Sectors trimmed → GC skips them
  Saved: Time + erase cycles
```

### 6.4 Sync Operation

**Journal Sync**:
```c
dhara_map_sync(&map, &err)
   ↓
1. Flush any pending journal updates to NAND
2. Write checkpoint with current epoch
3. Ensure all metadata is persistent
   ↓
Power failure after sync:
  → All writes before sync are guaranteed persistent
  → Can recover to this state
```

---

## 7. Garbage Collection

### 7.1 GC Algorithm

**Why Garbage Collection?**
```
Problem: NAND can't overwrite, must erase first

Write sector 0 → Page 100
Write sector 0 again → Page 450 (new page)
Now: Page 100 is stale (contains old data)

After many writes:
  Pages in use: 500
  Stale pages: 1000
  Free pages: 548
  Total: 2048 pages

Need to reclaim stale pages!
```

**Dhara GC Strategy**:

```c
int dhara_map_gc(struct dhara_map *m, dhara_error_t *err)
{
    // Step 1: Find block with most stale pages
    dhara_block_t victim = find_best_victim_block(m);

    // Step 2: Read all valid pages from victim block
    for (page in victim_block) {
        if (page_is_valid(page)) {
            // Page still referenced by some sector
            dhara_sector_t sector = find_sector_for_page(page);
            uint8_t data[PAGE_SIZE];

            // Read valid data
            dhara_nand_read(nand, page, 0, PAGE_SIZE, data);

            // Copy to new page
            dhara_page_t new_page = allocate_free_page();
            dhara_nand_prog(nand, new_page, data);

            // Update mapping
            update_journal(sector, new_page);
        }
        // Else: stale page, don't copy
    }

    // Step 3: Erase victim block
    dhara_nand_erase(nand, victim);

    // Step 4: Mark all pages in block as free
    mark_block_free(victim);

    return 0;
}
```

**Victim Selection**:
```
Goal: Minimize valid page copying

Scoring blocks:
  Block 0: 60 valid, 4 stale → score = 4 (poor choice)
  Block 5: 10 valid, 54 stale → score = 54 (good choice)
  Block 10: 0 valid, 64 stale → score = 64 (best choice!)

Select block with highest stale page count
```

### 7.2 GC Triggering

**Automatic GC** (interleaved with writes):
```c
// In dhara_map_write()

static uint32_t write_count = 0;

int dhara_map_write(struct dhara_map *m, dhara_sector_t s,
                    const uint8_t *data, dhara_error_t *err)
{
    // ... write data ...

    write_count++;

    // Trigger GC based on ratio
    if (write_count % m->gc_ratio == 0) {
        dhara_map_gc(m, err);
    }

    return 0;
}
```

**GC Ratio Effect**:
```
GC_RATIO=1:
  Every write triggers GC
  Very clean, but slow

GC_RATIO=4 (default):
  GC every 4th write
  Balanced

GC_RATIO=16:
  GC every 16th write
  Fast writes, but occasional pause
```

**Manual GC**:
```c
// Application can trigger GC during idle time
void idle_task(void)
{
    while (1) {
        sleep(60);  // Every minute

        // Trigger GC
        dhara_map_gc(&map, &err);
    }
}
```

### 7.3 GC Performance Impact

**GC Timing**:
```
Best case (all stale pages):
  Erase: 5 ms
  Total: 5 ms

Worst case (60 valid pages):
  Read 60 pages: 60 × 80 µs = 4.8 ms
  Write 60 pages: 60 × 450 µs = 27 ms
  Erase: 5 ms
  Total: 36.8 ms

Average case (30 valid pages):
  Read 30 pages: 2.4 ms
  Write 30 pages: 13.5 ms
  Erase: 5 ms
  Total: 20.9 ms
```

**Write Latency Distribution**:
```
GC_RATIO=4:

Write 1: 450 µs (no GC)
Write 2: 450 µs (no GC)
Write 3: 450 µs (no GC)
Write 4: 450 µs + 20 ms = 20.45 ms (with GC)
Write 5: 450 µs (no GC)
...

Average: (3 × 450 + 20450) / 4 = 5450 µs = 5.45 ms
Max: 20.45 ms
```

### 7.4 GC and Flash Lifetime

**Write Amplification**:
```
User writes 1 GB over time:
  Direct writes: 1 GB
  GC copies: 0.3 GB (30% valid data moved)
  Journal overhead: 0.05 GB (5%)
  ─────────────────────────
  Total NAND writes: 1.35 GB

Write amplification: 1.35x

Compare to no FTL:
  User must erase before every write
  Write amplification: >2x (must copy block contents)

Dhara saves flash lifetime!
```

**Erase Cycle Distribution**:
```
Without wear leveling:
  Block 0 (frequently used): 50,000 erases → WORN OUT
  Block 100 (rarely used): 100 erases → Wasted capacity

With Dhara wear leveling:
  Block 0: 10,000 erases
  Block 100: 9,500 erases
  All blocks: ~10,000 erases (evenly distributed)

Result: Use full flash lifetime capacity
```

---

## 8. Performance Characteristics

### 8.1 Throughput

**Sequential Read**:
```
LittleFS reads sectors 0-100:

  Without cache:
    100 × (journal read + data read) = 100 × 160 µs = 16 ms

  With cache:
    4 journal reads (cache) + 100 data reads = 100 × 80 µs = 8 ms

Throughput: (100 × 2048) / 8 ms = 25.6 MB/s
```

**Sequential Write** (without GC):
```
LittleFS writes sectors 0-100:
  100 × (data write + journal update)
  = 100 × (450 + 50) µs
  = 50 ms

Throughput: (100 × 2048) / 50 ms = 4.1 MB/s
```

**Sequential Write** (with GC, ratio=4):
```
100 writes:
  75 writes without GC: 75 × 500 µs = 37.5 ms
  25 writes with GC: 25 × 21 ms = 525 ms
  Total: 562.5 ms

Throughput: (100 × 2048) / 562.5 ms = 0.36 MB/s

Note: This is worst-case! Real-world is much better
because GC only copies valid pages.
```

### 8.2 Latency

**Operation Latencies**:

| Operation | Best Case | Typical | Worst Case |
|-----------|-----------|---------|------------|
| Sector Read (cached) | 80 µs | 100 µs | 200 µs |
| Sector Read (uncached) | 160 µs | 200 µs | 400 µs |
| Sector Write (no GC) | 500 µs | 600 µs | 1 ms |
| Sector Write (with GC) | 5 ms | 20 ms | 40 ms |
| Sync | 1 ms | 2 ms | 5 ms |

### 8.3 Capacity

**Usable Capacity**:
```
Total NAND: 256 MB (2048 blocks)
  - Bad blocks: 2% = 5 MB
  - Journal metadata: 256 KB
  - GC reserve (GC_RATIO=4): ~6% = 15 MB
  ───────────────────────────────────
  Usable: ~235 MB

Efficiency: 235/256 = 91.8%
```

**GC Ratio vs Capacity**:
```
GC_RATIO=1: Usable = 75% (64 MB reserved for GC)
GC_RATIO=4: Usable = 92% (20 MB reserved)
GC_RATIO=8: Usable = 96% (10 MB reserved)

Trade-off: Higher ratio = more capacity, less predictability
```

### 8.4 Comparison Summary

**Dhara vs Direct NAND**:

| Metric | Direct NAND | Dhara FTL |
|--------|-------------|-----------|
| Read Speed | 25 MB/s | 20 MB/s (cache overhead) |
| Write Speed | 4 MB/s | 2 MB/s (GC + journal) |
| Wear Leveling | Manual | Automatic |
| Bad Blocks | Manual | Transparent |
| Power-Loss Safe | No | Yes (journal) |
| Write Amplification | 1.0x | 1.35x |
| Usable Capacity | 100% | 92% |
| Lifetime | Uneven wear | 10x better |

**Verdict**: Dhara trades some performance for reliability and lifetime

---

## Summary

This part covered Dhara FTL integration in TizenRT:

**Key Takeaways**:

1. **Dhara FTL**: Open-source FTL with wear leveling and journaling
2. **Architecture**: Sector mapping layer between LittleFS and NAND
3. **Integration**: Clean callbacks to MTD NAND driver
4. **Configuration**: GC ratio and read cache tunable
5. **Garbage Collection**: Automatic reclamation of stale pages
6. **Performance**: ~92% capacity, automatic wear leveling, power-loss safe
7. **Benefits**: Extends flash lifetime 10x, transparent bad block handling

**When to Use**:
- ✓ For maximum flash lifetime (IoT devices with 5-10 year target)
- ✓ When write patterns are unpredictable
- ✓ When power-loss protection is critical
- ✗ For ultra-low latency requirements (use direct NAND)
- ✗ For read-mostly workloads (overhead not justified)

---

**Document Version**: 1.0
**Last Updated**: 2025
**Target Platform**: RTL8730E with W25N NAND Flash
**Complete Documentation Series**: Parts 1-5

**Related Documents**:
- Part 1: Overview and Architecture
- Part 2: Hardware and Configuration
- Part 3: Software Layers and Integration
- Part 4: Operations and Error Handling
- Part 5: Dhara FTL Integration (this document)
