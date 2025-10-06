# TizenRT NAND Flash - Part 6: Two-Layer Bad Block Management
## Detailed Analysis of Dhara FTL and NAND Driver Layers

---

## Table of Contents
1. [Overview of Two-Layer Bad Block Management](#overview-of-two-layer-bad-block-management)
2. [Layer 1: NAND Driver Bad Block Management](#layer-1-nand-driver-bad-block-management)
3. [Layer 2: Dhara FTL Bad Block Management](#layer-2-dhara-ftl-bad-block-management)
4. [Interaction Between Layers](#interaction-between-layers)
5. [Bad Block Statistics and Tracking](#bad-block-statistics-and-tracking)
6. [Complete Operation Flows](#complete-operation-flows)
7. [Error Handling and Recovery](#error-handling-and-recovery)
8. [Comparison and Best Practices](#comparison-and-best-practices)

---

## 1. Overview of Two-Layer Bad Block Management

### 1.1 Why Two Layers?

The TizenRT NAND flash system implements **two independent but cooperating layers** of bad block management:

```
┌─────────────────────────────────────────────────────────────┐
│                      Application                             │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│                    LittleFS Layer                            │
│  • Sees perfect block device (no bad blocks visible)        │
│  • Focuses on filesystem operations                         │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│      LAYER 2: Dhara FTL Bad Block Management                │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ Responsibilities:                                      │ │
│  │ • Logical-to-physical mapping (hide bad blocks)        │ │
│  │ • Skip bad blocks during allocation                    │ │
│  │ • Track bad block count (bb_current, bb_last)          │ │
│  │ • Capacity calculation accounting for bad blocks       │ │
│  │ • Recovery from runtime bad block failures             │ │
│  └────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────┘
                           │ Calls MTD interface
┌──────────────────────────▼──────────────────────────────────┐
│      LAYER 1: NAND Driver Bad Block Management              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │ Responsibilities:                                      │ │
│  │ • Physical bad block detection (OOB marker check)      │ │
│  │ • Bad block marking (write 0x00 to OOB bytes 0-1)      │ │
│  │ • Factory bad block identification                     │ │
│  │ • Runtime bad block detection (erase/write failures)   │ │
│  │ • Direct NAND flash access                             │ │
│  └────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│              NAND Flash Hardware (W25N)                      │
│  • OOB area stores bad block markers                        │
│  • Factory marks bad blocks during production               │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Division of Responsibilities

| Aspect | NAND Driver Layer | Dhara FTL Layer |
|--------|------------------|-----------------|
| **Detection** | Reads OOB bytes 0-1 from first page | Calls NAND driver's is_bad() |
| **Marking** | Writes 0x00 to OOB bytes 0-1 | Calls NAND driver's mark_bad() |
| **Tracking** | No tracking (stateless) | Tracks bb_current, bb_last |
| **Handling** | Returns error to caller | Skips to next block automatically |
| **Recovery** | None | Relocates data, updates journal |
| **Visibility** | Exposes bad blocks to FTL | Hides bad blocks from LittleFS |

### 1.3 Cooperation Model

```
Operation Flow Example: Write to sector fails due to bad block

1. LittleFS: write(sector=100)
       ↓
2. Dhara: dhara_map_write(sector=100)
       ├─ Allocate page 5000
       ├─ dhara_nand_prog(page=5000) → calls NAND driver
       ↓
3. NAND Driver: Write fails (P_FAIL bit set)
       ├─ Returns error to Dhara
       ↓
4. Dhara: Receives error
       ├─ Calls dhara_nand_mark_bad(block=78)
       │    ↓
       │  NAND Driver: Writes 0x00 to OOB
       │
       ├─ Increments bb_current counter
       ├─ Allocates next block (block 79)
       ├─ Retries write on new block
       ├─ Updates journal with new mapping
       ↓
5. LittleFS: Receives success
       └─ Unaware of bad block encountered

Result: Bad block handled transparently
```

---

## 2. Layer 1: NAND Driver Bad Block Management

### 2.1 MTD Driver Interface

**Location**: `os/fs/driver/mtd/w25n.c` or generic MTD layer

**Key Functions**:
```c
// Check if block is bad
int (*isbad)(FAR struct mtd_dev_s *dev, off_t block);

// Mark block as bad
int (*markbad)(FAR struct mtd_dev_s *dev, off_t block);
```

### 2.2 Bad Block Detection (is_bad)

**Implementation in Custom FTL** (`lfs_nand_ftl.c:103-124`):

```c
static u8 NF_IsBad(NAND_FTL_DeviceTypeDef *nand, u32 addr, u8 *value)
{
    Flash_InfoTypeDef *info = &nand->MemInfo;
    u8 data[2];
    u32 block_addr;
    u8 ret;

    // Step 1: Calculate block address (first page of block)
    block_addr = NF_GetBlockAddr(nand, addr);

    // For 64 pages/block:
    // Mask = ~(64-1) << 12 = 0xFFFF0000
    // Example: addr=0xA0A5678 → block_addr=0xA080000

    // Step 2: Read OOB bytes 0-1 from first page
    // PageSize = 2048, so offset 2048 accesses OOB area
    ret = NAND_Page_Read(block_addr,           // First page of block
                        info->PageSize,         // Offset 2048 (OOB start)
                        2,                      // Read 2 bytes
                        data);                  // Buffer

    // Step 3: Check bad block marker
    // Good block: [0xFF, 0xFF]
    // Bad block:  [0x00, 0x**] or [0x**, 0x00]

    if ((data[0] != NF_GOOD_BLOCK) || (data[1] != NF_GOOD_BLOCK)) {
        *value = 1;  // Bad block detected
        FS_DBG(FS_WARNING, "Bad block detected at 0x%08X", addr);
    } else {
        *value = 0;  // Good block
    }

    return ret;
}
```

**Constants**:
```c
#define NF_GOOD_BLOCK    0xFF
#define NF_BAD_BLOCK     0x00
```

**OOB Area Structure**:
```
NAND Page Layout (2048 + 64 bytes):

Offset 0-2047 (2048 bytes):
┌────────────────────────────────────────┐
│          Main Data Area                │
│      (User data / File content)        │
└────────────────────────────────────────┘

Offset 2048-2111 (64 bytes):
┌────────────────────────────────────────┐
│         Out-of-Band (OOB) Area         │
│  ┌──────────────────────────────────┐  │
│  │ Byte 0: Bad Block Marker (byte 0)│  │ ← Used for bad block detection
│  │ Byte 1: Bad Block Marker (byte 1)│  │ ← Used for bad block detection
│  ├──────────────────────────────────┤  │
│  │ Byte 2-63: Spare area            │  │ (ECC, metadata, user data)
│  └──────────────────────────────────┘  │
└────────────────────────────────────────┘

Bad Block Marker Values:
  Good Block:   [0xFF, 0xFF] - Both bytes are 0xFF
  Bad Block:    [0x00, any]  - Byte 0 is 0x00
             OR [any, 0x00]  - Byte 1 is 0x00
```

**Detection Algorithm**:
```c
// Pseudocode
function is_block_bad(block_number):
    first_page = block_number * 64  // 64 pages per block

    // Read OOB bytes 0-1 from first page
    oob_data = read_oob(first_page, offset=0, length=2)

    if (oob_data[0] == 0xFF AND oob_data[1] == 0xFF):
        return FALSE  // Good block
    else:
        return TRUE   // Bad block
```

**Example Detection Scenarios**:

```
Scenario 1: Factory Bad Block
  Block 15, First page OOB: [0x00, 0x00]
  → is_bad() returns TRUE
  → Block marked bad during manufacturing

Scenario 2: Runtime Bad Block (just failed)
  Block 42, First page OOB: [0x00, 0xFF]
  → is_bad() returns TRUE
  → Block failed during use, recently marked

Scenario 3: Good Block
  Block 100, First page OOB: [0xFF, 0xFF]
  → is_bad() returns FALSE
  → Block is usable
```

### 2.3 Bad Block Marking (mark_bad)

**Implementation** (`lfs_nand_ftl.c:134-158`):

```c
static u8 NF_MarkBad(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    Flash_InfoTypeDef *info = &nand->MemInfo;
    u8 data[2] = {NF_BAD_BLOCK, NF_BAD_BLOCK};  // [0x00, 0x00]
    u32 block_addr;
    u8 ret;

    // Step 1: Get block address (first page)
    block_addr = NF_GetBlockAddr(nand, addr);

    // Step 2: Write bad block marker to OOB bytes 0-1
    ret = NAND_Page_Write(block_addr,      // First page of block
                         info->PageSize,    // Offset 2048 (OOB start)
                         2,                 // Write 2 bytes
                         data);             // [0x00, 0x00]

    if (ret == 0) {
        FS_DBG(FS_WARNING, "Block 0x%08X marked as bad", addr);
        ret = HAL_OK;
    } else {
        FS_DBG(FS_ERROR, "Failed to mark block 0x%08X as bad", addr);
        ret = HAL_ERR_HW;
    }

    return ret;
}
```

**Marking Process**:
```
1. Before Marking:
   Block 42, Page 0 OOB: [0xFF, 0xFF, 0xFF, ...]
   Block status: Good

2. Erase/Write fails on Block 42

3. Mark Bad Operation:
   Write [0x00, 0x00] to OOB bytes 0-1 of Page 0

4. After Marking:
   Block 42, Page 0 OOB: [0x00, 0x00, 0xFF, ...]
   Block status: Bad (permanently)

5. Future Checks:
   NF_IsBad(block 42) → returns TRUE
   Block will be skipped by all layers
```

**Important Notes**:
- **Permanent**: Once marked, block is bad forever (OOB can't be erased)
- **First Page Only**: Only first page of block needs marker
- **Two Bytes**: Both bytes 0-1 set to 0x00 for redundancy
- **No Erase Needed**: Can write to OOB even if page has data

### 2.4 Runtime Bad Block Detection

**Trigger Points**:

1. **Erase Failure** (`lfs_spinand.c:332-353`):
```c
u8 NAND_Erase(u32 PageAddr)
{
    // Send WRITE ENABLE
    NAND_WriteEn();

    // Send BLOCK ERASE command
    NAND_TxCmd(FLASH_cmd_block_e, 3, Addr, 0, NULL);

    // Wait for completion
    status = NAND_WaitBusy(WAIT_FLASH_BUSY);

    // Check E_FAIL bit (bit 2)
    if (status & flash_init_para.FLASH_EFail_bit) {
        // Erase failed!
        return status;  // Non-zero = error
    }

    return 0;  // Success
}
```

2. **Write Failure** (`lfs_spinand.c:417-436`):
```c
u8 NAND_Page_Write_Program_Execute(u32 PageAddr)
{
    // Send PROGRAM EXECUTE command
    NAND_TxCmd(FLASH_cmd_page_write, 3, Addr, 0, NULL);

    // Wait for programming
    status = NAND_WaitBusy(WAIT_FLASH_BUSY);

    // Check P_FAIL bit (bit 3)
    if (status & flash_init_para.FLASH_PFail_bit) {
        // Program failed!
        return status;  // Non-zero = error
    }

    return 0;  // Success
}
```

**Status Register Bits** (W25N NAND):
```
Bit Position  Name     Description
────────────  ───────  ─────────────────────────────
[0]           OIP      Operation In Progress (1=busy)
[1]           WEL      Write Enable Latch
[2]           E_FAIL   Erase Failure (1=failed)
[3]           P_FAIL   Program Failure (1=failed)
[5:4]         ECCS     ECC Status
[6]           ECCS1    Additional ECC bit
[7]           Reserved
```

**Failure Handling** (`lfs_nand_ftl.c:167-185`):
```c
static u8 NF_EraseBlock(NAND_FTL_DeviceTypeDef *nand, u32 addr)
{
    u8 ret;

    // Attempt erase
    ret = NAND_Erase(addr);

    if (ret == 0) {
        // Erase succeeded
        return HAL_OK;
    } else if (ret == 0xFF) {
        // Timeout
        return HAL_TIMEOUT;
    } else {
        // Erase failed - mark block as bad
        FS_DBG(FS_ERROR, "Block erase failed at 0x%08X: 0x%02X", addr, ret);

        ret = NF_MarkBad(nand, addr);

        if (ret == HAL_OK) {
            // Successfully marked bad
            return UERR_NAND_WORN_BLOCK;  // Signal runtime bad block
        } else {
            // Couldn't even mark it bad!
            return HAL_ERR_HW;
        }
    }
}
```

---

## 3. Layer 2: Dhara FTL Bad Block Management

### 3.1 Dhara Interface to NAND Driver

**Callback Functions** (`dhara.c:490-500`):

```c
// Check if block is bad (calls MTD driver)
int dhara_nand_is_bad(FAR const struct dhara_nand *n, dhara_block_t bno)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;

    // Direct passthrough to MTD driver
    return MTD_ISBAD(dev->mtd, bno);

    // Returns:
    //   0 = good block
    //   1 = bad block
}

// Mark block as bad (calls MTD driver)
void dhara_nand_mark_bad(FAR const struct dhara_nand *n, dhara_block_t bno)
{
    FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;

    // Direct passthrough to MTD driver
    MTD_MARKBAD(dev->mtd, bno);

    // No return value - best effort
}
```

**Key Point**: Dhara doesn't implement bad block detection/marking itself. It delegates to the NAND driver layer.

### 3.2 Bad Block Tracking

**Data Structures** (`dhara/journal.h:105-111`):

```c
struct dhara_journal {
    // ... other fields ...

    /* Bad-block counters */
    dhara_block_t bb_current;  // Bad blocks before current head
    dhara_block_t bb_last;     // Estimated total bad blocks in chip

    // ...
};
```

**Counter Meanings**:

```
Visual Representation of bb_current and bb_last:

NAND Flash (2048 blocks):
┌─────────────────────────────────────────────────────────┐
│ Block 0     [Good]                                      │
│ Block 1     [Good]                                      │
│ Block 2     [BAD]  ← Factory bad                        │
│ ...                                                     │
│ Block 100   [Good]                                      │
│ Block 101   [BAD]  ← Runtime bad                        │
│ ...                                                     │
│ Block 500   [Good] ← Current head position              │
│             ▲                                           │
│             └─ bb_current = 2 (bad blocks before here) │
│ ...                                                     │
│ Block 1000  [BAD]  ← Factory bad (not counted yet)     │
│ ...                                                     │
│ Block 2047  [Good]                                      │
└─────────────────────────────────────────────────────────┘
                      ▲
                      └─ bb_last = 3 (estimated total from last epoch)

bb_current: Counts bad blocks from block 0 to current head
bb_last:    Best estimate of total bad blocks in entire chip
```

**Counter Updates**:

1. **bb_current Increment** - When bad block encountered during allocation:
```c
// In journal.c:629, 689, 711
if (dhara_nand_is_bad(j->nand, block)) {
    j->bb_current++;  // Found a bad block
    skip_to_next_block();
}
```

2. **bb_last Update** - When epoch rolls over (head wraps around):
```c
// In journal.c:211-216
static void roll_stats(struct dhara_journal *j)
{
    j->bb_last = j->bb_current;  // Save current count as estimate
    j->bb_current = 0;            // Reset for new epoch
    j->epoch++;                   // Increment epoch
}
```

3. **Initialization** - Conservative estimate at startup:
```c
// In journal.c:193-194
j->bb_last = j->nand->num_blocks >> 6;  // Estimate: 1.6% bad blocks
                                        // 2048 >> 6 = 32 blocks
j->bb_current = 0;
```

### 3.3 Bad Block Avoidance During Allocation

**Block Skipping Logic** (`dhara/journal.c:527-535`):

```c
// When allocating next block for journal tail
if (is_aligned(j->tail, j->nand->log2_ppb)) {
    dhara_block_t blk = j->tail >> j->nand->log2_ppb;
    int i;

    // Retry up to DHARA_MAX_RETRIES times (8)
    for (i = 0; i < DHARA_MAX_RETRIES; i++) {
        // Skip if block is bad
        if ((blk == (j->head >> j->nand->log2_ppb)) ||
            !dhara_nand_is_bad(j->nand, blk)) {

            // Found good block (or reached head)
            j->tail = blk << j->nand->log2_ppb;

            if (j->tail == j->head) {
                j->root = DHARA_PAGE_NONE;  // Journal full
            }

            return 0;
        }

        // Block is bad, try next
        blk++;
    }

    // Tried 8 blocks, all bad!
    dhara_set_error(err, DHARA_E_TOO_BAD);
    return -1;
}
```

**Erase with Bad Block Handling** (`dhara/journal.c:620-633`):

```c
int journal_enqueue(struct dhara_journal *j, dhara_error_t *err)
{
    // Try to erase block at head
    for (int i = 0; i < DHARA_MAX_RETRIES; i++) {
        dhara_block_t blk = j->head >> j->nand->log2_ppb;

        // Check if block is bad
        if (!dhara_nand_is_bad(j->nand, blk)) {
            // Block is good, erase it
            return dhara_nand_erase(j->nand, blk, err);
        }

        // Block is bad, skip it
        j->bb_current++;  // Increment bad block counter

        if (skip_block(j, err) < 0) {
            return -1;  // Can't skip (journal full)
        }
    }

    // All 8 attempts failed
    dhara_set_error(err, DHARA_E_TOO_BAD);
    return -1;
}
```

**Maximum Retries**:
```c
#define DHARA_MAX_RETRIES    8

// If 8 consecutive blocks are bad:
//   → DHARA_E_TOO_BAD error
//   → Extremely rare (probability ~10^-24 for random distribution)
```

### 3.4 Runtime Bad Block Recovery

**Scenario**: Write fails during journal operation

**Recovery Flow** (`dhara/journal.c:680-695`):

```c
// Attempt to write metadata page
int write_checkpoint(struct dhara_journal *j)
{
    dhara_error_t my_err = DHARA_E_NONE;

    // Try to write checkpoint page
    if (dhara_nand_prog(j->nand, j->head, j->page_buf, &my_err) < 0) {
        // Write failed!

        if (my_err != DHARA_E_BAD_BLOCK) {
            // Other error (not bad block)
            dhara_set_error(err, my_err);
            return -1;
        }

        // Bad block encountered during write
        j->bb_current++;  // Increment counter

        // Mark the block as bad
        dhara_nand_mark_bad(j->nand, j->head >> j->nand->log2_ppb);

        // Skip to next block
        if (skip_block(j, err) < 0) {
            return -1;  // Can't skip
        }

        // Retry write on next block (handled by caller)
    }

    return 0;
}
```

**Recovery from Block Failure During Recovery** (`dhara/journal.c:636-650`):

```c
// If block fails while recovering from previous failure
int recover_from(struct dhara_journal *j, dhara_error_t cause,
                dhara_error_t *err)
{
    dhara_page_t old_head = j->head;

    // Mark the current head bad immediately (unless it holds metadata)
    if ((j->recover_meta == DHARA_PAGE_NONE) ||
        !align_eq(j->recover_meta, old_head, j->nand->log2_ppb)) {

        // Safe to mark bad now
        dhara_nand_mark_bad(j->nand, old_head >> j->nand->log2_ppb);
    } else {
        // Defer marking until recovery complete
        j->flags |= DHARA_JOURNAL_F_BAD_META;
    }

    // Start recovery process
    // ... (copy data from failed block to new block)

    return 0;
}
```

### 3.5 Capacity Calculation with Bad Blocks

**Capacity Formula** (`dhara/journal.c:460-468`):

```c
dhara_page_t dhara_journal_capacity(const struct dhara_journal *j)
{
    // Use worst-case estimate of bad blocks
    const dhara_block_t max_bad = (j->bb_last > j->bb_current) ?
                                   j->bb_last : j->bb_current;

    // Calculate good blocks
    const dhara_block_t good_blocks = j->nand->num_blocks - max_bad - 1;
                                      // -1 for checkpoint overhead

    // Calculate checkpoint periods
    const int log2_cpb = j->nand->log2_ppb - j->log2_ppc;
    const dhara_page_t good_cps = good_blocks << log2_cpb;

    // Capacity = checkpoints × (checkpoint_size - 1)
    // (-1 because last page of checkpoint is metadata)
    return good_cps * ((1 << j->log2_ppc) - 1);
}
```

**Example Calculation**:
```
NAND: 2048 blocks, 64 pages/block
Checkpoint period: 64 pages (log2_ppc = 6)
Bad blocks: bb_last = 40

Step 1: Good blocks
  = 2048 - 40 - 1 = 2007 blocks

Step 2: Checkpoints per block
  = 64 pages / 64 checkpoint_period = 1 checkpoint/block

Step 3: Total checkpoints
  = 2007 checkpoints

Step 4: User pages per checkpoint
  = 64 - 1 = 63 pages (1 page for metadata)

Step 5: Total capacity
  = 2007 × 63 = 126,441 pages
  = 126,441 × 2048 bytes
  = 258,951,168 bytes
  = ~247 MB

Compare to:
  Without bad blocks: 2048 × 63 = 129,024 pages = ~252 MB
  Loss: ~5 MB (2% capacity loss for bad blocks)
```

---

## 4. Interaction Between Layers

### 4.1 Call Sequence for Bad Block Check

**Complete Call Stack**:

```
Application: write(fd, data, 2048)
   ↓
LittleFS: lfs_file_write()
   ↓
Dhara: dhara_write(sector=100)
   ↓
Dhara Map: dhara_map_write(sector=100, data)
   ├─ Allocate new page
   ├─ dhara_journal_enqueue() → Need to allocate block
   │    ↓
   │  Check if block is bad:
   │    ├─ dhara_nand_is_bad(nand, block=78)
   │    │    ↓
   │    │  [LAYER 2 → LAYER 1 call]
   │    │    ↓
   │    │  dhara.c: dhara_nand_is_bad()
   │    │    ├─ MTD_ISBAD(mtd, block=78)
   │    │    │    ↓
   │    │    │  MTD: w25n_isbad() or custom FTL
   │    │    │    ├─ NF_IsBad(nand, block_addr)
   │    │    │    │    ↓
   │    │    │    │  NAND_Page_Read(first_page, offset=2048, len=2)
   │    │    │    │    ↓
   │    │    │    │  SPI NAND Driver: Read OOB
   │    │    │    │    ↓
   │    │    │    │  Hardware: Read [0xFF, 0xFF] or [0x00, 0x**]
   │    │    │    │    ↓
   │    │    │    └─ Return: 0 (good) or 1 (bad)
   │    │    │    ↓
   │    │    └─ Return to Dhara
   │    │    ↓
   │    └─ If bad: skip block, try next
   │    └─ If good: erase and use
   │
   └─ Write data to allocated page

Total function calls: 8-10 layers deep
Time: ~50 µs (if cached) or ~200 µs (if OOB read needed)
```

### 4.2 Call Sequence for Bad Block Marking

**Complete Call Stack**:

```
Dhara: Write fails with E_FAIL
   ↓
Dhara Journal: recover_from()
   ├─ Need to mark block bad
   ├─ dhara_nand_mark_bad(nand, block=78)
   │    ↓
   │  [LAYER 2 → LAYER 1 call]
   │    ↓
   │  dhara.c: dhara_nand_mark_bad()
   │    ├─ MTD_MARKBAD(mtd, block=78)
   │    │    ↓
   │    │  MTD: w25n_markbad() or custom FTL
   │    │    ├─ NF_MarkBad(nand, block_addr)
   │    │    │    ↓
   │    │    │  NAND_Page_Write(first_page, offset=2048, len=2, data=[0x00, 0x00])
   │    │    │    ↓
   │    │    │  SPI NAND Driver: Write OOB
   │    │    │    ↓
   │    │    │  Hardware: Write [0x00, 0x00] to OOB bytes 0-1
   │    │    │    ↓
   │    │    └─ Return (success or failure)
   │    │    ↓
   │    └─ Return to Dhara
   │
   ├─ Increment bb_current
   ├─ Skip to next block
   └─ Retry operation

Time: ~500 µs (page write to OOB)
```

### 4.3 Information Flow

```
Direction: Upward (NAND → Dhara)
────────────────────────────────
• Bad block status (good/bad)
• Operation results (success/fail)
• ECC status (optional)

Direction: Downward (Dhara → NAND)
──────────────────────────────────
• Bad block checks (is_bad queries)
• Bad block marking (mark_bad commands)
• Read/write/erase requests

Direction: Horizontal (within layer)
────────────────────────────────────
Layer 1 (NAND):
  • OOB access for markers
  • Status register checks

Layer 2 (Dhara):
  • bb_current/bb_last tracking
  • Journal updates
  • Capacity calculations
```

### 4.4 Synchronization Points

**Journal Checkpoint** - Bad block counts saved to NAND:

```c
// When writing checkpoint (journal.c:783-789)
hdr_put_magic(j->page_buf);
hdr_set_epoch(j->page_buf, j->epoch);
hdr_set_tail(j->page_buf, j->tail);
hdr_set_bb_current(j->page_buf, j->bb_current);  // ← Saved to NAND
hdr_set_bb_last(j->page_buf, j->bb_last);        // ← Saved to NAND

dhara_nand_prog(j->nand, j->head + 1, j->page_buf, &err);
```

**Journal Resume** - Bad block counts restored from NAND:

```c
// When resuming from NAND (journal.c:439-440)
j->bb_current = hdr_get_bb_current(j->page_buf);  // ← Restored
j->bb_last = hdr_get_bb_last(j->page_buf);        // ← Restored
```

**Checkpoint Header Format**:
```
Offset  Size  Field         Description
──────  ────  ────────────  ─────────────────────────────
0-2     3     Magic         "Dha" (0x44 0x68 0x61)
3       1     Epoch         Current epoch number
4-7     4     Tail          Tail pointer
8-11    4     bb_current    Bad blocks before head
12-15   4     bb_last       Estimated total bad blocks
16+     ...   User data     Application metadata
```

---

## 5. Bad Block Statistics and Tracking

### 5.1 Statistics Collection

**Where Statistics are Kept**:

| Statistic | Location | Persistence | Purpose |
|-----------|----------|-------------|---------|
| **bb_current** | dhara_journal.bb_current | Checkpointed to NAND | Count bad blocks before head |
| **bb_last** | dhara_journal.bb_last | Checkpointed to NAND | Estimate total bad blocks |
| **Factory bad** | OOB markers in NAND | Permanent | Physical marking |
| **Runtime bad** | OOB markers + counters | Permanent + checkpointed | Tracking wear-out |

**Example Statistics Evolution**:

```
Time T0 (Fresh NAND):
  bb_current = 0
  bb_last = 32 (conservative estimate: 1.6% of 2048)
  Factory bad blocks: ~20 (actual, discovered during use)

Time T1 (After 1st epoch, head wrapped around):
  bb_current = 0 (reset)
  bb_last = 20 (updated with actual count from epoch 0)
  Factory bad: 20
  Runtime bad: 0

Time T2 (During 2nd epoch, some blocks failed):
  bb_current = 25 (20 factory + 5 runtime encountered so far)
  bb_last = 20
  Factory bad: 20
  Runtime bad: 5

Time T3 (After 2nd epoch):
  bb_current = 0 (reset)
  bb_last = 25 (updated with epoch 1 count)
  Factory bad: 20
  Runtime bad: 5

Time T4 (During 3rd epoch):
  bb_current = 27 (encountered in current epoch)
  bb_last = 25
  Factory bad: 20
  Runtime bad: 7 (2 more failed)
```

### 5.2 Capacity Impact Calculation

**Formula with Dynamic Bad Block Count**:

```c
// Real-time capacity calculation
dhara_page_t current_capacity(struct dhara_journal *j)
{
    // Use maximum of current and last epoch count
    dhara_block_t estimated_bad = MAX(j->bb_current, j->bb_last);

    // Subtract from total
    dhara_block_t usable = j->nand->num_blocks - estimated_bad - 1;

    // Calculate user pages
    dhara_page_t pages = usable * (pages_per_checkpoint - 1);

    return pages;
}
```

**Capacity Over Time**:

```
Initial (T0):
  Blocks: 2048
  Estimated bad: 32 (conservative)
  Usable: 2048 - 32 - 1 = 2015
  Capacity: 2015 × 63 = 126,945 pages ≈ 248 MB

After discovery (T1):
  Blocks: 2048
  Actual bad: 20
  Usable: 2048 - 20 - 1 = 2027
  Capacity: 2027 × 63 = 127,701 pages ≈ 249 MB
  Gain: +1 MB (better than estimated!)

After wear (T3):
  Blocks: 2048
  Total bad: 25
  Usable: 2048 - 25 - 1 = 2022
  Capacity: 2022 × 63 = 127,386 pages ≈ 249 MB
  Loss: 0.3 MB (5 blocks worn out)

After heavy wear (T10):
  Blocks: 2048
  Total bad: 100
  Usable: 2048 - 100 - 1 = 1947
  Capacity: 1947 × 63 = 122,661 pages ≈ 240 MB
  Loss: 9 MB (80 blocks worn out)
```

### 5.3 Bad Block Distribution Patterns

**Random Distribution** (typical):
```
Block numbers with bad blocks:
  15, 42, 89, 157, 203, 298, 401, 523, ...

  Spread evenly across chip
  Probability: ~1-2% factory, increases with wear
```

**Clustered Distribution** (rare, manufacturing defect):
```
Block numbers with bad blocks:
  100, 101, 102, 103, 104, 105, 106, 107

  Multiple consecutive bad blocks
  Can trigger DHARA_E_TOO_BAD if 8+ consecutive
```

**Handling Clusters**:
```c
// DHARA_MAX_RETRIES = 8

if (consecutive_bad_blocks >= 8):
    return DHARA_E_TOO_BAD
    → Fatal error
    → Indicates defective NAND chip

Probability of 8 consecutive bad blocks:
  With 2% bad block rate: (0.02)^8 ≈ 2.56 × 10^-14
  With 5% bad block rate: (0.05)^8 ≈ 3.9 × 10^-11

  → Extremely unlikely for random distribution
  → If it occurs, chip is likely defective
```

---

## 6. Complete Operation Flows

### 6.1 Read from Good Block

```
Application: read(fd, buf, 2048)
   ↓
LittleFS: lfs_file_read(file, buf, 2048)
   ↓
Dhara: dhara_read(sector=50)
   ├─ dhara_map_find(sector=50) → page=5000
   ├─ Block check? NO (read doesn't check bad blocks)
   │  (Assume page 5000 was written successfully, so block is good)
   ├─ dhara_nand_read(page=5000, buf)
   │    ↓
   │  MTD_READ(offset=5000×2048, len=2048, buf)
   │    ↓
   │  NAND_Page_Read() → Success
   │    ↓
   └─ Return data to LittleFS
   ↓
Application: Receives 2048 bytes

Time: ~80 µs (no bad block check needed)
```

**Why no bad block check on read?**
- If page exists in map, block must be good (was written successfully)
- Dhara never maps sectors to bad blocks
- Optimization: Saves 50-200 µs per read

### 6.2 Write to Good Block (First Time)

```
Application: write(fd, buf, 2048)
   ↓
LittleFS: lfs_file_write(file, buf, 2048)
   ↓
Dhara: dhara_write(sector=100)
   ├─ dhara_map_write(sector=100, buf)
   ├─ Need new page
   ├─ dhara_journal_enqueue()
   │    ↓
   │  Step 1: Check if need new block
   │    if (head % 64 == 0):  // Block boundary
   │      Need to erase new block
   │    ↓
   │  Step 2: Check if block is bad
   │    block_num = head >> 6  // e.g., block 78
   │    is_bad = dhara_nand_is_bad(nand, 78)
   │      ↓
   │    MTD_ISBAD(78)
   │      ↓
   │    NF_IsBad() → Read OOB → [0xFF, 0xFF]
   │      ↓
   │    Return: 0 (good block)
   │    ↓
   │  Step 3: Erase block (it's good)
   │    dhara_nand_erase(nand, 78)
   │      ↓
   │    NAND_Erase() → Success
   │    ↓
   │  Step 4: Advance head to first page of block
   │    head = 78 << 6 = 4992
   │    ↓
   ├─ Write data to page 4992
   ├─ dhara_nand_prog(page=4992, buf)
   │    ↓
   │  NAND_Page_Write() → Success
   │    ↓
   ├─ Update map: sector 100 → page 4992
   ├─ Update journal checkpoint
   └─ Return success
   ↓
Application: write() returns 2048

Time: ~5500 µs (erase + write + bad block check)
```

### 6.3 Write to Bad Block (Skip to Next)

```
Application: write(fd, buf, 2048)
   ↓
Dhara: dhara_write(sector=101)
   ├─ Need new page
   ├─ dhara_journal_enqueue()
   │    ↓
   │  Step 1: At block boundary
   │    head = 5056 (block 79)
   │    ↓
   │  Step 2: Check if block 79 is bad
   │    is_bad = dhara_nand_is_bad(nand, 79)
   │      ↓
   │    NF_IsBad() → Read OOB → [0x00, 0xFF]  ← Bad!
   │      ↓
   │    Return: 1 (bad block)
   │    ↓
   │  Step 3: Bad block detected
   │    bb_current++  // Increment counter: 26 → 27
   │    ↓
   │  Step 4: Skip to next block
   │    skip_block()
   │      head = 5120 (block 80)
   │    ↓
   │  Step 5: Check if block 80 is bad
   │    is_bad = dhara_nand_is_bad(nand, 80)
   │      ↓
   │    NF_IsBad() → Read OOB → [0xFF, 0xFF]  ← Good!
   │      ↓
   │    Return: 0 (good block)
   │    ↓
   │  Step 6: Erase block 80
   │    dhara_nand_erase(nand, 80) → Success
   │    ↓
   ├─ Write data to page 5120
   ├─ Update map: sector 101 → page 5120
   └─ Return success
   ↓
Application: write() returns 2048

Time: ~5700 µs (2 bad block checks + erase + write)
Note: Slightly slower due to extra bad block check
```

### 6.4 Erase Fails → Mark Bad → Retry

```
Dhara: Writing sector 200
   ├─ Need new block
   ├─ Check block 100: good
   ├─ Erase block 100
   │    ↓
   │  dhara_nand_erase(nand, 100)
   │    ↓
   │  NAND_Erase(block 100)
   │    ↓
   │  Hardware: E_FAIL bit = 1  ← Erase failed!
   │    ↓
   │  Return: DHARA_E_BAD_BLOCK
   │    ↓
   ├─ Erase failed! Recover:
   ├─ bb_current++  // Increment: 27 → 28
   ├─ dhara_nand_mark_bad(nand, 100)
   │    ↓
   │  MTD_MARKBAD(100)
   │    ↓
   │  NF_MarkBad() → Write [0x00, 0x00] to OOB
   │    ↓
   │  Success: Block 100 now marked bad permanently
   │    ↓
   ├─ Skip to block 101
   ├─ Check block 101: good
   ├─ Erase block 101: success
   ├─ Write data to block 101
   └─ Return success
   ↓
Application: Unaware of failure, receives success

Time: ~11000 µs (failed erase + mark bad + retry)
```

### 6.5 Write Fails → Mark Bad → Retry

```
Dhara: Writing journal checkpoint
   ├─ Write to page 6400 (block 100)
   │    ↓
   │  dhara_nand_prog(page=6400, data)
   │    ↓
   │  NAND_Page_Write()
   │    ↓
   │  Hardware: P_FAIL bit = 1  ← Write failed!
   │    ↓
   │  Return: DHARA_E_BAD_BLOCK
   │    ↓
   ├─ Write failed! Recover:
   ├─ bb_current++  // Increment: 28 → 29
   ├─ dhara_nand_mark_bad(nand, 100)
   │    ↓
   │  Block 100 marked bad
   │    ↓
   ├─ Skip to block 101
   ├─ Write to page 6464 (block 101): success
   └─ Return success

Time: ~1000 µs (failed write + mark bad + retry)
```

---

## 7. Error Handling and Recovery

### 7.1 Error Code Translation

**Error Flow Across Layers**:

```
Hardware Status     NAND Driver          Dhara FTL            LittleFS
────────────────    ────────────────    ────────────────    ────────────
E_FAIL bit = 1  →   UERR_NAND_WORN_  →  DHARA_E_BAD_     →  LFS_ERR_
                    BLOCK                BLOCK                CORRUPT

P_FAIL bit = 1  →   UERR_NAND_WORN_  →  DHARA_E_BAD_     →  LFS_ERR_
                    BLOCK                BLOCK                CORRUPT

OOB = [0x00,.] →    is_bad() = 1     →  Skip block       →  (transparent)

Timeout         →   HAL_TIMEOUT      →  DHARA_E_NAND_    →  LFS_ERR_IO
                                        (error passthrough)
```

**Dhara Error Codes** (`dhara/error.h`):

```c
typedef enum {
    DHARA_E_NONE = 0,           // No error
    DHARA_E_BAD_BLOCK,          // Bad block encountered
    DHARA_E_ECC,                // ECC error
    DHARA_E_TOO_BAD,            // Too many consecutive bad blocks
    DHARA_E_RECOVER,            // Recovery operation in progress
    DHARA_E_CORRUPT_MAP,        // Map corruption detected
    DHARA_E_NOT_FOUND,          // Sector not found
    DHARA_E_MAP_FULL,           // Map capacity exhausted
    DHARA_E_JOURNAL_FULL,       // Journal capacity exhausted
} dhara_error_t;
```

**Error Translation** (`dhara.c:132-153`):

```c
static int dhara_convert_result(dhara_error_t err)
{
    switch (err) {
        case DHARA_E_NONE:
            return 0;  // Success

        case DHARA_E_BAD_BLOCK:
        case DHARA_E_ECC:
        case DHARA_E_TOO_BAD:
        case DHARA_E_CORRUPT_MAP:
            return -EBADMSG;  // Bad message (data corruption)

        case DHARA_E_JOURNAL_FULL:
        case DHARA_E_MAP_FULL:
            return -ENOSPC;  // No space

        case DHARA_E_NOT_FOUND:
            return -ENOENT;  // No entry

        default:
            return -EIO;  // I/O error
    }
}
```

### 7.2 Recovery Strategies

**Strategy 1: Skip Bad Block**
```c
// Used when allocating new blocks
Result: Transparent to upper layers
Cost: Capacity reduction (1 block = 128 KB)
Time: ~50 µs (bad block check)
```

**Strategy 2: Relocate Data**
```c
// Used when block fails mid-operation
Process:
  1. Mark failed block as bad
  2. Allocate new block
  3. Copy valid data to new block
  4. Update journal mappings
  5. Continue operation

Result: Transparent to upper layers
Cost: Time (~20 ms for relocation)
```

**Strategy 3: Recovery Mode**
```c
// Used when failure occurs during checkpoint write
Process:
  1. Mark failed block as bad
  2. Dump buffered metadata to temporary page
  3. Skip to new block
  4. Reconstruct journal from temporary metadata
  5. Mark temporary block bad if it also failed

Result: Power-loss safe recovery
Cost: Time (~50 ms worst case)
```

### 7.3 Edge Cases

**Case 1: Too Many Consecutive Bad Blocks**
```
Scenario: Encounter 8 consecutive bad blocks

Result: DHARA_E_TOO_BAD error
Action:
  - Report to LittleFS as ENOSPC
  - LittleFS tries different area
  - If persists, filesystem full

Likelihood: Extremely rare (~10^-24 for random distribution)
Indication: Defective NAND chip
```

**Case 2: Bad Block in Journal Root**
```
Scenario: Journal root block fails

Recovery:
  1. Save current journal state
  2. Mark root block bad
  3. Find new root block
  4. Restore journal state
  5. Continue operation

Result: Successful recovery
Time: ~100 ms
```

**Case 3: Multiple Failures During Recovery**
```
Scenario: Block fails while recovering from previous failure

Recovery:
  1. Mark first failed block bad
  2. Begin recovery
  3. Second block fails
  4. Mark second block bad (deferred if holding metadata)
  5. Continue recovery with third block

Result: Multi-level recovery successful
Limit: Up to DHARA_MAX_RETRIES (8) failures
```

### 7.4 Unrecoverable Scenarios

**Scenario 1: All Good Blocks Exhausted**
```
Trigger: Total bad blocks ≥ num_blocks - 1

Symptom: DHARA_E_JOURNAL_FULL on all operations

Action:
  - Filesystem becomes read-only
  - No new writes possible
  - Data can still be read

Recovery: None (replace NAND chip)
```

**Scenario 2: Catastrophic Chip Failure**
```
Trigger: Large area of chip fails simultaneously

Symptom:
  - Multiple DHARA_E_TOO_BAD errors
  - Rapid increase in bb_current
  - Many consecutive failures

Action:
  - Emergency checkpoint
  - Mark filesystem as degraded
  - Attempt to salvage critical data

Recovery: Replace NAND chip, restore from backup
```

---

## 8. Comparison and Best Practices

### 8.1 Layer Comparison

| Aspect | NAND Driver Layer | Dhara FTL Layer |
|--------|------------------|-----------------|
| **Complexity** | Simple, direct | Complex, stateful |
| **State** | Stateless | Stateful (counters, journal) |
| **Visibility** | Physical blocks | Logical sectors |
| **Detection Speed** | 50-200 µs | Cached, ~1 µs |
| **Marking Speed** | 500 µs | Same (delegates to driver) |
| **Recovery** | None (just reports) | Full recovery with retry |
| **Persistence** | OOB markers (permanent) | Checkpoints (durable) |
| **Overhead** | None | ~2-5% for metadata |

### 8.2 Best Practices

**For NAND Driver Layer**:

1. **Always check OOB bytes 0 and 1**
   ```c
   // Both bytes must be 0xFF for good block
   if ((oob[0] == 0xFF) && (oob[1] == 0xFF))
       return GOOD_BLOCK;
   ```

2. **Mark bad immediately on failure**
   ```c
   if (erase_failed || write_failed) {
       mark_bad(block);
       return ERROR;
   }
   ```

3. **Use first page of block for marker**
   ```c
   // Always use page 0 of block for bad block marker
   page_addr = block_num << log2_pages_per_block;
   ```

4. **Never try to erase bad blocks**
   ```c
   if (is_bad(block)) {
       return ERROR;  // Don't attempt erase
   }
   ```

**For Dhara FTL Layer**:

1. **Check bad blocks before allocation**
   ```c
   while (is_bad(block)) {
       bb_current++;
       block++;
   }
   ```

2. **Update counters immediately**
   ```c
   if (block_failed) {
       bb_current++;
       mark_bad(block);
   }
   ```

3. **Checkpoint counters regularly**
   ```c
   // Save bb_current and bb_last to NAND
   write_checkpoint();
   ```

4. **Use conservative estimates**
   ```c
   // Initial estimate: 1.6% bad blocks
   bb_last = num_blocks >> 6;
   ```

**For Application Layer**:

1. **Handle ENOSPC gracefully**
   ```c
   if (write() returns ENOSPC) {
       // Cleanup old files
       // Retry write
   }
   ```

2. **Don't assume fixed capacity**
   ```c
   // Query capacity dynamically
   statvfs("/mnt", &stat);
   capacity = stat.f_blocks * stat.f_bsize;
   ```

3. **Monitor bad block growth**
   ```c
   // Periodic health check
   if (bad_blocks > threshold) {
       plan_replacement();
   }
   ```

### 8.3 Monitoring and Diagnostics

**Recommended Monitoring**:

```c
// Periodic health check (run daily/weekly)
void check_nand_health(void)
{
    // Get Dhara statistics
    dhara_journal_t *j = get_journal();

    uint32_t total_bad = MAX(j->bb_current, j->bb_last);
    uint32_t total_blocks = j->nand->num_blocks;
    float bad_percent = (total_bad * 100.0) / total_blocks;

    printf("NAND Health:\n");
    printf("  Total blocks: %u\n", total_blocks);
    printf("  Bad blocks: %u (%.1f%%)\n", total_bad, bad_percent);
    printf("  Current epoch: %u\n", j->epoch);
    printf("  bb_current: %u\n", j->bb_current);
    printf("  bb_last: %u\n", j->bb_last);

    // Alert if bad block rate exceeds threshold
    if (bad_percent > 5.0) {
        printf("WARNING: High bad block rate!\n");
    }

    if (bad_percent > 10.0) {
        printf("CRITICAL: Consider replacing NAND!\n");
    }
}
```

**Log Bad Block Events**:

```c
// Log when bad block is marked
void log_bad_block(uint32_t block, const char *reason)
{
    time_t now = time(NULL);
    fprintf(log, "[%lu] Bad block %u: %s\n", now, block, reason);

    // Trigger alert if multiple in short time
    check_bad_block_rate();
}
```

### 8.4 Lifetime Estimation

**Formula**:
```
Lifetime = (total_blocks × erase_cycles_per_block) / writes_per_day

Example:
  Total blocks: 2048
  Erase cycles: 100,000 per block
  Writes/day: 1 GB = ~8,000 blocks
  Bad blocks: 20 initially, +100 over lifetime

  Usable blocks: 2048 - 120 = 1928
  Total erases: 1928 × 100,000 = 192,800,000
  Days: 192,800,000 / 8,000 = 24,100 days
  Years: 24,100 / 365 ≈ 66 years

With Dhara wear leveling: All blocks wear evenly
Without wear leveling: Some blocks fail at 1 year
```

---

## Summary

This document provided detailed analysis of two-layer bad block management:

**Key Takeaways**:

1. **Two Independent Layers**: NAND driver (physical) and Dhara FTL (logical)
2. **Clear Division**: Driver detects/marks, Dhara tracks/recovers
3. **OOB Markers**: Permanent bad block marking in bytes 0-1
4. **Dhara Counters**: bb_current and bb_last track statistics
5. **Transparent Handling**: Bad blocks hidden from LittleFS/application
6. **Robust Recovery**: Multi-level recovery with retries
7. **Capacity Impact**: ~2-5% loss for typical bad block count

**Complete Flow**:
- Application writes → LittleFS → Dhara checks block → NAND driver reads OOB → Dhara skips if bad → Write succeeds transparently

**Best Practices**:
- Monitor bad block growth
- Use conservative estimates
- Checkpoint statistics regularly
- Handle ENOSPC gracefully
- Plan for replacement at >10% bad blocks

---

**Document Version**: 1.0
**Last Updated**: 2025
**Target Platform**: RTL8730E with W25N NAND Flash + Dhara FTL
**Complete Documentation Series**: Parts 1-6

