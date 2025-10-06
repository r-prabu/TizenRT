# TizenRT NAND Flash with LittleFS - Part 2: Hardware and Configuration
## For RTL8730E Chipset - Beginner's Guide

---

## Table of Contents
1. [Hardware Overview](#hardware-overview)
2. [RTL8730E Platform](#rtl8730e-platform)
3. [NAND Flash Specifications](#nand-flash-specifications)
4. [Memory Map and Partitions](#memory-map-and-partitions)
5. [Configuration Parameters](#configuration-parameters)
6. [Boot and Initialization](#boot-and-initialization)
7. [Manufacturer Support](#manufacturer-support)

---

## 1. Hardware Overview

### 1.1 System Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    RTL8730E SoC                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │          ARM Cortex-A32 Core                         │   │
│  │  • 32-bit ARMv7-A architecture                       │   │
│  │  • NEON SIMD engine                                  │   │
│  │  • Double precision FPU                              │   │
│  └──────────────────────────────────────────────────────┘   │
│                         ▲                                    │
│                         │                                    │
│  ┌──────────────────────▼──────────────────────────────┐   │
│  │          AXI/AHB/APB Bus Matrix                      │   │
│  └──┬──────────┬──────────┬──────────┬─────────────────┘   │
│     │          │          │          │                      │
│  ┌──▼──────┐ ┌▼────────┐ ┌▼────────┐ ┌▼─────────────────┐ │
│  │ DDR RAM │ │ SPI NOR │ │  SPIC   │ │  Other Periph.  │ │
│  │  63 MB  │ │  32 MB  │ │ (SPI    │ │  (UART, I2C,    │ │
│  │         │ │ (Code)  │ │ Master) │ │   GPIO, etc.)   │ │
│  └─────────┘ └─────────┘ └────┬────┘ └─────────────────┘ │
└─────────────────────────────────┼───────────────────────────┘
                                  │ SPI Bus
                                  │ (4-bit, 100 MHz)
                         ┌────────▼────────┐
                         │  W25N NAND Flash│
                         │     256 MB      │
                         │   (Data Storage)│
                         └─────────────────┘
```

### 1.2 Physical Connections

**SPI Interface Signals**:
```
RTL8730E SPIC          W25N NAND Flash
────────────           ───────────────
SPIC_CLK    ──────────> CLK         (SPI Clock, up to 100 MHz)
SPIC_CS     ──────────> CS#         (Chip Select, active low)
SPIC_DIO0   <────────> DI/IO0      (Data I/O bit 0 / MOSI)
SPIC_DIO1   <────────> DO/IO1      (Data I/O bit 1 / MISO)
SPIC_DIO2   <────────> WP#/IO2     (Write Protect / IO bit 2)
SPIC_DIO3   <────────> HOLD#/IO3   (Hold / IO bit 3)
GND         ──────────> VSS         (Ground)
3.3V        ──────────> VCC         (Power, 2.7V - 3.6V)
```

**Signal Modes**:
| Mode | Signals Used | Direction | Speed |
|------|-------------|-----------|-------|
| Standard SPI | CLK, CS#, DIO0, DIO1 | DIO0→NAND, DIO1←NAND | 12.5 MB/s |
| Dual SPI | CLK, CS#, DIO0, DIO1 | Both bidirectional | 25 MB/s |
| Quad SPI | CLK, CS#, DIO0-3 | All bidirectional | 50 MB/s |

### 1.3 Power and Timing

**Power Supply**:
- **VCC**: 2.7V - 3.6V (typically 3.3V)
- **Operating Current**:
  - Active read: ~25 mA
  - Active program: ~30 mA
  - Standby: <100 µA
- **Power-Down**: <10 µA

**Timing Specifications**:
```
Operation          Typical    Maximum
─────────────────  ────────   ────────
Page Read          25 µs      60 µs
Page Program       200 µs     700 µs
Block Erase        3 ms       10 ms
Power-up           <1 ms      5 ms
```

---

## 2. RTL8730E Platform

### 2.1 Processor Details

**CPU**: ARM Cortex-A32
- **Architecture**: ARMv7-A (32-bit)
- **ISA**: Thumb-2
- **SIMD**: NEON Advanced SIMD
- **FPU**: VFPv4 Double Precision Floating Point

**Memory Configuration**:
```
Memory Type     Start Address    Size        Usage
─────────────   ──────────────   ─────────   ─────────────────
DDR RAM         0x60100000       63 MB       Program execution
SPI NOR Flash   0x08000000       32 MB       Bootloader, kernel
SPI NAND Flash  0x0A000000       256 MB      User data, logs
```

### 2.2 SPIC Controller

**SPIC (SPI Controller) Features**:
- Master-only mode
- Programmable clock divider (up to 100 MHz)
- Dual/Quad SPI support
- TX FIFO: 32 bytes
- RX FIFO: 32 bytes
- DMA support (not used for NAND in this implementation)

**SPIC Register Map** (Key Registers):
```c
Offset   Name      Description
──────   ────────  ─────────────────────────────────
0x00     CTRLR0    Control Register 0 (SPI mode, frame format)
0x04     CTRLR1    Control Register 1 (data frames)
0x08     SSIENR    SSI Enable Register
0x10     SER       Slave Enable Register
0x18     BAUDR     Baud Rate Select
0x28     SR        Status Register
0x2C     IMR       Interrupt Mask Register
0x30     ISR       Interrupt Status Register
0x60     DR[0]     Data Register (FIFO access)
0x64     DR[1]     Data Register
...
0xA0     RX_NDF    RX Number of Data Frames
0xA4     TX_NDF    TX Number of Data Frames
```

**CTRLR0 Register Bits** (SPI Mode Configuration):
```
Bits     Field       Description
───────  ──────────  ───────────────────────────────────
[1:0]    DFS         Data Frame Size (0=8bit, 1=16bit)
[3:2]    FRF         Frame Format (0=Motorola SPI)
[5:4]    SCPH/SCPOL  Clock Phase/Polarity
[7:6]    TMOD        Transfer Mode:
                       0 = TX and RX
                       1 = TX only
                       2 = RX only
                       3 = EEPROM read
[9:8]    SLV_OE      Slave Output Enable
[15:12]  CMD_CH      Command Channel (0=1bit, 2=4bit)
[17:16]  ADDR_CH     Address Channel (0=1bit, 2=4bit)
[19:18]  DATA_CH     Data Channel (0=1bit, 1=2bit, 2=4bit)
```

**Example Configuration for Quad SPI Read**:
```c
// Configure SPIC for Quad Output Read (1-1-4)
spi_flash->CTRLR0 =
    DFS(0) |           // 8-bit frames
    FRF(0) |           // Motorola SPI
    TMOD(3) |          // RX only (EEPROM read mode)
    CMD_CH(0) |        // Command on 1 bit
    ADDR_CH(0) |       // Address on 1 bit
    DATA_CH(2);        // Data on 4 bits (quad)
```

### 2.3 Memory Architecture

**Address Space Layout**:
```
0xFFFFFFFF  ┌─────────────────────┐
            │   Reserved          │
            ├─────────────────────┤
0x70000000  │   Peripheral Regs   │  SPIC registers at 0x5000_0000
            ├─────────────────────┤
0x64000000  │   DDR RAM (63 MB)   │  Main memory
0x60100000  ├─────────────────────┤
            │   Reserved          │
            ├─────────────────────┤
0x0C000000  │   NAND Flash        │  256 MB (via SPIC)
0x0A000000  ├─────────────────────┤  (Memory-mapped for reads)
            │   Reserved          │
            ├─────────────────────┤
0x0A000000  │   SPI NOR Flash     │  32 MB (code storage)
0x08000000  ├─────────────────────┤
            │   Reserved          │
            ├─────────────────────┤
0x00000000  │   Internal ROM      │  Bootloader
            └─────────────────────┘
```

**Cache Configuration**:
- **I-Cache**: 32 KB instruction cache
- **D-Cache**: 32 KB data cache
- **Write Buffer**: 4 entries
- **Cache Line**: 32 bytes

---

## 3. NAND Flash Specifications

### 3.1 Winbond W25N Series

**Supported Models**:
| Model | Capacity | Device ID | Planes | Status |
|-------|----------|-----------|--------|--------|
| W25N01GV | 128 MB | 0xEF 0xAA 0x21 | 1 | Supported |
| W25N02KV | 256 MB | 0xEF 0xAA 0x22 | 1 | Primary target |
| W25N02KV | 256 MB | 0xEF 0xAB 0x22 | 2 | Multi-die variant |

**W25N02KV Specifications** (Primary Target):
```
Parameter                    Value
──────────────────────────   ─────────────────
Manufacturer                 Winbond (0xEF)
Device ID                    0xAA (single-die) / 0xAB (dual-die)
Capacity ID                  0x22 (256 MB)
Organization                 2048 blocks × 64 pages × 2KB
Page Size                    2048 + 64 bytes (main + OOB)
Block Size                   128 KB
Total Blocks                 2048
Total Capacity               256 MB
Planes/Dies                  1 or 2 (depending on variant)
Bad Blocks (Max)             40 (initial) + growth over lifetime
ECC                          Internal 1-bit ECC per 512 bytes
                             (4-bit correction per page)
Endurance                    100,000 erase cycles
Data Retention               10 years
Operating Voltage            2.7V - 3.6V
SPI Modes                    Standard, Dual, Quad
Max Frequency                104 MHz (read), 80 MHz (program)
```

### 3.2 NAND Flash Internal Architecture

**Block Structure**:
```
W25N02KV Internal Organization:

Die 0 (128 MB):
  ┌─────────────────────────────────┐
  │  Block 0 (128 KB)               │
  │  ├─ Page 0 (2112 bytes)         │
  │  ├─ Page 1                      │
  │  │  ...                          │
  │  └─ Page 63                     │
  ├─────────────────────────────────┤
  │  Block 1                        │
  │  ...                             │
  │  Block 1023                     │
  └─────────────────────────────────┘

Die 1 (128 MB): [Only for 0xAB variant]
  ┌─────────────────────────────────┐
  │  Block 1024                     │
  │  ...                             │
  │  Block 2047                     │
  └─────────────────────────────────┘
```

**Page Structure**:
```
┌────────────────────────────────────────────────────┐
│  Column Address 0x000 - 0x7FF (2048 bytes)        │
│  ┌──────────────────────────────────────────────┐ │
│  │         Main Data Area                       │ │
│  │         (User data)                          │ │
│  │         2048 bytes                           │ │
│  └──────────────────────────────────────────────┘ │
├────────────────────────────────────────────────────┤
│  Column Address 0x800 - 0x83F (64 bytes)          │
│  ┌──────────────────────────────────────────────┐ │
│  │  OOB (Out-of-Band) Area                     │ │
│  │  Byte 0-1:  Bad Block Marker                │ │
│  │  Byte 2-63: Spare area (ECC, metadata)      │ │
│  │  64 bytes total                              │ │
│  └──────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────┘
```

### 3.3 NAND Flash Commands

**Standard SPI NAND Commands**:
```c
Command                  Code    Description
──────────────────────   ────    ─────────────────────────────────
// Device Management
RESET                    0xFF    Software reset
READ ID                  0x9F    Read manufacturer and device ID

// Register Access
GET FEATURES             0x0F    Read status/config registers
SET FEATURES             0x1F    Write status/config registers

// Read Operations
PAGE READ                0x13    Transfer page from array to cache
READ FROM CACHE          0x03    Read cache (standard, 1-1-1)
FAST READ                0x0B    Read cache with dummy byte
FAST READ DUAL OUTPUT    0x3B    Read cache (1-1-2)
FAST READ DUAL IO        0xBB    Read cache (1-2-2)
FAST READ QUAD OUTPUT    0x6B    Read cache (1-1-4)
FAST READ QUAD IO        0xEB    Read cache (1-4-4)

// Write Operations
WRITE ENABLE             0x06    Enable write/erase operations
WRITE DISABLE            0x04    Disable write/erase operations
PROGRAM LOAD             0x02    Load data to cache (1-1-1)
QUAD PROGRAM LOAD        0x32    Load data to cache (1-1-4)
RANDOM PROGRAM LOAD      0x84    Update cache (1-1-1)
QUAD RANDOM PROGRAM LOAD 0x34    Update cache (1-1-4)
PROGRAM EXECUTE          0x10    Transfer cache to array

// Erase Operations
BLOCK ERASE              0xD8    Erase 128 KB block
```

**Command Sequences**:

**Read Sequence**:
```
1. PAGE READ (0x13)
   ┌──────┬──────┬──────┬──────┐
   │ 0x13 │ PA23 │ PA15 │ PA7  │  PA = Page Address
   │      │ -PA16│ -PA8 │ -PA0 │
   └──────┴──────┴──────┴──────┘
   Wait for busy (25-60 µs)

2. READ FROM CACHE (0x0B - Fast Read)
   ┌──────┬──────┬──────┬──────┬──────────┬────────┐
   │ 0x0B │ CA15 │ CA7  │Dummy │ Data...  │ Data   │
   │      │ -CA8 │ -CA0 │ 0x00 │          │        │
   └──────┴──────┴──────┴──────┴──────────┴────────┘
   CA = Column Address (0x000 - 0x83F)
```

**Write Sequence**:
```
1. WRITE ENABLE (0x06)
   ┌──────┐
   │ 0x06 │
   └──────┘

2. PROGRAM LOAD (0x02)
   ┌──────┬──────┬──────┬──────────┬────────┐
   │ 0x02 │ CA15 │ CA7  │ Data...  │ Data   │
   │      │ -CA8 │ -CA0 │          │        │
   └──────┴──────┴──────┴──────────┴────────┘

3. PROGRAM EXECUTE (0x10)
   ┌──────┬──────┬──────┬──────┐
   │ 0x10 │ PA23 │ PA15 │ PA7  │
   │      │ -PA16│ -PA8 │ -PA0 │
   └──────┴──────┴──────┴──────┘
   Wait for busy (200-700 µs)
```

**Erase Sequence**:
```
1. WRITE ENABLE (0x06)
   ┌──────┐
   │ 0x06 │
   └──────┘

2. BLOCK ERASE (0xD8)
   ┌──────┬──────┬──────┬──────┐
   │ 0xD8 │ PA23 │ PA15 │ PA7  │
   │      │ -PA16│ -PA8 │ -PA0 │
   └──────┴──────┴──────┴──────┘
   Wait for busy (3-10 ms)
```

### 3.4 Status and Configuration Registers

**Status Register (Address 0xC0)**:
```
Bit     Name    Description
──────  ──────  ───────────────────────────────────────
[0]     OIP     Operation In Progress (1=busy, 0=ready)
[1]     WEL     Write Enable Latch (1=enabled, 0=disabled)
[2]     E_FAIL  Erase Failure (1=failed, 0=success)
[3]     P_FAIL  Program Failure (1=failed, 0=success)
[5:4]   ECCS    ECC Status:
                  00 = No errors
                  01 = 1-4 bits corrected
                  10 = Not corrected (error)
                  11 = Reserved
[6]     ECCS1   Additional ECC status bit
[7]     Reserved
```

**Configuration Register (Address 0xB0)**:
```
Bit     Name    Description
──────  ──────  ───────────────────────────────────────
[0]     QE      Quad Enable (1=quad enabled, 0=disabled)
[1]     ECC-E   ECC Enable (1=enabled, 0=disabled)
[2]     SR1_L   Status Register Lock
[3]     TB      Top/Bottom Block Protect
[4]     BP0     Block Protect bit 0
[5]     BP1     Block Protect bit 1
[6]     BP2     Block Protect bit 2
[7]     BP3     Block Protect bit 3
```

**Protection Register (Address 0xA0)**:
```
Bit     Name    Description
──────  ──────  ───────────────────────────────────────
[0]     SRP0    Status Register Protect 0
[1]     WP-E    Write Protect Enable
[2]     TB      Top/Bottom protect
[3]     BP0     Block Protect bit 0
[4]     BP1     Block Protect bit 1
[5]     BP2     Block Protect bit 2
[6]     BP3     Block Protect bit 3
[7]     SRP1    Status Register Protect 1
```

**Die Select Register (Address 0xD0)** [For dual-die variants]:
```
Bit     Name    Description
──────  ──────  ───────────────────────────────────────
[0]     DS      Die Select (0=die 0, 1=die 1)
[7:1]   Reserved
```

---

## 4. Memory Map and Partitions

### 4.1 Flash Partition Layout

**Primary Flash (SPI NOR - 32 MB)**:
```
Offset      Size        Type        Name         Description
──────────  ──────────  ──────────  ───────────  ─────────────────────
0x00000000  60 KB       none        bl1          Bootloader stage 1
0x0000F000  40 KB       none        reserved     Reserved space
0x00019000  12 KB       none        ftl          FTL metadata
0x0001C000  400 KB      none        ss           Secure Storage
0x00080000  5228 KB     kernel      kernel_a     Kernel A (active)
0x00598000  5228 KB     kernel      kernel_b     Kernel B (backup)
0x00AB0000  20512 KB    none        reserved     Large reserved area
0x01EB0000  1280 KB     ftl         ftl_data     FTL data area
0x01FF0000  8 KB        bootparam   bootparam    Boot parameters
0x01FF2000  (end)
```

**Secondary Flash (SPI NAND - 256 MB)**:
```
Offset      Size        Type        Name         Description
──────────  ──────────  ──────────  ───────────  ─────────────────────
0x00000000  128 MB      littlefs    lfs          LittleFS partition
0x08000000  128 MB      romfs       romfs        Read-only FS
0x10000000  (end)
```

**Configuration** (`build/configs/rtl8730e/flat_dev_ddr_nand/defconfig`):
```bash
# Primary Flash Partitions
CONFIG_FLASH_PART_SIZE="60,40,12,400,5228,5228,20512,1280,8,"
CONFIG_FLASH_PART_TYPE="none,none,none,none,kernel,kernel,none,ftl,bootparam,"
CONFIG_FLASH_PART_NAME="bl1,reserved,ftl,ss,kernel,kernel,reserved,reserved,bootparam,"

# Secondary Flash (NAND) Partitions
CONFIG_SECOND_FLASH_PART_SIZE="131072,131072,"
CONFIG_SECOND_FLASH_PART_TYPE="littlefs,romfs,"
CONFIG_SECOND_FLASH_PART_NAME="lfs,romfs,"
```

### 4.2 Partition Details

**LittleFS Partition (128 MB)**:
```
Purpose:  User data storage, application files, logs
Location: NAND Flash 0x00000000 - 0x07FFFFFF
Blocks:   1024 blocks × 128 KB = 128 MB
Mount:    /mnt (automatic mount at boot)
Features:
  - Power-loss protection
  - Wear leveling
  - Bad block handling
  - Dynamic allocation
```

**ROMFS Partition (128 MB)**:
```
Purpose:  Read-only file system (firmware updates, static assets)
Location: NAND Flash 0x08000000 - 0x0FFFFFFF
Blocks:   1024 blocks × 128 KB = 128 MB
Mount:    /rom (on-demand mount)
Features:
  - Compressed
  - Read-only
  - Fast access
```

### 4.3 Block Allocation Example

**LittleFS Initial State After Format**:
```
Block 0:     Superblock (filesystem metadata)
Block 1:     Root directory
Block 2-3:   Free block list (metadata)
Block 4-1023: Free for allocation

As files are created:
Block 4:     /mnt/file1.txt (data)
Block 5:     /mnt/logs/ (directory)
Block 6-10:  /mnt/logs/system.log (data)
...
```

**Bad Block Distribution** (Example):
```
Good blocks:  2008 blocks
Factory bad:  20 blocks (marked in OOB byte 0-1 as 0x00)
Runtime bad:  0 blocks initially, grows over device lifetime

LittleFS effective capacity:
  = 2008 blocks × 128 KB
  = 251 MB usable (out of 256 MB total)

Overhead for metadata: ~2-5%
Final user capacity: ~238 MB
```

---

## 5. Configuration Parameters

### 5.1 Kernel Configuration (defconfig)

**File**: `build/configs/rtl8730e/flat_dev_ddr_nand/defconfig`

**Key NAND-Related Settings**:

**NAND Flash Driver**:
```bash
CONFIG_MTD=y                              # Memory Technology Device support
CONFIG_MTD_NAND=y                         # NAND flash support
CONFIG_MTD_W25N=y                         # Winbond W25N driver

# NAND Geometry
CONFIG_MTD_NAND_MAXNUMBLOCKS=2048         # Max blocks: 2048
CONFIG_MTD_NAND_MAXNUMPAGESPERBLOCK=64    # Pages per block: 64
CONFIG_MTD_NAND_MAXPAGEDATASIZE=2048      # Page size: 2048 bytes
CONFIG_MTD_NAND_MAXPAGESPARESIZE=128      # Spare size: 128 bytes
CONFIG_MTD_NAND_MAXSPAREECCBYTES=64       # ECC bytes: 64
CONFIG_MTD_NAND_BLOCKCHECK=y              # Enable bad block checking
CONFIG_MTD_NAND_MAXSPAREEXTRABYTES=64     # Extra spare bytes: 64
```

**Flash Translation Layer**:
```bash
CONFIG_MTD_DHARA=y                        # Dhara FTL
CONFIG_DHARA_GC_RATIO=4                   # Garbage collection ratio
CONFIG_DHARA_READ_NCACHES=4               # Read cache count
```

**File System Support**:
```bash
CONFIG_FS_LITTLEFS=y                      # LittleFS filesystem
CONFIG_FS_ROMFS=y                         # ROM filesystem
CONFIG_FS_SMARTFS=y                       # SMART filesystem (alternative)
CONFIG_MTD_FTL=y                          # Generic FTL support
CONFIG_MTD_SMART=y                        # SMART MTD support
```

**Flash Configuration**:
```bash
# Primary Flash (SPI NOR)
CONFIG_FLASH_START_ADDR=0x8000000
CONFIG_FLASH_SIZE=33554432                # 32 MB
CONFIG_BOARD_FLASH_32M=y

# Secondary Flash (NAND)
CONFIG_SECOND_FLASH_START_ADDR=0x0A000000
CONFIG_ARCH_BOARD_HAVE_SECOND_FLASH=y
CONFIG_SECOND_FLASH_PARTITION=y
CONFIG_SECOND_FLASH_MINOR=1               # Device node: /dev/mtdblock1

# Mount Points
CONFIG_USERFS_MNTPT="/mnt0"               # Primary mount
CONFIG_USERFS_EXT_MNTPT="/mnt"            # NAND mount
CONFIG_AUTOMOUNT=y                        # Auto-mount at boot
CONFIG_AUTOMOUNT_USERFS=y
```

**Memory Configuration**:
```bash
CONFIG_RAM_DDR=y                          # Use DDR RAM
CONFIG_RAM_START=0x60100000
CONFIG_RAM_SIZE=66060288                  # ~63 MB
CONFIG_BOOT_RUNFROMSDRAM=y                # Run from DDR
```

### 5.2 LittleFS Configuration

**Runtime Configuration** (`littlefs_adapter.c`):
```c
struct lfs_config g_nand_lfs_cfg = {
    // Block device operations
    .read  = lfs_nand_read,
    .prog  = lfs_nand_prog,
    .erase = lfs_nand_erase,
    .sync  = lfs_diskio_sync,

    // Thread safety
    .lock   = lfs_diskio_lock,
    .unlock = lfs_diskio_unlock,

    // Geometry configuration
    .read_size      = 2048,               // Minimum read: 2 KB (1 page)
    .prog_size      = 2048,               // Minimum program: 2 KB (1 page)
    .block_size     = 131072,             // Block size: 128 KB
    .block_count    = 1024,               // 1024 blocks in LFS partition

    // Cache configuration
    .cache_size     = 2048,               // Cache: 1 page
    .lookahead_size = 8,                  // Lookahead: 8 blocks

    // Wear leveling
    .block_cycles   = 100,                // Wear level every 100 erases

    // Metadata redundancy
    .metadata_max   = 256,                // Max metadata size
};
```

**Configuration Trade-offs**:

| Parameter | Value | Impact |
|-----------|-------|--------|
| `read_size=2048` | Aligned to page | ✓ Efficient<br>✗ Read amplification for small files |
| `prog_size=2048` | Aligned to page | ✓ Efficient<br>✗ Write amplification |
| `cache_size=2048` | 1 page | ✓ Low RAM usage (2 KB)<br>✗ More flash access |
| `lookahead_size=8` | 8 blocks | ✓ Fast allocation<br>✗ Some RAM overhead |
| `block_cycles=100` | Moderate | ✓ Balanced wear leveling<br>✗ Some overhead |

**Memory Usage**:
```
LittleFS RAM consumption:
  - Read cache:       2048 bytes
  - Program cache:    2048 bytes
  - Lookahead buffer: 8 bytes (1 bit per block for 8 blocks)
  - Metadata cache:   ~512 bytes
  - File handles:     ~64 bytes each
  ─────────────────────────────
  Total per mount:    ~5 KB + (64 × open_files)
```

### 5.3 SPI Configuration

**SPI Parameters** (`lfs_spinand.c`):
```c
// SPI Mode
#define CONFIG_W25N_SPIMODE      SPIDEV_MODE0
    // Mode 0: CPOL=0 (clock idle low), CPHA=0 (sample on leading edge)

// SPI Frequency
#define CONFIG_W25N_SPIFREQUENCY 100000000    // 100 MHz

// Quad SPI
#define QUAD_SPI_ENABLED         1             // Enable 4-bit mode

// Dummy Cycles (for different read modes)
#define NAND_DM_CYCLE_READ       0             // Standard read: 0 dummy
#define NAND_DM_CYCLE_FREAD      8             // Fast read: 8 dummy
#define NAND_DM_CYCLE_2O         8             // Dual output: 8 dummy
#define NAND_DM_CYCLE_2IO        4             // Dual I/O: 4 dummy
#define NAND_DM_CYCLE_4O         8             // Quad output: 8 dummy
#define NAND_DM_CYCLE_4IO        4             // Quad I/O: 4 dummy
```

**Timing Diagrams**:

**Standard SPI (Mode 0)**:
```
CLK:   ──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐  ┌──
         └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘  └──┘
CS#:   ────┐                                          ┌───
           └──────────────────────────────────────────┘

MOSI:  ────<D7><D6><D5><D4><D3><D2><D1><D0>───────────────
                 ▲   ▲   ▲   ▲   ▲   ▲   ▲   ▲
                Sample points (on rising edge)
```

**Quad SPI (4-bit)**:
```
CLK:   ──┐  ┌──┐  ┌──┐  ┌──
         └──┘  └──┘  └──┘
CS#:   ────┐              ┌───
           └──────────────┘

IO0:   ────<D0><D4>───────
IO1:   ────<D1><D5>───────
IO2:   ────<D2><D6>───────
IO3:   ────<D3><D7>───────
          ▲   ▲
        Nibble 0, Nibble 1
        (1 byte in 2 clocks)
```

---

## 6. Boot and Initialization

### 6.1 Boot Sequence

```
Power On
   │
   ▼
┌──────────────────────────────────────┐
│  ROM Bootloader (in SoC)            │
│  • Initialize clocks                 │
│  • Initialize DDR                    │
│  • Load BL1 from NOR flash          │
└───────────────┬──────────────────────┘
                │
                ▼
┌──────────────────────────────────────┐
│  BL1 (Bootloader Stage 1)           │
│  • Initialize peripherals            │
│  • Verify kernel signature           │
│  • Load kernel from NOR flash        │
└───────────────┬──────────────────────┘
                │
                ▼
┌──────────────────────────────────────┐
│  TizenRT Kernel Boot                │
│  • Initialize scheduler              │
│  • Mount root filesystem             │
│  • Initialize drivers                │
└───────────────┬──────────────────────┘
                │
                ▼
┌──────────────────────────────────────┐
│  Board Initialization                │
│  (rtl8730e_boot.c)                   │
│  • rtl8730e_board_initialize()      │
│  • rtl8730e_flash_initialize()      │
└───────────────┬──────────────────────┘
                │
                ▼
┌──────────────────────────────────────┐
│  NAND Flash Initialization           │
│  • Detect NAND (read ID)             │
│  • Initialize NAND FTL               │
│  • Register MTD device               │
└───────────────┬──────────────────────┘
                │
                ▼
┌──────────────────────────────────────┐
│  LittleFS Mount                      │
│  • lfs_mount("/mnt")                 │
│  • Read superblock                   │
│  • Verify filesystem                 │
└───────────────┬──────────────────────┘
                │
                ▼
┌──────────────────────────────────────┐
│  User Applications Start             │
│  • init process                      │
│  • Shell                             │
│  • User apps                         │
└──────────────────────────────────────┘
```

### 6.2 NAND Initialization Details

**Code Flow** (`rtl8730e_boot.c:374-381`):
```c
void rtl8730e_flash_initialize(void)
{
#ifdef CONFIG_MTD_W25N
    // Initialize SPI bus
    struct spi_dev_s *spi = up_spiinitialize(NAND_SPI_PORT);

    // Initialize W25N NAND driver
    struct mtd_dev_s *mtd = w25n_initialize(spi);
    if (mtd == NULL) {
        lldbg("w25n Init failed\n");
        return;
    } else {
        lldbg("w25n initialized\n");
    }

    // Register MTD device
    register_mtddriver("/dev/mtdblock1", mtd, 0, NULL);
#endif
}
```

**Detailed Initialization** (`lfs_nand_ftl.c:223-279`):
```c
u8 NAND_FTL_Init(NAND_FTL_DeviceTypeDef *nand)
{
    // Step 1: Reset NAND
    NAND_TxCmd(0xFF, 0, NULL, 0, NULL);

    // Step 2: Read Manufacturer and Device ID
    u8 flash_ID[3];
    NAND_RxCmd(FLASH_cmd_rd_id, 0, NULL, 2, flash_ID);

    // Micron uses 3-byte ID
    if (flash_ID[0] == 0x2C) {
        NAND_RxCmd(FLASH_cmd_rd_id, 0, NULL, 3, flash_ID);
    }

    nand->MemInfo.MID = flash_ID[0];      // Manufacturer ID
    nand->MemInfo.DID = flash_ID[1];      // Device ID
    nand->MemInfo.ExtDID = flash_ID[2];   // Extended ID (if applicable)

    // Step 3: Read current configuration
    u8 reg = NAND_GetStatus(NAND_REG_CFG);

    // Step 4: Enable internal ECC
    reg |= NAND_CFG_ECC_ENABLE;

    // Step 5: Configure quad mode (if supported)
    if (flash_init_para.FLASH_QuadEn_bit) {
        reg |= flash_init_para.FLASH_QuadEn_bit;
    }

    // Step 6: Write configuration
    NAND_SetStatus(NAND_REG_CFG, reg);

    // Step 7: Manufacturer-specific initialization
    ret = NAND_FTL_MfgInit(nand);

    // Step 8: Mark as initialized
    nand->Initialized = 1;
    nand->LastErasedBlockAddr = 0xFFFFFFFF;

    return ret;
}
```

### 6.3 Manufacturer Detection

**Supported Manufacturers**:
```c
#define NAND_MFG_DOSILICON    0xE5    // Dosilicon
#define NAND_MFG_GIGADEVICE   0xC8    // GigaDevice
#define NAND_MFG_MACRONIX     0xC2    // Macronix
#define NAND_MFG_MICRON       0x2C    // Micron
#define NAND_MFG_WINBOND      0xEF    // Winbond
```

**Detection Example**:
```
READ ID command (0x9F):
  Send:  [0x9F]
  Recv:  [0xEF][0xAA][0x22]
         │    │    │
         │    │    └─ Capacity: 0x22 = 256 MB
         │    └────── Device: 0xAA = W25N series
         └─────────── Manufacturer: 0xEF = Winbond

Result: Winbond W25N02KV (256 MB NAND)
```

### 6.4 ONFI Parameter Page

**ONFI**: Open NAND Flash Interface (industry standard)

**Parameter Page Structure**:
```
Offset  Size  Description
──────  ────  ─────────────────────────────────
0-3     4     Signature "ONFI" (0x4F 0x4E 0x46 0x49)
4-5     2     Revision number
6-7     2     Features supported
...
32-43   12    Manufacturer name (ASCII)
44-63   20    Device model (ASCII)
64      1     JEDEC manufacturer ID
65-79   15    Date code
80-83   4     Bytes per page (little endian)
84-85   2     Spare bytes per page
...
92-95   4     Pages per block
96-99   4     Blocks per LUN
100     1     Number of LUNs
...
103-104 2     Max bad blocks per LUN
...
112     1     Number of ECC bits required
...
254-255 2     CRC16 checksum
```

**Example Parameter Page Read**:
```
Manufacturer: "WINBOND     "
Model:        "W25N02KV            "
Page Size:    2048 (0x00000800)
OOB Size:     64 (0x0040)
Pages/Block:  64 (0x00000040)
Blocks/LUN:   1024 (0x00000400)
LUNs:         1 or 2
Max Bad:      40 (0x0028)
ECC Required: 1 bit per 512 bytes

Calculated Capacity:
  = 2048 × 64 × 1024 × 1
  = 134,217,728 bytes
  = 128 MB per LUN
  = 256 MB total (2 LUNs for dual-die variant)
```

---

## 7. Manufacturer Support

### 7.1 Winbond W25N

**Manufacturer ID**: 0xEF

**Key Characteristics**:
- **Quad SPI**: Enabled by default
- **Multi-die Support**: W25N02KV (0xAB) has 2 dies
- **Die Selection**: Command 0xC2
- **Buffer Mode**: Must be in buffer read mode (not continuous)
- **ECC**: Internal 1-bit ECC per 512 bytes

**Initialization** (`lfs_nand_ftl_mfg.c:198-218`):
```c
static u8 NAND_FTL_Winbond_Init(NAND_FTL_DeviceTypeDef *nand)
{
    // Configure buffer read mode on all dies
    if (nand->MemInfo.Targets > 1) {
        for (u8 i = 0; i < nand->MemInfo.Targets; i++) {
            NAND_FTL_Winbond_SelectTarget(nand, i);

            // Set bit 3 of CFG register (buffer read mode)
            NAND_SetStatusBits(NAND_REG_CFG,
                              NF_CFG_WINBOND_BUF_READ,
                              ENABLE);
        }
    } else {
        NAND_SetStatusBits(NAND_REG_CFG,
                          NF_CFG_WINBOND_BUF_READ,
                          ENABLE);
    }

    return HAL_OK;
}
```

**Die Selection** (for W25N02KV with 2 dies):
```c
static u8 NAND_FTL_Winbond_SelectTarget(NAND_FTL_DeviceTypeDef *nand,
                                        u8 target)
{
    if (nand->MemInfo.Targets > 1) {
        // Send die select command (0xC2)
        NAND_TxCmd(0xC2, 0, NULL, 1, &target);
    }
    return HAL_OK;
}
```

**Register Configuration**:
```
Register 0xB0 (Configuration):
  Bit 0 (QE):     1 (Quad enabled by default)
  Bit 1 (ECC-E):  1 (ECC enabled)
  Bit 3 (BUF):    1 (Buffer read mode)

Register 0xA0 (Protection):
  Bit 1 (WP-E):   0 (Write protect disabled by default)
```

### 7.2 Other Supported Manufacturers

**Micron (0x2C)**:
- Quad mode always enabled (cannot be disabled)
- Uses 3-byte device ID
- Supports dual and quad I/O modes
- Different ECC status format

**GigaDevice (0xC8)**:
- Requires dummy byte in read commands
- Variable address phase length (2 or 3 bytes)
- No dual/quad I/O support (only output modes)
- 4-level ECC status

**Macronix (0xC2)**:
- Extended ECC status register (command 0x7C)
- Special parameter page access sequence
- No dual/quad I/O support

**Dosilicon (0xE5)**:
- Similar to Macronix
- 2-level ECC reporting
- Parameter page access requires CFG manipulation

---

## Summary

This part covered the hardware and configuration details:

**Key Points**:
1. **RTL8730E Platform**: ARM Cortex-A32 with SPIC controller
2. **W25N NAND**: 256MB, 2KB pages, 128KB blocks
3. **Partitions**: 128MB LittleFS + 128MB ROMFS
4. **Configuration**: Extensive kernel config options
5. **Boot Sequence**: ROM → BL1 → Kernel → NAND Init → LittleFS Mount
6. **Manufacturer Support**: Winbond, Micron, GigaDevice, Macronix, Dosilicon

**Next Part**: Software implementation details, function call flows, and integration

---

**Document Version**: 1.0
**Last Updated**: 2025
**Target Platform**: RTL8730E with W25N NAND Flash
