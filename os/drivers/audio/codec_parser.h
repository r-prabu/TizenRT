#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


#define SYU645B_MAGIC_HEADER 0x53593642 /* "SY6B" */
#define ALC1019_MAGIC_HEADER 0x414C4331 /* "ALC1" */
#define ALC5658_MAGIC_HEADER 0x414C4332 /* "ALC2" */

#define SCRIPT_ID_RESET         0x01 
#define SCRIPT_ID_STOP          0x02 
#define SCRIPT_ID_INIT          0x03 
#define SCRIPT_ID_MUTE_ON       0x04 
#define SCRIPT_ID_MUTE_OFF      0x05 
#define SCRIPT_ID_VOLUME        0x06 
#define SCRIPT_ID_RATE_32K      0x07 
#define SCRIPT_ID_RATE_44K      0x08 
#define SCRIPT_ID_RATE_48K      0x09 
#define SCRIPT_ID_RATE_96K      0x0A 
#define SCRIPT_ID_DQ_PRESET     0x0B 

#define MAX_ENTRY_TYPE_SIZE 32


typedef struct {
    uint16_t addr;        /* Register address (16-bit to support 2-byte addressing) */
    uint8_t type;         /* Data length */
    uint16_t delay;       /* Delay after write (ms) */
    uint8_t values[MAX_ENTRY_TYPE_SIZE]; /* Register values (variable length) */
} script_entry_t;

typedef struct {
    uint8_t id;           /* Script identifier */
    uint8_t entry_count;  /* Number of entries */
    script_entry_t *entries; /* Array of entries */
} codec_script_t;

typedef struct {
    uint32_t magic;       /* Magic identifier for codec type */
    uint8_t script_count; /* Number of scripts */
    uint8_t addr_size;    /* Size of register address (1 or 2 bytes) */
    codec_script_t *scripts; /* Array of scripts */
} codec_resource_t;

int load_resource(const char *filename);
codec_script_t *find_script(uint8_t script_id);
void free_resource(void);
