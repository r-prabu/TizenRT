#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define SYU645B_MAGIC_HEADER    0x53593642 /* "SY6B" */ 
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

#define SYU645B_REG_DATA_TYPE_MAX 20

typedef struct { 
    uint8_t addr; /* Register address */ 
    uint8_t type; /* Data length */ 
    uint16_t delay; /* Delay after write (ms) */ 
    uint8_t values[20]; /* Register values (variable length) */ 
} script_entry_t;

typedef struct { 
    uint8_t id; /* Script identifier */ 
    uint8_t entry_count; /* Number of entries */ 
    script_entry_t *entries; /* Array of entries */ 
} codec_script_t;

typedef struct { 
    uint32_t magic; /* Magic identifier "SY6B" */ 
    uint8_t script_count; /* Number of scripts */ 
    codec_script_t *scripts; /* Array of scripts */ 
} codec_resource_t;


int syu645b_load_resource(const char *filename);
codec_script_t *syu645b_find_script(uint8_t script_id);
void syu645b_free_resource(void);
