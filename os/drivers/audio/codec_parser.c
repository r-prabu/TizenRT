#include <tinyara/config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <unistd.h>
// #include <stdio.h>
#include <stdlib.h>
// #include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>

#include "codec_parser.h"
#include "syu645b.h"

#if 0
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
#endif

static codec_resource_t *g_codec_resource = NULL; 

/** * Read a little-endian uint16_t from buffer */
static uint16_t read_uint16(const uint8_t *buffer)
{ 
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8); 
}

/** * Read a little-endian uint32_t from buffer */ 
static uint32_t read_uint32(const uint8_t *buffer)
{ 
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24); 
} 

/* Get script name from ID */
const char *get_script_name(uint8_t id)
{
    switch (id) {
        case 0x01: return "RESET";
        case 0x02: return "STOP";
        case 0x03: return "INIT";
        case 0x04: return "MUTE_ON";
        case 0x05: return "MUTE_OFF";
        case 0x06: return "VOLUME";
        case 0x07: return "RATE_32K";
        case 0x08: return "RATE_44K";
        case 0x09: return "RATE_48K";
        case 0x0A: return "RATE_96K";
        case 0x0B: return "DQ_PRESET";
        default:   return "UNKNOWN";
    }
}

static void dump_config(const uint8_t *buffer)
{
     /* Verify magic header */
    uint32_t magic = read_uint32(buffer);
    if (magic != SYU645B_MAGIC_HEADER) {
        fprintf("Error: Invalid file format (magic=0x%08X)\n", magic);
        // free(buffer);
        return 1;
    }
    
    /* Parse file */
    uint8_t script_count = buffer[4];
    printf("SYU645B Configuration File\n");
    printf("Magic: 0x%08X\n", magic);
    printf("Script Count: %d\n\n", script_count);
    
    size_t offset = 5; /* Start after header and script count */
    
    for (int i = 0; i < script_count; i++) {
        /* Parse script header */
        uint8_t script_id = buffer[offset++];
        uint8_t entry_count = buffer[offset++];
        
        printf("Script %d: %s (ID: 0x%02X)\n", i + 1, get_script_name(script_id), script_id);
        printf("  Entries: %d\n", entry_count);
        
        /* Parse entries */
        for (int j = 0; j < entry_count; j++) {
            uint8_t addr = buffer[offset++];
            uint8_t type = buffer[offset++];
            uint16_t delay = read_uint16(&buffer[offset]);
            offset += 2;
            
            printf("  Entry %d:\n", j + 1);
            printf("    Address: 0x%02X\n", addr);
            printf("    Type: %d bytes\n", type);
            printf("    Delay: %d ms\n", delay);
            printf("    Values: ");
            
            for (int k = 0; k < type; k++) {
                printf("%02X ", buffer[offset++]);
            }
            printf("\n");
        }
        printf("\n");
    }
}

/** * Load and parse SYU645B resource file */ 
int syu645b_load_resource(const char *filename)
{ 
    int fd; 
    int ret = OK; 
    uint8_t *buffer = NULL; 
    size_t filesize; 
    struct stat st; 
    /* Free previous resource if exists */ 
    if (g_codec_resource != NULL) { 
        syu645b_free_resource(); 
    } 
    /* Open resource file */
    fd = open(filename, O_RDONLY); 
    if (fd < 0) { 
        return -ENOENT; 
    } 
    /* Get file size */ 
    if (fstat(fd, &st) < 0) { 
        ret = -EINVAL; 
        goto errout_with_fd; 
    } 
    filesize = st.st_size; 
    /* Read entire file */ 
    buffer = (uint8_t *)kmm_zalloc(filesize); 
    if (buffer == NULL) { 
        ret = -ENOMEM; 
        goto errout_with_fd; 
    } 
    if (read(fd, buffer, filesize) != filesize) { 
        ret = -EIO; 
        goto errout_with_buffer; 
    } 
    // dump_config(buffer);
    /* Verify minimum size and magic header */ 
    if (filesize < 5 || read_uint32(buffer) != SYU645B_MAGIC_HEADER) { 
        ret = -EINVAL; 
        goto errout_with_buffer; 
    } 
    /* Allocate resource structure */ 
    g_codec_resource = (codec_resource_t *)kmm_zalloc(sizeof(codec_resource_t)); 
    if (g_codec_resource == NULL) { 
        ret = -ENOMEM; 
        goto errout_with_buffer; 
    } 
    /* Parse header */ 
    g_codec_resource->magic = SYU645B_MAGIC_HEADER; 
    g_codec_resource->script_count = buffer[4]; 
    /* Allocate scripts array */ 
    g_codec_resource->scripts = (codec_script_t *)kmm_zalloc( g_codec_resource->script_count * sizeof(codec_script_t)); 
    if (g_codec_resource->scripts == NULL) { 
        ret = -ENOMEM; 
        lldbg("No memory\n");
        goto errout_with_resource; 
    } 
    /* Parse scripts */ 
    size_t offset = 5; 
    /* Start after header and script count */ 
    for (int i = 0; i < g_codec_resource->script_count; i++) { 
        /* Check if we have enough data for script header */ 
        if (offset + 2 > filesize) { 
            ret = -EINVAL; 
            lldbg("Not proper data\n");
            goto errout_with_scripts; 
        } 
        /* Parse script header */ 
        g_codec_resource->scripts[i].id = buffer[offset++]; 
        g_codec_resource->scripts[i].entry_count = buffer[offset++]; 
        /* Allocate entries array */ 
        g_codec_resource->scripts[i].entries = (script_entry_t *)kmm_zalloc( g_codec_resource->scripts[i].entry_count * sizeof(script_entry_t)); 
        if (g_codec_resource->scripts[i].entries == NULL) { 
            ret = -ENOMEM; 
            lldbg("No memory for entries \n");
            goto errout_with_scripts; 
        } 
        /* Parse entries */ 
        for (int j = 0; j < g_codec_resource->scripts[i].entry_count; j++) { 
            /* Check if we have enough data for entry header */ 
            if (offset + 4 > filesize) { 
                ret = -EINVAL; 
                 lldbg("No proper data for entries \n");
                goto errout_with_entries; 
            } 
            script_entry_t *entry = &g_codec_resource->scripts[i].entries[j]; /* Parse entry header */ 
            entry->addr = buffer[offset++]; 
            entry->type = buffer[offset++]; 
            entry->delay = read_uint16(&buffer[offset]); 
            offset += 2; 
            /* Validate type */ 
            if (entry->type < 2 || entry->type > 20) { 
                ret = -EINVAL;
                lldbg("No proper entry type \n"); 
                goto errout_with_entries; 
            } 
            /* Check if we have enough data for values */ 
            if (offset + entry->type > filesize) { 
                ret = -EINVAL; 
                lldbg("data more than file size \n"); 
                goto errout_with_entries; 
            } 
            /* Copy values */ 
            memcpy(entry->values, &buffer[offset], entry->type); 
            offset += entry->type; 
        } 
    } 
    /* Success */ 
    kmm_free(buffer); 
    close(fd); 
    return OK; 
errout_with_entries: 
    /* Free already allocated entries */ 
    for (int i = 0; i < g_codec_resource->script_count; i++) { 
        if (g_codec_resource->scripts[i].entries != NULL) { 
            kmm_free(g_codec_resource->scripts[i].entries); 
        } 
    } 
errout_with_scripts: 
    kmm_free(g_codec_resource->scripts);

errout_with_resource: 
    kmm_free(g_codec_resource); 
    g_codec_resource = NULL;

errout_with_buffer: 
    kmm_free(buffer); 

errout_with_fd: 
    close(fd); 
    return ret; 
}

/** * Free SYU645B resource */ 
void syu645b_free_resource(void)
{ 
    if (g_codec_resource != NULL) { 
        for (int i = 0; i < g_codec_resource->script_count; i++) { 
            if (g_codec_resource->scripts[i].entries != NULL) { 
                free(g_codec_resource->scripts[i].entries); 
            } 
        } 
        if (g_codec_resource->scripts != NULL) { 
            free(g_codec_resource->scripts); 
        } 
        free(g_codec_resource); 
        g_codec_resource = NULL; 
    } 
}

/** * Find script by ID */ 
codec_script_t *syu645b_find_script(uint8_t script_id)
{ 
    if (g_codec_resource == NULL) { 
        return NULL; 
    } 
    for (int i = 0; i < g_codec_resource->script_count; i++) { 
        if (g_codec_resource->scripts[i].id == script_id) { 
            return &g_codec_resource->scripts[i]; 
        } 
    } 
    return NULL; 
}

#if 0 
/** * Apply script to codec */ 
int syu645b_apply_script(struct i2c_dev_s *i2c_dev, uint8_t script_id)
{ 
    codec_script_t *script = syu645b_find_script(script_id); 
    if (script == NULL) { 
        return -EINVAL; 
    } 
    for (int i = 0; i < script->entry_count; i++) { 
        script_entry_t *entry = &script->entries[i]; 
        /* Write register address and values */ 
        i2c_write(i2c_dev, entry->addr); 
        for (int j = 0; j < entry->type; j++) { 
            i2c_write(i2c_dev, entry->values[j]); 
        } 
        /* Apply delay if needed */ 
        if (entry->delay > 0) { 
            usleep(entry->delay * 1000); 
        } 
    } 
    return OK; 
}
#endif