#include <tinyara/config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>

#include "codec_parser.h"

// #ifdef CONFIG_AUDIO_SYU645B
// #include "syu645bscripts.h"
// #elif defined(CONFIG_AUDIO_ALC1019)
// #include "alc1019scripts.h"
// #elif defined(CONFIG_AUDIO_ALC5658)
// #include "alc5658scripts.h"
// #endif



static codec_resource_t *g_codec_resource = NULL;

/* Helper functions to read little-endian values */
static uint16_t read_uint16(const uint8_t *buffer)
{
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

static uint32_t read_uint32(const uint8_t *buffer)
{
    return (uint32_t)buffer[0] |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
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
    if (magic != SYU645B_MAGIC_HEADER && magic != ALC1019_MAGIC_HEADER && magic != ALC5658_MAGIC_HEADER) {
        fprintf(stderr, "Error: Invalid file format (magic=0x%08X)\n", magic);
        return;
    }

    /* Parse file */
    uint8_t script_count = buffer[4];
    uint8_t addr_size = buffer[5];
    printf("Codec Configuration File\n");
    printf("Magic: 0x%08X\n", magic);
    printf("Script Count: %d\n", script_count);
    printf("Address Size: %d bytes\n\n", addr_size);

    size_t offset = 6; /* Start after header, script count, and address size */

    for (int i = 0; i < script_count; i++) {
        /* Parse script header */
        uint8_t script_id = buffer[offset++];
        uint8_t entry_count = buffer[offset++];

        printf("Script %d: %s (ID: 0x%02X)\n", i + 1, get_script_name(script_id), script_id);
        printf("  Entries: %d\n", entry_count);

        /* Parse entries */
        for (int j = 0; j < entry_count; j++) {
            uint16_t addr = (addr_size == 2) ? read_uint16(&buffer[offset]) : buffer[offset];
            offset += addr_size;
            uint8_t type = buffer[offset++];
            uint16_t delay = read_uint16(&buffer[offset]);
            offset += 2;

            printf("  Entry %d:\n", j + 1);
            printf("    Address: 0x%0*X\n", addr_size * 2, addr);
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

/* Get file size using lseek */
static off_t get_file_size(int fd)
{
    off_t current_pos, size;

    /* Save current position */
    current_pos = lseek(fd, 0, SEEK_CUR);
    if (current_pos < 0) {
        lldbg("lseek to start failed \n");
        return -1;
    }

    /* Seek to end to get size */
    size = lseek(fd, 0, SEEK_END);
    if (size < 0) {
        lldbg("lseek to end failed \n");
        return -1;
    }

    /* Restore original position */
    if (lseek(fd, current_pos, SEEK_SET) < 0) {
        lldbg("lseek restore failed \n");
        return -1;
    }

    return size;
}

/* Load and parse resource file */
int load_resource(const char *filename)
{
    int i, j;
    int fd;
    int ret = OK;
    uint8_t *buffer = NULL;
    size_t filesize;

    /* Free previous resource if exists */
    if (g_codec_resource != NULL) {
        free_resource();
    }

    /* Open resource file */
    fd = open(filename, O_RDONLY);
    if (fd < 0) {
        lldbg("not able to open resource file %d \n", fd);
        return -ENOENT;
    }

    /* Get file size */
    filesize = get_file_size(fd);
    if (filesize < 0) {
        close(fd);
        return -EINVAL;
    }

    lldbg("File size: %lld \n", (long long)filesize);

    /* Verify minimum file size for header */
    if (filesize < 6) {
        ret = -EINVAL;
        goto errout_with_fd;
    }

    /* Read entire file */
    buffer = (uint8_t *)malloc(filesize);
    if (buffer == NULL) {
        ret = -ENOMEM;
        goto errout_with_fd;
    }
    size_t bytes_read = read(fd, buffer, filesize);
    if (bytes_read != filesize) {
        ret = -EIO;
        goto errout_with_buffer;
    }

    dump_config(buffer);

    /* Verify magic header */
    uint32_t magic = read_uint32(buffer);
    if (magic != SYU645B_MAGIC_HEADER && magic != ALC1019_MAGIC_HEADER && magic != ALC5658_MAGIC_HEADER) {
        ret = -EINVAL;
        goto errout_with_buffer;
    }

    /* Allocate resource structure */
    g_codec_resource = (codec_resource_t *)malloc(sizeof(codec_resource_t));
    if (g_codec_resource == NULL) {
        ret = -ENOMEM;
        goto errout_with_buffer;
    }

    /* Initialize resource structure */
    memset(g_codec_resource, 0, sizeof(codec_resource_t));

    /* Parse header */
    g_codec_resource->magic = magic;
    g_codec_resource->script_count = buffer[4];
    g_codec_resource->addr_size = buffer[5];

    /* Validate script count */
    if (g_codec_resource->script_count == 0) {
        ret = -EINVAL;
        goto errout_with_resource;
    }

    /* Allocate scripts array */
    g_codec_resource->scripts = (codec_script_t *)malloc(g_codec_resource->script_count * sizeof(codec_script_t));
    if (g_codec_resource->scripts == NULL) {
        ret = -ENOMEM;
        goto errout_with_resource;
    }

    /* Initialize scripts array */
    memset(g_codec_resource->scripts, 0, g_codec_resource->script_count * sizeof(codec_script_t));

    /* Parse scripts */
    size_t offset = 6; /* Start after header, script count, and address size */
    for (i = 0; i < g_codec_resource->script_count; i++) {
        /* Check if we have enough data for script header */
        if (offset + 2 > filesize) {
            ret = -EINVAL;
            goto errout_with_scripts;
        }
        /* Parse script header */
        g_codec_resource->scripts[i].id = buffer[offset++];
        g_codec_resource->scripts[i].entry_count = buffer[offset++];

        /* Validate entry count */
        if (g_codec_resource->scripts[i].entry_count == 0) {
            ret = -EINVAL;
            goto errout_with_scripts;
        }

        /* Allocate entries array */
        g_codec_resource->scripts[i].entries = (script_entry_t *)malloc(g_codec_resource->scripts[i].entry_count * sizeof(script_entry_t));
        if (g_codec_resource->scripts[i].entries == NULL) {
            ret = -ENOMEM;
            goto errout_with_scripts;
        }

        /* Initialize entries array */
        memset(g_codec_resource->scripts[i].entries, 0, g_codec_resource->scripts[i].entry_count * sizeof(script_entry_t));

        /* Parse entries */
        for (j = 0; j < g_codec_resource->scripts[i].entry_count; j++) {
            /* Check if we have enough data for entry header */
            if (offset + g_codec_resource->addr_size + 4 > filesize) {
                ret = -EINVAL;
                goto errout_with_entries;
            }
            script_entry_t *entry = &g_codec_resource->scripts[i].entries[j];
            /* Parse entry header */
            entry->addr = (g_codec_resource->addr_size == 2) ? read_uint16(&buffer[offset]) : buffer[offset];
            offset += g_codec_resource->addr_size;
            entry->type = buffer[offset++];
            entry->delay = read_uint16(&buffer[offset]);
            offset += 2;
            /* Validate type */
            if (entry->type < 2 || entry->type > MAX_ENTRY_TYPE_SIZE) {
                ret = -EINVAL;
                goto errout_with_entries;
            }
            /* Check if we have enough data for values */
            if (offset + entry->type > filesize) {
                ret = -EINVAL;
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
    for (int k = 0; k <= i; k++) {
        if (g_codec_resource->scripts[k].entries != NULL) {
            kmm_free(g_codec_resource->scripts[k].entries);
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

/* Free resource */
void free_resource(void)
{
    if (g_codec_resource != NULL) {
        for (int i = 0; i < g_codec_resource->script_count; i++) {
            if (g_codec_resource->scripts[i].entries != NULL) {
                kmm_free(g_codec_resource->scripts[i].entries);
            }
        }
        if (g_codec_resource->scripts != NULL) {
            kmm_free(g_codec_resource->scripts);
        }
        kmm_free(g_codec_resource);
        g_codec_resource = NULL;
    }
}

/* Find script by ID */
codec_script_t *find_script(uint8_t script_id)
{
    if (g_codec_resource == NULL || g_codec_resource->scripts == NULL) {
        return NULL;
    }
    for (int i = 0; i < g_codec_resource->script_count; i++) {
        if (g_codec_resource->scripts[i].id == script_id) {
            return &g_codec_resource->scripts[i];
        }
    }
    return NULL;
}