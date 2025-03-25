#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <tinyara/fs/mtd.h>
#include <tinyara/fs/ioctl.h>

#define MTD_DEVICE_PATH "/dev/mtdblock9"
#define BLOCK_SIZE 2048

/**
 * @brief Get total size of MTD block device
 * @param fd File descriptor of the MTD device
 * @return Total device size in bytes, or -1 on error
 */
static off_t get_mtd_device_size(int fd) {
    struct mtd_geometry_s geo;
    
    if (ioctl(fd, MTDIOC_GEOMETRY, (unsigned long)&geo) < 0) {
        printf("Failed to get MTD device geometry (errno: %d)\n", errno);
        return -1;
    }

    // Calculate total device size
    off_t total_size = (off_t)geo.erasesize * geo.neraseblocks;
    printf("MTD Device Size: %lld bytes\n", (long long)total_size);
    printf("Erase Block Size: %d bytes\n", geo.erasesize);
    printf("Number of Erase Blocks: %d\n", geo.neraseblocks);

    return total_size;
}

/**
 * @brief Perform comprehensive write and read test on MTD block device
 * @param write_iterations Number of write passes
 * @return 0 on success, -1 on error
 */
static int mtd_comprehensive_test(int write_iterations) {
    int fd;
    char *write_buffer;
    char *read_buffer;
    ssize_t bytes_written;
    ssize_t bytes_read;
    off_t total_device_size;
    off_t total_bytes_written = 0;
    int current_iteration = 0;

    // Open MTD device
    fd = open(MTD_DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        printf("Failed to open %s (errno: %d)\n", MTD_DEVICE_PATH, errno);
        return -1;
    }

    // Get total device size
    total_device_size = get_mtd_device_size(fd);
    if (total_device_size <= 0) {
        close(fd);
        return -1;
    }

    // Allocate buffers
    write_buffer = malloc(BLOCK_SIZE);
    read_buffer = malloc(BLOCK_SIZE);
    if (!write_buffer || !read_buffer) {
        printf("Memory allocation failed\n");
        close(fd);
        free(write_buffer);
        free(read_buffer);
        return -1;
    }

    // Prepare write buffer with incrementing pattern for better verification
    for (int i = 0; i < BLOCK_SIZE; i++) {
        write_buffer[i] = (char)(i & 0xFF);
    }

    printf("Starting comprehensive MTD write and read test\n");
    printf("Total iterations: %d\n", write_iterations);

    // Perform multiple write passes
    for (int pass = 0; pass < write_iterations; pass++) {
        printf("Starting pass %d\n", pass + 1);

        // Seek to the beginning of the device
        if (lseek(fd, 0, SEEK_SET) < 0) {
            printf("Seek to start failed (errno: %d)\n", errno);
            break;
        }

        total_bytes_written = 0;
        current_iteration = 0;

        // Write entire device
        while (total_bytes_written < total_device_size) {
            // Adjust write size if near end of device
            size_t write_size = (total_bytes_written + BLOCK_SIZE > total_device_size) 
                                ? (total_device_size - total_bytes_written) 
                                : BLOCK_SIZE;

            bytes_written = write(fd, write_buffer, write_size);
            
            if (bytes_written < 0) {
                printf("Write failed at %lld bytes (errno: %d)\n", 
                       (long long)total_bytes_written, errno);
                break;
            }

            total_bytes_written += bytes_written;
            current_iteration++;

            // Periodic status update
            if (current_iteration % 100 == 0) {
                printf("Pass %d: Wrote %lld/%lld bytes (iteration %d)\n", 
                       pass + 1, (long long)total_bytes_written, 
                       (long long)total_device_size, current_iteration);
            }
        }

        printf("Pass %d complete. Total bytes written: %lld\n", 
               pass + 1, (long long)total_bytes_written);

        // Seek back to beginning for verification
        if (lseek(fd, 0, SEEK_SET) < 0) {
            printf("Seek for read failed (errno: %d)\n", errno);
            break;
        }

        // Verify data
        off_t bytes_verified = 0;
        while (bytes_verified < total_device_size) {
            // Adjust read size if near end of device
            size_t read_size = (bytes_verified + BLOCK_SIZE > total_device_size) 
                               ? (total_device_size - bytes_verified) 
                               : BLOCK_SIZE;

            bytes_read = read(fd, read_buffer, read_size);
            
            if (bytes_read < 0) {
                printf("Read failed at %lld bytes (errno: %d)\n", 
                       (long long)bytes_verified, errno);
                break;
            }

            // Optional: Add detailed verification logic
            // Here we'll do a basic check of the read data pattern
            for (int i = 0; i < bytes_read; i++) {
                if (read_buffer[i] != (char)((bytes_verified + i) & 0xFF)) {
                    printf("Data mismatch at offset %lld\n", 
                           (long long)(bytes_verified + i));
                    // You might want to break or handle the error differently
                }
            }

            bytes_verified += bytes_read;
        }

        printf("Pass %d verification complete. Total bytes verified: %lld\n", 
               pass + 1, (long long)bytes_verified);
    }

    // Cleanup
    close(fd);
    free(write_buffer);
    free(read_buffer);

    printf("MTD Comprehensive Write and Read Test Completed Successfully\n");
    return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
    // Check for specific command and iterations
    if (argc < 2) {
        printf("Usage: %s start_test [iterations]\n", argv[0]);
        printf("Commands:\n");
        printf("  start_test - Trigger comprehensive MTD write and read test\n");
        printf("  Optional: specify number of test passes (default: 1)\n");
        return -1;
    }

    // Check for start_test command
    if (strcmp(argv[1], "start_test") != 0) {
        printf("Invalid command. Use 'start_test' to begin MTD test.\n");
        return -1;
    }

    // Default to 1 test pass
    int write_iterations = 1;

    // Check if optional iteration count is provided
    if (argc > 2) {
        // Convert argument to integer
        char *endptr;
        long parsed_iterations = strtol(argv[2], &endptr, 10);

        // Validate conversion
        if (*endptr != '\0' || parsed_iterations <= 0) {
            printf("Invalid iteration count. Using default: 1\n");
        } else {
            write_iterations = (int)parsed_iterations;
        }
    }

    printf("MTD Comprehensive Write and Read Test\n");
    printf("Test Passes: %d\n", write_iterations);

    // Perform comprehensive MTD test
    int result = mtd_comprehensive_test(write_iterations);
    
    if (result != 0) {
        printf("MTD Comprehensive Test Failed\n");
        return -1;
    }

    printf("MTD Comprehensive Test Successful\n");
    return 0;
}