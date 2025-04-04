  /****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <errno.h>

#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>

#include <tinyara/fs/mtd.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/

#define EB	(128 * 1024)

uint16_t buffer[1024];
extern struct mtd_dev_s * abhi_mtd;



void check_blocks(void)
{
	int ret;
	int block;

	for (block = 0; block < 2048; block++) {
		ret = MTD_ISBAD(abhi_mtd, block);
		if (ret != OK) {
			printf("BAD BLOCK %d\n", block);
			continue;
		}
		
		// else{
		// 	ret = MTD_ERASE(abhi_mtd, block, 1);
		// 	if (ret != 1) {
		// 		printf("ERASE ERROR %d\n", ret);
		// 		continue;
		// 	}
		// }

		// for (int j = 0; j < 2048; j++) {
		// 	buffer[j] = block;
		// }
		// /*Write to the first page in the block*/
		// ret = MTD_BWRITE(abhi_mtd, block * 64, 1, buffer);
		// if (ret != 1) printf("write ERROR %d\n", ret);
	}
	printf("check blocks Done!\n");
}

void write_blocks(void)
{
	int ret;
	int block;

	for (block = 0; block < 1024; block++) {
		ret = MTD_ISBAD(abhi_mtd, block);
		if (ret != OK) {
			printf("BAD BLOCK %d\n", block);
			continue;
		}
		else{
			ret = MTD_ERASE(abhi_mtd, block, 1);
			if (ret != 1) {
				printf("ERASE ERROR %d\n", ret);
				continue;
			}
		}

		for (int j = 0; j < 1024; j++) {
			buffer[j] = block;
		}
		/*Write to the first page in the block*/
		ret = MTD_BWRITE(abhi_mtd, block * 64, 1, (uint8_t*)buffer);
		if (ret != 1) printf("write ERROR %d\n", ret);
	}
	printf("Write blocks Done!\n");
}

void Read_blocks(void)
{
	int block = 0;
	int ret;

	for (block = 0; block < 2048; block++) {
		ret = MTD_ISBAD(abhi_mtd, block);
		if (ret != OK) {
			printf("BAD BLOCK %d\n", block);
			continue;
		}
		else {
			// read first page of every block
			ret = MTD_BREAD(abhi_mtd, block * 64, 1, (uint8_t*)buffer);
			printf("%u(%d) : ", block, ret);

			for (int j = 0; j < 1024; j++) {
				if (buffer[j] != block)
					printf("%u(%u) ", buffer[j], j);
			}
			printf("\n");
		}
	}
}

 void check_read1(int block)
 {
	int ret;
	if((block < 0) || (block > 2048))
	{
		printf("Invalid block input received \n");
	} else {
		ret = MTD_BREAD(abhi_mtd, block * 64, 1, (uint8_t*)buffer);
		printf("%u(%d) : \n", block, ret);
		for (int j = 0; j < 1024; j++) {
	
			printf("%u(%u) ", buffer[j], j);
			// if (buffer[j] == block)
			// {
			// 	printf("%u(%u) ", buffer[j], j);
			// }
			if(j%16 == 0)
			printf("\n");
		}
	}
}

void show_help(void) {
    printf("Download test - Available Commands:\n");
    printf("========================================\n");
    printf("    check_blocks       - scan the complete for bad blocks\n");
    printf("    write_blocks      - erase & write only the first page with block number \n");
    printf("    Read_blocks       - Read the first page of block & only prints the mismatch content \n");
    printf("    exit            - Exit the shell\n");
    printf("========================================\n");
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	int blocks = 0;

	printf("Hello, World!!\n");
	if(argc < 2)
	{
		show_help();
		return 0;
	}
	if (strcmp(argv[1], "check_blocks") == 0)
	{
		check_blocks();
	}
	else if(strcmp(argv[1], "write_blocks") == 0)
	{
		write_blocks();
	}
	else if(strcmp(argv[1], "Read_blocks") == 0)
	{
		Read_blocks();
	}
	else if(strcmp(argv[1], "test") == 0 )
	{
		blocks = (int)atoi(argv[2]);
		check_read1(blocks);
	}
	else
	{
		printf("Invalid command. Type `hello help` for usage.\n");
	}

	// if (argc < 2) {
    //     show_help();
    //     return 0;
    // }
	// if (strcmp(argv[1], "check_blocks")== 0) {
	// 	check_blocks();
    // } else if (strcmp(argv[1], "write_blocks") == 0 ) {
	// 	write_blocks();
    // } else if (strcmp(argv[1], "Read_blocks") == 0) {
	// 	Read_blocks();
    // }
    // else if (strcmp(argv[1], "help") == 0) {
    //     show_help();
    // } else {
    //     printf("Invalid command. Type `hello help` for usage.\n");
    // }

	// extern struct mtd_dev_s * abhi_mtd;
	return 0;

#if 0
#if 0
	block = 0;

	for (int i = 0; i < 256; i++) {
		
		int ret;
	       
		while (true) {
			ret = MTD_ISBAD(abhi_mtd, block);

			if (ret != OK) {
				printf("BAD BLOCK %d\n", block);
				block++;
				continue;
			} else {
				break;
			}
		}

		ret = MTD_ERASE(abhi_mtd, block, 1);

                if (ret != 1) printf("ERASE ERROR %d\n", ret);
#if 1
		for (int j = 0; j < 2048; j++) {
			buffer[j] = i;
		}
		
		/*Write to the first page in the block*/
		ret = MTD_BWRITE(abhi_mtd, block * 64, 1, buffer);
		if (ret != 1) printf("write ERROR %d\n", ret);
		block++;
#endif
	}
#endif
	block = 0;
	for (int i = 0; i < 256; i++) {
		// read first page of every block
		
		int ret;
	       
		while (true) {
                        ret = MTD_ISBAD(abhi_mtd, block);

                        if (ret != OK) {
                                printf("BAD BLOCK %d\n", block);
                                block++;
                                continue;
                        } else {
                                break;
                        }
                }

		ret = MTD_BREAD(abhi_mtd, block * 64, 1, buffer);
			
		printf("%u(%d) : ", block, ret);	

		for (int j = 0; j < 2048; j++) {
			if (buffer[j] != i)
				printf("%u(%u) ", buffer[j], j);
		}

		printf("\n");
		block++;
	}

	return 0;
#if 0
	int read_fd;
	int write_fd;
	int ret;
	int total_size;
	int copy_size;
	int read_size;
	uint32_t crc_hash = 0;
	
	char * mydata = malloc(sizeof(char) * EB);

	if (mydata == NULL) {
		printf("alloc failed\n");
		return ERROR;
	}	

	read_fd = open("/mnt0/romfs", O_RDONLY);

	if (read_fd < 0) {
		printf("Failed to open %s: errno %d\n", "/mnt0/romfs", get_errno());
		return ERROR;
	}

	write_fd = open("/dev/mtdblock10", O_WRONLY);
	if (write_fd < 0) {
		printf("Failed to open %s: errno %d\n", "mtdblock9", get_errno());
		return ERROR;
	}
#if 0
	for (int i = 0; i < 2048; i++) mydata[i] = 0xA;

	ret = write(write_fd, mydata, 2048);
	if (ret != 2048) {
		printf("Failed to write buffer : %d\n", ret);
		return ERROR;
	} else {
		printf("write is done\n");
	}
#endif

	do {
		ret = read(read_fd, mydata, EB);

		if (ret > 0) {
			int ret2 = write(write_fd, mydata, ret);
			if (ret2 != ret) {
				printf("write failed\n", ret2);
			}
			printf("transfered %d bytes\n", ret);
		} else {
			printf("read failed ret : %d\n", ret);
		}

	} while (ret > 0);

	close(write_fd);

	return 0;
#endif
#endif

}
