/******************************************************************************
 * Copyright (c) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *****************************************************************************/

#include <cerrno>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "platform.h"
#include "resource_mgr.h"

/*
The inputs to the test application can be controlled 
by the following file. User need to create this file in
pdk_#_##_##_##\packages\ti\platform\lib\platform_utils_test\bin
directory. 

The syntax to the input file is as follows
<input string> = <value 1 => enable, 0 => disable
*/

char    input_file_name[] = "../testconfig/platform_test_input.txt";
#define MAX_LINE_LENGTH 40

/* The input strings for the input config file is given below */
/* Input flags */
#define INIT_PLL1             "init_pll"
#define INIT_DDR3             "init_ddr"
#define INIT_UART             "init_uart"
#define INIT_TCSL             "init_tcsl"
#define INIT_PHY              "init_phy"
#define INIT_ECC              "init_ecc"

#define PRINT_INFO            "print_info"
#define PRINT_CURRENT_CORE_ID "print_current_core_id"
#define PRINT_SWITCH_STATE    "print_switch_state"
#define TEST_EEPROM           "test_eeprom"
#define TEST_NAND             "test_nand"
#define TEST_NOR              "test_nor"
#define TEST_LED              "test_led"
#define TEST_UART             "test_uart"
#define TEST_EXT_MEM          "run_external_memory_test"
#define TEST_INT_MEM          "run_internal_memory_test"

/* Input parameters */
#define INIT_CONFIG_PLL1_PLLM      "init_config_pll1_pllm"
#define INIT_CONFIG_UART_BAUDRATE  "init_config_uart_baudrate"

#define NAND_TEST_BLOCK_NUMBER     "nand_test_block_number"

#define EEPROM_TEST_SLAVE_ADDRESS  "eeprom_test_slave_address"

#define LED_TEST_LOOP_COUNT        "led_test_loop_count"
#define LED_TEST_LOOP_DELAY        "led_test_loop_delay"

#define EXT_MEM_TEST_BASE_ADDR     "ext_mem_test_base_addr"
#define EXT_MEM_TEST_LENGTH        "ext_mem_test_length"

#define INT_MEM_TEST_CORE_ID       "int_mem_test_core_id"

/* The default values for the input strings are given below */
#define DEF_INIT_CONFIG_PLL1_PLLM      0
#define DEF_INIT_CONFIG_UART_BAUDRATE  0

#define DEF_NAND_TEST_BLOCK_NUMBER    1000

#define DEF_NOR_TEST_SECTOR_NUMBER    10

#define DEF_EEPROM_TEST_SLAVE_ADDRESS 0x51

#define DEF_LED_TEST_LOOP_COUNT    1
#define DEF_LED_TEST_LOOP_DELAY    3000000

#define DEF_EXT_MEM_TEST_BASE_ADDR 0x80000000
#define DEF_EXT_MEM_TEST_LENGTH    0x20000000

#define DEF_INT_MEM_TEST_CORE_ID   1

#undef TEST_READ_IMAGE

#ifdef TEST_READ_IMAGE
#define IMAGE_STORE_ADDR        0x80000000
#define READ_EEPROM_IMAGE       1
#define READ_NAND_OOB_IMAGE     1
#endif

typedef struct test_config {
    uint8_t print_info;
    uint8_t print_current_core_id;
    uint8_t print_switch_state;
    uint8_t test_eeprom;
    uint8_t test_nand;
    uint8_t test_nor;
    uint8_t test_led;
    uint8_t test_uart;
    uint8_t test_external_mem;
    uint8_t test_internal_mem;

    uint32_t init_config_pll1_pllm;
    uint32_t init_config_uart_baudrate;
    uint32_t nand_test_block_number;
    uint32_t nor_test_sector_number;
    uint32_t eeprom_test_slave_address;
    uint32_t led_test_loop_count;
    uint32_t led_test_loop_delay;
    uint32_t ext_mem_test_base_addr;
    uint32_t ext_mem_test_length;
    uint32_t int_mem_test_core_id;
} test_config;

#define CHECK_SET_FLAG(ARG1, ARG2)     \
    do { \
        if(strcmp(key, ARG1) == 0) { \
            if((strcmp(data, "1") == 0) || \
                    (strcmp(data, "y") == 0) || \
                    (strcmp(data, "yes") == 0) || \
                    (strcmp(data, "true") == 0)){ \
                ARG2 = 1; \
            } else { \
                ARG2 = 0; \
            } \
            continue; \
        } \
    } while(0)

#define CHECK_SET_PARAM(ARG1, ARG2)     \
    do { \
        if(strcmp(key, ARG1) == 0) { \
            errno = 0; \
            temp = (uint32_t) strtol(data, &ep, 0); \
            if (errno == 0 && ep != data) { \
                ARG2 = temp; \
            } \
            continue; \
        } \
    } while(0)

#define PRINT_VARIABLE(F, VAR) platform_write(#VAR"\t= "#F"\n", VAR)

/* OSAL functions for Platform Library */
uint8_t *Osal_platformMalloc (uint32_t num_bytes, uint32_t alignment)
{
	return malloc(num_bytes);
}

void Osal_platformFree (uint8_t *dataPtr, uint32_t num_bytes)
{
    /* Free up the memory */
    if (dataPtr)
    {
        free(dataPtr);
    }
}

void Osal_platformSpiCsEnter(void)
{
    /* Get the hardware semaphore.
     *
     * Acquire Multi core CPPI synchronization lock
     */
    while ((CSL_semAcquireDirect (PLATFORM_SPI_HW_SEM)) == 0);

    return;
}

void Osal_platformSpiCsExit (void)
{
    /* Release the hardware semaphore
     *
     * Release multi-core lock.
     */
    CSL_semReleaseSemaphore (PLATFORM_SPI_HW_SEM);

    return;
}

/******************************************************************************
 * Function:    parse_input_file  
 ******************************************************************************/
static inline void parse_input_file(FILE * fpr, test_config *args, 
        platform_init_flags *flags)
{
    char line[MAX_LINE_LENGTH + 1];
    char tokens[] = " :=;\n";
    char *key, *data, *ep;
    uint32_t temp;

    memset(line, 0, MAX_LINE_LENGTH + 1);

    while (fgets(line, MAX_LINE_LENGTH + 1, fpr)) {
        key  = (char *)strtok(line, tokens);
        data = (char *)strtok(NULL, tokens);

        if(strlen(data) == 0) {
            continue;
        }

        CHECK_SET_FLAG(INIT_PLL1, flags->pll);
        CHECK_SET_FLAG(INIT_DDR3, flags->ddr);
        CHECK_SET_FLAG(INIT_TCSL, flags->tcsl);
        CHECK_SET_FLAG(INIT_PHY,  flags->phy);
        CHECK_SET_FLAG(INIT_ECC,  flags->ecc);

        CHECK_SET_FLAG(PRINT_INFO, args->print_info);
        CHECK_SET_FLAG(PRINT_CURRENT_CORE_ID, args->print_current_core_id);
        CHECK_SET_FLAG(PRINT_SWITCH_STATE, args->print_switch_state);
        CHECK_SET_FLAG(TEST_EEPROM, args->test_eeprom);
        CHECK_SET_FLAG(TEST_NAND, args->test_nand);
        CHECK_SET_FLAG(TEST_NOR, args->test_nor);
        CHECK_SET_FLAG(TEST_LED, args->test_led);
        CHECK_SET_FLAG(TEST_UART, args->test_uart);
        CHECK_SET_FLAG(TEST_EXT_MEM, args->test_external_mem);
        CHECK_SET_FLAG(TEST_INT_MEM, args->test_internal_mem);

        CHECK_SET_PARAM(INIT_CONFIG_PLL1_PLLM, args->init_config_pll1_pllm);
        CHECK_SET_PARAM(INIT_CONFIG_UART_BAUDRATE, args->init_config_uart_baudrate);
        CHECK_SET_PARAM(NAND_TEST_BLOCK_NUMBER, args->nand_test_block_number);
        CHECK_SET_PARAM(EEPROM_TEST_SLAVE_ADDRESS, args->eeprom_test_slave_address);
        CHECK_SET_PARAM(LED_TEST_LOOP_COUNT, args->led_test_loop_count);
        CHECK_SET_PARAM(LED_TEST_LOOP_DELAY, args->led_test_loop_delay);
        CHECK_SET_PARAM(EXT_MEM_TEST_BASE_ADDR, args->ext_mem_test_base_addr);
        CHECK_SET_PARAM(EXT_MEM_TEST_LENGTH, args->ext_mem_test_length);
        CHECK_SET_PARAM(INT_MEM_TEST_CORE_ID, args->int_mem_test_core_id);
    }
}

/******************************************************************************
 * Function:    print_current_core_id  
 ******************************************************************************/
static void print_current_core_id(void)
{
    platform_write("Current core id is %d\n", platform_get_coreid());
}

/******************************************************************************
 * Function:    print_platform_info  
 ******************************************************************************/
static void print_platform_info(platform_info * p_info, test_config * args)
{
	uint32_t	i;

	PLATFORM_DEVICE_info *p_device;

    PLATFORM_EMAC_EXT_info emac_info;

    PRINT_VARIABLE(%s, p_info->version);

    PRINT_VARIABLE(%d, p_info->cpu.core_count);
    PRINT_VARIABLE(%s, p_info->cpu.name);
    PRINT_VARIABLE(%d, p_info->cpu.id);
    PRINT_VARIABLE(%d, p_info->cpu.revision_id);
    PRINT_VARIABLE(%d, p_info->cpu.silicon_revision_major);
    PRINT_VARIABLE(%d, p_info->cpu.silicon_revision_minor);
    PRINT_VARIABLE(%d, p_info->cpu.megamodule_revision_major);
    PRINT_VARIABLE(%d, p_info->cpu.megamodule_revision_minor);
    PRINT_VARIABLE(%d, p_info->cpu.endian);

    PRINT_VARIABLE(%s, p_info->board_name);
    PRINT_VARIABLE(%d, p_info->frequency);
    PRINT_VARIABLE(%d, p_info->board_rev);

    PRINT_VARIABLE(%d, p_info->led[PLATFORM_USER_LED_CLASS].count);
    PRINT_VARIABLE(%d, p_info->led[PLATFORM_SYSTEM_LED_CLASS].count);

    PRINT_VARIABLE(%d, p_info->emac.port_count);
    for (i = 0; i < PLATFORM_MAX_EMAC_PORT_NUM; i++)
    {
        platform_get_emac_info (i, &emac_info);
        if (emac_info.mode == PLATFORM_EMAC_PORT_MODE_PHY)
        {
            platform_write("EMAC port %d connected to the PHY.\n", emac_info.port_num);
            platform_write("MAC Address = %02x:%02x:%02x:%02x:%02x:%02x\n",
            		emac_info.mac_address[0], emac_info.mac_address[1],
            		emac_info.mac_address[2], emac_info.mac_address[3],
            		emac_info.mac_address[4], emac_info.mac_address[5]);
        }
    }

    if(args->test_nand) {
		p_device = platform_device_open(PLATFORM_DEVID_NAND512R3A2D, 0);

		if (p_device) {
			platform_write("\nNAND Device: \n");
			PRINT_VARIABLE(%d, p_device->device_id);
			PRINT_VARIABLE(%d, p_device->manufacturer_id);
			PRINT_VARIABLE(%d, p_device->width);
			PRINT_VARIABLE(%d, p_device->block_count);
			PRINT_VARIABLE(%d, p_device->page_count);
			PRINT_VARIABLE(%d, p_device->page_size);
			PRINT_VARIABLE(%d, p_device->spare_size);
			PRINT_VARIABLE(%d, p_device->column);
			PRINT_VARIABLE(%d, p_device->handle);
			PRINT_VARIABLE(%d, p_device->flags);
			PRINT_VARIABLE(%d, p_device->bboffset);
			if (p_device->bblist) {
				platform_write("Bad Block Table (only bad block numbers shown): \n");
				for (i=0; i < p_device->block_count; i++) {
					if (p_device->bblist[i] == 0x00) {
						platform_write("%d \n", i);
					}
				}
			}
			platform_device_close(p_device->handle);
		}
		else {
			platform_write("Could not open the NAND device errno = 0x%x\n", platform_errno);
		}
    }

    if(args->test_nor) {

		p_device = platform_device_open(PLATFORM_DEVID_NORN25Q128, 0);

		if (p_device) {
			platform_write("\nNOR Device: \n");
			PRINT_VARIABLE(%d, p_device->device_id);
			PRINT_VARIABLE(%d, p_device->manufacturer_id);
			PRINT_VARIABLE(%d, p_device->width);
			PRINT_VARIABLE(%d, p_device->block_count);
			PRINT_VARIABLE(%d, p_device->page_count);
			PRINT_VARIABLE(%d, p_device->page_size);
			PRINT_VARIABLE(%d, p_device->spare_size);
			PRINT_VARIABLE(%d, p_device->handle);
			PRINT_VARIABLE(%d, p_device->flags);
			PRINT_VARIABLE(%d, p_device->bboffset);
			if (p_device->bblist) {
				platform_write("Bad Block Table (only bad block numbers shown): \n");
				for (i=0; i < p_device->block_count; i++) {
					if (p_device->bblist[i] == 0x00) {
						platform_write("%d ", i);
					}
				}
			}
			platform_device_close(p_device->handle);
		}
		else {
			platform_write("Could not open the NOR device errno = 0x%x\n", platform_errno);
		}
    }

    if(args->test_eeprom) {

		p_device = platform_device_open(PLATFORM_DEVID_EEPROM50, 0);

		if (p_device) {
			platform_write("\nEEPROM Device (@ 0x50): \n");
			PRINT_VARIABLE(%d, p_device->device_id);
			PRINT_VARIABLE(%d, p_device->manufacturer_id);
			PRINT_VARIABLE(%d, p_device->width);
			PRINT_VARIABLE(%d, p_device->block_count);
			PRINT_VARIABLE(%d, p_device->page_count);
			PRINT_VARIABLE(%d, p_device->page_size);
			PRINT_VARIABLE(%d, p_device->spare_size);
			PRINT_VARIABLE(%d, p_device->handle);
			PRINT_VARIABLE(%d, p_device->flags);
			PRINT_VARIABLE(%d, p_device->bboffset);
			platform_device_close(p_device->handle);
		}
		else {
			platform_write("Could not open the EEPROM @50 device errno = 0x%x\n", platform_errno);
		}

		p_device = platform_device_open(PLATFORM_DEVID_EEPROM51, 0);

		if (p_device) {
			platform_write("\nEEPROM Device (@ 0x51): \n");
			PRINT_VARIABLE(%d, p_device->device_id);
			PRINT_VARIABLE(%d, p_device->manufacturer_id);
			PRINT_VARIABLE(%d, p_device->width);
			PRINT_VARIABLE(%d, p_device->block_count);
			PRINT_VARIABLE(%d, p_device->page_count);
			PRINT_VARIABLE(%d, p_device->page_size);
			PRINT_VARIABLE(%d, p_device->spare_size);
			PRINT_VARIABLE(%d, p_device->handle);
			PRINT_VARIABLE(%d, p_device->flags);
			PRINT_VARIABLE(%d, p_device->bboffset);
			platform_device_close(p_device->handle);
		}
		else {
			platform_write("Could not open the EEPROM @51 device errno = 0x%x\n", platform_errno);
		}
    }

    return;
}

/******************************************************************************
 * Function:    print_switch_state  
 ******************************************************************************/
static void print_switch_state(void)
{
    platform_write("User switch 1 state is %s\n",
            platform_get_switch_state(1) ? "ON" : "OFF");
}

/******************************************************************************
 * Function:    test_eeprom  
 ******************************************************************************/
static void test_eeprom(test_config * args)
{
    uint8_t   test_buf[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b};
    uint8_t * test_buf1;
    uint8_t * orig_buf;
	PLATFORM_DEVICE_info *p_device;


	if (args->eeprom_test_slave_address == 0x50) {
		p_device = platform_device_open(PLATFORM_DEVID_EEPROM50, 0);
	}
	else {
		p_device = platform_device_open(PLATFORM_DEVID_EEPROM51, 0);
	}

    if (p_device == NULL) {
       platform_write("test_eeprom: Could not open EEPROM device %x errno = 0x%x \n", args->eeprom_test_slave_address, platform_errno);
       return;
    }

    test_buf1 = malloc(12);
    orig_buf  = malloc(12);

    if(platform_device_read(p_device->handle, 0, orig_buf, 12) != Platform_EOK) {
        platform_write("test_eeprom: Read failed, errno = 0x%x\n", platform_errno);
        goto free;
    }

    if(platform_device_write(p_device->handle,0, test_buf, 12) != Platform_EOK) {
        platform_write("test_eeprom: Write test data failed, errno = 0x%x\n", platform_errno);
        goto free;
    }

    if(platform_device_read(p_device->handle, 0, test_buf1, 12) != Platform_EOK) {
        platform_write("test_eeprom: Read test data failed, errno = 0x%x\n",platform_errno);
        goto free;
    }

    if (memcmp(test_buf, test_buf1, 12) != 0) {
        platform_write("test_eeprom: Data verification failed\n");
        goto free;
    } else {
        platform_write("test_eeprom: passed\n");
    }

    if(platform_device_write(p_device->handle,0, orig_buf, 12) != Platform_EOK) {
        platform_write("test_eeprom: Write back original data failed, errno = 0x%x\n", platform_errno);
    }

free:
	platform_device_close(p_device->handle);
    free(test_buf1);
    free(orig_buf);
}

/******************************************************************************
 * Function:    test_nand  
 ******************************************************************************/
static void test_nand(test_config * args)
{
    uint8_t 				* buf1, * buf2, * buf_orig;
    /*uint16_t 				i, sp_flag = 0;*/
    uint32_t				offset;
	PLATFORM_DEVICE_info 	*p_device;


	buf1 = NULL; buf2 = NULL; buf_orig = NULL;

	/* Open the device */
    p_device = platform_device_open(PLATFORM_DEVID_NAND512R3A2D, 0);
    if (p_device == NULL) {
       platform_write("test_nand: Could not open NAND device  errno = 0x%x \n", platform_errno);
       goto NAND_TEST_DONE;
    }

    /* We need a buffer to read a page in */
    buf_orig = (uint8_t *) malloc(p_device->page_size);
    if (buf_orig == NULL) {
        platform_write("test_nand: Can't allocate %d bytes for buf_orig\n", p_device->page_size);
        goto NAND_TEST_DONE;
    }

    /* Read the test block page 0 - save the contents so we can write back */
    platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0 );
    if (platform_device_read(p_device->handle, offset, buf_orig, p_device->page_size) != Platform_EOK) {
        platform_write("test_nand: Unable to read block %d page %d errno = 0x%x\n", args->nand_test_block_number, 0, platform_errno );
        goto NAND_TEST_DONE;
    }

    /* Get a new buffer so we can test writing some values */
    buf1 = (uint8_t *) malloc(p_device->page_size);
    if (buf1 == NULL) {
        platform_write("test_nand: Can't allocate %d bytes for buf1\n", p_device->page_size);
        goto NAND_TEST_DONE;
    }

    /* Erase our test block so we can write to it */
    if (platform_device_erase_block(p_device->handle, args->nand_test_block_number) != Platform_EOK) {
        platform_write("test_nand: Unable to erase (%d) block %d errno = 0x%x\n", args->nand_test_block_number, platform_errno);
        goto NAND_TEST_DONE;
    }

    /* Fill in a test pattern */
    memset(buf1, 0x6d, p_device->page_size);

    /* Write the test pattern to the test block */
    platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0 );
    if (platform_device_write(p_device->handle, offset, buf1, p_device->page_size) != Platform_EOK) {
        platform_write("test_nand: Unable to write (%d) block %d page %d errno = 0x%x \n", args->nand_test_block_number, 0, platform_errno);
        goto NAND_TEST_DONE;
    }

    /* Get another buffer to read back in the test pattern so we can compare it */
    buf2 = (uint8_t *) malloc(p_device->page_size);
    if (buf2 == NULL) {
        platform_write("test_nand: Can't allocate %d bytes for buf2\n", p_device->page_size);
        /* save back the original page */
        platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0);
        platform_device_write(p_device->handle, offset, buf_orig, p_device->page_size);
        goto NAND_TEST_DONE;
    }

    memset(buf2, 0xff, p_device->page_size);
    platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0);
    if (platform_device_read(p_device->handle, offset, buf2, p_device->page_size) != Platform_EOK) {
        platform_write("test_nand: Unable to read block %d page %d errno = 0x%x\n", args->nand_test_block_number, 0, platform_errno );
        goto NAND_TEST_DONE;
    }

    /* Compare to see if what we wrote is what we read */
    if(memcmp(buf1, buf2, p_device->page_size))
    {
        platform_write("test_nand: failed for block %d. What we wrote is not what we read. \n", args->nand_test_block_number);
    } else {
        platform_write("test_nand: passed\n");
    }

    /* Restore the original buffer */
    if (platform_device_erase_block(p_device->handle, args->nand_test_block_number) != Platform_EOK) {
        platform_write("test_nand: Unable to erase block %d errno = 0x%x \n", args->nand_test_block_number,platform_errno);
        goto NAND_TEST_DONE;
    }
    platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0);
    platform_device_write(p_device->handle, offset, buf_orig, p_device->page_size);

#if 0
    /* Spare area Read test */
    platform_write ("test_nand: Testing the Spare Read functionality now\n");

    /* Save the original spare area */
    platform_device_read_spare_data(p_device->handle, args->nand_test_block_number, buf1);

    /* Create a new buffer to get a new ECC bytes */
    for (i = 0; i < p_device->page_size ; i++) {
    	buf2[i] = ~buf_orig[i];
    }

    /* Nand write a new page would put a different spare area bytes */
    platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0);
    platform_device_write(p_device->handle, offset, buf2, p_device->page_size);

    /* Read the new Spare area byets*/
    platform_device_read_spare_data(p_device->handle, args->nand_test_block_number, buf2);
    
    sp_flag = 1;
    for (i = 0; i < p_device->spare_size; i++) {
    	if ( buf1[i] != buf2[i] ) {
    		sp_flag = 0;         
    	}    	
    }
    if (sp_flag == 1) { /* same spare area could mean write is not OK or SP Read NOK */
    	platform_write (" test_nand: Spare Read might be Not OK\n");
    }

    /* Otherwise, write back the original buffer to get the original spare bytes*/
    platform_blocknpage_to_offset(p_device->handle, &offset, args->nand_test_block_number, 0);
    platform_device_write(p_device->handle, offset, buf_orig,  p_device->page_size);

    /* Read the new spare bytes */
    platform_device_read_spare_data(args->nand_test_block_number, 0, buf2);
    
    sp_flag = 1;
    for (i = 0; i < p_device->spare_size; i++) {
    	if ( buf1[i] != buf2[i] ) {
    		sp_flag = 0;         
    	}    	
    }
    if (sp_flag == 1) { /* Expecting same Spare Area as before to Pass this test */
    	platform_write (" test_nand: Spare area Read passed\n");
    }
    else {
    	platform_write (" test_nand: Spare area Read failed\n");
    }
#endif

NAND_TEST_DONE:

	if (buf1)
		free(buf1);
	if (buf2)
		free(buf2);
	if (buf_orig)
		free(buf_orig);

    platform_device_close(p_device->handle);

    return;
}

/******************************************************************************
 * Function:    test_nor 
 ******************************************************************************/
static void test_nor(test_config * args, platform_info * p_info)
{
    uint8_t * test_buf0;
    uint8_t * test_buf1;
    uint8_t * orig_buf;
    uint32_t  i;
    uint32_t  address;
	PLATFORM_DEVICE_info 	*p_device;

#define SPI_NOR_TEST_SIZE (64*1024)

    platform_init_flags   flags;
    platform_init_config  config;
    
    memset(&flags, 0, sizeof(platform_init_flags));
    memset(&config, 0, sizeof(platform_init_config));

    p_device = platform_device_open(PLATFORM_DEVID_NORN25Q128, 0);

    if (p_device == NULL) {
       platform_write("test_nand: Could not open NOR device  errno = 0x%x \n", platform_errno);
       return;
    }

    test_buf0 = malloc(SPI_NOR_TEST_SIZE);
    test_buf1 = malloc(SPI_NOR_TEST_SIZE);
    orig_buf  = malloc(SPI_NOR_TEST_SIZE);

    /* Initialize the test pattern */
    for (i=0; i < SPI_NOR_TEST_SIZE; i++) {
        test_buf0[i] ^= 0xAB;
    }

    address = args->nor_test_sector_number * 64 * 1024; /* each sector is 64kb */
    if(platform_device_read(p_device->handle, address, orig_buf, SPI_NOR_TEST_SIZE) != Platform_EOK) {
        platform_write("test_nor: Read failed errno = 0x%x\n", platform_errno);
        goto free;
    }

    if(platform_device_erase_block (p_device->handle, args->nor_test_sector_number) != Platform_EOK) {
    	platform_write("test_nor: Erase Failed for sector %d  errno = 0x%x\n", args->nor_test_sector_number, platform_errno);
    }
    	
    if(platform_device_read(p_device->handle, address, test_buf1, SPI_NOR_TEST_SIZE) != Platform_EOK) {
        platform_write("test_nor: Read test data failed errno = 0x%x\n", platform_errno);
    }
    
    if(platform_device_write(p_device->handle, address, test_buf0, SPI_NOR_TEST_SIZE) != Platform_EOK) {
        platform_write("test_nor: Write test data failed errno = 0x%x\n", platform_errno);
    }

    if(platform_device_read(p_device->handle, address, test_buf1, SPI_NOR_TEST_SIZE) != Platform_EOK) {
        platform_write("test_nor: Read test data failed errno = 0x%x\n", platform_errno);
    }

    if (memcmp(test_buf0, test_buf1, SPI_NOR_TEST_SIZE) != 0) {
        platform_write("test_nor: Data verification failed\n");
    } else {
        platform_write("test_nor: passed\n");
    }
    
    if(platform_device_erase_block (p_device->handle, args->nor_test_sector_number) != Platform_EOK) {
    	platform_write("test_nor: Erase Failed errno = 0x%x\n", platform_errno);
    }
    
    if(platform_device_write(p_device->handle, address, orig_buf, SPI_NOR_TEST_SIZE) != Platform_EOK) {
        platform_write("test_nor: Write back original data failed errno = 0x%x\n", platform_errno);
    }

free:
    free(test_buf0);
    free(test_buf1);
    free(orig_buf);

	platform_device_close(p_device->handle);

}

/******************************************************************************
 * Function:    test_led  
 ******************************************************************************/
static void test_led(test_config * args, platform_info * p_info)
{
    int32_t i,j;
    int32_t max_loop = args->led_test_loop_count;

    do {
        for (j = PLATFORM_USER_LED_CLASS; j < PLATFORM_END_LED_CLASS; j++) {;
            for (i = 0; i < p_info->led[j].count; i++) {
                platform_write("LED %d ON\n", i);
                platform_led(i, PLATFORM_LED_ON, (LED_CLASS_E)j);
                platform_delay(args->led_test_loop_delay);
                platform_write("LED %d OFF\n", i);
                platform_led(i, PLATFORM_LED_OFF, (LED_CLASS_E) j);
                platform_delay(args->led_test_loop_delay);                
                platform_write("LED %d ON\n", i);
                platform_led(i, PLATFORM_LED_ON, (LED_CLASS_E)j);
            }
        }
    } while (--max_loop);

    /* turn on all the LEDS (default state) after the test */
}

/******************************************************************************
 * Function:    test_internal_memory  
 ******************************************************************************/
static void test_internal_memory(test_config * args)
{
    if(platform_internal_memory_test(args->int_mem_test_core_id) != Platform_EOK) {
        platform_write("Internal memory test failed for core %d at address 0x%x \n",
            args->int_mem_test_core_id, platform_errno);
    } else {
        platform_write("Internal memory test (for core %d) passed\n",
            args->int_mem_test_core_id);
    }
}

/******************************************************************************
 * Function:    test_external_memory  
 ******************************************************************************/
static void test_external_memory(test_config * args)
{
    int32_t status;

    status = platform_external_memory_test(args->ext_mem_test_base_addr, 
            (args->ext_mem_test_base_addr + args->ext_mem_test_length));
    if (status == PLATFORM_ERRNO_INVALID_ARGUMENT) {
        platform_write("Invalid Arguments passed for the test\n");        
    } else if (status != Platform_EOK) {
        platform_write("External memory test failed at address = 0x%x \n", platform_errno);
    } else {
        platform_write("External memory test passed\n");
    }
}

/******************************************************************************
 * Function:    test_uart  
 ******************************************************************************/
static void test_uart(test_config * args)
{
    uint8_t 	message[] = "\r\nThis is a Platform UART API unit test ...\r\n";
    int 		length = strlen((char *)message);
    uint8_t 	buf;
    int 		i;
    WRITE_info 	setting;

    platform_uart_set_baudrate(args->init_config_uart_baudrate);
    
    /* Don't echo to the uart since we are testing on it */
    setting = platform_write_configure (PLATFORM_WRITE_PRINTF);

    platform_write("Open a serial port console in a PC connected to\n");
    platform_write("the board using UART and set its baudrate to %d\n", args->init_config_uart_baudrate);
    platform_write("You should see following message --- %s", message);
    platform_write("Type 10 characters in serial console\n");

    for (i = 0; i < length; i++) {
        platform_uart_write(message[i]);
    }
    
    for (i = 0; i < 10; i++) {
        platform_errno = PLATFORM_ERRNO_RESET;
        if (platform_uart_read(&buf, 30 * 10000000) == Platform_EOK) {
        	platform_write("Char %d = %c\n", i, buf);
        } else {
        	platform_write("Char %d = %c (errno: 0x%x)\n", i, buf, platform_errno);
        }
    }

    platform_write_configure (setting);
}

#if READ_EEPROM_IMAGE
#define MAX_EEPROM_SIZE     0x10000
uint32_t write_eeprom_image =   0;

void
read_eeprom_image(void)
{
    FILE                    *fp;
    char                    *eeprom50 = "eeprom50.bin";
    char                    *eeprom51 = "eeprom51.bin";
	PLATFORM_DEVICE_info    *p_device;
    uint8_t                 *buf;
    uint32_t                size;

    /* Read 64KB from EEPROM 0x50 and save it to eeprom50.bin */
    fp = fopen(eeprom50, "wb");
    if (fp == NULL)
    {
        printf("Error in opening %s file\n", eeprom50);
        return;
    }

    p_device = platform_device_open(PLATFORM_DEVID_EEPROM50, 0);
    if (p_device == NULL) 
    {
       platform_write("read_eeprom_image: Could not open EEPROM device 0x50 errno = 0x%x \n", platform_errno);
       return;
    }

    buf = (uint8_t *)IMAGE_STORE_ADDR;

    /* Read 64K bytes from EEPROM 0x50, and store in external memory */
    if(platform_device_read(p_device->handle, 0, buf, MAX_EEPROM_SIZE) != Platform_EOK) 
    {
        platform_write("read_eeprom_image: Read failed on 0x50, errno = 0x%x\n", platform_errno);
        platform_device_close(p_device->handle);
        return;
    }
    platform_device_close(p_device->handle);

    /* Write the data to a binary file */
    size = fwrite(buf, 1, MAX_EEPROM_SIZE, fp);
    if (size != MAX_EEPROM_SIZE)
    {
        platform_write("read_eeprom_image: write to binary failed\n");
    }
    fclose(fp);

    /* Read 64KB from EEPROM 0x51 and save it to eeprom51.bin */
    fp = fopen(eeprom51, "wb");
    if (fp == NULL)
    {
        printf("Error in opening %s file\n", eeprom51);
        return;
    }

    p_device = platform_device_open(PLATFORM_DEVID_EEPROM51, 0);
    if (p_device == NULL) 
    {
       platform_write("read_eeprom_image: Could not open EEPROM device 0x51 errno = 0x%x \n", platform_errno);
       return;
    }

    buf = (uint8_t *)(IMAGE_STORE_ADDR + MAX_EEPROM_SIZE);

    /* Read 64K bytes from EEPROM 0x51, and store in external memory */
    if(platform_device_read(p_device->handle, 0, buf, MAX_EEPROM_SIZE) != Platform_EOK) 
    {
        platform_write("read_eeprom_image: Read failed on 0x50, errno = 0x%x\n", platform_errno);
        platform_device_close(p_device->handle);
        return;
    }
    platform_device_close(p_device->handle);

    /* Write the data to a binary file */
    size = fwrite(buf, 1, MAX_EEPROM_SIZE, fp);
    if (size != MAX_EEPROM_SIZE)
    {
        platform_write("read_eeprom_image: write to binary failed\n");
    }
    fclose(fp);

    if (!write_eeprom_image)
    {
        return;
    }

    /* Read eeprom50.bin and write it to EEPROM 0x50 */
    fp = fopen(eeprom50, "rb");
    if (fp == NULL)
    {
        printf("Error in opening %s file\n", eeprom50);
        return;
    }

    buf = (uint8_t *)IMAGE_STORE_ADDR;
    size = fread(buf, 1, MAX_EEPROM_SIZE, fp);
    fclose(fp);
    if (size != MAX_EEPROM_SIZE)
    {
        platform_write("read_eeprom_image: read binary failed\n");
        return;
    }

    p_device = platform_device_open(PLATFORM_DEVID_EEPROM50, 0);
    if (p_device == NULL) 
    {
       platform_write("read_eeprom_image: Could not open EEPROM device 0x50 errno = 0x%x \n", platform_errno);
       return;
    }

    if(platform_device_write(p_device->handle, 0, buf, MAX_EEPROM_SIZE) != Platform_EOK) 
    {
        platform_write("read_eeprom_image: write failed on 0x50, errno = 0x%x\n", platform_errno);
        platform_device_close(p_device->handle);
        return;
    }

    platform_device_close(p_device->handle);

    /* Read eeprom51.bin and write it to EEPROM 0x51 */
    fp = fopen(eeprom51, "rb");
    if (fp == NULL)
    {
        printf("Error in opening %s file\n", eeprom51);
        return;
    }

    buf = (uint8_t *)(IMAGE_STORE_ADDR + MAX_EEPROM_SIZE);
    size = fread(buf, 1, MAX_EEPROM_SIZE, fp);
    fclose(fp);
    if (size != MAX_EEPROM_SIZE)
    {
        platform_write("read_eeprom_image: read binary failed\n");
        return;
    }

    p_device = platform_device_open(PLATFORM_DEVID_EEPROM51, 0);
    if (p_device == NULL)
    {
       platform_write("read_eeprom_image: Could not open EEPROM device 0x51 errno = 0x%x \n", platform_errno);
       return;
    }

    if(platform_device_write(p_device->handle, 0, buf, MAX_EEPROM_SIZE) != Platform_EOK)
    {
        platform_write("read_eeprom_image: write failed on 0x51, errno = 0x%x\n", platform_errno);
    }

    platform_device_close(p_device->handle);

    return;
}
#endif


#if READ_NAND_OOB_IMAGE

#define NAND_READ_START_OFFSET (16384) /* start byte address of block 1 */
#define NAND_READ_SIZE 0xA00000

void
read_nand_oob_image(void)
{
    FILE                    *fp = 0;
    char                    *nand_oob = "nand_oob.bin";
    uint8_t                 *buf, *buf2;
    PLATFORM_DEVICE_info    *p_device = 0;
    uint32_t start_block, end_block;
    uint32_t start_page, end_page;
    uint32_t offset_start, offset_end;
    uint32_t i, j, write_idx;
    uint32_t size;

    fp = fopen(nand_oob, "wb");
    if (fp == NULL)
    {
        printf("Error in opening %s file\n", nand_oob);
        return;
    }

	/* Open the device */
    p_device = platform_device_open(PLATFORM_DEVID_NAND512R3A2D, 0);
    if (p_device == NULL) {
       platform_write("read_nand_oob_image: Could not open NAND device  errno = 0x%x \n", platform_errno);
       goto NAND_TEST_DONE;
    }

    buf = (uint8_t *) IMAGE_STORE_ADDR;

    offset_start = NAND_READ_START_OFFSET;
    offset_end   = NAND_READ_START_OFFSET + NAND_READ_SIZE;

    platform_offset_to_blocknpage(p_device->handle, offset_start, &start_block, &start_page);
    platform_offset_to_blocknpage(p_device->handle, offset_end, &end_block, &end_page);

    write_idx = 0;
    for (i = start_block; i < end_block; i++) 
    {
        while (p_device->bblist[i] != 0xff)
        {
            i++;
            end_block++;
        }

        for (j = 0; j < p_device->page_count; j++) {
        	if(platform_device_read_spare_data (p_device->handle, i, j, &buf[write_idx]) != Platform_EOK) {
        		platform_write("read_nand_oob_image: Spare read failed on NAND, errno = 0x%x\n", platform_errno);
                goto NAND_TEST_DONE;
        	}
            write_idx += p_device->spare_size;
        }

        /* Write the data to a binary file */
        size = fwrite(&buf[write_idx-(p_device->spare_size*p_device->page_count)], 1, p_device->spare_size * p_device->page_count, fp);
        if (size != (p_device->spare_size * p_device->page_count))
        {
            platform_write("read_nand_oob_image: write to binary (spare) failed\n");
        }
    }

NAND_TEST_DONE:

	if (p_device) platform_device_close(p_device->handle);
    if(fp) fclose(fp);

    fp = fopen(nand_oob, "rb");
    if (fp == NULL)
    {
    	platform_write("Error in opening %s file\n", nand_oob);
        return;
    }

    buf2 = (uint8_t *) (IMAGE_STORE_ADDR + 0x100000);
    size = fread(buf2, 1, write_idx, fp);
    fclose(fp);
    if (size != MAX_EEPROM_SIZE)
    {
        platform_write("read_nand_oob_image: read binary failed\n");
        return;
    }
    for (i=0; i<write_idx; i++)
    {
    	if(buf[i] != buf2[i])
        {
            platform_write("read_nand_oob_image: memory compare failed\n");
            return;
        }
    }

}
#endif

/******************************************************************************
 * Function:    main  
 ******************************************************************************/
void main (int argc, char *argv[])
{
    platform_init_flags  init_flags;
    platform_init_config init_config;
    platform_info        p_info;
    FILE * fpr = NULL;
    test_config args;

    /* Set default values */
    memset(&args, 0x01, sizeof(test_config));
    memset(&init_flags, 0x01, sizeof(platform_init_flags));
    args.init_config_pll1_pllm     = DEF_INIT_CONFIG_PLL1_PLLM;
    args.init_config_uart_baudrate = DEF_INIT_CONFIG_UART_BAUDRATE;
    args.nand_test_block_number    = DEF_NAND_TEST_BLOCK_NUMBER;
    args.nor_test_sector_number    = DEF_NOR_TEST_SECTOR_NUMBER;
    args.eeprom_test_slave_address = DEF_EEPROM_TEST_SLAVE_ADDRESS;
    args.led_test_loop_count       = DEF_LED_TEST_LOOP_COUNT;
    args.led_test_loop_delay       = DEF_LED_TEST_LOOP_DELAY;
    args.ext_mem_test_base_addr    = DEF_EXT_MEM_TEST_BASE_ADDR;
    args.ext_mem_test_length       = DEF_EXT_MEM_TEST_LENGTH;
    args.int_mem_test_core_id      = DEF_INT_MEM_TEST_CORE_ID;

    if (argc > 1) {
        fpr = fopen(argv[1], "r");
        if (fpr == NULL) {
            printf("Error in opening %s input file\n", argv[1]);
        }
    } else {
        fpr = fopen(input_file_name, "r");
        if (fpr == NULL) {
            printf("Error in opening %s input file\n", input_file_name);
        }
    }

    if (fpr) {
        parse_input_file(fpr, &args, &init_flags);
    }

    fclose(fpr);

    init_config.pllm    = args.init_config_pll1_pllm;

    if (platform_init(&init_flags, &init_config) != Platform_EOK) {
    	printf("Platform failed to initialize, errno = 0x%x \n", platform_errno);
    }

    platform_write_configure(PLATFORM_WRITE_PRINTF);
    platform_uart_init();
    platform_uart_set_baudrate(args.init_config_uart_baudrate);

    platform_get_info(&p_info);

    if(args.print_info) {
        print_platform_info(&p_info, &args);
    }

    if(args.print_current_core_id) {
        print_current_core_id();
    }

    if(args.print_switch_state) {
        print_switch_state();
    }

    if(args.test_uart) {
        platform_write("UART test start\n");
        test_uart(&args);
        platform_write("UART test complete\n");
    }

    if(args.test_eeprom) {
        platform_write("EEPROM test start\n");
        test_eeprom(&args);
        platform_write("EEPROM test complete\n");
    }

    if(args.test_nand) {
        platform_write("NAND test start\n");
        test_nand(&args);
        platform_write("NAND test complete\n");
    }

    if(args.test_nor) {
        platform_write("NOR test start\n");
        test_nor(&args, &p_info);
        platform_write("NOR test complete\n");
    }

    if(args.test_led) {
        platform_write("LED test start\n");
        test_led(&args, &p_info);
        platform_write("LED test complete\n");
    }

    if(args.test_internal_mem) {
        platform_write("Internal memory test start\n");
        test_internal_memory(&args);
        platform_write("Internal memory test complete\n");
    }

    if(args.test_external_mem) {
        platform_write("External memory test start\n");
        test_external_memory(&args);
        platform_write("External memory test complete\n");
    }

#if READ_EEPROM_IMAGE
    read_eeprom_image();
#endif

#if READ_NAND_OOB_IMAGE
    read_nand_oob_image();
#endif

    platform_write("Test completed\n");
}

