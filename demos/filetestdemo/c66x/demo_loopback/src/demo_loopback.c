/*
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 * 
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
 *
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
*/


/**
 *  FILE:  demo_loopback.c
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "mailBox.h"

#include "filetestdemo.h"

#define TESTMSG_MAX_BUFFER 8*1024*1024
#define NUM_DSP_CORES   8


#pragma  DATA_SECTION(process_buf,".process_buf");
uint8_t process_buf[NUM_DSP_CORES*TESTMSG_MAX_BUFFER];

/* Boot time init configuration */
#pragma DATA_SECTION(boot_config,".boot_config");
filetest_boot_config_t boot_config;

#pragma DATA_SECTION(host2dspmailbox,".host2dspmailbox")
uint8_t host2dspmailbox[DEMO_CONFIG_CORES_PER_CHIP*DEMO_PER_MAILBOX_MEM_SIZE];

#pragma DATA_SECTION(dsp2hostmailbox,".dsp2hostmailbox")
uint8_t dsp2hostmailbox[DEMO_CONFIG_CORES_PER_CHIP*DEMO_PER_MAILBOX_MEM_SIZE];


#define GPIO_IN_DATA 0x02320020	/* for DSP number */

extern cregister volatile unsigned int DNUM;

uint32_t local_dsp_id, local_node_id;

/* Read note Id from GPIO Register */
int32_t demo_loopback_test_getDspId(void)
{
  uint32_t dsp_id=0;

  if(boot_config.magic_number == 0xbabeface)
  {
    dsp_id = boot_config.dsp_number;
  }  else {
   dsp_id = (((*(unsigned int*) GPIO_IN_DATA) & 0x6) >> 1);		/* GPIO 1~2 */
  }

  return(dsp_id);
}

/* Function to trap exceptions */
void demo_loopback_test_assert(int32_t statement, int32_t node_id, const char *error)
{
  volatile int32_t dbg_halt = 1;

  if(!statement) {
    printf("%s (%d)\n",error,node_id);
    while(dbg_halt);
  }
}

/* Simple test code to open mailbox , receive any messages and
 *  loop back the buffer to output buffer and send message back */
void demo_loopback_test(void)
{
  int32_t node_id, ret_val;
  uint32_t        size;
  uint32_t        trans_id;
  demoTestMsg_t   testMsg;
//  uint32_t        read_cnt, write_cnt;
  void *rxMailBoxHandle, *txMailBoxHandle;
  mailBox_config_t mailBox_config;
  uint32_t core_id = DNUM;
  uint32_t mailboxallocsize;

  local_dsp_id = demo_loopback_test_getDspId();
  local_node_id = MAILBOX_MAKE_DSP_NODE_ID(local_dsp_id,core_id);
  node_id = local_node_id;

      /* Allocate instance memory for mailbox */
  mailboxallocsize = mailBox_get_alloc_size();
  rxMailBoxHandle = (void *)malloc(mailboxallocsize);
  demo_loopback_test_assert( (rxMailBoxHandle != NULL), node_id, "ERROR: malloc rx handle ");

  /* Initialise Mail boxes for both Host -> DSP and DSP -> Host */
  mailBox_config.mem_start_addr = (uint32_t )host2dspmailbox + (core_id * DEMO_PER_MAILBOX_MEM_SIZE);
  mailBox_config.mem_size = DEMO_PER_MAILBOX_MEM_SIZE;
  mailBox_config.max_payload_size = DEMO_MAILBOX_MAX_PAYLOAD_SIZE;
  demo_loopback_test_assert( (mailBox_create(rxMailBoxHandle, MAILBOX_MAKE_HOST_NODE_ID(0),
      MAILBOX_MEMORY_LOCATION_LOCAL, MAILBOX_DIRECTION_RECEIVE, &mailBox_config) == 0),
      node_id, "ERROR: mailBox_init(host --> dsp) ");
  printf("mailbox_create(host --> dsp %d) Done. \n",node_id);

  txMailBoxHandle = (void *)malloc(mailboxallocsize);
  demo_loopback_test_assert( (txMailBoxHandle != NULL), node_id, "ERROR: malloc tx handle ");

  mailBox_config.mem_start_addr = (uint32_t )dsp2hostmailbox + (core_id * DEMO_PER_MAILBOX_MEM_SIZE);

  demo_loopback_test_assert( (mailBox_create(txMailBoxHandle, MAILBOX_MAKE_HOST_NODE_ID(0),
      MAILBOX_MEMORY_LOCATION_LOCAL, MAILBOX_DIRECTION_SEND, &mailBox_config) == 0),
      node_id, "ERROR: mailBox_init(dsp --> host) ");
  printf("mailbox_create(host <-- dsp %d) Done. \n",node_id);

  /* Open Mail boxes for both Host -> DSP and DSP -> Host */
  ret_val = mailBox_open(rxMailBoxHandle);
  demo_loopback_test_assert( (ret_val != -1), node_id, "ERROR: mailBox_open(host --> dsp) ");
  printf("mailbox_open(host --> dsp %d) Done. \n",node_id);

  ret_val = mailBox_open(txMailBoxHandle);
  demo_loopback_test_assert( (ret_val != -1), node_id, "ERROR: mailBox_open(host --> dsp) ");
  printf("mailbox_open(host <-- dsp %d) Done. \n",node_id);

  for(;;)
  {
	/* Check and wait for message from host */
	do {
	  ret_val = mailBox_query(rxMailBoxHandle);
	} while(!ret_val);

	/* Read mailbox to see if any messages */
    ret_val = mailBox_read(rxMailBoxHandle, (uint8_t *)&testMsg, &size, &trans_id);
    demo_loopback_test_assert( (ret_val == 0), node_id, "ERROR: mailBox_read(dsp <-- host) ");

    /* Insert Processing here */
    memcpy(&process_buf[DNUM*TESTMSG_MAX_BUFFER], (void *)testMsg.input_addr, testMsg.input_size);
    testMsg.output_size = testMsg.input_size;
    memcpy((void *)testMsg.output_addr, &process_buf[DNUM*TESTMSG_MAX_BUFFER], testMsg.output_size);

    /* Write Mailbox to send message to host */
    ret_val = mailBox_write(txMailBoxHandle, (uint8_t *)&testMsg, sizeof(testMsg), trans_id);
    demo_loopback_test_assert( (ret_val == 0), node_id, "ERROR: mailBox_write(dsp --> host) ");
  }
  /* Free mailbox handle memory */
  /* Not needed as it is infinite loop */
/*  free(txMailBoxHandle);
    free(rxMailBoxHandle);*/
}
