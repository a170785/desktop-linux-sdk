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
 *  FILE:  demo_filetest.c
 *         This software demonstrates the PCIe APIs using a simple file test.
 *         It reads data from input file in chunks defined by 
 *         demo_payload_size.  Then sends the data to DSP 
 *         through PCIE using pcie driver and mailbox APIs.
 *         DSP in turn copies the date into the output buffer and sends message
 *         back to the Host.  The received data is written to the output file.
 *         This supports 3 different mechanisms to pass data from and to DSP.
 *         a) DMA transfer to DSP DDR 
 *         b) Mapping of Host buffer to DSP memory directly.
 *         c) Direct mem copy into DDR through pcie_drv_read & pcie_drv_write APIs.
 *         d) Statically mapped Huge buffer to DSP (This test uses only transfers
 *            per DSP. not per core)
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

/* SDK Module header files */
#include "pciedrv.h"
#include "cmem_drv.h"
#include "mailBox.h"
#include "bufmgr.h"

/* Demo header files */
#include "demo.h"
#include "filetestdemo.h"

/******************************************************************************/
/* Stores rx and tx mail ids for all the cores                                */
void  *rx_mailbox_handle[DEMO_FILETEST_MAX_NUM_CORES],
        *tx_mailbox_handle[DEMO_FILETEST_MAX_NUM_CORES];


/******************************************************************************/
/* Temporary buffer for input                                                 */
int8_t inputBuf[DEMO_FILETEST_MAX_PAYLOAD_BUFFER_SIZE];

/******************************************************************************/
/* Temporary buffer for output                                                */
int8_t outputBuf[DEMO_FILETEST_MAX_PAYLOAD_BUFFER_SIZE];

/* Buffer pool handles for Input and Output buffers */
void *c66InputBufPool[DEMO_FILETEST_MAX_NUM_CHIPS];
void *c66OutputBufPool[DEMO_FILETEST_MAX_NUM_CHIPS];
void *hostDmaBufPool;

#define MAX_NUM_BUFFERS_PER_LIST  4
/* Structure to keep track of Buffer descriptors mapped to DSP addr */
typedef struct _dsp_addr_desc_map_entry_t {
  uint32_t num_buffers;
  cmem_host_buf_desc_t buf_desc_list[MAX_NUM_BUFFERS_PER_LIST];
  uint32_t dsp_addr;
} dsp_addr_desc_map_entry_t;

dsp_addr_desc_map_entry_t dsp_addr_desc_map_table[DEMO_FILETEST_MAX_NUM_CHIPS][32];

cmem_host_buf_desc_t huge_buf_desc[DEMO_FILETEST_MAX_NUM_CHIPS];
uint32_t dsp_mapped_huge_buf_addr[DEMO_FILETEST_MAX_NUM_CHIPS];

uint32_t demo_payload_size=DEMO_FILETEST_MAX_PAYLOAD_BUFFER_SIZE;
uint32_t num_of_dsps;

/* mailbox locations */
uint32_t dsp2hostmailbox, host2dspmailbox;

/**
 *  Variables used for profiling on Desktop
 */
static int64_t total_write_time = 0;
static int32_t total_write_time_cnt = 0;
static int64_t total_read_time = 0;
static int32_t total_read_time_cnt = 0;
static int64_t total_dma_write_time = 0;
static int32_t total_dma_write_time_cnt = 0;
static int64_t total_dma_read_time = 0;
static int32_t total_dma_read_time_cnt = 0;
static int64_t memcpy_read_time=0, memcpy_write_time=0;
static int64_t total_mailbox_write_time=0, mailbox_write_min_time=0x7fffffffffffffffll, mailbox_write_max_time=0;
static int32_t total_mailbox_write_cnt=0, total_mailbox_read_cnt=0;
static int64_t total_mailbox_read_time=0, mailbox_read_min_time=0x7fffffffffffffffll, mailbox_read_max_time=0;

static int64_t file_read_time= 0, file_write_time=0;
static int32_t file_read_time_cnt=0, file_write_time_cnt=0;
/**
 *  @brief Function clock_diff returns difference between two timeval
 *  @param[in]     start            start time
 *  @param[in]     end              end time
 *  @retval        deltaTime
 */
static int64_t clock_diff (struct timespec *start, struct timespec *end)
{
    return (end->tv_sec - start->tv_sec) * 1000000000 
             + end->tv_nsec - start->tv_nsec;
}

/**
 *  @brief Function dsp_dma_write() Write to DSP memory using dsp dma over PCIe
 *                  This code splits up the buffer into the allowed contiguous 
 *                  buffers and inititates DMA to do the transfer to DSP
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Address to write
 *  @param[in]     buf            Source Buffer pointer
 *  @param[in]     size           size of Memory write in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */

int32_t dsp_dma_write(int32_t dsp_id, uint32_t addr, uint8_t *buf, 
  uint32_t size)
{
  int32_t ret_val;
  cmem_host_buf_desc_t host_buf_desc[MAX_CONTIGUOUS_XFER_BUFFERS];
  cmem_host_frame_desc_t host_frame_desc;
  uint32_t remaining_cpy_size, cpy_size, transfer_size, offset;
  uint32_t num_remain_buffers, num_buffers, transfer_buffers, remaining_size;
  uint32_t trans_id;
  int i;
  struct timespec tp_start, tp_end;
#ifdef DEMO_VERBOSE
  printf(
    "\n Debug: dma_write dsp_id %d, addr 0x%x, buffer addr 0x%p, size 0x%x\n",
    dsp_id, addr, buf, size);
#endif
  /* Calculate total number of host buffers required to fit the data */
  num_buffers = (size+DSP_OB_REGION_SIZE-1)/DSP_OB_REGION_SIZE;
  remaining_size = size;
  offset = 0;
 
/* Initiate one transfer at a time based on what fits within the allowed
   contiguous buffers per DMA transaction */
  while(num_buffers) {
    if(num_buffers > MAX_CONTIGUOUS_XFER_BUFFERS) {
      transfer_buffers = MAX_CONTIGUOUS_XFER_BUFFERS;
      transfer_size = transfer_buffers * DSP_OB_REGION_SIZE;
    }
    else {
      transfer_buffers = num_buffers;
      transfer_size = remaining_size;
    }
  
    /* Allocate Host buffer */
    for(i=0; i< transfer_buffers; i++) {
      ret_val = bufmgrAlloc(hostDmaBufPool, 1, &host_buf_desc[i]);
      printf("\n Allocated buffer %p\n",host_buf_desc[i].userAddr);
      if(ret_val != 0) {
        printf("ERROR: dma buffer allocation failed\n");
        return(-1);
      }
    }
    clock_gettime(CLOCK_MONOTONIC, &tp_start);
    /* Copy from buffer content into DMA buffers */
    remaining_cpy_size = transfer_size;
    i=0;
    while(remaining_cpy_size)
    {
      if(remaining_cpy_size >= host_buf_desc[i].length) {
        cpy_size = host_buf_desc[i].length;
      } else {
        cpy_size = remaining_cpy_size;
      }

      memcpy(host_buf_desc[i].userAddr, buf+offset, cpy_size);
      remaining_cpy_size-= cpy_size;
      offset+=cpy_size;
      i++;
    }
    /* Populate details of data in frame descriptor */
    host_frame_desc.bufDescP = host_buf_desc;
    host_frame_desc.numBuffers = transfer_buffers;
    host_frame_desc.frameStartOffset = 0;
    host_frame_desc.frameSize = transfer_size;

//    clock_gettime(CLOCK_MONOTONIC, &tp_start);

    /* Initiate DMA */
    ret_val = pciedrv_dma_write_initiate(dsp_id, addr, &host_frame_desc,
               PCIEDRV_DMA_XFER_BLOCKING, &trans_id);
    if(ret_val != 0) {
      printf("ERROR: DMA initiate failed\n");
      return(-1);
    }

#if 0  /*   Not used as we are using Blocking call currently */
    printf("\nDebug: Dma Initiate Complete %d\n", trans_id);fflush(stdout);
    /* Wait for dma complete */
    while(pciedrv_dma_check(dsp_id, trans_id));
#endif
    clock_gettime(CLOCK_MONOTONIC, &tp_end);
    total_dma_write_time += clock_diff (&tp_start, &tp_end);
    total_dma_write_time_cnt ++;

    /* Free the Buffer descriptors */
    for(i=0; i< transfer_buffers; i++) {
      ret_val = bufmgrFreeDesc(hostDmaBufPool, &host_buf_desc[i]);
     printf("\n Free buffer %p\n",host_buf_desc[i].userAddr);
      if(ret_val != 0) {
        printf("ERROR: dma buffer free failed\n");
        return(-1);
      }
    }
    num_buffers -= transfer_buffers;
    remaining_size -=transfer_size;
    addr += transfer_size;
  }

  return(0);

}
/**
 *  @brief Function dsp_dma_read() Read to DSP memory using dsp dma over PCIe
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Address to read
 *  @param[in]     buf            Destination Buffer pointer
 *  @param[in]     size           size of Memory write in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dsp_dma_read(int32_t dsp_id, uint32_t addr, uint8_t *buf, uint32_t size)
{
  int32_t ret_val;
  cmem_host_buf_desc_t host_buf_desc[MAX_CONTIGUOUS_XFER_BUFFERS];
  cmem_host_frame_desc_t host_frame_desc;
  uint32_t remaining_cpy_size, cpy_size, transfer_size, offset;
  uint32_t num_remain_buffers, num_buffers, transfer_buffers, remaining_size;
  uint32_t trans_id;
  int i;
  struct timespec tp_start, tp_end;

#ifdef DEMO_VERBOSE
  printf(
    "\n Debug: dma_read dsp_id %d, addr 0x%x, buffer addr 0x%p, size 0x%x\n", 
    dsp_id, addr, buf, size);
#endif
  /* Calculate total number of host buffers required to fit the data */
  num_buffers = (size+DSP_OB_REGION_SIZE-1)/DSP_OB_REGION_SIZE;
  remaining_size = size;
  offset = 0;

  /* Initiate one transfer at a time based on what fits within the allowed
     contiguous buffers per DMA transaction */
  while(num_buffers) {
    if(num_buffers > (MAX_CONTIGUOUS_XFER_BUFFERS)) {
      transfer_buffers = (MAX_CONTIGUOUS_XFER_BUFFERS);
      transfer_size = transfer_buffers * DSP_OB_REGION_SIZE;
    }
    else {
      transfer_buffers = num_buffers;
      transfer_size = remaining_size;
    }
    /* Allocate Host buffer */
    for(i=0; i< transfer_buffers; i++) {
      ret_val = bufmgrAlloc(hostDmaBufPool, 1, &host_buf_desc[i]);

      if(ret_val != 0) {
        printf("ERROR: dma buffer allocation failed\n");
        return(-1);
      }
    }

   /* Populate details of data in frame descriptor */
    host_frame_desc.bufDescP = host_buf_desc;
    host_frame_desc.numBuffers = transfer_buffers;
    host_frame_desc.frameStartOffset = 0;
    host_frame_desc.frameSize = transfer_size;

    clock_gettime(CLOCK_MONOTONIC, &tp_start);

    /* Initiate DMA */
    ret_val = pciedrv_dma_read_initiate(dsp_id, addr, &host_frame_desc,
                PCIEDRV_DMA_XFER_BLOCKING, &trans_id);
    if(ret_val != 0) {
      printf("ERROR: DMA initiate failed\n");
      return(-1);
    }
 
#if 0  /*   Not used as we are using Blocking call currently */
    printf("\nDebug: Dma Initiate Complete %d\n", trans_id);fflush(stdout);
    /* Wait for dma complete */
    while(pciedrv_dma_check(dsp_id, trans_id));
#endif
    clock_gettime(CLOCK_MONOTONIC, &tp_end);
    total_dma_read_time += clock_diff (&tp_start, &tp_end);
    total_dma_read_time_cnt ++;
 
    /* Copy from dma buffers into buffer */
    remaining_cpy_size =  host_frame_desc.frameSize;
    i=0;
    while(remaining_cpy_size)
    {
      if(remaining_cpy_size > host_buf_desc[i].length)
      {
        cpy_size = host_buf_desc[i].length;
      } else
      {
        cpy_size = remaining_cpy_size;
      }
      memcpy(buf+offset, host_buf_desc[i].userAddr, cpy_size);  
      remaining_cpy_size -= cpy_size;
      offset+=cpy_size;
      i++;
    }

    /* Free Buffer Descriptors */
    for(i=0; i< transfer_buffers; i++) {
      ret_val = bufmgrFreeDesc(hostDmaBufPool, &host_buf_desc[i]);
      if(ret_val != 0) {
        printf("ERROR: dma buffer free failed\n");
        return(-1);
      }
    }
    num_buffers -= transfer_buffers;
    remaining_size -=transfer_size;
    addr += transfer_size;
  }
  return(0);

}
/**
 *  @brief Function demo667x4_assert Routine to trap exceptions
 *  @param[in]         statement    Boolean statement
 *  @param[in]         node_id      Node Id
 *  @param[in]         errorMsg     Error string
 *  @retval        None         
 *  @pre  
 *  @post 
 */
void demo667x4_assert(int32_t statement, int32_t node_id, const int8_t *errorMsg)
{
  volatile int32_t dbg_halt = 1;

  if(!statement) {
    printf("%s (%d)\n",errorMsg, node_id);
    while(dbg_halt);
  }
}


/**
 *  @brief Function demo667x4_initDspBufs : Initialise buffer manager 
 *                  for all chips 
 *  @param             none
 *  @retval            none       
 *  @pre  
 *  @post 
 */

void demo667x4_initDspBufs(void)
{
  int32_t i;
  int32_t ret_val;

  /* Allocate buffer pools from DSP memory for Input and Output */
  for ( i = 0; i < num_of_dsps; i++)
  {
    ret_val = bufmgrCreate2(
      &c66InputBufPool[i], 
      (void *) DEMO_FILETEST_INPUT_POOL_BASEADDR, 
      DEMO_FILETEST_POOL_SIZE, 
      demo_payload_size, 
      0);
    demo667x4_assert( (ret_val == 0), i,
        "ERROR: Buffer Create2 : input pool) ");
    
    ret_val = bufmgrCreate2(
      &c66OutputBufPool[i], 
      (void *) DEMO_FILETEST_OUTPUT_POOL_BASEADDR, 
      DEMO_FILETEST_POOL_SIZE, 
      demo_payload_size, 
      0);
    demo667x4_assert( (ret_val == 0), i,
        "ERROR: Buffer Create2 : output pool) ");
  }
}
/**
 *  @brief Function demo667x4_deletePools Delete allocated pools
 *  @param          None
 *  @retval         None
 */
void demo667x4_deletePools(void)
{
  int32_t i;

  for ( i = 0; i < num_of_dsps; i++) {
    bufmgrDelete(&c66InputBufPool[i]);
    bufmgrDelete(&c66OutputBufPool[i]);
  }
}

/**
 *  @brief Function demo667x4_Config : Configure and initialise 
 *                  required components for demo
 *  @param[in]         nodeBits   Node bit map
 *  @retval                       0: success ; -1: fail       
 *  @pre  
 *  @post 
 */

uint32_t demo667x4_Config(uint32_t nodeBits)
{
  int32_t node_id, ret_val;
  uint32_t size, trans_id = 0xC0DE0000, size_rx, trans_id_rx;
  uint32_t core_id, dsp_id;
  mailBox_config_t mailBox_config;
  uint32_t mailboxallocsize;

  /*** Create mailboxes ***/
  for ( node_id = 0; node_id < num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP; node_id++)
  {
    
    if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
    {
      core_id = node_id%(DEMO_CONFIG_CORES_PER_CHIP);
      dsp_id = node_id/DEMO_CONFIG_CORES_PER_CHIP;
#ifdef DEMO_VERBOSE
      printf("Creating Mailbox (%x --> %x)\n", MAILBOX_MAKE_HOST_NODE_ID(0),node_id);
#endif
      mailboxallocsize = mailBox_get_alloc_size();
      tx_mailbox_handle[node_id] = (void *)malloc(mailboxallocsize);
      demo667x4_assert( (tx_mailbox_handle[node_id] != NULL), node_id, "ERROR: malloc Tx handle ");
     mailBox_config.mem_start_addr = host2dspmailbox + (core_id * DEMO_PER_MAILBOX_MEM_SIZE);
      mailBox_config.mem_size = DEMO_PER_MAILBOX_MEM_SIZE;
      mailBox_config.max_payload_size = DEMO_MAILBOX_MAX_PAYLOAD_SIZE;
      ret_val = mailBox_create(tx_mailbox_handle[node_id], MAILBOX_MAKE_DSP_NODE_ID(dsp_id,core_id),
        MAILBOX_MEMORY_LOCATION_REMOTE, MAILBOX_DIRECTION_SEND, &mailBox_config);
      demo667x4_assert( (ret_val == 0 ), node_id,
        "ERROR: mailBox_create(host --> dsp) ");
      printf("Created Mailbox Tx id: 0x%8x Handle %p\n",node_id, tx_mailbox_handle[node_id] );
#ifdef DEMO_VERBOSE
      printf("Creating Mailbox (%x --> %x)\n",node_id,MAILBOX_MAKE_HOST_NODE_ID(0));
#endif
      rx_mailbox_handle[node_id] = (void *)malloc(mailboxallocsize);
      demo667x4_assert( (rx_mailbox_handle[node_id] != NULL), node_id, "ERROR: malloc Rx handle ");
       mailBox_config.mem_start_addr = dsp2hostmailbox + (core_id * DEMO_PER_MAILBOX_MEM_SIZE);
      ret_val = mailBox_create(rx_mailbox_handle[node_id], MAILBOX_MAKE_DSP_NODE_ID(dsp_id,core_id),
        MAILBOX_MEMORY_LOCATION_REMOTE, MAILBOX_DIRECTION_RECEIVE, &mailBox_config);
      demo667x4_assert( (ret_val == 0), node_id, 
        "ERROR: mailBox_create(dsp --> host) ");
      printf("Created Mailbox Rx id: 0x%8x, Handle %p\n",node_id, rx_mailbox_handle[node_id] );
    }
  }
  /***  Open Mailboxes ***/
  for ( node_id=0; node_id < (num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP); node_id++)
  {
    if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
    {
      printf("nodeBits %x decision%x \n",(((uint32_t) 1) << node_id),  
        nodeBits & (((uint32_t) 1) << node_id) );
      printf("Opening Mailbox(%x --> %x)\n", MAILBOX_MAKE_HOST_NODE_ID(0),node_id);
      demo667x4_assert( (mailBox_open(tx_mailbox_handle[node_id]) == 0), 
        node_id, "ERROR: mailBox_init(host --> dsp) ");
      printf("Opening Mailbox(%x --> %x)\n",node_id, MAILBOX_MAKE_HOST_NODE_ID(0));
      demo667x4_assert( (mailBox_open(rx_mailbox_handle[node_id]) == 0),
        node_id, "ERROR: mailBox_init(dsp --> host) ");
    }
  }

  printf("\t\t\tMailboxed Created!\n");

  sleep(1);

  return 0;
}
/**
 *  @brief Function demo667x4_Config_free : Free memory allocated by demo667x4_Config
 *  @param[in]         nodeBits   Node bit map
 *  @retval            none      
 *  @pre  
 *  @post 
 */

void demo667x4_Config_free(uint32_t nodeBits)
{
  int32_t node_id;

  /*** Create mailboxes ***/
  for ( node_id = 0; node_id < (num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP); node_id++)
  {
    if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
    {
      free(rx_mailbox_handle[node_id]);
      free(tx_mailbox_handle[node_id]);
    }
  }
}
/**
 *  @brief Function get_free_entry_from_desc_map_table : Desc map table keeps track of  
 *         the entries used for mapping to dsp memory range. This routine returns a free
 *         entry from the table. Entry free is marked by num_buffers = 0
 *  @param[in]         dsp_id   DSP or Chip number
 *  @retval            desc map entry       
 *  @pre  
 *  @post 
 */
dsp_addr_desc_map_entry_t *get_free_entry_from_desc_map_table(uint32_t dsp_id) 
{
  int i;

  for(i=0; i< 32; i++)
  {
    if(dsp_addr_desc_map_table[dsp_id][i].num_buffers==0)
    { 
      return(&dsp_addr_desc_map_table[dsp_id][i]);
    }
  }
  return(NULL);
}
/**
 *  @brief Function get_free_entry_from_desc_map_table : This routine returns the
 *         entry from the table, which is mapped to the dsp address.
 *  @param[in]         dsp_id   DSP or Chip number
 *  @param[in]         dsp_addr input dsp address
 *  @retval            desc map entry           
 *  @pre  
 *  @post 
 */
dsp_addr_desc_map_entry_t *get_match_desc_map_table_entry(uint32_t dsp_id,
  uint32_t dsp_addr)
{
  int i;

  for(i=0; i< 32; i++)
  {
    if((dsp_addr_desc_map_table[dsp_id][i].dsp_addr==dsp_addr)
       && (dsp_addr_desc_map_table[dsp_id][i].num_buffers != 0))
    { 
      return(&dsp_addr_desc_map_table[dsp_id][i]);
    }
  }
  return(NULL);
}
/**
 *  @brief Function read_and_send_data_to_dsp_through_memcpy : This routine reads a block of data from input file
 *         and writes to Input buffer in DSP address space through pciedrv_dsp_write api. 
 *         Also sends a mail to the DSP through Mailbox
 *  @param[in]         node_id   Node id
 *  @param[in]         trans_id  Transaction id to be used with mailbox 
 *  @param[in]         inFile    Input file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void read_and_send_data_to_dsp_through_memcpy(int32_t node_id, uint32_t trans_id, FILE *inFile)
{
  void *inBufHandle, *outBufHandle;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  struct timespec tp_start, tp_end;
  bufmgrDesc_t bufDesc;
  int64_t tmp_value;

  inBufHandle = c66InputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];
  outBufHandle = c66OutputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];

  /* Read Buffer from file and send to Mailbox */
  dspMsg.input_size = fread(inputBuf,1,demo_payload_size,inFile);
  /* Allocate Input buffer from pool */
  ret_val = bufmgrAlloc(inBufHandle, 1, &bufDesc);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, 
    "ERROR: bufmgrAlloc() did not find free input buffer.");
  printf("bufmgrAlloc(%d) = 0x%08X\n", node_id, (uint32_t)(intptr_t)(bufDesc.userAddr));
  dspMsg.input_addr = (uint32_t)(intptr_t)(bufDesc.userAddr);
  /* Allocate Output buffer from pool */
  ret_val = bufmgrAlloc(outBufHandle, 1, &bufDesc);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, 
    "ERROR: bufmgrAlloc() did not find free output buffer.");
  printf("bufmgrAlloc(%d) = 0x%08X\n", node_id, (uint32_t)(intptr_t)(bufDesc.userAddr));
  dspMsg.output_addr = (uint32_t)(intptr_t)(bufDesc.userAddr);
  /* Write data from temporary buffer to Input buffer */
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  pciedrv_dsp_write((node_id / DEMO_CONFIG_CORES_PER_CHIP),dspMsg.input_addr,
    inputBuf,dspMsg.input_size);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  if(dspMsg.input_size ==demo_payload_size) {
    total_write_time += clock_diff (&tp_start, &tp_end);
    total_write_time_cnt ++;
  }

  printf(
    "\nMsg (%08X) to --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",
    trans_id,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,
    dspMsg.output_size);
  printf("\ntx_mailbox node_id %x, handle (%p)\n",node_id, tx_mailbox_handle[node_id]);
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
   /* Send message through Mailbox */ 
  ret_val = mailBox_write(tx_mailbox_handle[node_id], (uint8_t *)&dspMsg, sizeof(demoTestMsg_t), 
              trans_id);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_write_min_time)
    mailbox_write_min_time = tmp_value;
  if(tmp_value > mailbox_write_max_time)
    mailbox_write_max_time = tmp_value;
  total_mailbox_write_time += tmp_value;
  total_mailbox_write_cnt++;

  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_write(host --> dsp) ");

}
/**
 *  @brief Function read_and_send_data_to_dsp_through_mapping : This routine reads a block of data from input file
 *         and writes to Input buffer in Host address space and maps to DSP memory range. 
 *         Also sends a mail to the DSP through Mailbox
 *  @param[in]         node_id   Node id
 *  @param[in]         trans_id  Transaction id to be used with mailbox 
 *  @param[in]         inFile    Input file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void read_and_send_data_to_dsp_through_mapping(int32_t node_id, uint32_t trans_id, FILE *inFile)
{
  uint32_t i, num_buffers, remaining_size, read_size, write_size ;
  struct timespec tp_start, tp_inter, tp_end;
  bufmgrDesc_t *inbufDesc_list;
  bufmgrDesc_t *outbufDesc_list;
  dsp_addr_desc_map_entry_t  *dsp_addr_desc_map_entryP;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  uint32_t fread_size;
  int64_t tmp_value;

  /* Allocate input buffers */
  num_buffers = (demo_payload_size+DSP_OB_REGION_SIZE-1)
                  /DSP_OB_REGION_SIZE;
  /* Find a free entry for buffer descriptor list */
  dsp_addr_desc_map_entryP = get_free_entry_from_desc_map_table((node_id / DEMO_CONFIG_CORES_PER_CHIP));
  demo667x4_assert( (dsp_addr_desc_map_entryP !=NULL), node_id, 
            "ERROR: Unable to find desc map entry.");
  dsp_addr_desc_map_entryP->num_buffers = num_buffers;
  inbufDesc_list = dsp_addr_desc_map_entryP->buf_desc_list;
  for(i=0; i< num_buffers; i++) {
    ret_val = bufmgrAlloc(hostDmaBufPool, 1, &inbufDesc_list[i]);
    demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, 
      "ERROR: bufmgrAlloc() did not find free input buffer.");
    printf("bufmgrAlloc(%d) = %p\n", node_id, (inbufDesc_list[i].userAddr));
  }
  /* Read data into input buffers */
  dspMsg.input_size = 0;
  remaining_size = demo_payload_size;
  i= 0;
  while(remaining_size)
  {
    if( remaining_size >= DSP_OB_REGION_SIZE) 
    {
      read_size = DSP_OB_REGION_SIZE;
    } 
    else 
    {
      read_size = remaining_size;
    }
    clock_gettime(CLOCK_MONOTONIC, &tp_start);
    /* Read Buffer from file and send to Mailbox */
//    fread_size = fread(inbufDesc_list[i].userAddr, 1,read_size,inFile);
    fread_size = fread(inputBuf, 1,read_size,inFile);
    clock_gettime(CLOCK_MONOTONIC, &tp_inter);
    memcpy(inbufDesc_list[i].userAddr,inputBuf, read_size);
    clock_gettime(CLOCK_MONOTONIC, &tp_end);
    if(fread_size == read_size)
    {
      file_read_time += clock_diff (&tp_start, &tp_inter);
      memcpy_read_time += clock_diff(&tp_inter, &tp_end);
      file_read_time_cnt ++;
    }

    dspMsg.input_size += fread_size;
    remaining_size -= read_size;
    i++;
  }
  /* Allocate DSP range */
  ret_val = pciedrv_dsp_memrange_alloc((node_id / DEMO_CONFIG_CORES_PER_CHIP),
    demo_payload_size, &dsp_addr_desc_map_entryP->dsp_addr);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: Memory range alloc failed.");


  clock_gettime(CLOCK_MONOTONIC, &tp_start);

  /* Map Input buffers to dsp range */
  ret_val = pciedrv_map_hostbufs_to_dsp_memrange((node_id / DEMO_CONFIG_CORES_PER_CHIP),
              num_buffers, 
              inbufDesc_list, 
              (uint32_t) dsp_addr_desc_map_entryP->dsp_addr);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: Map to dsp Memory failed.");
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  if(dspMsg.input_size ==demo_payload_size) {
    total_write_time += clock_diff (&tp_start, &tp_end);
    total_write_time_cnt ++;
  }


  dspMsg.input_addr =  dsp_addr_desc_map_entryP->dsp_addr;

  /* Find a free entry for buffer descriptor list */
  dsp_addr_desc_map_entryP = get_free_entry_from_desc_map_table((node_id / DEMO_CONFIG_CORES_PER_CHIP));
  demo667x4_assert( (dsp_addr_desc_map_entryP !=NULL), node_id, 
    "ERROR: Unable to find desc map entry.");
  dsp_addr_desc_map_entryP->num_buffers = num_buffers;
  outbufDesc_list =  dsp_addr_desc_map_entryP->buf_desc_list;
  /* Allocate output buffers */
  for(i=0; i< num_buffers; i++) {
    ret_val = bufmgrAlloc(hostDmaBufPool, 1, &outbufDesc_list[i]);
    demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, 
      "ERROR: bufmgrAlloc() did not find free output buffer.");
    printf("bufmgrAlloc(%d) = %p\n", node_id, (outbufDesc_list[i].userAddr));
  }
  /* Allocate DSP range */
  ret_val = pciedrv_dsp_memrange_alloc((node_id / DEMO_CONFIG_CORES_PER_CHIP),
    demo_payload_size, &dsp_addr_desc_map_entryP->dsp_addr);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: Memory range alloc failed.");
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  /* Map Input buffers to dsp range */
  ret_val = pciedrv_map_hostbufs_to_dsp_memrange((node_id / DEMO_CONFIG_CORES_PER_CHIP),
              num_buffers, 
              outbufDesc_list, 
              (uint32_t) dsp_addr_desc_map_entryP->dsp_addr);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: Map to dsp Memory failed.");
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: dma read failed");
  if(dspMsg.input_size ==demo_payload_size) {
    total_read_time += clock_diff (&tp_start, &tp_end);
    total_read_time_cnt ++;
  }

  dspMsg.output_addr =  dsp_addr_desc_map_entryP->dsp_addr;

  printf("\nMsg (%08X) to --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",trans_id,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_write(tx_mailbox_handle[node_id], (uint8_t *)&dspMsg, sizeof(demoTestMsg_t), trans_id);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_write_min_time)
    mailbox_write_min_time = tmp_value;
  if(tmp_value > mailbox_write_max_time)
    mailbox_write_max_time = tmp_value;
  total_mailbox_write_time += tmp_value;
  total_mailbox_write_cnt++;

  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_write(host --> dsp) ");
}
/**
 *  @brief Function read_and_send_data_to_dsp_through_dma : This routine reads a block of data from input file
 *         and writes to Input buffer in DSP address space through DMA. 
 *         Also sends a mail to the DSP through Mailbox
 *  @param[in]         node_id   Node id
 *  @param[in]         trans_id  Transaction id to be used with mailbox 
 *  @param[in]         inFile    Input file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void read_and_send_data_to_dsp_through_dma(int32_t node_id, uint32_t trans_id, FILE *inFile)
{
  void *inBufHandle, *outBufHandle;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  bufmgrDesc_t bufDesc;
  struct timespec tp_start, tp_end;
  int64_t tmp_value;

  inBufHandle = c66InputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];
  outBufHandle = c66OutputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];

  /* Allocate Input buffer from Input pool*/
  ret_val = bufmgrAlloc(inBufHandle, 1, &bufDesc);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrAlloc() did not find free input buffer.");
  printf("bufmgrAlloc(%d) = 0x%08X\n", node_id, (uint32_t)(intptr_t)bufDesc.userAddr);

  dspMsg.input_addr = (uint32_t)(intptr_t)bufDesc.userAddr;

  /* Allocate Output buffer from Output pool*/
  ret_val = bufmgrAlloc(outBufHandle, 1, &bufDesc);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrAlloc() did not find free output buffer.");
  printf("bufmgrAlloc(%d) = 0x%08X\n", node_id, (uint32_t)(intptr_t)bufDesc.userAddr);

  dspMsg.output_addr = (uint32_t)(intptr_t)bufDesc.userAddr;
  demo667x4_assert( (dspMsg.output_addr != 0), node_id, "ERROR: bufmgrAlloc() did not find free output buffer.");
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  dspMsg.input_size = fread(inputBuf,1,demo_payload_size,inFile);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  if(dspMsg.input_size ==demo_payload_size) {
    file_read_time += clock_diff (&tp_start, &tp_end);
    file_read_time_cnt ++;
  }

  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  /* Write date to DSP buffer from Host buffer */
  ret_val = dsp_dma_write((node_id / DEMO_CONFIG_CORES_PER_CHIP),dspMsg.input_addr,inputBuf,dspMsg.input_size);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: dma write failed");
  if(dspMsg.input_size ==demo_payload_size) {
    total_write_time += clock_diff (&tp_start, &tp_end);
    total_write_time_cnt ++;
  }
  printf("\nMsg (%08X) to --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",
     trans_id,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_write(tx_mailbox_handle[node_id], (uint8_t *)&dspMsg, sizeof(demoTestMsg_t), trans_id);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_write_min_time)
    mailbox_write_min_time = tmp_value;
  if(tmp_value > mailbox_write_max_time)
    mailbox_write_max_time = tmp_value;
  total_mailbox_write_time += tmp_value;
  total_mailbox_write_cnt++;

  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_write(host --> dsp) ");
}
/**
 *  @brief Function read_and_send_data_to_dsp_through_huge_buffer : This routine reads a block of data from input file
 *         and writes to huge host  buffer mapped to dsp memory range. 
 *         Also sends a mail to the DSP through Mailbox
 *  @param[in]         dsp_id    DSP id
 *  @param[in]         trans_id  Transaction id to be used with mailbox 
 *  @param[in]         inFile    Input file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void read_and_send_data_to_dsp_through_huge_buffer(int32_t dsp_id, uint32_t trans_id, FILE *inFile)
{
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  int32_t node_id = (dsp_id*DEMO_CONFIG_CORES_PER_CHIP);
  struct timespec tp_start, tp_end;
  int64_t tmp_value;

  printf("\n Buffer address %p\n", huge_buf_desc[dsp_id].userAddr);
  dspMsg.input_size = fread(huge_buf_desc[dsp_id].userAddr, 1,demo_payload_size,inFile);
  dspMsg.input_addr =  dsp_mapped_huge_buf_addr[dsp_id];
  dspMsg.output_size = 0;
  /* currently setting input same as output */
  dspMsg.output_addr =  dsp_mapped_huge_buf_addr[dsp_id];

  printf("\nMsg (%08X) to --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",trans_id,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_write(tx_mailbox_handle[node_id], (uint8_t *)&dspMsg, sizeof(demoTestMsg_t), trans_id);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_write_min_time)
    mailbox_write_min_time = tmp_value;
  if(tmp_value > mailbox_write_max_time)
    mailbox_write_max_time = tmp_value;
  total_mailbox_write_time += tmp_value;
  total_mailbox_write_cnt++;

  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_write(host --> dsp) ");
}

/**
 *  @brief Function get_data_through_mapped_memory_and_write_to_file : This routine gets message from  
 *         mailBox from the specified DSP node, and gets the data from the Host buffer mapped to dsp space
 *         And write the data to a file 
 *  @param[in]         node_id   Node id
 *  @param[in]         outFile   Output file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void get_data_through_mapped_memory_and_write_to_file(int32_t node_id,  FILE *outFile)
{
  uint32_t i, num_buffers, remaining_size, write_size ;
  struct timespec tp_start, tp_end, tp_inter;
  bufmgrDesc_t *inbufDesc_list;
  bufmgrDesc_t *outbufDesc_list;
  dsp_addr_desc_map_entry_t  *dsp_addr_desc_map_entryP;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  uint32_t    size_rx, trans_id_rx;
  int64_t tmp_value;

  /* Get message from mailbox */
  printf("\nrx_mailbox node_id %x, handle (%p)\n",node_id, rx_mailbox_handle[node_id]);
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_read(rx_mailbox_handle[node_id], (uint8_t *)&dspMsg, &size_rx, &trans_id_rx);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_read_min_time)
    mailbox_read_min_time = tmp_value;
  if(tmp_value > mailbox_read_max_time)
    mailbox_read_max_time = tmp_value;
  total_mailbox_read_time += tmp_value;
  total_mailbox_read_cnt++;

  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_read(host <-- dsp) ");

  printf("\nMsg (%08X) from --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",trans_id_rx,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  dsp_addr_desc_map_entryP = get_match_desc_map_table_entry(
    (node_id / DEMO_CONFIG_CORES_PER_CHIP), dspMsg.output_addr);
  outbufDesc_list =  dsp_addr_desc_map_entryP->buf_desc_list;
  num_buffers= dsp_addr_desc_map_entryP->num_buffers ;
  /* Write data from output buffers to file */
  remaining_size = dspMsg.output_size;
  i=0;
  while(remaining_size)
  {
    if( remaining_size >= DSP_OB_REGION_SIZE) 
    {
      write_size = DSP_OB_REGION_SIZE;
    } 
    else 
    {
      write_size = remaining_size;
    }
   clock_gettime(CLOCK_MONOTONIC, &tp_start);
    /* Read from Buffer  and write to file  */
//    fwrite(outbufDesc_list[i].userAddr, 1,write_size,outFile);
    memcpy(outputBuf, outbufDesc_list[i].userAddr, write_size);
    clock_gettime(CLOCK_MONOTONIC, &tp_inter);
    fwrite(outputBuf, 1,write_size,outFile);
    clock_gettime(CLOCK_MONOTONIC, &tp_end);
 
    file_write_time += clock_diff (&tp_inter, &tp_end);
    memcpy_write_time += clock_diff (&tp_start, &tp_inter);
    file_write_time_cnt ++;

    remaining_size -= write_size;
    i++;
  }
  /* Now free the DSP outbound region */
  ret_val = pciedrv_dsp_memrange_free(
              (node_id / DEMO_CONFIG_CORES_PER_CHIP),
              demo_payload_size,
              dsp_addr_desc_map_entryP->dsp_addr);
  /* Free the Host buffers */
  for(i=0; i< num_buffers; i++) {
    ret_val = bufmgrFreeDesc(hostDmaBufPool, &outbufDesc_list[i]);
    demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrFreeDesc() did not free input buffer.");
    printf("bufmgrFreeDesc(%d) = %p\n", node_id, outbufDesc_list[i].userAddr);
  }
  dsp_addr_desc_map_entryP->num_buffers = 0;

  /* Get matching table entry */
  dsp_addr_desc_map_entryP = get_match_desc_map_table_entry(
                         (node_id / DEMO_CONFIG_CORES_PER_CHIP), dspMsg.input_addr);
  inbufDesc_list =  dsp_addr_desc_map_entryP->buf_desc_list;
  /* Now free the DSP outbound region */
  ret_val = pciedrv_dsp_memrange_free(
              (node_id / DEMO_CONFIG_CORES_PER_CHIP),
              demo_payload_size,
              dsp_addr_desc_map_entryP->dsp_addr);
  /* Free the Host buffers */
  for(i=0; i< num_buffers; i++) {
    ret_val = bufmgrFreeDesc(hostDmaBufPool, &inbufDesc_list[i]);
    demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrFreeDesc() did not free output buffer.");
    printf("bufmgrFreeDesc(%d) = %p\n", node_id, inbufDesc_list[i].userAddr);
  }
  dsp_addr_desc_map_entryP->num_buffers = 0;
}
/**
 *  @brief Function get_data_through_dma_and_write_to_file : This routine gets message from  
 *         mailBox from the specified DSP node, and gets the data from the DSP Buffer through DSP DMA
 *         And write the data to a file 
 *  @param[in]         node_id   Node id
 *  @param[in]         outFile   Output file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void get_data_through_dma_and_write_to_file(int32_t node_id,  FILE *outFile)
{
  void *inBufHandle, *outBufHandle;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  bufmgrDesc_t bufDesc;
  struct timespec tp_start, tp_end;
  uint32_t  size_rx, trans_id_rx;
  int64_t tmp_value;

  inBufHandle = c66InputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];
  outBufHandle = c66OutputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];

  printf("\nrx_mailbox node_id %x, handle (%p)\n",node_id, rx_mailbox_handle[node_id]);
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_read(rx_mailbox_handle[node_id], (uint8_t *)&dspMsg, &size_rx, &trans_id_rx);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_read_min_time)
    mailbox_read_min_time = tmp_value;
  if(tmp_value > mailbox_read_max_time)
    mailbox_read_max_time = tmp_value;
  total_mailbox_read_time += tmp_value;
  total_mailbox_read_cnt++;
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_read(host <-- dsp) ");

  printf("\nMsg (%08X) from --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",
    trans_id_rx,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = dsp_dma_read((node_id / DEMO_CONFIG_CORES_PER_CHIP),dspMsg.output_addr,outputBuf,dspMsg.output_size);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: dma read failed");
  if(dspMsg.output_size ==demo_payload_size) {
    total_read_time += clock_diff (&tp_start, &tp_end);
    total_read_time_cnt ++;
  }
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  fwrite(outputBuf,1,dspMsg.output_size,outFile);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  if(dspMsg.output_size ==demo_payload_size) {
    file_write_time += clock_diff (&tp_start, &tp_end);
    file_write_time_cnt ++;
  }

  printf("bufmgrFree(%d) = 0x%08X\n", node_id, dspMsg.input_addr);
  ret_val = bufmgrFreeUserAddr( inBufHandle, (void *)(intptr_t)dspMsg.input_addr);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrFreeUserAddr() for input buffer.");

  printf("bufmgrFree(%d) = 0x%08X\n", node_id, dspMsg.output_addr);
  ret_val = bufmgrFreeUserAddr( outBufHandle, (void *)(intptr_t)dspMsg.output_addr);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrFreeUserAddr() for output buffer.");

}
/**
 *  @brief Function get_data_through_memcpy_and_write_to_file : This routine gets message from  
 *         mailBox from the specified DSP node, and gets the data from the DSP Buffer using 
 *         the pciedrv_dsp_read API. And write the data to a file 
 *  @param[in]         node_id   Node id
 *  @param[in]         outFile   Output file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
/**
 *  @brief Function get_data_through_huge_buffer_and_write_to_file : This routine gets message from  
 *         mailBox from the specified DSP node, and gets the data from the huge Host buffer mapped to dsp space
 *         And write the data to a file 
 *  @param[in]         dsp_id    DSP id
 *  @param[in]         outFile   Output file descriptor 
 *  @retval            none       
 *  @pre  
 *  @post 
 */
void get_data_through_huge_buffer_and_write_to_file(int32_t dsp_id,  FILE *outFile)
{
  uint32_t i, num_buffers, remaining_size, write_size ;
  struct timespec tp_start, tp_end;
  bufmgrDesc_t *inbufDesc_list;
  bufmgrDesc_t *outbufDesc_list;
  dsp_addr_desc_map_entry_t  *dsp_addr_desc_map_entryP;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  uint32_t    size_rx, trans_id_rx;
  int32_t node_id = (dsp_id*DEMO_CONFIG_CORES_PER_CHIP);
  int64_t tmp_value;

  /* Get message from mailbox */
  printf("\nrx_mailbox node_id, %x handle (%p)\n",node_id, rx_mailbox_handle[node_id]);
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_read(rx_mailbox_handle[node_id], (uint8_t *)&dspMsg, &size_rx, &trans_id_rx);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_read_min_time)
    mailbox_read_min_time = tmp_value;
  if(tmp_value > mailbox_read_max_time)
    mailbox_read_max_time = tmp_value;
  total_mailbox_read_time += tmp_value;
  total_mailbox_read_cnt++;
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_read(host <-- dsp) ");

  printf("\nMsg (%08X) from --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",trans_id_rx,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  /* Read from buffer and write to file */
  fwrite(huge_buf_desc[dsp_id].userAddr+(dspMsg.output_addr- dsp_mapped_huge_buf_addr[dsp_id]), 1,dspMsg.output_size, outFile);
}

void get_data_through_memcpy_and_write_to_file(int32_t node_id,  FILE *outFile)
{
  void *inBufHandle, *outBufHandle;
  demoTestMsg_t dspMsg;
  int32_t ret_val;
  bufmgrDesc_t bufDesc;
  struct timespec tp_start, tp_end;
  uint32_t  size_rx, trans_id_rx;
  int64_t tmp_value;

  inBufHandle = c66InputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];
  outBufHandle = c66OutputBufPool[(node_id / DEMO_CONFIG_CORES_PER_CHIP)];

  printf("\nrx_mailbox node_id %x handle (%p)\n",node_id, rx_mailbox_handle[node_id]);
  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = mailBox_read(rx_mailbox_handle[node_id], (uint8_t *)&dspMsg, &size_rx, &trans_id_rx);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  tmp_value =  clock_diff(&tp_start, &tp_end);
  if(tmp_value < mailbox_read_min_time)
    mailbox_read_min_time = tmp_value;
  if(tmp_value > mailbox_read_max_time)
    mailbox_read_max_time = tmp_value;
  total_mailbox_read_time += tmp_value;
  total_mailbox_read_cnt++;
   demo667x4_assert( (ret_val == 0), node_id, "ERROR: mailBox_read(host <-- dsp) ");

  printf("\nMsg (%08X) from --> %2d : input_addr = 0x%08X, input_size = %10d, output_addr = 0x%08X, output_size = %10d \n\n",
    trans_id_rx,node_id,dspMsg.input_addr,dspMsg.input_size,dspMsg.output_addr,dspMsg.output_size);

  clock_gettime(CLOCK_MONOTONIC, &tp_start);
  ret_val = pciedrv_dsp_read((node_id / DEMO_CONFIG_CORES_PER_CHIP),dspMsg.output_addr,outputBuf,dspMsg.output_size);
  clock_gettime(CLOCK_MONOTONIC, &tp_end);
  demo667x4_assert( (ret_val == 0), node_id, "ERROR: dma read failed");
  if(dspMsg.output_size ==demo_payload_size) {
    total_read_time += clock_diff (&tp_start, &tp_end);
    total_read_time_cnt ++;
  }
  fwrite(outputBuf,1,dspMsg.output_size,outFile);

  printf("bufmgrFree(%d) = 0x%08X\n", node_id, dspMsg.input_addr);
  ret_val = bufmgrFreeUserAddr( inBufHandle, (void *)(intptr_t)dspMsg.input_addr);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrFreeUserAddr() for input buffer.");

  printf("bufmgrFree(%d) = 0x%08X\n", node_id, dspMsg.output_addr);
  ret_val = bufmgrFreeUserAddr( outBufHandle, (void *)(intptr_t)dspMsg.output_addr);
  demo667x4_assert( (ret_val == BUF_MGR_ERROR_NONE), node_id, "ERROR: bufmgrFreeUserAddr() for output buffer.");

 }

/**
 *  @brief Function do_filetest :  Do file test: Demo to send data and 
 *                  receive data to DSP
 *  @param[in]         nodeBits   Node bit map
 *  @param[in]         inFile     Input file pointer
 *  @param[in]         outFile    Output file pointer
 *  @param[in]         flag       0 : Memcopy test 1: DSP Map test 2: DMA test
 *  @retval            none 
 *  @pre  
 *  @post 
 */

void do_filetest(uint32_t nodeBits, FILE *inFile, FILE *outFile, uint32_t flag)
{
  int32_t node_id, ret_val;
  uint32_t trans_id = 0xC0DE0000 ;
  uint32_t read_cnt, write_cnt;

  printf("Starting Test...\n");
  for ( node_id = 0; node_id < (num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP); node_id++)
  {
    if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
    {
      /* Send message through the mechanism chosen for demo */
      switch(flag)
      {
        case DEMO_MODE_DSPMAP_TEST:
/*         NO Priming for map test */
//          read_and_send_data_to_dsp_through_mapping(node_id, trans_id++, inFile);
          break;
        case DEMO_MODE_DMA_TEST:
          read_and_send_data_to_dsp_through_dma(node_id, trans_id++, inFile);
          break;

        case DEMO_MODE_MEMCPY_TEST:
        default:
          read_and_send_data_to_dsp_through_memcpy(node_id, trans_id++, inFile);
          break;
      }
    } 
  }
  
  printf("Priming done.\n");

  while (!feof(inFile))
  {

    /* Read next buffer from file and send to Maibox */
    for ( node_id = 0; node_id < (num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP); node_id++)
    {
      if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
      {
        /* Send message through the mechanism chosen for demo */
        switch(flag)
        {
          case DEMO_MODE_DSPMAP_TEST:
            read_and_send_data_to_dsp_through_mapping(node_id, trans_id++, inFile);
            break;
          case DEMO_MODE_DMA_TEST:
            read_and_send_data_to_dsp_through_dma(node_id, trans_id++, inFile);
            break;
          case DEMO_MODE_MEMCPY_TEST:
          default:
            read_and_send_data_to_dsp_through_memcpy(node_id, trans_id++, inFile);
          break;
        }
      }
    }

    /* Read any processed frames back from DSP and write to file */ 
    for ( node_id = 0; node_id < (num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP); node_id++)
    {
      if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
      {     
        /* Wait for message from  DSP node, through mailBOx */   
        do {
          ret_val = mailBox_query(rx_mailbox_handle[node_id]);
        } while(!ret_val);

        /* Read mailBox and read data through mechanism chosen for demo */
        switch(flag)
        {
          case DEMO_MODE_DSPMAP_TEST:
            get_data_through_mapped_memory_and_write_to_file(node_id, outFile);
            break;
          case DEMO_MODE_DMA_TEST:
            get_data_through_dma_and_write_to_file(node_id, outFile);
            break;
          case DEMO_MODE_MEMCPY_TEST:
          default:
            get_data_through_memcpy_and_write_to_file(node_id, outFile);
          break;
        }
      }
    }
  }

  /* Read any frames still pending to be received and write to file */
  for ( node_id = 0; node_id < (num_of_dsps*DEMO_CONFIG_CORES_PER_CHIP); node_id++)
  {
    if( nodeBits & (((uint32_t) 1) << (node_id%32)) )
    {
      /* Read mailBox and read data through mechanism chosen for demo */
      switch(flag)
      {
        case DEMO_MODE_DSPMAP_TEST:
//          get_data_through_mapped_memory_and_write_to_file(node_id, outFile);
          break;
        case 2:
          /* Wait for message from  DSP node, through mailBOx */   
          do {
            ret_val = mailBox_query(rx_mailbox_handle[node_id]);
          } while(!ret_val);
          get_data_through_dma_and_write_to_file(node_id, outFile);
          break;
        case 0:
        default:
          /* Wait for message from  DSP node, through mailBOx */   
          do {
          ret_val = mailBox_query(rx_mailbox_handle[node_id]);
          } while(!ret_val);
          get_data_through_memcpy_and_write_to_file(node_id, outFile);
          break;
      }
    }
  }
  printf(" End of Test...\n");fflush(stdout);

}
/**
 *  @brief Function do_mode3_filetest :  Do file test: Demo to send data and 
 *                  receive data to DSP
 *  @param[in]         nodeBits   Node bit map
 *  @param[in]         inFile     Input file pointer
 *  @param[in]         outFile    Output file pointer
 *  @retval            none 
 *  @pre  
 *  @post 
 */

void do_mode3_filetest(uint32_t nodeBits, FILE *inFile, FILE *outFile)
{
  int32_t node_id, ret_val;
  uint32_t trans_id = 0xC0DE0000 ;
  uint32_t read_cnt, write_cnt;
  int i;
  uint32_t num_dsps;

  num_dsps = num_of_dsps;

  if(demo_payload_size > DEMO_HOST_HUGE_BUFFER_SIZE)
  {
    printf("\n ERROR: payload size > Huge buffer size \n");
    exit(-1);
  }
  /* Free any dynamic memory allocated */
  if(cmem_drv_free(0,
         HOST_BUF_TYPE_DYNAMIC ,  NULL)!=0)
  {
    printf("ERROR: contiguous memory free failed\n");
    exit(-1);
  }

  /* Meaning of dsp bits is here number of dsps to run test on */
  /* Allocate contiguous memory for DMA */
  if(cmem_drv_alloc(num_dsps, DEMO_HOST_HUGE_BUFFER_SIZE ,
    HOST_BUF_TYPE_DYNAMIC ,  huge_buf_desc)!=0)
    {
      printf("ERROR: contiguous memory allocation failed\n");
      exit(-1);
    }
    printf("\n Contiguous memory allocation complete : Num Buffers %d\n", num_dsps); 
    for( i=0; i < num_dsps; i++) 
    {
      printf("Allocated buffers: Buff no :%d, Buffer length %x\n", i, huge_buf_desc[i].length);
        /* Map entire memory */
        /* Allocate DSP range */
        ret_val = pciedrv_dsp_memrange_alloc(i,
          DEMO_HOST_HUGE_BUFFER_SIZE, &dsp_mapped_huge_buf_addr[i]);
        demo667x4_assert( (ret_val == 0), i, "ERROR: Memory range alloc failed.");

        /* Map Input buffers to dsp range */
        ret_val = pciedrv_map_hostbufs_to_dsp_memrange(i,
              1, 
              &huge_buf_desc[i], 
              dsp_mapped_huge_buf_addr[i]);
        demo667x4_assert( (ret_val == 0), i, "ERROR: Map to dsp Memory failed.");
    }
  printf("\n Huge memory configuration done \n");
  while (!feof(inFile))
  {

    /* Read next buffer from file and send to Maibox */
    for ( i = 0; i  < num_dsps; i++)
    {
      read_and_send_data_to_dsp_through_huge_buffer(i, trans_id++, inFile);
    }

    /* Read any processed frames back from DSP and write to file */ 
    for ( i = 0; i < num_dsps; i++)
    {
        node_id = (i*DEMO_CONFIG_CORES_PER_CHIP);
        /* Wait for message from  DSP node, through mailBOx */   
        do {
          ret_val = mailBox_query(rx_mailbox_handle[node_id]);
        } while(!ret_val);

       get_data_through_huge_buffer_and_write_to_file(i, outFile);
    }
  }
  /* Free buffers */
  if(cmem_drv_free(MAX_NUM_HOST_DSP_BUFFERS,
       HOST_BUF_TYPE_DYNAMIC ,  &huge_buf_desc[0/*i*/])!=0)
  {
    printf("ERROR: contiguous memory free failed\n");
    exit(-1);
  }
  /* Close contiguous memory driver */
  if(cmem_drv_close() != 0) {
    printf("ERROR: Error closing driver API\n");
  }
}


/**
 *  @brief Function main :  Main function
 *  @param[in]         argc       Command line argument count
 *  @param[in]         argv       Command line arguments
 *  @retval            none 
 *  @pre  
 *  @post 
 */
void main(int32_t argc, int8_t **argv)
{
  FILE *inFile, *outFile;
  uint32_t dspBits = 0;
  int32_t  i, core_id;
  cmem_host_buf_desc_t buf_desc[MAX_NUM_HOST_DSP_BUFFERS];
  pciedrv_open_config_t pciedrv_open_config, *pciedrv_open_configP=NULL;
  uint32_t flag = 0;
  int32_t ret_val;
  uint32_t max_payload_size;
  void *dsp_image_handle;
  uint32_t dsp_entry_point;

  /* Read parameters from command line */
   if(argc < 8)
  {
    printf("Usage: %s <input_file> <output_file> \
<test type flag: 0: Memcpy 1: DSP Map 2: DMA 3: Hugemem > \
        <num_of_dsps> <dsp_bits> < payload size in hex > <dsp_image>\n",argv[0]);
    exit(-1);
  }
  /* Open input file for reading */
  inFile = fopen(argv[1],"rb");
  if(inFile == NULL)
  {
    printf("Error opening input file: %s\n", argv[1]);
    exit(-1);
  }
  /* Open output file */
  outFile = fopen(argv[2],"wb");
  if(outFile == NULL)
  {
    fclose(inFile);
    printf("Error opening output file: %s\n", argv[2]);
    exit(-1);
  }

  /* Record flag to indicate type of test*/
  flag = atoi(argv[3]);

  sscanf (argv[4], "%x", &dspBits);
  printf("dspBits = %08X \n",dspBits);

  if(dspBits == 0)
    dspBits = 1;

  printf("dspBits = %08X \n",dspBits);

  sscanf(argv[5], "%d", &num_of_dsps);
  if(num_of_dsps > DEMO_FILETEST_MAX_NUM_CHIPS)
  {
    num_of_dsps = DEMO_FILETEST_MAX_NUM_CHIPS;
    printf("\n Number of dsps limited to %d",num_of_dsps);
  }

  sscanf(argv[6], "%x", &demo_payload_size);

  /* Get DSP Image */
  if(dnldmgr_get_image(argv[7],&dsp_image_handle, &dsp_entry_point) != 0)
  {
    printf("\n ERROR: Get DSP image failed ");
    exit(-1);
  }

  if(dnldmgr_get_symbol_address(dsp_image_handle, "host2dspmailbox", &host2dspmailbox) != 0)
  {
    printf("\n ERROR: Symbol host2dspmailbox not found ");
    exit(-1);
  }
  if(dnldmgr_get_symbol_address(dsp_image_handle, "dsp2hostmailbox", &dsp2hostmailbox) != 0)
  {
    printf("\n ERROR: Symbol dsp2hostmailbox not found ");
    exit(-1);
  }
  /* free DSP Image handle */
  dnldmgr_free_image(dsp_image_handle);

  /* Check limits on payload size */
  max_payload_size = DEMO_FILETEST_MAX_PAYLOAD_BUFFER_SIZE;
  if(demo_payload_size > max_payload_size)
  {
    demo_payload_size = max_payload_size;
    printf("\n Payload size limited to 0x%x",demo_payload_size);
  }

  /* Configure pciedrv if required */
  if(flag != 0)
  {
    memset(&pciedrv_open_config, 0 , sizeof(pciedrv_open_config_t));
    /*Set  open config parameters */
    /* These defines used are in demo.h */
    pciedrv_open_config.dsp_outbound_reserved_mem_size = DSP_OB_MEM_REGION_SIZE_RESERVED;
    pciedrv_open_config.dsp_outbound_block_size = DSP_OB_REGION_SIZE;
    pciedrv_open_config.max_dma_transactions = MAX_DMA_TRANSACTIONS;

    /* These define are in filetestdemo.h : common to DSP */
    pciedrv_open_config.start_dma_chan_num = START_DMA_NUM;
    pciedrv_open_config.num_dma_channels = NUM_DMA_CHANNELS_FOR_HOST_DSP_XFERS;
    pciedrv_open_config.start_param_set_num = START_PARAM_SET_NUM;
    pciedrv_open_config.num_param_sets = NUM_PARAM_SETS;

    pciedrv_open_configP = &pciedrv_open_config;
  }
  
  /* Open pcie driver */
  if(pciedrv_open(pciedrv_open_configP) != 0) {
    printf("Driver open error \n");
    goto err_driver_open;
  }
 
  printf("Driver opened!\n");

  /* Depending on mode : allocate memory buffers */
  if(flag != DEMO_MODE_MEMCPY_TEST)
  { 
    /* Open contiguous memory driver */
    if(cmem_drv_open() !=0) {
      printf("\nERROR: dma mem driver open failed \n");
      exit(-1);
    }
  }
    switch(flag) 
    {
      case DEMO_MODE_DSPMAP_TEST:
        /* Initialise buffer desc table */
        memset(&dsp_addr_desc_map_table, 0, sizeof(dsp_addr_desc_map_table));
      case DEMO_MODE_DMA_TEST: 
        /* Free any dynamic memory allocated */
        if(cmem_drv_free(0,
           HOST_BUF_TYPE_DYNAMIC ,  buf_desc)!=0)
        {
          printf("ERROR: contiguous memory free failed\n");
          exit(-1);
        }
        /* Allocate contiguous memory for DMA */
        if(cmem_drv_alloc(MAX_NUM_HOST_DSP_BUFFERS, HOST_CMEM_BUFFER_SIZE,
          HOST_BUF_TYPE_DYNAMIC ,  buf_desc)!=0)
        {
          printf("ERROR: contiguous memory allocation failed\n");
          exit(-1);
        }
        printf("\n Contiguous memory allocation complete : Num Buffers %d\n",MAX_NUM_HOST_DSP_BUFFERS); 

        /* Create buffer pool for Host DSP buffers */
        if(bufmgrCreate(&hostDmaBufPool, MAX_NUM_HOST_DSP_BUFFERS, buf_desc ) != 0)
        {
          printf("ERROR:Memory pool initialisation failed\n");
          exit(-1);
        }
        /* Initialise Buffer Manager */
        demo667x4_initDspBufs();
        break;

      case DEMO_MODE_MEMCPY_TEST:
         /* Initialise Buffer Manager */
        demo667x4_initDspBufs();
        break;

      default:
	/*Nothing to do for this case */
       break;

     }  
  /* Configure required sdk modules */
  demo667x4_Config(dspBits);
  printf("\t\t\tConfiguration Complete!\n");

   /* Run test based on flag */
  switch(flag) {
    case DEMO_MODE_HUGE_BUF_TEST:
      do_mode3_filetest(dspBits,inFile,outFile);
      break;

    default:
     /* Run the actual test */
      do_filetest(dspBits,inFile,outFile, flag);
      break;
  }

  /* Free resources */
  demo667x4_Config_free(dspBits);
  printf("\n Resources free complete\n");fflush(stdout);
  
  /* Delete buffers and pools, based on flag */
  switch(flag)
  {
    case DEMO_MODE_DMA_TEST:
      /*Test complete :  Delete Buffer pools */
      demo667x4_deletePools();

    case DEMO_MODE_DSPMAP_TEST: 
      bufmgrDelete(&hostDmaBufPool);
      if(cmem_drv_free(MAX_NUM_HOST_DSP_BUFFERS,
           HOST_BUF_TYPE_DYNAMIC ,  buf_desc)!=0)
        {
          printf("ERROR: contiguous memory free failed\n");
          exit(-1);
        }
      /* Close contiguous memory driver */
      if(cmem_drv_close() != 0) {
        printf("ERROR: Error closing driver API\n");
      }
      break;
   
    case DEMO_MODE_MEMCPY_TEST:
      /*Test complete :  Delete Buffer pools */
      demo667x4_deletePools();
      break;
   
  }

  /* Close pcie driver */
  pciedrv_close();

  /* Print stats */
  printf("\n Payload size : %d bytes ",demo_payload_size); 
  if(total_dma_read_time_cnt && total_dma_write_time_cnt)
  {
   printf ("\nTotal_dma_read_time=%lld ns total_dma_read_cnt=%d", (long long)total_dma_read_time, total_dma_read_time_cnt);
   printf ("\nTotal_dma_write_time=%lld ns total_dma_write_cnt=%d", (long long)total_dma_write_time, total_dma_write_time_cnt);
   printf("\n dma_read_time %lld ns , dma write_time %lld ns ", 
      (long long)(total_dma_read_time/total_dma_read_time_cnt),
      (long long)(total_dma_write_time/total_dma_write_time_cnt));
   printf("\n Transfer rate in Mbytes /sec Read %d, Write %d\n", (uint32_t)((((uint64_t)1000 * demo_payload_size)/(total_dma_read_time/total_dma_read_time_cnt))),
     (uint32_t)((((uint64_t)1000 * demo_payload_size)/(total_dma_write_time/total_dma_write_time_cnt))));
  }
  if(total_read_time_cnt && total_write_time_cnt)
  {

    printf ("\nTotal_read_time=%lld ns total_read_cnt=%d", (long long)total_read_time, total_read_time_cnt);
    printf ("\nTotal_write_time=%lld ns total_write_cnt=%d", (long long)total_write_time, total_write_time_cnt);
    printf("\n read_time %lld ns , total write_time %lld ns ", 
      (long long)(total_read_time/total_read_time_cnt),
      (long long)(total_write_time/total_write_time_cnt));
    printf("\n Transfer rate in Mbytes /sec Read %d, Write %d\n", 
        (uint32_t)((((uint64_t)1000 * demo_payload_size)/(total_read_time/total_read_time_cnt))),
        (uint32_t)((((uint64_t)1000 * demo_payload_size)/(total_write_time/total_write_time_cnt))));
  }
  if( file_read_time_cnt && file_write_time_cnt)
  {
    printf ("\ntotal file_read_time=%lld ns total file_read_cnt=%d", (long long)file_read_time, file_read_time_cnt);
    printf ("\nfile_write_time=%lld ns file_write_cnt=%d", (long long)file_write_time, file_write_time_cnt);
    printf("\n file read_time %lld ns , file write_time %lld ns ", 
        (long long)(file_read_time/file_read_time_cnt),
        (long long)(file_write_time/file_write_time_cnt));

    printf ("\nmemcpy_read_time=%lld ns memcpy_write_time=%lld ns\n", (long long)(memcpy_read_time/file_read_time_cnt), (long long)(memcpy_write_time/file_write_time_cnt));
  }
  if(total_mailbox_read_cnt && total_mailbox_write_cnt)
  {
    printf ("\ntotal mailbox_read_time=%lld ns total_mailbox_write_time=%lld ns",
      (long long)(total_mailbox_read_time), (long long)(total_mailbox_write_time));
    printf ("\ntotal mailbox_read_cnt=%lld  total_mailbox_write_cnt=%lld",
        (long long)(total_mailbox_read_cnt), (long long)(total_mailbox_write_cnt));
    printf ("\n mailbox_read_min_time=%lld ns mailbox_write_min_time=%lld ns",
      (long long)(mailbox_read_min_time), (long long)(mailbox_write_min_time));
    printf ("\n mailbox_read_max_time=%lld ns mailbox_write_max_time=%lld ns",
      (long long)(mailbox_read_max_time), (long long)(mailbox_write_max_time));
    printf ("\n mailbox_read_time=%lld ns mailbox_write_time=%lld ns",
      (long long)(total_mailbox_read_time/total_mailbox_read_cnt), (long long)(total_mailbox_write_time/total_mailbox_write_cnt));
  }
  printf("\nTest Complete\n");
close_files_and_exit:


err_driver_open:
  fclose(inFile);
  fclose(outFile); 
exit(0);

}
/*** nothing past this point ***/
