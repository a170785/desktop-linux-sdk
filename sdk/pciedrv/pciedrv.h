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



#ifndef _PCIEDRV_H
#define _PCIEDRV_H
#include <stdint.h>
#include "inc/buffdesc.h"

#define PCIEDRV_DMA_XFER_BLOCKING     0x0
#define PCIEDRV_DMA_XFER_NON_BLOCKING 0x1


typedef struct _pciedrv_open_config_t {
/* Driver uses DSP DMA to transfer data from and to DSP 
   Some dma channels need to be reserved for use by host.
   The following parameters specifies the starting dma channel 
   number and the number of dma channels reserved for host use */
   uint16_t start_dma_chan_num;
   uint16_t num_dma_channels;
/* Some DMA param sets need to be reserved for use by host.
   The following parameters specifies the starting dma param sets 
   and the number of param sets reserved for host use */
   uint16_t start_param_set_num;
   uint16_t num_param_sets;

/* DSP PCIE end point outbound regions are configurable through PCIE registers.
    For example in C6678, the region starting from 0x6000 0000-0x6fffffff is 
    available for mapping on the DSP. In some cases ( for example to reserve 
    for  global shared memory), you may not want to use the whole region for
    dynamic mapping. A portion of this memory can be marked as reserved, so 
    that this region for all DSPs are not touched by the driver. This parameter
    "dsp_outbound_reserved_mem_Size" specifies the initial region to be marked
    as reserved. If no global shared memory is required, this can be set to 0*/
  uint32_t dsp_outbound_reserved_mem_size;

/* The following parameter specifies the DSP Outbound block size configured as a global parameter.  The sizes can be configured as 1 MB, 2 MB, 4 MB or 8 MB only*/
   uint32_t dsp_outbound_block_size;

/* Driver keeps track of DMA transactions initiated using the DSP DMA resources
   the following parameter specifies the max number of dma transactions to be 
   tracked simultaneously per dsp */
   uint16_t max_dma_transactions;

} pciedrv_open_config_t;

typedef struct _pciedrv_device_info_t {
  uint32_t switch_device;
  char sysfspath[256];
//  uint32_t ddr_mem_size;
} pciedrv_device_info_t;

/**
 *  @brief Function pciedrv_open() Open the PCIe driver 
 * 	   Driver searches for the TI Shannon PCIE devices on the bus
 *         Probes the device to make them active and sets up access to 
 *         PCIE BAR regions to communicate with the DSP
 *  @param[in]     open_config   Open configuration pointer;
 *                 NULL-> no resource allocation functionality
 *                 (To use DMA, DSP outbound memory allocation i
 *                 open_config  should be specified.)
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_open(pciedrv_open_config_t *open_config);
/**
 *  @brief Function pciedrv_close() Closes the PCIe driver  and free all
 *         allocated resources
 *  @param[in, out]     none
 *  @retval        0 for success, -1 for failure
 *  @pre  pciedrv_open should have called before calling this routine
 *  @post 
 */
int32_t pciedrv_close(void);

/**
 *  @brief Function pciedrv_dsp_write() Write to DSP memory using 
 *                  memcpy over PCIe
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Address to write
 *  @param[in]     buf            Source Buffer pointer
 *  @param[in]     size           size of Memory write in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_write(int32_t dsp_id, uint32_t addr, uint8_t *buf,
  uint32_t size);

/**
 *  @brief Function pciedrv_dsp_read() Read from DSP memory using
 *                  memcpy over PCIe
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Address to write
 *  @param[in]     buf            Destination Buffer pointer
 *  @param[in]     size           size of Memory read in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_read(int32_t dsp_id, uint32_t addr, uint8_t *buf, 
  uint32_t size);

/**
 *  @brief Function pciedrv_dsp_set_entry_point() Set entry point by
 *         writing to reserved location used with dsp reset through pcie
 *  @param[in]     dsp_id         DSP or device id
 *  @param[in]     core_id        DSP core id
 *  @param[in]     entry_point    Entry point address
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_set_entry_point(uint32_t dsp_id, uint32_t core_id,
  uint32_t entry_point);


typedef enum 
{
 PCIEDRV_PCIEEP_SET_INTA_SET=1,  /* Set interrupt A for DSP EP*/
 PCIEDRV_PCIEEP_SET_INTA_CLR,    /* Clear interrupt A for DSP EP*/
 PCIEDRV_PCIEEP_SET_INTB_SET,    /* Set interrupt B for DSP EP*/
 PCIEDRV_PCIEEP_SET_INTB_CLR,    /* Clear interrupt B for DSP EP*/
 PCIEDRV_PCIEEP_SET_MASTER_PRIV_SET /* Set master privilege for pcie access,
                                       Used with dsp reset through PCIe.
                                       Required to access Boot config registers
                                       through PCIE  */
} pciedrv_pcieep_set_type_t;
 

/**
 *  @brief Function pciedrv_pcieep_set_config() Set PCIE endpoint configuration
 *  @param[in]     dsp_id         DSP or device id
 *  @retval        0: success -1: Fail
 *  @pre  
 *  @post 
 */
int32_t pciedrv_pcieep_set_config(uint32_t dsp_id, 
  pciedrv_pcieep_set_type_t pcieep_set_type);

/**
 *  @brief Function pciedrv_dma_read_initiate() Initiate Read DMA from 
 *         DSP Address to provided contiguous host buffers. Flag determines
 *         blocking or non-blocking.
 *  @param[in]    dsp_id                DSP or device id
 *  @param[in]    dspAddr               DSP Address to read
 *  @param[in]    frame_desc            Frame descriptor to read data into 
 *  @param[in]    flag                  Flag to indicate wait till completion
 *                                      or not ( blocking/non-blocking)
 *  @param[out]   trans_id              Returned transaction id
 *  @retval       0: for success, -1 for failure 
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dma_read_initiate(int32_t dsp_id,
  uint32_t dspAaddr, cmem_host_frame_desc_t *frame_desc, uint32_t flag,  
  uint32_t *trans_id);

/**
 *  @brief Function pciedrv_dma_write_initiate() Initiate Write DMA to
 *            DSP Address from provided contiguous host buffers. Flag 
 *            determines blocking or unblocking.
 *  @param[in]    dsp_id                DSP or device id
 *  @param[in]    dspAddr               DSP Address to write
 *  @param[in]    frame_desc            Frame descriptor of data to be written
 *  @param[in]    flag                  Flag to indicate wait till completion
 *                                      or not ( blocking/non-blocking)
 *  @param[out]   trans_id              Returned transaction id
 *  @retval       0: for success, -1 for failure 
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dma_write_initiate(int32_t dsp_id,
  uint32_t dspAddr, cmem_host_frame_desc_t *frame_desc,  uint32_t flag, 
  uint32_t *trans_id);

/**
 *  @brief Function pciedrv_dma_check() Check if the dma operation is complete
 *                 Used in case of a DMA initiate (NON blocking). If
 *                 the dma is complete, free transaction 
 *  @param[in]    dsp_id      DSP or device id
 *  @param[in]    trans_id    Transaction id to check
 *  @retval        -1 for failure, 0 : DMA complete 1: DMA in progress
 *  @pre  call to pciedrv_dma_read_initiate or pciedrv_dma_write_initiate
 *  @post 
 */
int32_t pciedrv_dma_check(uint32_t dsp_id, uint32_t trans_id);

/**
 *  @brief Function pciedrv_dsp_memrange_alloc() Allocate dsp outbound mem range
 *         The allocated areas can be used to map Host buffer, so that DSP can 
 *         access the host  buffers directly
 *         If reserved memory is needed, alloc can be called at the beginning of 
 *         application execution and freed only at the end of application
 *  @param[in]    dsp_id          DSP or device id
 *  @param[in]    mem_size        size of dsp outbound memory to allocate
 *  @param[out]   dsp_start_addr  Allocated DSP start address
 *  @retval       0: success, -1 : failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_memrange_alloc(uint32_t dsp_id, uint32_t mem_size, 
  uint32_t *dsp_start_addr);

/**
 *  @brief Function pciedrv_dsp_memrange_free() Free dsp outbound mem range
 * ( If reserved memory is needed, It is recommended to call alloc at the
 * beginning of application execution and this will avoid defragmentation 
 * due to repeated aloc and free)
 *  @param[in]    dsp_id      DSP or device id
 *  @param[in]    mem_size    size of dsp outbound memory to allocate
 *  @param[out]   dsp_start_addr  Allocated DSP start address
 *  @retval       0: success, -1 : failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_memrange_free(uint32_t dsp_id, uint32_t mem_size, 
  uint32_t dsp_start_addr);

/**
 *  @brief Function pciedrv_map_hostbufs_to_dsp_memrange() Map host buffers to the 
 *         allocated DSP outbound memory range 
 *  @param[in]    dsp_id          DSP or device id
 *  @param[in]    num_of_bufs     Number of host buffers
 *  @param[in]    buf_desc        list of host buffer descriptors 
 *  @param[out]   dsp_start_addr  DSP start address to map 
 *  @retval       0: success, -1 : failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_map_hostbufs_to_dsp_memrange(uint32_t dsp_id, uint32_t num_of_bufs,
  cmem_host_buf_desc_t buf_desc[], uint32_t dsp_start_addr);

/**
 *  @brief Function pciedrv_get_pcie_addr() Get pcie_address for bar window region
 *  @param[in]    dsp_id        DSP or device id
 *  @param[in]    bar_num       Bar number
 *  @retval       pcie address of Bar window region
 *  @pre  
 *  @post 
 */
uint32_t pciedrv_get_pcie_addr(uint32_t dsp_id, uint32_t bar_num);

/**
 *  @brief Function pciedrv_get_num_devices() Get number of the TI devices on the PCI bus
 *  @retval       number of devices
 *  @pre          pciedrv_open should be called before calling this function
 *  @post 
 */
uint32_t pciedrv_get_num_devices(void);

/**
 *  @brief Function pciedrv_get_pci_info() Get pci information of all the TI devices
 *  @param[in]    devices_info : To populat the devices information
 *  @param[in]    bar_num       Bar number
 *  @retval       0: success, -1 : failure
 *  @pre          pciedrv_open should be called before calling this function
 *  @post 
 */
int32_t pciedrv_get_pci_info(pciedrv_device_info_t *devices_info);

/**
 *  @brief Function pciedrv_map_dspaddr_to_inbound() Maps the inbound range of a DSP.  
 *                       This maps the DSPs memory to be available for inbound access.
 *  @param[in]    dsp_id : ID of the DSP 
 *  @param[in]    dsp_addr : Address to map for inbound access.
 *  @param[in]    size : Size to map. 
 *  @retval       0: success, -1 : failure
 *  @pre          pciedrv_open should be called before calling this function
 *  @post         pciedrv_map_outbound_to_rmt_dspaddr() can be called after this to make 
 *                this memory visible on another DSP.
 *
 *  E.g. 
 *
 *  -- Make 0x80000000 - 0x8FFFFFFF on DSP 0 available for other DSPs to map  
 *       (Size that can be mapped depends on alignment, IB window size)
 *
 *  ret = pciedrv_map_dspaddr_to_inbound( 0, 0x80000000, 0x10000000);
 *
 *  -- Map DSP 0's 0x80000000 - 0x803FFFFF on DSP 1 at "dsp_1_addr"
 *  ret = pciedrv_map_outbound_to_rmt_dspaddr( 1, 0, 0x80000000, 0x400000, &dsp_1_addr);
 *  
 *  -- Map DSP 0's 0x80400000 - 0x807FFFFF on DSP 2 at "dsp_2_addr"
 *  ret = pciedrv_map_outbound_to_rmt_dspaddr( 2, 0, 0x80400000, 0x400000, &dsp_2_addr);
 *  
 */
int32_t pciedrv_map_dspaddr_to_inbound(int32_t dsp_id, uint32_t dsp_addr, uint32_t size);


/**
 *  @brief Function pciedrv_free_inbound_mapping() Will free the inbound register so 
 *                                                 that it may be remapped.
 *
 *  @param[in]    dsp_id : ID of the DSP 
 *  @param[in]    dsp_addr : Address to map for inbound access.
 *  @param[in]    size : Size to map.
 *  @retval       0: success, -1 : failure
 */
int32_t pciedrv_free_inbound_mapping(int32_t dsp_id, uint32_t dsp_addr, uint32_t size);

/**
 *  @brief Function pciedrv_map_outbound_to_rmt_dspaddr() Maps the outbound range of a DSP.  
 *
 *                       This maps a remote DSP memory (which has been made available through
 *                       pciedrv_map_dspaddr_to_inbound()), to another DSPs outbound range.
 *
 *  @param[in]    dsp_id : ID of the DSP which outbound range will be mapped.
 *  @param[in]    rmt_dsp_id: ID of the DSP whose memory will be mapped to dsp_id.
 *  @param[in]    rmt_dsp_addr : Address on remote DSP which will be mapped to dsp_id.
 *  @param[in]    size : Size to map. 
 *  @param[out]   mapped_dsp_addr: location on dsp_id where the remote DSPs memory has been mapped.
 *  @retval       0: success, -1 : failure
 *  @pre          pciedrv_open should be called before calling this function
 *  @pre          pciedrv_map_dspaddr_to_inbound() should be called before this to make 
 *                the remote memory available to be mapped.
 *
 *  E.g. 
 *
 *  -- Make 0x80000000 - 0x8FFFFFFF on DSP 0 available for other DSPs to map  
 *       (Size that can be mapped depends on alignment, IB window size)
 *
 *  ret = pciedrv_map_dspaddr_to_inbound( 0, 0x80000000, 0x10000000);
 *
 *  -- Map DSP 0's 0x80000000 - 0x803FFFFF on DSP 1 at "dsp_1_addr"
 *  ret = pciedrv_map_outbound_to_rmt_dspaddr( 1, 0, 0x80000000, 0x400000, &dsp_1_addr);
 *  
 *  -- Map DSP 0's 0x80400000 - 0x807FFFFF on DSP 2 at "dsp_2_addr"
 *  ret = pciedrv_map_outbound_to_rmt_dspaddr( 2, 0, 0x80400000, 0x400000, &dsp_2_addr);
 *  
 */
int32_t pciedrv_map_outbound_to_rmt_dspaddr(int32_t dsp_id, int32_t rmt_dsp_id, uint32_t rmt_dsp_addr, uint32_t size, uint32_t *mapped_dsp_addr);



#endif /* _PCIEDRV_H */

