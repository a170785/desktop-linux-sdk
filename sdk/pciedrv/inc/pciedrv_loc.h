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



#ifndef _PCIEDRV_LOC_H
#define _PCIEDRV_LOC_H
#include <stdint.h>

#define MAX_NUM_BAR_WINDOWS	  5

#define DMA_QUEUE_RD         0
#define DMA_QUEUE_WR         1
#define DMA_CHAN_RD          2
#define DMA_CHAN_WR          3
#define DMA_CHAN_DSP_TO_DSP  4

#define PARAM_SET_RD          0
#define PARAM_SET_WR          1
#define PARAM_SET_DSP_TO_DSP  2

#define DMA_TIMEOUT  100000

/**
 * Represents a memory region (essentially a BAR)
 */
typedef struct memregion_s {
     /**
      *	1 if present (in the sense that it was setup under enumeration).
      */
     int is_present;
     /** 
      *	 >0 if used 0 otherwise
      */
     uint32_t use_count;
     /**
      * 1 if this is the configuration region 0 otherwise
      */
     int is_config;
     /**
      * 1 if memory is at 64-bit addresses.
      */
     int is_64;
     /** 
      * BAR this region is bound to
      */     
     int bar;
     /** 
      * Inbound address translation used by this region (Note that
      *	this representation implies a one-to-one usage between bars
      *	and inbound translation registers which is actually not
      *	necessary but currently the custom in this application).
      */
     int ib_bar;

     /**
      *	Address the region has been mapped to in virtual space, or
      *	NULL if not mapped.
      */
     volatile uint8_t* mem;

     /**
      *	Address the region has been mapped to on the DSP, or NULL if
      *	not mapped.
      */
     uint32_t dspmem;

     /** 
      *	 Size 
      */
     size_t size;

} memregion_t;


/**
   Struct representing one keystone device
 */
typedef struct pcie_end_point_s {
     /**
	Memory regions 
     **/
     memregion_t memregions[MAX_NUM_BAR_WINDOWS];
} pcie_end_point_t;

typedef struct driver_inst_s
{
  struct pci_device **pcidevices;
  pcie_end_point_t *pcieendpoint;
} driver_inst_t;


typedef struct _dma_transaction_t {
    uint32_t trans_id;
    uint16_t dma_chan_number;
    uint16_t param_set_number;
    uint16_t num_param_sets;
    uint16_t ob_reg_number;
    uint16_t num_ob_registers;
    uint32_t flags;
} dma_transaction_t;

typedef struct _remote_map_transaction_t {
    uint32_t trans_id;
    uint16_t ob_reg_number;
    uint16_t num_ob_registers;
    uint32_t flags;
} remote_map_transaction_t;
#define DMA_TRANSACTION_USED 0x1 

#define REMOTE_MAP_TRANSACTION_USED 0x1

#define PCIE_USPACE_FLAG_RESOURCE_INIT_DONE 0x1

typedef struct _pciedrv_context_t {
  uint32_t num_devices;
  uint32_t num_ob_instances;
  uint32_t trans_id_count;
  uint16_t num_param_sets;
  uint16_t num_dma_channels;
  uint16_t max_dma_transactions;
  uint16_t max_remote_map_transactions;
  uint32_t dsp_outbound_block_size;
  uint32_t local_processor_id;
  uint32_t flag;
} pciedrv_context_t;


typedef struct _pciedrv_obreg_alloc_inst_t {
  uint32_t flags;
  uint16_t ob_reg_number;
} pciedrv_obreg_alloc_inst_t;

typedef struct _pciedrv_dma_alloc_inst_t {
  uint32_t flags;
  uint16_t dma_chan_number;
} pciedrv_dma_alloc_inst_t;

typedef struct _pciedrv_param_set_alloc_inst_t {
  uint32_t flags;
  uint16_t param_set_number;
} pciedrv_param_set_alloc_inst_t;

    

#define OBREG_FLAG_ALLOCATED	 0x1
#define DMA_FLAG_ALLOCATED	 0x1


#define EP_MAP_OFFSET_ALIGN      0x000FFFFF  /* Mapping alignment */
#define EP_MAP_ALIGN             0xFFF00000  /* Mapping alignment */
#define minOffsetAlign(x,y)      (((x)<(y))?(x):(y))
#endif /* _PCIEDRV_LOC_H */

