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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pciaccess.h>
#include <bfd.h>
#include <semaphore.h>
#include <sys/stat.h>

#include "pciedrv.h"
#include "pciedrv_loc.h"
#include "c6678pciedefs.h"

//#define PCIEDRV_VERBOSE

#define SYS_BUS_PCI "/sys/bus/pci/devices"

static char* progname = "pciedrv";

/* PCIe application registers virtual address (mapped to PCI window) */
#define PCI_REGV(minor, reg)          (volatile unsigned *) (driverinst.pcieendpoint[minor].memregions[0].mem + reg)


pciedrv_context_t pciedrv_context;
driver_inst_t driverinst;

pciedrv_obreg_alloc_inst_t **obreg_alloc_inst;

pciedrv_dma_alloc_inst_t **dma_alloc_inst;
pciedrv_param_set_alloc_inst_t **param_set_alloc_inst;
dma_transaction_t **dma_transactions;

static sem_t **bar_sem;
static sem_t *allocation_sem;


/* Local functions */
static int32_t obreg_alloc(uint32_t dsp_id, uint16_t num_contiguous_regions, uint16_t *obreg_num);
static int32_t obreg_free(uint32_t dsp_id, uint16_t num_contiguous_regions, uint16_t obreg_num);
static int32_t config_contiguous_ob_range(uint32_t dsp_id, uint32_t num_of_buffers,
  cmem_host_buf_desc_t buf_desc[], uint16_t ob_num_start, uint16_t *num_ob_regions);
static int32_t alloc_map_frame_desc_to_dsp_memrange(uint32_t dsp_id, cmem_host_frame_desc_t *frame_desc,
    uint32_t *dsp_start_addr, uint16_t *start_ob_reg_no, uint16_t *num_regions);
/**
   Find devices matching the specified vendor and device id returning
   them in a malloc'ed array of pointers to malloc'ed pci_device
   structs.  It is the callers responsibility to free all of these
   mallocations. Returns the number of devices or a value less than 0
   on error (errno set as well).

   \param vendor_id vendor id to look for
   \param device_id device id to look for
   \param devices pointer to where to store the pointer to the devices.
  */

static int find_devices(int vendor_id, int device_id, struct pci_device*** devices)
{
      struct pci_id_match matcher;
      struct pci_device** int_devices;
      struct pci_device *device;
      struct pci_device_iterator* pci_iterator;
      int n_devices = 0, i = 0;

      matcher.vendor_id = vendor_id;
      matcher.device_id = device_id;
      matcher.subvendor_id = PCI_MATCH_ANY;
      matcher.subdevice_id = PCI_MATCH_ANY;
      matcher.device_class = 0;
      matcher.device_class_mask = 0;
      matcher.match_data =  0;

      pci_iterator = pci_id_match_iterator_create(&matcher);     
      if (pci_iterator == NULL) {
	   fprintf(stderr, "%s: Unable to allocate memory for pci probe\n", progname);
	   return(errno);
      }

      /* Count devices found.*/
      while (pci_device_next(pci_iterator) != NULL) {
	   n_devices++;
      }
      pci_iterator_destroy(pci_iterator);

      if (n_devices == 0) {
	   return 0;
      }

      /*  Allocate array of devices. */
      if ((int_devices = malloc(n_devices*sizeof(struct pci_device*))) == NULL) {
	   return(errno);
      }

      /* Go through devices again and store pointers to the devices */
      pci_iterator = pci_id_match_iterator_create(&matcher);     
      if (pci_iterator == NULL) {
	   fprintf(stderr, "%s: Unable to allocate memory for pci probe\n", progname);
	   return errno;
      }
      i = 0;
      while ((device = pci_device_next(pci_iterator)) != NULL) {
	   if (i <= n_devices) {
		int_devices[i++] = device;	       
	   } else {
		fprintf(stderr, "%s: Unable to handle apperance of new card between scans\n", progname);
		errno = EAGAIN;
		return errno;
	   }
      }
      pci_iterator_destroy(pci_iterator);

      *devices = int_devices;
      return n_devices;
 }


static int setup_mem_regions( struct pci_device* pcidevice, pcie_end_point_t *pcieendpoint)
{        
  int i;
  char name[256];


  /**
    Go through all possible mem regions (as defined in the
    pciaccess library), map its memory and try to find the config
    space
  */
  for (i = 0; i < MAX_NUM_BAR_WINDOWS; i++) {
    /* TODO: work around to avoid repeated Sudo access */
  /* Make sure the file resource permissions are set */
  snprintf(name, 255, "%s/%04x:%02x:%02x.%1u/resource%u",
    SYS_BUS_PCI,
    pcidevice->domain,
    pcidevice->bus,
    pcidevice->dev,
    pcidevice->func,
    i);
  chmod(name, (S_IROTH | S_IWOTH | S_IRGRP | S_IWGRP | S_IRUSR | S_IWUSR));
  snprintf(name, 255, "%s/%04x:%02x:%02x.%1u/resource%u_wc",
    SYS_BUS_PCI,
    pcidevice->domain,
    pcidevice->bus,
    pcidevice->dev,
    pcidevice->func,
    i);
  chmod(name, (S_IROTH | S_IWOTH | S_IRGRP | S_IWGRP | S_IRUSR | S_IWUSR));

  // Map region if it is larger than 0
  if (pcidevice->regions[i].size > 0) {
#ifdef PCIEDRV_VERBOSE
    printf(" DEBUG: beginning check 1 .0x%x. 0x%x\n", (uint32_t )pcidevice, (uint32_t ) pcieendpoint );
#endif
    pcieendpoint->memregions[i].use_count = 0;
#ifdef PCIEDRV_VERBOSE
    printf(" DEBUG: Use count set ..\n");
#endif
    pcieendpoint->memregions[i].bar = i;
    pcieendpoint->memregions[i].ib_bar = 0;
    pcieendpoint->memregions[i].size = pcidevice->regions[i].size;
    pcieendpoint->memregions[i].is_64 = pcidevice->regions[i].is_64;
    if (pci_device_map_range(pcidevice, 
      pcidevice->regions[i].base_addr,
      pcidevice->regions[i].size,
      PCI_DEV_MAP_FLAG_WRITABLE,
      (void**) &(pcieendpoint->memregions[i].mem)))
   {
      fprintf(stderr, "%s: pci_device_map_range fail 0x%x \n", progname, errno);
      return(-1);
    }
      
//    fprintf(stderr, "%d to %p\n", i, (pcieendpoint->memregions)[i].mem);
	       
    // Check if this is the config area.
    // FIXME - should be a safer way to check this.
    if (*(pcieendpoint->memregions[i].mem) == TI667X_PCI_PID) {
      pcieendpoint->memregions[i].is_config = 1;
    }	       	       
    } else {
      // If size <= 0 the region is not present - and we will
      // not touch it.
      pcieendpoint->memregions[i].is_present = 0;
      pcieendpoint->memregions[i].use_count = 1;
      pcieendpoint->memregions[i].is_config = 0;
      pcieendpoint->memregions[i].bar = i;
      pcieendpoint->memregions[i].ib_bar = 0;
      pcieendpoint->memregions[i].size = 0;
      pcieendpoint->memregions[i].is_64 = pcidevice->regions[i].is_64;      
    }
  }
  return 0;
}

/* Check if requested address range lies within those supported */

/* C6678 L2 global range
* core0 L2: 0x10800000 ~ 0x10880000
* core1 L2: 0x11800000 ~ 0x11880000
* core2 L2: 0x12800000 ~ 0x12880000
* core3 L2: 0x13800000 ~ 0x13880000
* core4 L2: 0x14800000 ~ 0x14880000
* core5 L2: 0x15800000 ~ 0x15880000
* core6 L2: 0x16800000 ~ 0x16880000
* core7 L2: 0x17800000 ~ 0x17880000
*/
static int is_in_l2sram(unsigned start, unsigned size)
{
    unsigned core_offset;
    int i;

    for(i=0; i<8; i++)
    {
        core_offset = 0x10000000 | (i<<24);
        if( (start >= core_offset + TI667X_EP_L2SRAM_BASE) && 
            ((start+size) <= core_offset + TI667X_EP_L2SRAM_BASE + TI667X_EP_L2SRAM_MAX_SIZE) )
            return 1;
    }
    return 0;
}

#define is_in_msmcsram(a, l) \
    ((a >= TI667X_EP_MSMCSRAM_BASE) && ((a + l) <= (TI667X_EP_MSMCSRAM_BASE + TI667X_EP_MSMCSRAM_MAX_SIZE)))

#define is_in_ddr(a, l) \
    ((a >= TI667X_EP_DDR3_BASE) && ((a + l) <= (TI667X_EP_DDR3_BASE + TI667X_EP_DDR3_MAX_SIZE)))

#define is_in_chip_cfg_space(a, l)  \
    ((a >= TI667X_EP_CHIP_CFG_BASE) && ((a + l) <= (TI667X_EP_CHIP_CFG_BASE + TI667X_EP_CHIP_CFG_MAX_SIZE)))


/**
 *  @brief Function getMemRegion() This routine determines which address range  is mapped to 
 *                                which BAR Region.
 *  @param[in]     dsp_id         DSP chip number     
 *  @param[in]     addr           DSP Address 
 *  @param[in]     size           size of transaction
 *  @retval                       Memory region number or -1 if out of range
 *  @pre  
 *  @post 
 */

static int32_t getMemRegion(uint32_t dsp_id, uint32_t dsp_addr, uint32_t size)
{
  uint32_t bar_num;

  /* Regions 2 and 4 are reserved for DSP usage */
  if(is_in_l2sram(dsp_addr,size) || is_in_chip_cfg_space(dsp_addr,size) 
      || is_in_msmcsram(dsp_addr,size))
  {
    bar_num = 1;
    return(bar_num);
  }
  if(is_in_ddr(dsp_addr, size))
  {
    bar_num = 3;
    return(bar_num);
  }
  else
	return(-1);
}

static int32_t get_dsp2dsp_remote_map_memregion(uint32_t dsp_id, uint32_t dsp_addr, uint32_t size)
{
  uint32_t bar_num;

  /* Region 2 is reserved for DSP 2 DSP config spece map usage */
  if(is_in_chip_cfg_space(dsp_addr, size))
  {
    bar_num = 2;
    return (bar_num);
  }
  /* Region 4 is reserved for DSP 2 DSP local_l2/msmc/ddr memory map usage */
  else if( is_in_l2sram(dsp_addr,size) ||
           is_in_msmcsram(dsp_addr,size) || 
           is_in_ddr(dsp_addr, size))
  {
    bar_num = 4;
    return(bar_num);
  }
  else
  return(-1);
}


static uint32_t dspaddr_of_obregion(uint16_t ob_region_no)
{
 return(PCIE_DATA+ (ob_region_no * pciedrv_context.dsp_outbound_block_size));

}
static uint16_t obregion_of_dspaddr(uint32_t dsp_addr)
{
  return((dsp_addr-PCIE_DATA)/pciedrv_context.dsp_outbound_block_size);
}
/**
 *  @brief Function ti667x_ep_setup_bar() Map Dsp address to BAR registers
 *                                which BAR Region.
 *  @param[in]     dsp_id         DSP chip number
 *  @param[in]     bar_num        BAR region number
 *  @param[in]     addr           Address to be mapped
 *  @retval                       Memory region number or -1 if out of range
 *  @pre
 *  @post
 */

static int ti667x_ep_setup_bar(uint32_t dsp_id, uint32_t bar_num, uint32_t addr)
{
  int ret;
  uint32_t ib_trans_num;
  uint32_t read_val;

  if (bar_num > MAX_NUM_BAR_WINDOWS)
    return -1;

  if(driverinst.pcieendpoint[dsp_id].memregions[bar_num].dspmem == addr)
    return 0;

  ib_trans_num = bar_num-1;
  *(PCI_REGV(dsp_id, IB_OFFSET(ib_trans_num)))= addr;
  read_val = *(PCI_REGV(dsp_id, IB_OFFSET(ib_trans_num)));
  if(read_val != addr)
  {
    ret = -1;
    goto error;
  }
  /* Store Bar dsp address */
  driverinst.pcieendpoint[dsp_id].memregions[bar_num].dspmem = addr; 
  return 0;

error:
    fprintf(stderr, "%s: Error in ti667x_ep_setup_bar, ib trans num %d!, ret = %d \n", 
      progname, ib_trans_num, ret);
    fprintf(stderr, "%s: Found: IB_BAR = %08X, IB_OFFSET = %08X. \n", 
      progname, *(PCI_REGV(dsp_id, IB_BAR(ib_trans_num))), read_val);
    fprintf(stderr, "%s: Expected: IB_BAR = %08X, IB_OFFSET = %08X. \n",progname, bar_num, addr);

    return ret;
}
static int ti667x_ep_init_bar(uint32_t dsp_id, uint32_t bar_num)
{
  int ret;
  uint32_t bar_val;
  uint32_t ib_trans_num;
  uint32_t read_val;

  if (bar_num > MAX_NUM_BAR_WINDOWS)
    return -1;

  ib_trans_num = bar_num-1;

  bar_val = driverinst.pcidevices[dsp_id]->regions[bar_num].base_addr;

  *(PCI_REGV(dsp_id, IB_BAR(ib_trans_num))) = bar_num;
  read_val = *(PCI_REGV(dsp_id, IB_BAR(ib_trans_num)));
  if(bar_num != read_val) {
      ret = -2;
      goto error;
  }

  *(PCI_REGV(dsp_id, IB_START_LO(ib_trans_num))) = bar_val;
  read_val = *(PCI_REGV(dsp_id, IB_START_LO(ib_trans_num)));
  if(bar_val != read_val) {
      ret = -4;
      goto error;
  }

  *(PCI_REGV(dsp_id, IB_START_HI(ib_trans_num))) = 0;
  read_val = *(PCI_REGV(dsp_id, IB_START_HI(ib_trans_num)));
  if(0 != read_val) {
      ret = -8;
      goto error;
  }

    return 0;

error:
    fprintf(stderr, "%s: Error in ti667x_ep_init_bar, ib trans num %d!, ret = %d \n", 
      progname, ib_trans_num, ret);
    fprintf(stderr, "%s: Found: IB_BAR = %08X, IB_START_LO = %08X, IB_START_HI = %08X \n", 
      progname, *(PCI_REGV(dsp_id, IB_BAR(ib_trans_num))),
        *(PCI_REGV(dsp_id, IB_START_LO(ib_trans_num))),
        *(PCI_REGV(dsp_id, IB_START_HI(ib_trans_num))));
    fprintf(stderr, "%s: Expected: IB_BAR = %08X, IB_START_LO = %08X, IB_START_HI = %08X. \n", 
        progname, bar_num,
        bar_val,
        0);

    return ret;
}

static int32_t ob_reg_cfg(uint32_t dsp_id, uint32_t ob_region_num, uint64_t region_base_addr)
{

    if (ob_region_num > 32)
        return -1;

    *(PCI_REGV(dsp_id, OB_OFFSET_INDEX(ob_region_num))) = (region_base_addr & 0xfff00000) | 1;
    *(PCI_REGV(dsp_id, OB_OFFSET_HI(ob_region_num))) = (region_base_addr >> 32 ) & 0xffffffff;
    return(0);
}
/**
 *  @brief Function pciedrv_ep_setup_ob_size() Set Ob size register
 *  @param[in]     dsp_id         DSP chip number                 
 *  @param[in]     ob_size        Outbound window size
 *  @retval        none
 *  @pre  
 *  @post 
 */
static void pciedrv_ep_setup_ob_size(uint32_t dsp_id, uint32_t ob_size)
{
    *(PCI_REGV(dsp_id, OB_SIZE)) = ob_size;
}

static int32_t pciedrv_dma_alloc(uint32_t dsp_id, uint16_t num_param_sets, uint16_t *dma_chan, uint16_t *param_set_no)
{
  int i,j;
  uint32_t bitmap, req_bitmap;

  req_bitmap =((1 << num_param_sets)-1);
  for(i=0; i < pciedrv_context.num_param_sets; i++) {
    if((i+num_param_sets) > pciedrv_context.num_param_sets)
       return(-1);

    bitmap = 0;
    sem_wait(&allocation_sem[dsp_id]);
    for(j=0; j< num_param_sets; j++) {
      bitmap += ((param_set_alloc_inst[dsp_id][i+j].flags  & DMA_FLAG_ALLOCATED) ? 0: 1) << j;
    }
       
    if(bitmap  == req_bitmap) {
       /* Critical section */
       *param_set_no = param_set_alloc_inst[dsp_id][i].param_set_number;
       for(j=0; j< num_param_sets; j++) {
         param_set_alloc_inst[dsp_id][i+j].flags |= DMA_FLAG_ALLOCATED;
       }
       sem_post(&allocation_sem[dsp_id]);
       break;
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  if(i == pciedrv_context.num_param_sets)
    return(-1);

  for(i=0; i < pciedrv_context.num_dma_channels; i++) {
    sem_wait(&allocation_sem[dsp_id]);
    if(!(dma_alloc_inst[dsp_id][i].flags  & DMA_FLAG_ALLOCATED)) {
       /* Critical section */
       *dma_chan = dma_alloc_inst[dsp_id][i].dma_chan_number;
       dma_alloc_inst[dsp_id][i].flags |= DMA_FLAG_ALLOCATED;
       sem_post(&allocation_sem[dsp_id]);
       return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
 /* alloation failed */
  return(-1);
}

static int32_t pciedrv_dma_free(uint32_t dsp_id, uint16_t dma_chan, uint16_t param_set_no, uint16_t num_param_sets)
{
  int i,j;

  for(i=0; i < pciedrv_context.num_param_sets; i++) {
    if((i+num_param_sets) > pciedrv_context.num_param_sets)
       return(-1);
    sem_wait(&allocation_sem[dsp_id]);
    if((param_set_alloc_inst[dsp_id][i].param_set_number ==param_set_no) 
      && (param_set_alloc_inst[dsp_id][i].flags  & DMA_FLAG_ALLOCATED)) {

      /* Critical section */
      for(j=0; j< num_param_sets; j++) {
        param_set_alloc_inst[dsp_id][i+j].flags &= (~DMA_FLAG_ALLOCATED);
      }
       sem_post(&allocation_sem[dsp_id]);
       break;
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  if(i == pciedrv_context.num_param_sets)
    return(-1);

  for(i=0; i < pciedrv_context.num_dma_channels; i++) {
    sem_wait(&allocation_sem[dsp_id]);
    if((dma_alloc_inst[dsp_id][i].dma_chan_number ==dma_chan) 
      && (dma_alloc_inst[dsp_id][i].flags  & DMA_FLAG_ALLOCATED)) {
       /* TODO: Add critical section */
       dma_alloc_inst[dsp_id][i].flags &= (~DMA_FLAG_ALLOCATED);
       sem_post(&allocation_sem[dsp_id]);
       return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
 /* Free  failed */
  return(-1);
}


static int32_t pciedrv_alloc_dma_transaction(uint32_t dsp_id, dma_transaction_t **dma_transaction)
{
  int i;
  dma_transaction_t *dma_transaction_list;

  dma_transaction_list = (dma_transaction_t *)dma_transactions[dsp_id] ;
   
  for(i=0; i < pciedrv_context.max_dma_transactions; i++) {
    sem_wait(&allocation_sem[dsp_id]);
    if(!(dma_transaction_list[i].flags & DMA_TRANSACTION_USED)) {
      pciedrv_context.trans_id_count++;
      dma_transaction_list[i].trans_id = pciedrv_context.trans_id_count;
      dma_transaction_list[i].flags |= DMA_TRANSACTION_USED;
      *dma_transaction = &(dma_transaction_list[i]);
      sem_post(&allocation_sem[dsp_id]);
      return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  return(-1);
}

static int32_t pciedrv_find_dma_transaction(uint32_t dsp_id, uint32_t trans_id, dma_transaction_t **dma_transaction)
{
  int i;
  dma_transaction_t *dma_transaction_list;

  dma_transaction_list = (dma_transaction_t *)dma_transactions[dsp_id] ;
   
  for(i=0; i < pciedrv_context.max_dma_transactions; i++) {
    sem_wait(&allocation_sem[dsp_id]);
    if((dma_transaction_list[i].flags & DMA_TRANSACTION_USED)
      && (dma_transaction_list[i].trans_id == trans_id)) {
      *dma_transaction = &(dma_transaction_list[i]);
      sem_post(&allocation_sem[dsp_id]);
      return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  return(-1);
  
}

static int32_t pciedrv_free_dma_transaction(uint32_t dsp_id, uint32_t trans_id)
{
  int i;
  dma_transaction_t *dma_transaction_list;

  dma_transaction_list = (dma_transaction_t *)dma_transactions[dsp_id] ;
   
  for(i=0; i < pciedrv_context.max_dma_transactions; i++) {
    sem_wait(&allocation_sem[dsp_id]);
    if((dma_transaction_list[i].flags & DMA_TRANSACTION_USED)
      && (dma_transaction_list[i].trans_id == trans_id)) {
//      dma_transaction_list[i].trans_id = 0;
      dma_transaction_list[i].flags &= (~DMA_TRANSACTION_USED);
      sem_post(&allocation_sem[dsp_id]);
      return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  return(-1);
  
}

/**
 *  @brief Function pciedrv_dsp_write() Write to DSP memory using memcpy over PCIe
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Address to write
 *  @param[in]     buf            Source Buffer pointer
 *  @param[in]     size           size of Memory write in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_write(int32_t dsp_id, uint32_t addr, uint8_t *buf, uint32_t size)
{
  int32_t bar_num, ret_val;
  uint32_t offset, offsetAlign;
  uint32_t xfer_size;

  /* Get the Bar Region number */
  bar_num = getMemRegion(dsp_id, addr, size);
  if(bar_num < 0)
  {
    fprintf(stderr, "%s: ERROR: pciedrv_dsp_write: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, addr, size);
    return(-1);
  }
  /* Calculate offset from base address to be written into BAR register */
  while(size > 0) 
  {
    offsetAlign = minOffsetAlign(EP_MAP_OFFSET_ALIGN, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-1);
    offset = addr & offsetAlign;
    xfer_size = ((offset+size) > driverinst.pcieendpoint[dsp_id].memregions[bar_num].size) ?
                  driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-offset : size;
    sem_wait(&bar_sem[dsp_id][bar_num]);

    /* Set up Bar register */
    ret_val = ti667x_ep_setup_bar(dsp_id, bar_num, addr & (~offsetAlign));
    if(ret_val < 0 )
    {
      fprintf(stderr, "%s: ERROR: pciedrv_dsp_write: Setup Bar register fail \n", progname);
      sem_post(&bar_sem[dsp_id][bar_num]);
      return(-1);
    }
    /* Copy data into memory through BAR Memory region */  
    memcpy((void *)(driverinst.pcieendpoint[dsp_id].memregions[bar_num].mem+offset), 
               buf,  xfer_size);
    sem_post(&bar_sem[dsp_id][bar_num]);
    addr += xfer_size;
    buf  += xfer_size;
    size -= xfer_size;
  }
  return(0);
  
}

/**
 *  @brief Function pciedrv_dsp_read() Read from DSP memory using memcpy over PCIe
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Address to read
 *  @param[in]     buf            Destination Buffer pointer
 *  @param[in]     size           size of Memory read in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_read(int32_t dsp_id, uint32_t addr, uint8_t *buf, uint32_t size)
{
  int32_t bar_num, ret_val;
  uint32_t offset, offsetAlign;
  uint32_t xfer_size;

  /* Get the Bar Region number */
  bar_num = getMemRegion(dsp_id, addr, size);
  if(bar_num < 0)
  {
    fprintf(stderr, "%s: ERROR: pciedrv_dsp_read: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, addr, size);
    return(-1);
  }
  while(size > 0) 
  {
    /* Calculate offset from base address to be written into BAR register */
    offsetAlign = minOffsetAlign(EP_MAP_OFFSET_ALIGN, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-1);
    offset = addr & offsetAlign;

    xfer_size = (offset+size > driverinst.pcieendpoint[dsp_id].memregions[bar_num].size) ?
                  driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-offset : size;
    sem_wait(&bar_sem[dsp_id][bar_num]);

    /* Set up Bar register */
    ret_val = ti667x_ep_setup_bar(dsp_id, bar_num, addr & (~offsetAlign));
    if(ret_val < 0 )
    {
      fprintf(stderr, "%s: ERROR:  pciedrv_dsp_read: Setup Bar register fail \n", progname);
      sem_post(&bar_sem[dsp_id][bar_num]);
      return(-1);
    }
    /* Copy data into memory through BAR Memory region */  
    memcpy(buf,
      (void *)(driverinst.pcieendpoint[dsp_id].memregions[bar_num].mem+offset),
      xfer_size);
    sem_post(&bar_sem[dsp_id][bar_num]);
    addr += xfer_size;
    buf  += xfer_size;
    size -= xfer_size;
  }
  return(0);
}

/**
 *  @brief Function pciedrv_resource_init() Initialise user space driver
 *                  Resource init is required to be called before using dma resources
 *                  (Required to call pciedrv_driver_open before calling this )
 *  @param[in]    dsp_outbound_reserved_mem_Size  size of dsp outbound memory
 *                region to be marked as reserved.
 *  @retval          0: success, -1 : failure
 *  @pre  
 *  @post 
 */
static int32_t resource_init(pciedrv_open_config_t *open_config)
{
  uint16_t num_reserved_obregions;
  int n_devices, i,j;
  uint16_t ob_reg_number;
  int32_t ret_val;
  n_devices =  pciedrv_context.num_devices;

  /* Calculate number of outbound regsions to be reserved */
  num_reserved_obregions 
    = open_config->dsp_outbound_reserved_mem_size/open_config->dsp_outbound_block_size;

  /* allocate memory for semaphore */
  allocation_sem = malloc(n_devices*sizeof(sem_t));
  if(allocation_sem ==NULL)
    return(-1);
  /* Initialise semaphores used for local allocation */
  for(j=0; j< n_devices; j++) {
    sem_init(&allocation_sem[j], 0, 1);
  }
  pciedrv_context.num_ob_instances = TOTAL_OB_REGIONS-num_reserved_obregions;
  
  obreg_alloc_inst = malloc(n_devices*sizeof(pciedrv_obreg_alloc_inst_t *));
  if(obreg_alloc_inst == NULL)
    return(-1);

  /* Initialise OB window allocation structures*/
  for(j=0; j< n_devices; j++) {
    obreg_alloc_inst[j] = malloc(pciedrv_context.num_ob_instances*sizeof(pciedrv_obreg_alloc_inst_t));
    if(obreg_alloc_inst[j] == NULL)
      goto obreg_alloc_perdevice_fail;
    for(i=0; i < pciedrv_context.num_ob_instances ; i++) {
      obreg_alloc_inst[j][i].ob_reg_number =  num_reserved_obregions+i;
      obreg_alloc_inst[j][i].flags = 0;
    }
  }
  /* Store number of dma channels */
  pciedrv_context.num_dma_channels = open_config->num_dma_channels;

  /* allocate memory for dma alloc inst */
  dma_alloc_inst = malloc(n_devices*sizeof(pciedrv_dma_alloc_inst_t *));
  if(dma_alloc_inst == NULL)
    goto dma_alloc_inst_fail;

  /* Initialise DMA channel allocation structures */
  for(j=0; j< n_devices; j++) {
    dma_alloc_inst[j] = malloc(pciedrv_context.num_dma_channels*sizeof(pciedrv_dma_alloc_inst_t));
    if(dma_alloc_inst[j] == NULL)
      goto dma_alloc_per_device_fail;
    for(i=0; i < open_config->num_dma_channels; i++) {
      dma_alloc_inst[j][i].dma_chan_number =  open_config->start_dma_chan_num+i;
      dma_alloc_inst[j][i].flags = 0;
    }
  }
  /* Store number of param sets */
  pciedrv_context.num_param_sets = open_config->num_param_sets;

  /* allocate memory for param set instance */
  param_set_alloc_inst = malloc(n_devices*sizeof(pciedrv_param_set_alloc_inst_t *));
  if(param_set_alloc_inst == NULL)
    goto param_set_alloc_fail;
  /* Initialise DMA  allocation structures */
  for(j=0; j< n_devices; j++) {
    param_set_alloc_inst[j] = malloc(pciedrv_context.num_param_sets*sizeof(pciedrv_param_set_alloc_inst_t));
    if(param_set_alloc_inst[j] == NULL)
      goto param_set_per_device_fail;
    for(i=0; i < open_config->num_param_sets; i++) {
      param_set_alloc_inst[j][i].param_set_number =  open_config->start_param_set_num+i;
      param_set_alloc_inst[j][i].flags = 0;
    }
  }
  /* Store number of dma transactions */
  pciedrv_context.max_dma_transactions = open_config->max_dma_transactions;

  /* Allocate memory for dma transaction pointers */
  dma_transactions = malloc(n_devices*sizeof(dma_transaction_t *));
  if(dma_transactions == NULL)
    goto dma_transaction_alloc_fail;
  /* Initialise dma transaction pool */
  for(j=0; j< n_devices; j++) {
    dma_transactions[j] = (dma_transaction_t *)malloc(open_config->max_dma_transactions*sizeof(dma_transaction_t));
    if(dma_transactions[j] ==NULL) {
      /* Memory allocation fail */
      goto dma_transaction_per_device_alloc_fail;
    }
    /* Initialise to zero */
    memset(dma_transactions[j], 0, open_config->max_dma_transactions*sizeof(dma_transaction_t));
  }
  /* Store number of dma transactions */
  pciedrv_context.dsp_outbound_block_size = open_config->dsp_outbound_block_size;
  for(i =0; i < n_devices; i++) {
    switch(open_config->dsp_outbound_block_size) 
    {
      case 0x100000:
        pciedrv_ep_setup_ob_size(i, OB_SIZE_1MB);
        break;   
      case 0x200000:
        pciedrv_ep_setup_ob_size(i, OB_SIZE_2MB);
        break;     
      case 0x400000:
        pciedrv_ep_setup_ob_size(i, OB_SIZE_4MB);
        break; 
      case 0x800000:
        pciedrv_ep_setup_ob_size(i, OB_SIZE_8MB);
        break;
      default:
        /* Unexpected buffer size value */
        goto outbound_size_config_invalid;
     }
   }

  pciedrv_context.flag |= PCIE_USPACE_FLAG_RESOURCE_INIT_DONE;

  return(0);
#if 1
outbound_size_config_invalid:
  j = n_devices;
#else
remote_map_transaction_per_device_alloc_fail:
  for(i=0; i< j; i++)
    free(remote_map_transactions[i]);
  free(remote_map_transactions);
remote_map_transaction_alloc_fail:
#endif
  j= n_devices;
dma_transaction_per_device_alloc_fail:
  for(i=0; i< j; i++)
    free(dma_transactions[i]);
  free(dma_transactions);
dma_transaction_alloc_fail:
  j = n_devices;
param_set_per_device_fail:
  for(i=0; i< j; i++)
    free(param_set_alloc_inst[i]);
  free(param_set_alloc_inst);
param_set_alloc_fail:
  j= n_devices;
dma_alloc_per_device_fail:
  for(i=0; i< j; i++)
    free(dma_alloc_inst[i]);
  free(dma_alloc_inst);
dma_alloc_inst_fail:
  j = n_devices;
obreg_alloc_perdevice_fail:
  for(i=0; i< j; i++)
    free(obreg_alloc_inst[i]);
  free(obreg_alloc_inst);
  return(-1);
}

/**
 *  @brief Function pciedrv_open() Open the PCIe driver
 *  @param[in, out]     none
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_open(pciedrv_open_config_t *open_config)
{
  int32_t ret_val;
  int n_devices, i,j;
  uint32_t val;
  uint32_t ti667x_dsp_id;
  uint16_t ob_reg_number;

  /* Initialize pciaccess library */
  pci_system_init();
  /* Patches to libpciaccess in some distributions fails to
     reset errno after trying and failing to open the /rom
     sysfs file.
  */
  if (errno) {
    errno = 0;
  }

  /* Find TI pcie devices on this host. */
  if ((n_devices = find_devices(TI667X_PCI_VENDOR_ID, TI667X_PCI_DEVICE_ID, 
    &driverinst.pcidevices)) <= 0)
  {
    fprintf(stderr, "%s: ERROR: No TI PCI devices found\n", progname);
    return -1;
  }
#ifdef PCIEDRV_VERBOSE
  printf("\n Found %d TI PCI devices \n", n_devices);
#endif
  if ((driverinst.pcieendpoint = malloc(n_devices*sizeof(pcie_end_point_t ))) == NULL) {
    return -1;
  }
  memset( driverinst.pcieendpoint, 0, n_devices*sizeof(pcie_end_point_t));
  /*
   * Probe devices (necessary to make them active):
  */
  for (i = 0; i < n_devices; i++) {
    if (pci_device_probe( driverinst.pcidevices[i])) {
      fprintf(stderr, "%s: ERROR: PCI Probe failed error code \n", progname);
      return -1;
    }

    /* Patches to libpciaccess in some distributions fails to
       reset errno after trying and failing to open the /rom
       sysfs file.
    */
    if (errno) {
      errno = 0;
    }
 
#ifdef PCIEDRV_VERBOSE
    printf("pci device probe complete %d \n  ",i);fflush(stdout);
#endif    
    if(setup_mem_regions( driverinst.pcidevices[i], &driverinst.pcieendpoint[i]) != 0) {
      fprintf(stderr, "%s: ERROR: Mapping region :Device %d \n", progname, i);
      return errno;
    }
#ifdef PCIEDRV_VERBOSE
  printf("Setup memory regions complete\n  ");fflush(stdout);
#endif   

    /* Set up default inbound access windows */
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ti667x_ep_setup_bar(i, 0, TI667X_EP_PCIE_BASE);

    ti667x_ep_init_bar(i, 1);
    ti667x_ep_setup_bar(i, 1, TI667X_EP_L2SRAM_BASE + 0x10000000);/* core0 L2 SRAM */
    /* Currently reserved for DSP usage */
    ti667x_ep_init_bar(i, 2);
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ti667x_ep_setup_bar(i, 2, TI667X_EP_MSMCSRAM_BASE); /* Reserved for DSP use */

    ti667x_ep_init_bar(i, 3);
    ti667x_ep_setup_bar(i, 3, TI667X_EP_DDR3_BASE);
    /* Currently reserved for DSP usage */
    ti667x_ep_init_bar(i, 4);

    /* Enable inbound  and outbound translation */
    val = *(PCI_REGV(i, CMD_STATUS));
    val |= (IB_XLAT_EN_VAL | OB_XLAT_EN_VAL);
    *(PCI_REGV(i, CMD_STATUS)) = val;

  }
  pciedrv_context.num_devices = n_devices;
  pciedrv_context.flag = 0;
#ifdef PCIEDRV_VERBOSE
  printf("Setup bar region complete\n  ");fflush(stdout);
#endif

  bar_sem = malloc(n_devices*sizeof(sem_t *));
  if(bar_sem ==NULL)
    return(-1);
#ifdef PCIEDRV_VERBOSE
  printf(" malloc complete for bar_sem \n");fflush(stdout);
#endif

  for(ti667x_dsp_id =0; ti667x_dsp_id < n_devices; ti667x_dsp_id++)
  {
    bar_sem[ti667x_dsp_id] = malloc(MAX_NUM_BAR_WINDOWS*sizeof(sem_t));
    if(bar_sem[ti667x_dsp_id] == NULL)
    {
      free(bar_sem);
      return(-1);
    }
    for(i=0; i < MAX_NUM_BAR_WINDOWS; i++) 
    { 
      sem_init(&bar_sem[ti667x_dsp_id][i], 0, 1);
    }
  }
#ifdef PCIEDRV_VERBOSE
  printf(" Allocation of semaphores complete \n");fflush(stdout);

#endif
  /* Only do resource init if configuration pointer non null */
  if(open_config)
  {
    if(resource_init(open_config) != 0)
    {
      fprintf(stderr, "%s: ERROR: Resource init fail \n", progname);
      goto err_exit_opencfg_fail;
    }
  }
#ifdef PCIEDRV_VERBOSE
  printf(" Open Driver done \n");  fflush(stdout);
#endif
  return(0);

err_exit_opencfg_fail:
  free(bar_sem);
  /* Free memory allocated */
  free(driverinst.pcidevices);
  free(driverinst.pcieendpoint);

  fprintf(stderr, "%s: ERROR: Open config failed \n", progname);
  return(-1);

}
/**
 *  @brief Function pciedrv_close() Closes the PCIe driver
 *  @param[in, out]     none
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_close(void)
{
  int n_devices, i, j;
  uint32_t ti667x_dsp_id;

  n_devices =  pciedrv_context.num_devices;
  if(pciedrv_context.flag & PCIE_USPACE_FLAG_RESOURCE_INIT_DONE) 
  {

    /* Destroy dma transactions semaphores */
    for(j=0; j< n_devices; j++) {
      if(dma_transactions[j] != NULL) {
        free(dma_transactions[j]); 
      }
    }
    free(dma_transactions);  
 
    for(j=0; j< n_devices; j++) {
      free(param_set_alloc_inst[j]);
    }
    free(param_set_alloc_inst);

    for(j=0; j< n_devices; j++) {
      free(obreg_alloc_inst[j]);
    }
    free(obreg_alloc_inst);

    for(j=0; j< n_devices; j++) {
      free(dma_alloc_inst[j]);
    }
    free(dma_alloc_inst);
    /* Destroy allocation semaphores */
    for(j=0; j< n_devices; j++) {
      sem_destroy(&allocation_sem[j]);
    }
    free(allocation_sem);
  }
  pciedrv_context.flag =0;
 
  for(ti667x_dsp_id =0; ti667x_dsp_id < n_devices; ti667x_dsp_id++)
  {
    for(i=0; i < MAX_NUM_BAR_WINDOWS; i++) {
      sem_destroy(&bar_sem[ti667x_dsp_id][i]);
    }
    free(bar_sem[ti667x_dsp_id]);
  }
  free(bar_sem);
  /* Free memory allocated */
  free(driverinst.pcidevices);
  free(driverinst.pcieendpoint);
#ifdef PCIEDRV_VERBOSE
  printf("\nPCIE Driver closed \n");
#endif
  return(0);
}
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
int32_t pciedrv_dsp_set_entry_point(uint32_t dsp_id, uint32_t core_id, uint32_t entry_point)
{
  int32_t bar_num, ret_val;
  uint32_t offset, offsetAlign;
  uint32_t size;
  uint32_t ib_map_dsp_addr, ib_map_size;

  ib_map_dsp_addr = TI667X_EP_L2SRAM_BASE + ((core_id + 0x10) << 24);
  ib_map_size = TI667X_EP_BOOTFLAG_OFFSET+4;
  /* Find memory region */
  bar_num = getMemRegion(dsp_id, ib_map_dsp_addr, ib_map_size );
  offsetAlign = minOffsetAlign(EP_MAP_OFFSET_ALIGN, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-1);
  if((bar_num < 0) 
    || (((ib_map_dsp_addr & offsetAlign)+ib_map_size) 
      > driverinst.pcidevices[dsp_id]->regions[bar_num].size))
  {
    fprintf(stderr, "%s: ERROR: pciedrv_dsp_set_entry_point: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, ib_map_dsp_addr, ib_map_size);
    return(-1);
  }

  sem_wait(&bar_sem[dsp_id][bar_num]);
  /* Set up bar register */
  ret_val = ti667x_ep_setup_bar(dsp_id, bar_num, ib_map_dsp_addr );
  if(ret_val != 0) {
    sem_post(&bar_sem[dsp_id][bar_num]);
    return (ret_val);
  }
  /* Write entry point */
  *((unsigned *)(driverinst.pcieendpoint[dsp_id].memregions[bar_num].mem+TI667X_EP_BOOTFLAG_OFFSET)) = entry_point;
  sem_post(&bar_sem[dsp_id][bar_num]);
#ifdef PCIEDRV_VERBOSE
  printf("Entry point is set to 0x%08x\n", entry_point);
#endif
  return(0);
}

/* Local functions */
int32_t pcieep_master_privilege_set(uint32_t dsp_id)
{
  int val;
  int val22;

  /* Enable priority */
  val = *(PCI_REGV(dsp_id, PRIORITY));
  val |= 0x10000;
  *(PCI_REGV(dsp_id, PRIORITY)) = val;
//  printf("Set Master priv DSP number %d \n", dsp_id);
  return(0);
}


int32_t pcieep_dsp_inta_set(uint32_t dsp_id)
{
  /* set interrupt A to wake up DSP */
  *(PCI_REGV(dsp_id, TI667X_EP_INTA_SET_OFFSET)) = 0x1;

  return(0);

}

int32_t pcieep_dsp_inta_clr(uint32_t dsp_id)
{
  /* set interrupt A to wake up DSP */
  *(PCI_REGV(dsp_id, TI667X_EP_INTA_CLR_OFFSET)) = 0x1;

  return(0);
}

int32_t pcieep_dsp_intb_set(uint32_t dsp_id)
{
  /* set interrupt B to wake up DSP */
  *(PCI_REGV(dsp_id, TI667X_EP_INTB_SET_OFFSET)) = 0x1;

  return(0);

}

int32_t pcieep_dsp_intb_clr(uint32_t dsp_id)
{
  /* set interrupt B to wake up DSP */
  *(PCI_REGV(dsp_id, TI667X_EP_INTB_CLR_OFFSET)) = 0x1;

  return(0);
}
/* Local functions end */

/**
 *  @brief Function pciedrv_pcieep_set_config() Set PCIE endpoint configuration
 *  @param[in]     dsp_id           DSP or device id
 *  @param[in]     pcieep_set_type  Specifies the type of configuration. see enum for details
 *  @retval        0: success -1: Fail
 *  @pre  
 *  @post 
 */
int32_t pciedrv_pcieep_set_config(uint32_t dsp_id, pciedrv_pcieep_set_type_t pcieep_set_type)
{
  switch(pcieep_set_type)
  {
    case PCIEDRV_PCIEEP_SET_INTA_SET:
    {
      pcieep_dsp_inta_set(dsp_id);
    }
    break;
    case PCIEDRV_PCIEEP_SET_INTA_CLR:
    {
      pcieep_dsp_inta_clr(dsp_id);
    }
    break;
    case PCIEDRV_PCIEEP_SET_INTB_SET:
    {
      pcieep_dsp_intb_set(dsp_id);
    }
    break;
    case PCIEDRV_PCIEEP_SET_INTB_CLR:
    {
      pcieep_dsp_intb_clr(dsp_id);
    }
    break;
    case PCIEDRV_PCIEEP_SET_MASTER_PRIV_SET:
    {
      return(pcieep_master_privilege_set(dsp_id));
    }
    break;
  }
}

/* ============================================================================
*  @func   do_dsp_dma  Does DMA transfer using DSP DMA (Assume alignment and  
*                      size multiple of PCIE_TRANSFER_SIZE )
*
*  @desc   Move DMAs contents between DSP memory & Host Memory. 
*  flag:   bitmap 0: Move data inside DSP; 0x1: Dest Host address 0x2: Src Host addr
*  
*  @modif  None.
*  ============================================================================
*/

#define FLAG_DST_ADDR_HOST   0x1
#define FLAG_SRC_ADDR_HOST   0x2

#define FLAG_XFER_NOT_WAIT_TILL_COMPLETION 0x4

#define EDMA_ACCESS_MAX_OFFSET 0x8000

#define MAX_DMA_SINGLE_XFER 0x400000

static int32_t do_dsp_dma(uint32_t dsp_id, uint32_t dma_chan_num, uint32_t param_set_num,
    uint32_t dstAddr, uint32_t srcAddr, uint32_t size, uint32_t flag)
{
  uint32_t tmp, tSize;
  uint32_t *dmaqnum_addr;
  uint32_t tmp_mask;

  uint32_t *dchmap_param; 
  int32_t ret_val = -1;
  uint32_t bar_num;
  uint8_t *mem_region_base;
  uint32_t dma_queue_num;
  uint16_t previous_dma_flag=0;
  uint32_t ib_map_dsp_addr, ib_map_size;

//  printf("Debug: do_dsp_dma: dsp_id : %d, dstAddr 0x%x, srcAddr 0x%x, size 0x%x, flag 0x%x\n", dsp_id, dstAddr, srcAddr, size, flag);
  ib_map_dsp_addr = EDMA_TPCC0_BASE_ADDRESS;
  ib_map_size = EDMA_ACCESS_MAX_OFFSET;
  bar_num = getMemRegion(dsp_id, ib_map_dsp_addr, ib_map_size );
  if((bar_num < 0)
   || (((ib_map_dsp_addr & (minOffsetAlign(EP_MAP_OFFSET_ALIGN, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-1)))
      +ib_map_size) > driverinst.pcidevices[dsp_id]->regions[bar_num].size))
  {
    fprintf(stderr, "%s: ERROR: do_dsp_dma: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, ib_map_dsp_addr, ib_map_size);
    return(-1);
  }

//  printf("Debug: do_dsp_dma: got bar number: %x\n", bar_num);
  sem_wait(&bar_sem[dsp_id][bar_num]);
//  printf("Debug: do_dsp_dma: Got semaphore\n");
  ret_val = ti667x_ep_setup_bar(dsp_id, bar_num, ib_map_dsp_addr);
  if(ret_val != 0) {
    sem_post(&bar_sem[dsp_id][bar_num]);
    return (ret_val);
  }
  mem_region_base = (uint8_t *)driverinst.pcieendpoint[dsp_id].memregions[bar_num].mem;

  if(flag & FLAG_DST_ADDR_HOST) {
     dma_queue_num = DMA_QUEUE_RD;
  }
  else if(flag & FLAG_SRC_ADDR_HOST) {
     dma_queue_num = DMA_QUEUE_WR;
  }
  else {
     /* TODO: Need to check whether this matters */
    dma_queue_num = DMA_QUEUE_WR;
  }


  dmaqnum_addr = (uint32_t *)((mem_region_base+ DMAQNUM0 + 4*((dma_chan_num >> 3) << 2)));
  tmp_mask = ~(0x7 << ((dma_chan_num & 0x0007) << 2));
  tmp  = *(dmaqnum_addr);
  tmp &= tmp_mask;
  tmp |= (dma_queue_num << ((dma_chan_num & 0x0007) << 2));
  *(dmaqnum_addr) = tmp;

  dchmap_param = ((uint32_t *)( mem_region_base + DCHMAP + 4 * dma_chan_num));
  *(dchmap_param) = PARAM_SET_OFFSET(param_set_num, PARAM_SET_OPT);

  /* Set the interrupt enable for 1st Channel (IER). */
  *((uint32_t *) ( mem_region_base+ IESR)) = (1 << dma_chan_num);

  /* Clear any pending interrupt (IPR). */
  *((uint32_t *) ( mem_region_base + ICR))= (1 << dma_chan_num);

while (1) {


  *((uint32_t *) (mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_SRC))) = srcAddr;
  *((uint32_t *) ( mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_DST))) = dstAddr;

  /* Calculate the A & B count */
  if (size >= MAX_DMA_SINGLE_XFER)  {
    tmp = MAX_DMA_SINGLE_XFER/PCIE_TRANSFER_SIZE;
    tSize = tmp*PCIE_TRANSFER_SIZE;
    size -= (tmp*PCIE_TRANSFER_SIZE);
    tmp <<= 16;
    tmp |= PCIE_TRANSFER_SIZE;
  } else if (size >= PCIE_TRANSFER_SIZE)  {
    tmp = size/PCIE_TRANSFER_SIZE;
    tSize = tmp*PCIE_TRANSFER_SIZE;
    size -= (tmp*PCIE_TRANSFER_SIZE);
    tmp <<= 16;
    tmp |= PCIE_TRANSFER_SIZE;
  }
  else {
      tmp = 0x10000|size;
      tSize = size;
      size = 0;
    }

  *((uint32_t *) ( mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_A_B_CNT))) = tmp;
  *((uint32_t *) (mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_SRC_DST_BIDX))) 
    = ((PCIE_TRANSFER_SIZE<<16)|PCIE_TRANSFER_SIZE);
  *((uint32_t *) (mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_LINK_BCNTRLD))) = 0xFFFF;
  if(previous_dma_flag) {
    *((uint32_t *) (mem_region_base + PARAM_SET_OFFSET((param_set_num-1), PARAM_SET_LINK_BCNTRLD)))
       = PARAM_SET_OFFSET(param_set_num, PARAM_SET_OPT);
  }
    
  *((uint32_t *) (mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_SRC_DST_CIDX))) = 0x0;
  /* C Count is set to 1 since mostly size will not be more than 1.75GB */
  *((uint32_t *) (mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_CCNT))) = 0x1;

    if (size != 0) {
      srcAddr += tSize;
      dstAddr += tSize;
     /* Populate the Param entry. */
     *((uint32_t *) ( mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_OPT))) 
        = 0x00400004 | (dma_chan_num << 12);
    } else {
      /* Populate the Param entry. */
      *((uint32_t *) ( mem_region_base + PARAM_SET_OFFSET(param_set_num, PARAM_SET_OPT))) 
        = 0x0010000C | (dma_chan_num << 12);
      break;
    }
  previous_dma_flag = 1;
  param_set_num++;
}
//    printf(" Enter any key to continue...");
//    getchar();

  /* Set the Event Enable Set Register. */
  *((uint32_t *) (mem_region_base + EESR)) = (1 << dma_chan_num);

  /* Set the event set register. */
  *((uint32_t *) (mem_region_base + ESR)) = (1 << dma_chan_num);

//  printf(" Enter any key to continue...");
//  getchar();

  if(!(flag & FLAG_XFER_NOT_WAIT_TILL_COMPLETION)) {
    do
    {
      usleep(1);
      tmp = *((volatile uint32_t *) (mem_region_base + IPR));
    } while(!(tmp & (1 << dma_chan_num)));
    /* Clear any pending interrupt. */
    *((uint32_t *) (mem_region_base + ICR)) = (1 << dma_chan_num);
  }
  sem_post(&bar_sem[dsp_id][bar_num]);
  return (0);
}
static int32_t check_dma_complete(uint32_t dsp_id, uint32_t dma_chan_num)
{
  int32_t ret_val = -1;
  uint32_t tmp;
  uint32_t bar_num;
  uint8_t *mem_region_base;
  uint32_t ib_map_dsp_addr, ib_map_size;

  ib_map_dsp_addr = EDMA_TPCC0_BASE_ADDRESS;
  ib_map_size = EDMA_ACCESS_MAX_OFFSET;
  bar_num = getMemRegion(dsp_id, ib_map_dsp_addr, ib_map_size );
  if((bar_num < 0)
    || (((ib_map_dsp_addr & (minOffsetAlign(EP_MAP_OFFSET_ALIGN, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-1)))
       +ib_map_size) > driverinst.pcidevices[dsp_id]->regions[bar_num].size))
  {
    fprintf(stderr, "%s: ERROR: check_dma_complete: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, ib_map_dsp_addr, ib_map_size);
    return(-1);
  }

//  printf("Debug: do_dsp_dma: got bar number: %x\n", bar_num);
  sem_wait(&bar_sem[dsp_id][bar_num]);
//  printf("Debug: do_dsp_dma: Got semaphore\n");
  ret_val = ti667x_ep_setup_bar(dsp_id, bar_num, ib_map_dsp_addr);
  if(ret_val != 0) {
    sem_post(&bar_sem[dsp_id][bar_num]);
    return (ret_val);
  }
//  printf("Debug: do_dsp_dma: Setup bar complete %d\n", bar_num);
  mem_region_base = (uint8_t *)driverinst.pcieendpoint[dsp_id].memregions[bar_num].mem;


  tmp = *((uint32_t *) (mem_region_base + IPR));
  if (tmp & (1 << dma_chan_num)) {
    sem_post(&bar_sem[dsp_id][bar_num]);
    return(0); 
  }
  sem_post(&bar_sem[dsp_id][bar_num]);
  return(1);
}
/**
 *  @brief Function pciedrv_free_trans_id() Free transaction id for dma transfer
 *  @param[in]    dsp_id                DSP or device id
 *  @param[in]    trans_id              transaction id
 *  @retval       0: for success, -1 for failure
 *  @pre
 *  @post
 */
static int32_t pciedrv_free_trans_id(uint32_t dsp_id, uint32_t trans_id)
{
  int32_t ret_val;
  dma_transaction_t *dma_transaction;

  ret_val = pciedrv_find_dma_transaction(dsp_id, trans_id, &dma_transaction);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_free_trans_id: Unable to find DMA transaction \n", progname);
    return(-1);
  }
  ret_val = obreg_free(dsp_id, dma_transaction->num_ob_registers, dma_transaction->ob_reg_number);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_free_trans_id: ob reg free failed \n", progname);
    return(-1);
  }
  ret_val = pciedrv_dma_free(dsp_id, dma_transaction->dma_chan_number, dma_transaction->param_set_number,
              dma_transaction->num_param_sets);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_free_trans_id: dma free failed \n", progname);
    return(-1);
  }
  ret_val = pciedrv_free_dma_transaction(dsp_id, trans_id);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_free_trans_id: dma transaction free failed \n", progname);
    return(-1);
  }
  return(0);
}


/**
 *  @brief Function pciedrv_dma_read_initiate() Free dma buffers
 *  @param[in]    dsp_id                DSP or device id
 *  @param[in]    addr                  DSP Address to read
 *  @param[in]    frame_desc            Frame descriptor to read data into
 *  @param[in]    flag                  Flag to indicate wait till completion or not
 *  @param[out]   trans_id              Returned transaction id
 *  @retval       0: for success, -1 for failure 
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dma_read_initiate(int32_t dsp_id,
   uint32_t addr, cmem_host_frame_desc_t *frame_desc, uint32_t flag,  uint32_t *trans_id)
{
  int32_t ret_val;
  uint32_t dsp_addr;
  dma_transaction_t *dma_transaction;
  int i;
  uint32_t size;
  uint32_t dma_flag;
  uint32_t num_concat_buffers, start_buffer_number, end_buffer_number;
  uint32_t start_buffer_offset, end_buffer_offset;
   
  size = frame_desc->frameSize;

#ifdef PCIEDRV_VERBOSE
  printf("Debug: enter dma_read_initiate dsp_id : %d, dsp_addr: 0x%x, num buffers %d buf 0x%x, Host buf: 0x%x host dma addr 0x%llx,size 0x%x \n",
    dsp_id, addr, num_concat_buffers, (uint32_t )buf_desc,(uint32_t )buf_desc[0].userAddr,
    buf_desc[0].physAddr, size);
#endif
  
  /* Allocate DMA transaction */
  ret_val = pciedrv_alloc_dma_transaction(dsp_id, &dma_transaction);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_read_initiate: dma transaction allocation failed \n", progname);
    return(-1);
  }
 
  ret_val = alloc_map_frame_desc_to_dsp_memrange(dsp_id, frame_desc, &dsp_addr,  &dma_transaction->ob_reg_number,
      &dma_transaction->num_ob_registers);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_read_initiate: alloc map frame desc failed \n", progname);
    goto obreg_fail;
  }


  dma_transaction->num_param_sets = (size + (MAX_DMA_SINGLE_XFER-1)) / MAX_DMA_SINGLE_XFER;
  if((size > PCIE_TRANSFER_SIZE) && (size % PCIE_TRANSFER_SIZE))
   dma_transaction->num_param_sets += 1;
 

  ret_val = pciedrv_dma_alloc(dsp_id, dma_transaction->num_param_sets,  &dma_transaction->dma_chan_number,  
    &dma_transaction->param_set_number);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_read_initiate: dma allocation failed \n", progname);
    goto dma_alloc_fail;    
  }

  if(flag & PCIEDRV_DMA_XFER_NON_BLOCKING) 
  {
    dma_flag = FLAG_DST_ADDR_HOST | FLAG_XFER_NOT_WAIT_TILL_COMPLETION;
  } else
  {
    dma_flag = FLAG_DST_ADDR_HOST ;
  }
  
  /* Initiate the DMA through DSP */
  ret_val =  do_dsp_dma(dsp_id, dma_transaction->dma_chan_number, dma_transaction->param_set_number, 
               dsp_addr, addr, size, dma_flag);
  if(ret_val < 0 )
  {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_read_initiate: dma operation fail \n", progname);
    goto dma_fail; 
  }
  if(flag & PCIEDRV_DMA_XFER_NON_BLOCKING)
  {
    /* Return transaction id */
    *trans_id = dma_transaction->trans_id;
#ifdef PCIEDRV_VERBOSE
    printf("\n Debug: Allocated transaction id %d\n",*trans_id);
#endif
  } else
  {
    /* free transaction id */
    ret_val = pciedrv_free_trans_id(dsp_id, dma_transaction->trans_id);
    if(ret_val != 0)
    {
      fprintf(stderr, "%s: ERROR:pciedrv_dma_read_initiate: free transaction id failed \n", progname);
      return(-1);
    }
  }
  return(0);
dma_fail:
  /* Free dma resource */
  pciedrv_dma_free(dsp_id, dma_transaction->dma_chan_number, dma_transaction->param_set_number,
              dma_transaction->num_param_sets);
dma_alloc_fail:
  obreg_free(dsp_id, dma_transaction->num_ob_registers, dma_transaction->ob_reg_number);
 
obreg_fail:
  pciedrv_free_dma_transaction(dsp_id, dma_transaction->trans_id);

  return(-1); 
}

/**
 *  @brief Function pciedrv_dma_write_initiate() Free dma buffers
 *  @param[in]    dsp_id                DSP or device id
 *  @param[in]    addr                  DSP Address to write
 *  @param[in]    frame_desc            Frame descriptor of data to be written
 *  @param[in]    flag                  Flag to indicate wait till completion or not
 *  @param[out]   trans_id              Returned transaction id
 *  @retval       0: for success, -1 for failure 
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dma_write_initiate(int32_t dsp_id,
  uint32_t addr, cmem_host_frame_desc_t *frame_desc,  uint32_t flag, uint32_t *trans_id)
{
  int32_t ret_val;
  uint32_t dsp_addr;
  dma_transaction_t *dma_transaction;
  int i;
  uint32_t num_xfer_chunks, size_residue, dma_xfer_size, residue_xfer_size;
  uint32_t size; 
  uint32_t dma_flag;
   uint32_t num_concat_buffers, start_buffer_number, end_buffer_number;
  uint32_t start_buffer_offset, end_buffer_offset;
   
  size = frame_desc->frameSize;

  /* Allocate DMA transaction */
  ret_val = pciedrv_alloc_dma_transaction(dsp_id, &dma_transaction);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_write_initiate: dma transaction allocation failed \n", progname);
    return(-1);
  }

  ret_val = alloc_map_frame_desc_to_dsp_memrange(dsp_id, frame_desc, &dsp_addr,  &dma_transaction->ob_reg_number,
       &dma_transaction->num_ob_registers);
   if(ret_val != 0) {
     fprintf(stderr, "%s: ERROR:pciedrv_dma_write_initiate: alloc map frame desc failed \n", progname);
     goto obreg_fail;
   }

  dma_transaction->num_param_sets = (size + (MAX_DMA_SINGLE_XFER-1)) / MAX_DMA_SINGLE_XFER;
  if((size > PCIE_TRANSFER_SIZE) && (size % PCIE_TRANSFER_SIZE))
   dma_transaction->num_param_sets += 1;
  ret_val = pciedrv_dma_alloc(dsp_id, dma_transaction->num_param_sets,  &dma_transaction->dma_chan_number,  
    &dma_transaction->param_set_number);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_write_initiate: dma allocation failed \n", progname);
    goto dma_alloc_fail;
  }

  if(flag & PCIEDRV_DMA_XFER_NON_BLOCKING) 
  {
    dma_flag = FLAG_SRC_ADDR_HOST | FLAG_XFER_NOT_WAIT_TILL_COMPLETION;
  } else
  {
    dma_flag = FLAG_SRC_ADDR_HOST ;
  }
 
  /* Initiate the DMA through DSP */
  ret_val =  do_dsp_dma(dsp_id, dma_transaction->dma_chan_number, dma_transaction->param_set_number, addr, 
               dsp_addr, size, dma_flag);
  if(ret_val < 0 )
  {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_write_initiate: dma operation fail \n", progname);
    goto dma_fail;
  }

  if(flag & PCIEDRV_DMA_XFER_NON_BLOCKING)
  {
    /* Return transaction id */
    *trans_id = dma_transaction->trans_id;
#ifdef PCIEDRV_VERBOSE
    printf("\n Debug: Allocated transaction id %d\n",*trans_id);
#endif
  } else
  {
    /* free transaction id */
    ret_val = pciedrv_free_trans_id(dsp_id, dma_transaction->trans_id);
    if(ret_val != 0)
    {
      fprintf(stderr, "%s: ERROR:pciedrv_dma_write_initiate: free transaction id failed \n", progname);
      return(-1);
    }
  }

  return(0);

dma_fail:
  /* Free dma resource */
  pciedrv_dma_free(dsp_id, dma_transaction->dma_chan_number, dma_transaction->param_set_number,
              dma_transaction->num_param_sets);
dma_alloc_fail:
  obreg_free(dsp_id, dma_transaction->num_ob_registers, dma_transaction->ob_reg_number);
 
obreg_fail:
  pciedrv_free_dma_transaction(dsp_id, dma_transaction->trans_id);

  return(-1);
}

/**
 *  @brief Function pciedrv_dma_check() Check if the dma operation is complete
 *  @param[in]    dsp_id      DSP or device id
 *  @param[in]    trans_id    Transaction id to check
 *  @retval        -1 for failure, 0 : DMA complete 1: DMA in progress
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dma_check(uint32_t dsp_id, uint32_t trans_id)
{
  int32_t ret_val;
  dma_transaction_t *dma_transaction;

  /* Find dma transaction corresponding to transaction id */
  ret_val = pciedrv_find_dma_transaction(dsp_id, trans_id, &dma_transaction);
  if(ret_val != 0) {
    fprintf(stderr, "%s: ERROR:pciedrv_dma_check: Unable to find DMA transaction \n", progname);
    return(-1);
  }
  /* Check if dma is complete */
  ret_val = check_dma_complete(dsp_id, dma_transaction->dma_chan_number);
  if(ret_val == 0) {
    /* DMA is complete : Free OB region */
    ret_val = obreg_free(dsp_id, dma_transaction->num_ob_registers, dma_transaction->ob_reg_number);
    if(ret_val != 0) {
      fprintf(stderr, "%s: ERROR:pciedrv_dma_check: ob reg free failed\n", progname);
      return(-1);
    }
    /* Free dma resource */
    ret_val = pciedrv_dma_free(dsp_id, dma_transaction->dma_chan_number, dma_transaction->param_set_number,
                dma_transaction->num_param_sets);
    if(ret_val != 0) {
      fprintf(stderr, "%s: ERROR:pciedrv_dma_check: dma free failed\n", progname);
      return(-1);
    }
    /* Free the dma transaction */
    ret_val = pciedrv_free_dma_transaction(dsp_id, trans_id);
    if(ret_val != 0) {
      fprintf(stderr, "%s: ERROR:pciedrv_dma_check: dma transaction free failed\n", progname);
      return(-1);
    }
  }
  return(ret_val);
}

static int32_t obreg_alloc(uint32_t dsp_id, uint16_t num_contiguous_regions, uint16_t *obreg_num)
{
  int i,j;
  uint32_t bitmap, req_bitmap;

  /* Calculate bitmap for contiguous regions */
  req_bitmap =((uint64_t)1 << num_contiguous_regions)-1;

  /* Search to see free regions available */
  for(i=0; i < pciedrv_context.num_ob_instances; i++) {
    if((i + num_contiguous_regions) > pciedrv_context.num_ob_instances)
    {
       fprintf(stderr, "%s: ERROR:obreg_alloc: contiguous regions start %d, num regions %d exceed available: %d\n", progname,
         i, (i + num_contiguous_regions),  pciedrv_context.num_ob_instances );
       return(-1);
    }

    bitmap = 0;
    sem_wait(&allocation_sem[dsp_id]);
    /* Calculate teh moving bit map */
    for(j=0; j< num_contiguous_regions; j++) {
      bitmap += ((uint32_t)((obreg_alloc_inst[dsp_id][i+j].flags  & OBREG_FLAG_ALLOCATED) ? 0: 1)) << j;
    }
       
    if(bitmap  == req_bitmap) {
       /* Found free region */
       *obreg_num = obreg_alloc_inst[dsp_id][i].ob_reg_number;
       for(j=0; j< num_contiguous_regions; j++) {
       obreg_alloc_inst[dsp_id][i+j].flags |= OBREG_FLAG_ALLOCATED;
      }
      sem_post(&allocation_sem[dsp_id]);
      return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  fprintf(stderr, "%s: ERROR:obreg_alloc: Allocation failed\n", progname);

  /* allocation failed */
  return(-1);
}

static int32_t obreg_free(uint32_t dsp_id, uint16_t num_contiguous_regions, uint16_t obreg_num)
{
  int i,j;

  /* Free the individual contiguous regions */
  for(i=0; i < pciedrv_context.num_ob_instances; i++) {
    if((i + num_contiguous_regions) > pciedrv_context.num_ob_instances)
    {
      fprintf(stderr, "%s: ERROR:obreg_free: Error in free\n", progname);
      return(-1);
    }
    sem_wait(&allocation_sem[dsp_id]);
    if((obreg_alloc_inst[dsp_id][i].ob_reg_number ==obreg_num) 
      && (obreg_alloc_inst[dsp_id][i].flags  & OBREG_FLAG_ALLOCATED)) {

      /* Critical section */
      for(j=0; j< num_contiguous_regions; j++) {
        obreg_alloc_inst[dsp_id][i+j].flags &= (~OBREG_FLAG_ALLOCATED);
      }
       sem_post(&allocation_sem[dsp_id]);
       return(0);
    }
    sem_post(&allocation_sem[dsp_id]);
  }
  fprintf(stderr, "%s: ERROR:obreg_free: Free failed\n", progname);
  /* Free  failed */
  return(-1);
}

/**
 *  @brief Function pciedrv_dsp_memrange_alloc() Allocate dsp ob region 
 *  @param[in]    mem_size        size of dsp outbound memory to allocate
 *  @param[out]   dsp_start_addr  Allocated DSP start address
 *  @retval       0: success, -1 : failure
 *  @pre  
 *  @post 
 */
int32_t pciedrv_dsp_memrange_alloc(uint32_t dsp_id, uint32_t mem_size, uint32_t *dsp_start_addr)
{
  uint16_t num_contiguous_regions;
  uint16_t obreg_num;

  /* Calculate number of regions */
  num_contiguous_regions= (mem_size+(pciedrv_context.dsp_outbound_block_size-1))/pciedrv_context.dsp_outbound_block_size;

  /* allocate regions */
  if(obreg_alloc(dsp_id, num_contiguous_regions, &obreg_num)!=0) {
    *dsp_start_addr = 0;
    return(-1);
  }
  /* Calculate start address */
  *dsp_start_addr = dspaddr_of_obregion(obreg_num);
  return(0);
}

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
int32_t pciedrv_dsp_memrange_free(uint32_t dsp_id, uint32_t mem_size, uint32_t dsp_start_addr)
{
  uint16_t num_contiguous_regions;
  uint16_t obreg_num;
  
  obreg_num = obregion_of_dspaddr(dsp_start_addr);
  
  num_contiguous_regions= (mem_size+(pciedrv_context.dsp_outbound_block_size-1))/pciedrv_context.dsp_outbound_block_size;
  
  if(obreg_free(dsp_id, num_contiguous_regions, obreg_num)!=0) {
    return(-1);
  }

  return(0);
  
}
/**
 *  @brief Function free_map_bufs_to_dsp_memrange() Map host buffers to the
 *         allocated DSP outbound memory range
 *  @param[in]    dsp_id          DSP or device id
 *  @param[in]    dsp_start_addr  DSP start address to map
 *  @param[in]    alloc_size      Allocated size
 *  @retval       0: success, -1 : failure
 *  @pre
 *  @post
 */
int32_t free_map_frame_desc_to_dsp_memrange(uint32_t dsp_id,
    uint32_t dsp_start_addr, uint32_t alloc_size)
{

  uint16_t num_contiguous_regions;
  uint16_t obreg_num;

  obreg_num = obregion_of_dspaddr(dsp_start_addr);

  num_contiguous_regions= alloc_size/pciedrv_context.dsp_outbound_block_size;

  if(obreg_free(dsp_id, num_contiguous_regions, obreg_num)!=0) {
    return(-1);
  }

  return(0);
}
/**
 *  @brief Function config_contiguous_ob_range() Configure contigous OB
 *          Range for DSP.
 *  @param[in]     dsp_id           DSP or device id
 *  @param[in]     num_of_buffers   Number of buffers
 *  @param[in]     buf_desc	    Array of buffers
 *  @param[out]    ob_num_start     Outbound window number for dsp
 *  @param[out]    num_ob_regions   Number of outbound regions
 *  @retval        0: for success, -1 for failure 
 *  @pre  
 *  @post 
 */

static int32_t config_contiguous_ob_range(uint32_t dsp_id, uint32_t num_of_buffers,
  cmem_host_buf_desc_t buf_desc[], uint16_t ob_num_start, uint16_t *num_ob_regions)
{
  int i,j;

  uint16_t ob_count=0;

  for(i=0; i< num_of_buffers; i++) {
    for(j = 0; j < (buf_desc[i].length+pciedrv_context.dsp_outbound_block_size-1)
       /pciedrv_context.dsp_outbound_block_size; j++) 
    {
      if(ob_reg_cfg(dsp_id, (ob_num_start)+ob_count,
       (buf_desc[i].physAddr & (~((uint64_t )pciedrv_context.dsp_outbound_block_size-1)))
       + j*pciedrv_context.dsp_outbound_block_size) != 0) 
      {
        fprintf(stderr, "%s: ERROR:config_contiguous_ob_range: .dsp_id %d, buff start %d, buff no %d\n", progname
          , dsp_id, ob_num_start, (ob_num_start+ob_count));
        return(-1);
      }
      ob_count++;
    }
  }
  *num_ob_regions = ob_count;
  return(0);
}


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
int32_t pciedrv_map_hostbufs_to_dsp_memrange(uint32_t dsp_id, uint32_t num_of_bufs, cmem_host_buf_desc_t buf_desc[], uint32_t dsp_start_addr)
{
   uint16_t ob_region_no;
   uint16_t ob_num_regions;

  ob_region_no = obregion_of_dspaddr(dsp_start_addr);
  
   return(config_contiguous_ob_range(dsp_id, num_of_bufs, buf_desc, ob_region_no, &ob_num_regions));
}

/**
 *  @brief Function alloc_map_bufs_to_dsp_memrange() Map host buffers to the
 *         allocated DSP outbound memory range
 *  @param[in]    dsp_id          DSP or device id
 *  @param[in]    frame_desc      Frame descriptor to map
 *  @param[out]   dsp_start_addr  DSP start address to map
 *  @param[out]   start_ob_reg_no Start of ob region number
 *  @param[out]   num_regions      Allocated size
 *  @retval       0: success, -1 : failure
 *  @pre
 *  @post
 */
static int32_t alloc_map_frame_desc_to_dsp_memrange(uint32_t dsp_id, cmem_host_frame_desc_t *frame_desc,
    uint32_t *dsp_start_addr, uint16_t *start_ob_reg_no, uint16_t *num_regions)
{
  int i,j;
  uint16_t ob_region_no;
  uint16_t ob_num_regions_alloc, ob_count;

  uint32_t  num_concat_buffers, start_buffer_number, end_buffer_number;
  uint32_t start_buffer_offset, end_buffer_offset, remaining_length, current_buffer_size;
  uint32_t size;
  int32_t ret_val;
  uint32_t map_offset, start_offset;

  *num_regions = 0;

  size = frame_desc->frameSize;
  num_concat_buffers = frame_desc->numBuffers;

  /* Calculate start buffer number and offset */
  start_buffer_number=0;
  start_buffer_offset = frame_desc->frameStartOffset;
  while(start_buffer_offset > frame_desc->bufDescP[start_buffer_number].length)
  {
     start_buffer_offset -= frame_desc->bufDescP[start_buffer_number].length;
     start_buffer_number++;
     if(start_buffer_number > num_concat_buffers)
     {
        /* Error in offset */
        fprintf(stderr, "%s: ERROR: alloc_map_frame_desc_to_dsp_memrange: Error in start offset \n", progname);
        return(-1);
     }
  }

  /* Calculate end buffer number and offset */
  end_buffer_number= start_buffer_number;
  end_buffer_offset= start_buffer_offset+size;
  while(end_buffer_offset > frame_desc->bufDescP[end_buffer_number].length)
  {
     end_buffer_offset -= frame_desc->bufDescP[end_buffer_number].length;
     end_buffer_number++;
     if(end_buffer_number > num_concat_buffers)
     {
        /* Error in offset+ length */
        fprintf(stderr, "%s: ERROR: alloc_map_frame_desc_to_dsp_memrange: Error in length & offset \n", progname);
        return(-1);
     }

  }

  num_concat_buffers = end_buffer_number-start_buffer_number+1;

  /* Calculate number of buffers */
  map_offset = (frame_desc->bufDescP[start_buffer_number].physAddr + start_buffer_offset) &
     (pciedrv_context.dsp_outbound_block_size-1);

  ob_num_regions_alloc = (map_offset + size + pciedrv_context.dsp_outbound_block_size-1)
      /pciedrv_context.dsp_outbound_block_size;

  ret_val = obreg_alloc(dsp_id, ob_num_regions_alloc, &ob_region_no);
  if(ret_val != 0)
  {
    fprintf(stderr, "%s: ERROR: alloc_map_frame_desc_to_dsp_memrange: obreg alloc failed \n", progname);
    return(-1);
  }
  ob_count=0;
  remaining_length = size;
  for(i=0; i < num_concat_buffers; i++) {
    if(ob_count > ob_num_regions_alloc)
    {
      fprintf(stderr, "%s: ERROR:alloc_map_frame_desc_to_dsp_memrange: difference in number of buffers allocated and mapped  Allocated %d Mapped %d\n", progname, ob_num_regions_alloc, ob_count);
      obreg_free(dsp_id, ob_num_regions_alloc, ob_region_no);
      return(-1);
    }
    if(i == 0)
    {
      start_offset = start_buffer_offset;
    } else
    {
      start_offset = 0;
      if(frame_desc->bufDescP[start_buffer_number+i].physAddr & (pciedrv_context.dsp_outbound_block_size-1))
      {
         obreg_free(dsp_id, ob_num_regions_alloc, ob_region_no);
         return(-1);
      }
    }
    current_buffer_size = frame_desc->bufDescP[start_buffer_number+i].length-start_offset;
    if(current_buffer_size > remaining_length)
      current_buffer_size = remaining_length;

    for(j = 0; j < (current_buffer_size + pciedrv_context.dsp_outbound_block_size-1)
        /pciedrv_context.dsp_outbound_block_size; j++)
    {
      /* Program the OB register */
      ob_reg_cfg(dsp_id,  ob_region_no+ob_count,
        ((frame_desc->bufDescP[start_buffer_number+i].physAddr+start_offset )
            & (~((uint64_t )pciedrv_context.dsp_outbound_block_size-1))) );
      ob_count++;
    }
    remaining_length -= current_buffer_size;

  }

  if(ob_count != ob_num_regions_alloc)
  {
    fprintf(stderr, "%s: ERROR:alloc_map_frame_desc_to_dsp_memrange: Final number of buffers allocated and mapped not match  Allocated %d Mapped %d\n", progname, ob_num_regions_alloc, ob_count);
    obreg_free(dsp_id, ob_num_regions_alloc, ob_region_no);
    return(-1);
  }
  *num_regions = ob_count;
  *start_ob_reg_no = ob_region_no;
  *dsp_start_addr = dspaddr_of_obregion(ob_region_no)+map_offset;
  return(0);

}

/**
 *  @brief Function pciedrv_alloc_map_frame_desc_to_dsp_memrange() Map host buffers to the
 *         allocated DSP outbound memory range
 *  @param[in]    dsp_id          DSP or device id
 *  @param[in]    frame_desc      Frame descriptor to map
 *  @param[out]   dsp_start_addr  DSP start address to map
 *  @param[out]   alloc_size      Allocated size
 *  @retval       0: success, -1 : failure
 *  @pre
 *  @post
 */
int32_t pciedrv_alloc_map_frame_desc_to_dsp_memrange(uint32_t dsp_id, cmem_host_frame_desc_t *frame_desc,
    uint32_t *dsp_start_addr,  uint32_t *alloc_size)
{

  uint16_t start_ob_reg_no;
  uint16_t num_regions;

  if(alloc_map_frame_desc_to_dsp_memrange(dsp_id, frame_desc, dsp_start_addr,
    &start_ob_reg_no, &num_regions)!= 0)
    return(-1);

  *dsp_start_addr = dspaddr_of_obregion(start_ob_reg_no);
  *alloc_size = num_regions *pciedrv_context.dsp_outbound_block_size;

  return(0);
}

/**
 *  @brief Function pciedrv_get_pcie_addr() Get pcie_address for bar window region
 *  @param[in]    dsp_id        DSP or device id
 *  @param[in]    bar_num       Bar number
 *  @retval       pcie address of Bar window region
 *  @pre  
 *  @post 
 */
uint32_t pciedrv_get_pcie_addr(uint32_t dsp_id, uint32_t bar_num)
{
  return(driverinst.pcidevices[dsp_id]->regions[bar_num].base_addr );
}

/**
 *  @brief Function pciedrv_get_num_devices() Get number of the TI devices on the PCI bus
 *  @retval       number of devices
 *  @pre          pciedrv_open should be called before calling this function
 *  @post 
 */
uint32_t pciedrv_get_num_devices(void)
{
  return(pciedrv_context.num_devices);
}

/**
 *  @brief Function pciedrv_get_pci_info() Get pci information of all the TI devices
 *  @param[in]    devices_info : To populat the devices information
 *  @param[in]    bar_num       Bar number
 *  @retval       0: success, -1 : failure
 *  @pre          pciedrv_open should be called before calling this function
 *  @post 
 */
int32_t pciedrv_get_pci_info(pciedrv_device_info_t *devices_info)
{
  int i;
  char name[256];
  char filestring[256];
  uint32_t read_value;
  struct pci_device* pcidevice;
  FILE *fp;
  int ret_value;

  for(i=0; i < pciedrv_context.num_devices; i++)
  {
    pcidevice = driverinst.pcidevices[i];
    snprintf(name, 255, "%s/%04x:%02x:%02x.%1u",
      SYS_BUS_PCI,
      pcidevice->domain,
      pcidevice->bus,
      pcidevice->dev,
      pcidevice->func,
      i);
    /* Follow the link to get the actual sysfs path */
    ret_value = readlink(name, devices_info[i].sysfspath, 255);
    if(ret_value < 0 )
    {
      return(-1);
    }
    devices_info[i].sysfspath[ret_value]= '\0';
#if 0
    snprintf(filestring, 255, "%s/%s/../vendor",
      SYS_BUS_PCI,
      pathname);

    fp = fopen(filestring, "r");
    fscanf(fp,"%x",&read_value);
    printf("\nSwitch vendor: %x", read_value);
    fclose(fp);
#endif
    /*Get the path name for switch device's device file one level below */
    snprintf(filestring, 255, "%s/%s/../device",
      SYS_BUS_PCI,
      devices_info[i].sysfspath);
    /* Open  file for reading */
    fp = fopen(filestring, "r");
    if(fp == NULL)
    {
      return(-1);
    }
    /* Get switch device number */
    ret_value = fscanf(fp,"%x",(uint32_t *)(&devices_info[i].switch_device));
    if(ret_value==0) 
      return(-1);

#ifdef PCIEDRV_VERBOSE
    printf("\ndsp_id %d Switch device: %x",i, devices_info[i].switch_device);
#endif
    fclose(fp);
  } 
  return 0;
}

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
 */
 int32_t pciedrv_map_dspaddr_to_inbound(int32_t dsp_id, uint32_t dsp_addr, uint32_t size)
{
  int32_t   bar_num, ret_val;
  uint32_t  offset, offsetAlign;

  bar_num = get_dsp2dsp_remote_map_memregion(dsp_id, dsp_addr, size);

  offsetAlign = minOffsetAlign(EP_MAP_OFFSET_ALIGN, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size-1);
  offset = dsp_addr & offsetAlign;
  
  if(bar_num < 0)
  {
    fprintf(stderr, "%s: ERROR: pciedrv_ob_cfg: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, dsp_addr, size);
    return(-1);
  }

  if ( (size + offset)  > driverinst.pcieendpoint[dsp_id].memregions[bar_num].size ) 
  {
    fprintf(stderr, "%s: ERROR:  pciedrv_bar_cfg: Size to be mapped to BAR is too large! (0x%X > 0x%zX)\n", 
      progname, size, driverinst.pcieendpoint[dsp_id].memregions[bar_num].size);
    return -1;
  }

  /* Acquire lock */
  sem_wait(&bar_sem[dsp_id][bar_num]);

  /* Set up Bar register */
  ret_val = ti667x_ep_setup_bar(dsp_id, bar_num, dsp_addr & (~offsetAlign));
  
  if(ret_val < 0 )
  {
    fprintf(stderr, "%s: ERROR:  pciedrv_dsp_read: Setup Bar register fail \n", progname);

    sem_post(&bar_sem[dsp_id][bar_num]);
    return(-1);
  }

  return 0;
}

/**
 *  @brief Function pciedrv_free_inbound_mapping() Will free the inbound register so 
 *                                                 that it may be remapped.
 *
 *  @param[in]    dsp_id : ID of the DSP 
 *  @param[in]    dsp_addr : Address to map for inbound access.
 *  @param[in]    size : Size to map. 
 *  @retval       0: success, -1 : failure
 */
int32_t pciedrv_free_inbound_mapping(int32_t dsp_id, uint32_t dsp_addr, uint32_t size)
{
  int32_t   bar_num;

  bar_num = get_dsp2dsp_remote_map_memregion(dsp_id, dsp_addr, size);

  if(bar_num < 0)
  {
    fprintf(stderr, "%s: ERROR: pciedrv_ob_cfg: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, dsp_addr, size);
    return(-1);
  }

  sem_post(&bar_sem[dsp_id][bar_num]);
}


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
 */
int32_t pciedrv_map_outbound_to_rmt_dspaddr(int32_t dsp_id, int32_t rmt_dsp_id, uint32_t rmt_dsp_addr, uint32_t size, uint32_t *mapped_dsp_addr)
{
  cmem_host_buf_desc_t buf_desc;
  int32_t   bar_num, ret_val;
  uint32_t  addr_offset, align_offset;
  uint16_t  num_of_regions_alloc, num_of_regions_map, start_ob_reg_no, num_regions;

  bar_num = get_dsp2dsp_remote_map_memregion(rmt_dsp_id, rmt_dsp_addr, size);

  if(bar_num < 0)
  {
    fprintf(stderr, "%s: ERROR: pciedrv_ob_cfg: Memory address out of range \n", progname);
    fprintf(stderr, "%s: Addr 0x%x Size 0x%x\n \n", progname, rmt_dsp_addr, size);
    return(-1);
  }

  if ( (rmt_dsp_addr < driverinst.pcieendpoint[rmt_dsp_id].memregions[bar_num].dspmem) || 
       ( (rmt_dsp_addr + size) > (driverinst.pcieendpoint[rmt_dsp_id].memregions[bar_num].dspmem 
                                 + driverinst.pcieendpoint[rmt_dsp_id].memregions[bar_num].size) ) ) 
  {
    fprintf(stderr, "%s: ERROR: pciedrv_ob_cfg: Remote DSP buffer is out of range of current BAR configuration!\n", progname);
    return -1;
  }

  addr_offset = rmt_dsp_addr - driverinst.pcieendpoint[rmt_dsp_id].memregions[bar_num].dspmem;

  align_offset = ((driverinst.pcidevices[rmt_dsp_id]->regions[bar_num].base_addr + addr_offset)
                  & (pciedrv_context.dsp_outbound_block_size-1));

  /* Calculate number of regions */
  num_of_regions_alloc= (align_offset + size +(pciedrv_context.dsp_outbound_block_size-1))
       /pciedrv_context.dsp_outbound_block_size;

  /* allocate regions */
  if(obreg_alloc(dsp_id, num_of_regions_alloc, &start_ob_reg_no)!=0) {
    *mapped_dsp_addr = 0;
    return(-1);
  }
  buf_desc.physAddr = driverinst.pcidevices[rmt_dsp_id]->regions[bar_num].base_addr + addr_offset;
  buf_desc.length = align_offset + size;

  ret_val = config_contiguous_ob_range(dsp_id, 1, &buf_desc, start_ob_reg_no, &num_of_regions_map);
  if(ret_val != 0)
  {
    return(-1);
  }
  if(num_of_regions_map != num_of_regions_alloc)
  {
    obreg_free(dsp_id, num_of_regions_alloc, start_ob_reg_no);
    return(-1);
  }
  num_regions = num_of_regions_map;
  *mapped_dsp_addr = dspaddr_of_obregion(start_ob_reg_no) + align_offset;

  return(0);
}

/*** nothing past this point ***/

