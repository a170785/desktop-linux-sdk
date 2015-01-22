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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <bfd.h>
#include "stdint.h"
#include "pcieLocalReset.h"
#include "pciedrv.h"
#include "c6678_boot_defs.h" 
#include "dnldmgr.h"

/* Block size in bytes when r/w data between GPP and DSP via DSP CPU */
#define DSP_LOAD_BLOCK_TRANSFER_SIZE          0x100      
#define IS_CORE_LOCAL_ADDR(addr) ((addr & 0xff000000) == 0)
#define LOCAL_TO_GLOBAL_ADDR(addr, core_id) (addr | ((core_id + 0x10) << 24))

/**
 *  @brief Function dsp_local_write() write to local memory of DSP
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     core_id        Core ID within the chip specified by dsp_id
 *  @param[in]     addr           DSP Address to write
 *  @param[in]     buf            Destination Buffer pointer
 *  @param[in]     size           size of Memory read in bytes
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
static int32_t dsp_local_write(int32_t dsp_id, int32_t core_id, uint32_t addr, uint8_t *buf, uint32_t size)
{
 int32_t ret;

 if (IS_CORE_LOCAL_ADDR(addr)) // It points to 'per core' address space.
 {
   addr = LOCAL_TO_GLOBAL_ADDR(addr, core_id);
 }

 ret = pciedrv_dsp_write(dsp_id, addr, (unsigned char *)buf, size);

 return(ret);	
}
/**
 *  @brief Function do_reg_read() Read register
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Register Address to read
 *  @retval        Register value
 *  @pre  
 *  @post 
 */

static uint32_t do_reg_read(uint32_t dsp_id, uint32_t addr)
{
  unsigned value;
  int32_t ret_value;

  ret_value = pciedrv_dsp_read( dsp_id, addr, (unsigned char *)&value, 4); 
  if(ret_value != 0)
  {
    fprintf(stderr, "\n do_reg_read: ERROR Reading Address %x\n", addr);
  }
  return(value);
}
/**
 *  @brief Function do_reg_write() Write register
 *  @param[in]     dsp_id         DSP Chip ID
 *  @param[in]     addr           DSP Register Address to write
 *  @retval        none
 *  @pre  
 *  @post 
 */

static void do_reg_write(unsigned dsp_id, uint32_t addr, uint32_t value)
{
  unsigned value_local;
  int32_t ret_value;

  value_local = value;
  ret_value = pciedrv_dsp_write( dsp_id, addr, (unsigned char *)&value_local, 4);  
  if(ret_value != 0)
  {
    fprintf(stderr, "\n do_reg_write: ERROR Writing to Address %x\n", addr);
  }

}

/* ============================================================================
*  @func   coreLocalReset
*
*  @desc   Reset a particular CorePac, 6678 Data Manual, section 7.4.4
*          initiated by LPSC MMRs
*
*  @modif  None.
*  ============================================================================
*/
void coreLocalReset(uint32_t dsp_id, uint32_t pid, uint32_t mid, uint32_t state) 
{
    uint32_t *pReg, temp, counter = 0;

    temp =  do_reg_read(dsp_id, PSC_BASE_ADDRESS + MDCTL(mid));  
    if (state == 0) {
        /* Reset assert */
        temp = ((temp & ~0x1F) | PSC_ENABLE) & (~0x100);
#ifdef DNLDMGR_VERBOSE
        printf("Start local reset assert for core dsp_id %d, (module id): %d ...\n", dsp_id, mid);
#endif
    } else 	{
        /* Reset de-assert */
        temp = (temp & ~0x1F) | PSC_ENABLE | (1 << 8);
#ifdef DNLDMGR_VERBOSE
        printf("Start local reset de-assert for core  dsp_id %d, (module id): %d ...\n",dsp_id, mid);
#endif
    }

    do_reg_write(dsp_id, PSC_BASE_ADDRESS + MDCTL(mid), temp);    /* Assert/De-assert local reset */

    /* No previous transition in progress */
    counter = 0;
    while (1) {
        temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTSTAT);
        if ((temp & (1 << pid)) == 0) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\ncoreLocalReset: Previous transition in progress dsp_id %d pid %d mid %d state: %d\n", dsp_id, pid, mid, state);
            break;
        }
    }

    do_reg_write(dsp_id, PSC_BASE_ADDRESS + PTCMD, (1 << pid)); 

    /* Current transition finished */
    counter = 0;
    while (1) {
        temp =  do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTSTAT);
        if ((temp & (1 << pid)) == 0) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\ncoreLocalReset: Current transition in progress dsp_id %d pid %d mid %d state: %d\n", dsp_id, pid, mid, state);
            break;
        }
    }

    /* Verifying state change */
    counter = 0;
    while (1) {
        temp =  do_reg_read(dsp_id, PSC_BASE_ADDRESS + MDSTAT(mid));
        if ((temp & 0x1F) == 3) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\ncoreLocalReset: MD stat for dsp_id %d pid %d mid %d state: %d timeout\n", dsp_id, pid, mid, state);
            break;
        }
    }

}

/* ============================================================================
*  @func   setPDState
*
*  @desc   Set a new power state for the specified domain id in a power controler
*          domain. Wait for the power transition to complete.
*
*      pid   -  power domain.
*      state -  new state value to set (1 = ON; 0 = OFF )
*
*  @modif  None.
*  ============================================================================
*/
void setPDState(uint32_t dsp_id, uint32_t pid, uint32_t state)
{
    uint32_t *pReg, mdctl, pdctl, temp, counter = 0;
    uint32_t mid;

    pdctl = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PDCTL(pid));

    /* No previous transition in progress */
    counter = 0;
    while (1) {
        temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTSTAT);
        if ((temp & (1 << pid)) == 0) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\n setPDState: dsp_id %d: Previous transition in progress pid %d state: %d\n",dsp_id, pid, state);
            break;
        }
    }
    /* Set power domain control */
    do_reg_write(dsp_id,  PSC_BASE_ADDRESS + PDCTL(pid), ((pdctl & (~ 0x1)) | state)); 

    /* Start power transition by setting PTCMD GO to 1 */
    temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTCMD);
    do_reg_write(dsp_id,  PSC_BASE_ADDRESS + PTCMD, temp | (0x1 << pid)); 

    /* Current transition finished */
    counter = 0;
    while (1) {
        temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTSTAT);
        if ((temp & (1 << pid)) == 0) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\n setPDState: dsp_id %d: Current transition in progress pid %d state: %d\n", dsp_id, pid, state);
            break;
        }
    }

}
/* ============================================================================
*  @func   setPscState
*
*  @desc   Set a new power state for the specified domain id in a power controler
*          domain. Wait for the power transition to complete.
*
*      pid   -  power domain.
*      mid   -  module id to use for module in the specified power domain
*      state -  new state value to set (0 = RESET; 3 = ENABLE)
*
*  @modif  None.
*  ============================================================================
*/
void setPscState(uint32_t dsp_id, uint32_t pid, uint32_t mid, uint32_t state)
{
    uint32_t *pReg, mdctl, pdctl, temp, counter = 0;

    mdctl = do_reg_read(dsp_id, PSC_BASE_ADDRESS + MDCTL(mid));
    pdctl = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PDCTL(pid));

    /* No previous transition in progress */
    counter = 0;
    while (1) {
        temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTSTAT);
        if ((temp & (1 << pid)) == 0) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\nsetPscState: dsp_id %d: Previous transition in progress pid %d mid %d state: %d\n",dsp_id, pid, mid, state);
            break;
        }
    }

    /* Set power domain control */
    do_reg_write(dsp_id,  PSC_BASE_ADDRESS + PDCTL(pid), (pdctl | 0x1)); 

    /* Set MDCTL NEXT to new state */
    mdctl = ((mdctl) & ~(0x1f)) | state;
    do_reg_write(dsp_id,  PSC_BASE_ADDRESS + MDCTL(mid), mdctl); 

    /* Start power transition by setting PTCMD GO to 1 */
    temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTCMD);
    do_reg_write(dsp_id,  PSC_BASE_ADDRESS + PTCMD, temp | (0x1 << pid)); 

    /* Current transition finished */
    counter = 0;
    while (1) {
        temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + PTSTAT);
        if ((temp & (1 << pid)) == 0) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\n setPscState: dsp_id %d: Current transition in progress pid %d mid %d state: %d\n", dsp_id, pid, mid, state);
            break;
        }
    }

    /* Verifying state change */
    counter = 0;
    while (1) {
        temp = do_reg_read(dsp_id, PSC_BASE_ADDRESS + MDSTAT(mid));
        if ((temp & 0x1F) == state) break;
        usleep(1000);
        counter ++;
        if (counter > 10) {
            fprintf(stderr, "\nsetPscState: dsp_id %d: MD stat for pid %d mid %d expected state: %d state: %d timeout\n", dsp_id, pid, mid, state, (temp & 0x1f));
            break;
        }
    }

}
/* ============================================================================
*  @func   set_PA_state_to_reset
*
*  @desc   Set the PA PDSPs to reset state 
*
*      dsp_id   -  Dsp id
*
*  @modif  None.
*  ============================================================================
*/
void set_PA_state_to_reset(uint32_t dsp_id)
{
  int i;

  /* Put each of the PDSPs into reset */
  for (i = 0; i < 6; i++)  {
    do_reg_write(dsp_id,  CSL_PA_SS_CFG_REGS + PDSP_CONTROL_OFFSET(i), 0); 
  }

  /* Reset packet ID */
  do_reg_write(dsp_id,  CSL_PA_SS_CFG_REGS + PDSP_PKT_ID_SOFT_RESET_OFFSET, 1); 

  /* Reset LUT2 */
  do_reg_write(dsp_id,  CSL_PA_SS_CFG_REGS + PDSP_LUT2_SOFT_RESET_OFFSET, 1); 

  /* Reset statistics */
  do_reg_write(dsp_id,  CSL_PA_SS_CFG_REGS + PDSP_STATS_SOFT_RESET_OFFSET, 1); 

  /* Reset timers */
  for (i = 0; i < 6; i++)
    do_reg_write(dsp_id,  CSL_PA_SS_CFG_REGS + PDSP_TIMER_CNTRL_REG_OFFSET(i), 0); 

  usleep(100);

}
/* ============================================================================
*  @func   set_SA_state_to_reset
*
*  @desc   Set the SA PDSPs to reset state 
*
*      dsp_id   -  Dsp id
*
*  @modif  None.
*  ============================================================================
*/
void set_SA_state_to_reset(uint32_t dsp_id)
{
  int i;

  /* Disable and reset PDSPs */
  for (i = 0; i < 2; i++)  {
    do_reg_write(dsp_id,  CSL_PA_SS_CFG_CP_ACE_CFG_REGS + PDSP_CONTROL_OFFSET(i), 0); 
  }
  usleep(100);
}
/* ============================================================================
*  @func   setBootAddrIpcgr
*
*  @desc   Write boot entry point into DSP_BOOT_ADDR0 and the send an IPC
*
*  @modif  None.
*  ============================================================================
*/
uint32_t setBootAddrIpcgr(uint32_t dsp_id, uint32_t core, uint32_t addr)  
{

    /* Unlock KICK0, KICK1 */
    do_reg_write(dsp_id,  CHIP_LEVEL_BASE_ADDRESS + KICK0, KICK0_UNLOCK); 
    do_reg_write(dsp_id,  CHIP_LEVEL_BASE_ADDRESS + KICK1, KICK1_UNLOCK); 

    /* Check if the last 10 bits of addr is 0 */
    if ((addr & 0x3ff) != 0) {
        fprintf(stderr, "\nsetBootAddrIpcgr: The address is not 1K aligned.\n");
        return 0;
    }

    do_reg_write(dsp_id,  CHIP_LEVEL_BASE_ADDRESS + DSP_BOOT_ADDR(core), addr); 
/* TODO: this IPCGR is not really required */
 //   do_reg_write(dsp_id,  CHIP_LEVEL_BASE_ADDRESS + IPCGR(core), 1); 

    usleep(1000); 

    return 1;
}


/* ============================================================================
*  @func   byteto32bits 
*
*  @desc   Convert 4 bytes to 32 bits long word
*
*  @modif  None.
*  ============================================================================
*/
uint32_t byteTo32bits(uint8_t *pDspCode)
{
    uint32_t i, temp;

    temp = *pDspCode++;
    for(i = 0; i < 3;i++) {
        temp <<= 8;
        temp |= *pDspCode++;
    }
    return(temp);
}

/* ============================================================================
*  @func   pushData
*
*  @desc   Parser function for DSP boot image array
*
*  @modif  None.
*  ============================================================================
*/
void pushData(uint32_t dsp_id, uint8_t *pDspCode, uint8_t coreNum, uint32_t *bootEntryAddr)
{
    uint32_t i, j, tempArray[DSP_LOAD_BLOCK_TRANSFER_SIZE/4];
    uint32_t size, section = 0, totalSize = 0;
    uint32_t count, remainder, startaddr;

    /* Get the boot entry address */
    *bootEntryAddr = byteTo32bits(pDspCode);
#ifdef DNLDMGR_VERBOSE
    printf("Boot entry address is 0x%8x\n", *bootEntryAddr);
#endif
    pDspCode +=4;

    while(1) {

        /* Get the size */
        size = byteTo32bits(pDspCode);
        if(size == 0) break;

        if ((size/4)*4 != size) {
            size = ((size/4)+1)*4;
        }

        totalSize += size;
        section++;
        pDspCode += 4;
        startaddr = byteTo32bits(pDspCode);

        pDspCode+= 4;

        count = size/DSP_LOAD_BLOCK_TRANSFER_SIZE;

        remainder = size - count * DSP_LOAD_BLOCK_TRANSFER_SIZE;

        for(i = 0; i < count; i++) {
            for (j = 0; j < DSP_LOAD_BLOCK_TRANSFER_SIZE/4; j++) {
                tempArray[j] = byteTo32bits(pDspCode);
                pDspCode += 4;
            }
            /* Transfer boot tables to DSP */
            dsp_local_write(dsp_id, coreNum, startaddr, (uint8_t *)tempArray, DSP_LOAD_BLOCK_TRANSFER_SIZE); 
            startaddr += DSP_LOAD_BLOCK_TRANSFER_SIZE;
        }

        for (j = 0; j < remainder/4; j++) {
            tempArray[j] = byteTo32bits(pDspCode);
            pDspCode += 4;
        }
        dsp_local_write(dsp_id, coreNum, startaddr, (uint8_t *)tempArray, remainder); 
    }
#ifdef DNLDMGR_VERBOSE
    printf("Total %d sections, 0x%x bytes of data written to core %d\n", section, totalSize, coreNum);
#endif
}
/**
 *  @brief Function dnldmgr_get_image() Get DSP image from file and parse image
 *  @param[in]     dsp_image_name    Dsp image name with path (any valid image hex or out file)
 *  @param[out]    dsp_image_handle  Returned image handle
 *  @param[out]     dsp_entry_point   Entry point address for Program
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dnldmgr_get_image(char *dsp_image_name,
  void **dsp_image_handle,  uint32_t *dsp_entry_point)
{
  bfd* dsp_bfd;
  char** matching;
  char* ptr;
  uint32_t nsize, nsyms;
  asymbol **symtab;
  symbol_info syminfo;
  int i;

    static char* boot_symbol_name = "_c_int00";
    static char* carg_symbol_name = "__c_args__";

    /* Open image file */
    dsp_bfd = bfd_openr(dsp_image_name, NULL);
  
    if(dsp_bfd == NULL) 
    {
      fprintf(stderr, "\nERROR:dnldmgr_get_image: %s Error Open image %s\n",  
        bfd_errmsg(bfd_get_error()), dsp_image_name); 
      return(-1); 
    }
    /* Check format with matching */	  
    if (!bfd_check_format_matches (dsp_bfd, bfd_object, &matching)) {
      fprintf(stderr, "\nERROR:dnldmgr_get_image %s: %s\n", dsp_image_name, 
        bfd_errmsg(bfd_get_error()));  
      if (bfd_get_error () == bfd_error_file_ambiguously_recognized) {
        for (ptr = *matching; ptr != NULL; ptr++) {
          fprintf(stderr, "%s: \n", ptr); 
          return(-1); 
        }
        free (matching);
      }
    }
    *dsp_entry_point = 0xffffffff;
    // Find boot address and address of mpi_rank.
    nsize = bfd_get_symtab_upper_bound (dsp_bfd);
    if ((symtab = malloc(nsize)) == NULL) {
      return (-1);
    }
    nsyms = bfd_canonicalize_symtab(dsp_bfd, symtab);
   
    for (i = 0; i < nsyms; i++) {
      if (strcmp(symtab[i]->name, boot_symbol_name) == 0) { 
        bfd_symbol_info(symtab[i], &syminfo);
	*dsp_entry_point = syminfo.value;
      } 
    }
  free(symtab);
  *dsp_image_handle = (void *)dsp_bfd;
  return(0);
}
/**
 *  @brief Function dnldmgr_get_symbol_address() Get DSP image from file
 *  @param[in]    dsp_image_handle  image handle
 *  @param[in]    symbol_string     Symbol string to find
 *  @param[out]   symbol_addr       Entry point address for Program
 *  @retval       0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dnldmgr_get_symbol_address(void *dsp_image_handle, char *symbol_string, uint32_t *symbol_addr)
{
  bfd* dsp_bfd;
  uint32_t nsyms, nsize;
  asymbol **symtab;
  symbol_info syminfo;
  int i;
  int32_t ret_val=-1;

  dsp_bfd = (bfd *)dsp_image_handle;

  // Find boot address and address of mpi_rank.
  nsize = bfd_get_symtab_upper_bound (dsp_bfd);
  if ((symtab = malloc(nsize)) == NULL) {
    return (-1);
  }

  nsyms = bfd_canonicalize_symtab(dsp_bfd, symtab);
   
  for (i = 0; i < nsyms; i++) {
    if (strcmp(symtab[i]->name, symbol_string) == 0) { 
      bfd_symbol_info(symtab[i], &syminfo);
      *symbol_addr = syminfo.value;
      ret_val = 0;
      break;
    } 
  }
  free(symtab);
  return(ret_val); 

}
/**
 *  @brief Function dnldmgr_free_image() Free image associated with the handle
 *  @param[in]    dsp_image_handle  Returned image handle
 *  @retval        none
 *  @pre  
 *  @post 
 */
void dnldmgr_free_image(void *dsp_image_handle)
{
  bfd* dsp_bfd=(bfd* )dsp_image_handle;

  bfd_close(dsp_image_handle);
}
static int32_t load_dsp_image(int32_t dsp_id, int32_t core_id,  void *dsp_image_handle)
{
  bfd* dsp_bfd;
  struct bfd_section* section;
  uint32_t target_addr,address;
  bfd_byte* s;
  uint32_t num_cores, core_num;
  int32_t ret=0;
  int i;

  dsp_bfd = (bfd* )dsp_image_handle;
  for (section = dsp_bfd->sections; section != NULL; section = section->next) {
      if (section->reloc_count != 0) {
        fprintf(stderr, 
          "\nERROR:load_dsp_image: currently unable to handle relocations (in %s)", 
	  section->name);
      }
      // Check if we should copy in the section
      if (section->flags & SEC_LOAD) {
        target_addr = section->lma;
#ifdef DNLDMGR_VERBOSE
        fprintf(stderr, "\n%s at 0x%08lx mapped to 0x%08x "
          "sz: %ld flags: 0x%08x", section->name, (unsigned long) section->lma,
          target_addr, (unsigned long)section->size, section->flags); 
#endif		
        if (! (section->flags & SEC_IN_MEMORY)) {
          if (!bfd_malloc_and_get_section (dsp_bfd, section, &s)) {
            fprintf(stderr, "\nERROR:dnldmgr_load_image: %s reading section %s", 
              bfd_errmsg(bfd_get_error()), 
              section->name);
            return(-1);
          }
        }
        num_cores = 1;
        core_num = core_id;
        if (IS_CORE_LOCAL_ADDR(target_addr)) // It points to 'per core' address space.
        {
          if(core_id == 0xffff) 
          {
            num_cores = TOTAL_NUM_CORES_PER_CHIP;
            core_num = 0;
          }
          for(i =  0; i < num_cores; i++)
          {
            address = LOCAL_TO_GLOBAL_ADDR(target_addr,core_num+i);
            ret |= pciedrv_dsp_write(dsp_id, address, (uint8_t *)s, section->size);
          }
        } else
        {
          ret = pciedrv_dsp_write(dsp_id, target_addr, (uint8_t *)s, section->size);     
        }

        if (ret != 0) {
          return(-1);
        }
        free(s);
      } else {
        /* Do nothing for now. */
        //fprintf(stderr, "%s flags %x\n", section->name, section->flags);
      }
    }
    return(0);
}
#define LOOP_CODE_LOCATION_ADDR 0xc000000
#define IDLE_INSTRUCTION_CODE   0x0001E000
int32_t downloadSimpleLoopCode(int32_t dsp_id)
{
  int i;
  int ret = 0;
  uint32_t write_value;

  write_value = IDLE_INSTRUCTION_CODE;
 
  /* Write 8 Idle instructions */
  for(i=0; i< 8; i++)
  {
   ret = pciedrv_dsp_write(dsp_id, (LOOP_CODE_LOCATION_ADDR+i*4), (uint8_t *)(&write_value)  , 4);
    if(ret != 0)
    {
      fprintf(stderr, "\nERROR:downloadSimpleLoopCode: dsp_id %x Failed writing loop code", dsp_id);
      return -1;
    }
  }
  return(ret);
}
/**
 *  @brief Function dnldmgr_reset_dsp() Puts DSP in reset
 *         or brings the dsp out of reset based on flag
 *         The boot image is loaded to DSP, prior to bringing the dsp
 *         out of reset, Loads boot configuration if any
 *         prior to bringing the dsp out of reset and then DSP jumps
 *         to entry point. If no boot image is supplied the precompiled
 *         boot image is used.
 *  @param[in]     dsp_id           DSP Chip Id
 *  @param[in]     flag             flag to indicate whether to  put in reset : 0
 *                                  or bring out of reset : 1
 *  @param[in]     dsp_image_handle  DSP Boot image hanlde, used when bringing out of reset
                                    If set to NULL, precompiled image is used
 *                                  Not used if flag is set to 0
 *  @param[in]     boot_entry_point Boot image entry point address
 *  @param[in]     boot_cfg         Pointer to Boot configuration for DSP; No 
 *                                  No boot configuration if set to NULL
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dnldmgr_reset_dsp(int32_t dsp_id,  int32_t flag, void *dsp_image_handle,
  uint32_t boot_entry_point, boot_cfg_t *boot_cfg)
{
  int ret = 0;
  int i;
  uint32_t bootEntryAddr;

  pciedrv_pcieep_set_config(dsp_id, PCIEDRV_PCIEEP_SET_MASTER_PRIV_SET);

  if(flag == 0)
  {
#ifdef DNLDMGR_VERBOSE
    printf("\n Resetting DSP : %d\n", dsp_id);
#endif
    /* Local reset of all cores */
    coreLocalReset(dsp_id, PD8,  LPSC_C0_TIM0, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD9,  LPSC_C1_TIM1, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD10, LPSC_C2_TIM2, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD11, LPSC_C3_TIM3, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD12, LPSC_C4_TIM4, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD13, LPSC_C5_TIM5, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD14, LPSC_C6_TIM6, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD15, LPSC_C7_TIM7, LOC_RST_ASSERT);

    /* Disable all other modules */
    setPscState(dsp_id, PD0, LPSC_MODRST0, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD0, LPSC_EMIF16_SPI, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD0, LPSC_TSIP, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD1, LPSC_DEBUG, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD1, LPSC_TETB_TRC, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD4, LPSC_SRIO, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD5, LPSC_HYPER, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD7, LPSC_MSMCRAM, PSC_SWRSTDISABLE);
    
    /* Put Netcp components in safe state for reset */
    set_SA_state_to_reset(dsp_id);
    set_PA_state_to_reset(dsp_id);

    setPscState(dsp_id, PD2, LPSC_SA, PSC_SWRSTDISABLE); 
    setPscState(dsp_id, PD2, LPSC_SGMII, PSC_SWRSTDISABLE);
    setPscState(dsp_id, PD2, LPSC_PA, PSC_SWRSTDISABLE);
    /* Turn off power domain for NETCP */
    setPDState(dsp_id, PD2, PSC_PD_OFF);
  }
  else if (flag == 1)
  {
#ifdef DNLDMGR_VERBOSE
    printf("\nLoad default boot code to all cores; dsp_id %d\n", dsp_id);
#endif
    /* Bring MSMCRAM out of reset to allow writing to MSMC */
    setPscState(dsp_id, PD7, LPSC_MSMCRAM, PSC_ENABLE);

    /*-------------------------------------------------------------------------
     * The following code is a work around to flush the cache, without this
     * Any dirty cache lines in L1D may cause corruption of the downloaded
     * image
     */
    ret = downloadSimpleLoopCode(dsp_id);
    if(ret !=0)
    {
      fprintf(stderr, "\nERROR: dnldmgr_reset_dsp: dsp_id %d Failed download loop code",dsp_id);
      return(-1);
    }
    for (i = 0; i < TOTAL_NUM_CORES_PER_CHIP; i++) {
     if (setBootAddrIpcgr(dsp_id, i, LOOP_CODE_LOCATION_ADDR ) == 0) {
        fprintf(stderr, "\nERROR: dnldmgr_reset_dsp: dsp_id %d, Core %d  set boot address failed !!! ", dsp_id, i);
        return(-1);
      }
    }   
    /* Enable required modules */
    setPscState(dsp_id, PD0, LPSC_MODRST0, PSC_ENABLE);

    /* Local out of reset of all cores */
    coreLocalReset(dsp_id, PD8,  LPSC_C0_TIM0, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD9,  LPSC_C1_TIM1, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD10, LPSC_C2_TIM2, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD11, LPSC_C3_TIM3, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD12, LPSC_C4_TIM4, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD13, LPSC_C5_TIM5, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD14, LPSC_C6_TIM6, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD15, LPSC_C7_TIM7, LOC_RST_DEASSERT);

    /* Local reset of all cores */
    coreLocalReset(dsp_id, PD8,  LPSC_C0_TIM0, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD9,  LPSC_C1_TIM1, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD10, LPSC_C2_TIM2, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD11, LPSC_C3_TIM3, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD12, LPSC_C4_TIM4, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD13, LPSC_C5_TIM5, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD14, LPSC_C6_TIM6, LOC_RST_ASSERT);
    coreLocalReset(dsp_id, PD15, LPSC_C7_TIM7, LOC_RST_ASSERT);

    /* Disable the enabled modules for cache flush*/
    setPscState(dsp_id, PD0, LPSC_MODRST0, PSC_SWRSTDISABLE);
    /*-------------------------------------------------------------------------*/

    if(dsp_image_handle ==NULL)
    {
      for (i = 0; i < TOTAL_NUM_CORES_PER_CHIP; i++) {
        pushData(dsp_id, localResetCode, i, &bootEntryAddr);
        if (setBootAddrIpcgr(dsp_id, i, bootEntryAddr) == 0) {
          fprintf(stderr, "\nERROR: dnldmgr_reset_dsp: dsp_id %d Core %d is not ready !!! ", dsp_id, i);
          return(-1);
        }
      }
    } 
    else
    {
      uint32_t num_cores,core_num;
      uint32_t address;

      /* Write image to DSP */
      ret = load_dsp_image(dsp_id, 0xffff, dsp_image_handle);
      if( ret != 0)
      {
          fprintf(stderr, "\nERROR:dnldmgr_reset_dsp: dsp_id %d ,Load image error", dsp_id);
          return(-1);

      }
      if(boot_entry_point ==0)
      {
        fprintf(stderr, "\nERROR:dnldmgr_reset_dsp: dsp_id %d, Entry point invalid", dsp_id);
        return(-1);
      }

      /* Load boot config if required */
      if(boot_cfg != NULL)
      {
        num_cores = TOTAL_NUM_CORES_PER_CHIP;
        core_num = 0;
        if (IS_CORE_LOCAL_ADDR(boot_cfg->config_dsp_addr)) // It points to 'per core' address space.
        {
         for(i=0;i < num_cores; i++)
          {
            address = LOCAL_TO_GLOBAL_ADDR(boot_cfg->config_dsp_addr ,(core_num +i));
            ret |= pciedrv_dsp_write(dsp_id, address, boot_cfg->config_p  , boot_cfg->config_size_b);
          }
        } else
        {
          ret = pciedrv_dsp_write(dsp_id, boot_cfg->config_dsp_addr, boot_cfg->config_p  , boot_cfg->config_size_b);
        }
        if(ret != 0)
        {
          fprintf(stderr, "\nERROR:dnldmgr_reset_dsp: dsp_id %d, Boot config write error", dsp_id);
          return -1;
        }
      }
      for (i = 0; i < TOTAL_NUM_CORES_PER_CHIP; i++) {
        if (setBootAddrIpcgr(dsp_id, i, boot_entry_point) == 0) {
          fprintf(stderr, "\nERROR: dnldmgr_reset_dsp: dsp_id %d Core %d is not ready !!! ", dsp_id, i);
          return(-1);
        }
      }
    }
#ifdef DNLDMGR_VERBOSE
    printf("\n Bringing  DSP out of reset \n");
#endif
    /* Enable all other modules */
    setPscState(dsp_id, PD0, LPSC_MODRST0, PSC_ENABLE);
    setPscState(dsp_id, PD0, LPSC_EMIF16_SPI, PSC_ENABLE);
    setPscState(dsp_id, PD0, LPSC_TSIP, PSC_ENABLE);
    setPscState(dsp_id, PD1, LPSC_DEBUG, PSC_ENABLE);
    setPscState(dsp_id, PD1, LPSC_TETB_TRC, PSC_ENABLE);

    /* Local out of reset of all cores */
    coreLocalReset(dsp_id, PD8,  LPSC_C0_TIM0, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD9,  LPSC_C1_TIM1, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD10, LPSC_C2_TIM2, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD11, LPSC_C3_TIM3, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD12, LPSC_C4_TIM4, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD13, LPSC_C5_TIM5, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD14, LPSC_C6_TIM6, LOC_RST_DEASSERT);
    coreLocalReset(dsp_id, PD15, LPSC_C7_TIM7, LOC_RST_DEASSERT);
  }
  return(0);
}
/**
 *  @brief Function dnldmgr_load_image() Loads image into the DSP 
 *                  writes boot configuration if required,  
 *                  writes Entry point to L2SRAM location 0x87fffc and sends
 *                  interrupt to DSP
 *  @param[in]     dsp_id          DSP Chip Id
 *  @param[in]     core_id         core Id
 *  @param[in]     dsp_image_handle Image handle 
 *  @param[in]     dsp_entry_point Entry point address for Program
 *  @param[in]     boot_cfg        Boot configuration for DSP
 *                                 No boot configuration if set to NULL
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dnldmgr_load_image(int32_t dsp_id, int32_t core_id, void *dsp_image_handle,
  uint32_t dsp_entry_point, boot_cfg_t *boot_cfg)
{
  int ret = 0;
  int i;
  uint32_t num_cores,core_num;
  uint32_t address;

  /* Write image to DSP */
  ret = load_dsp_image(dsp_id, core_id, dsp_image_handle);
  if( ret != 0)
  {
      fprintf(stderr, "\nERROR:dnldmgr_load_image: Load image error");
      return(-1);
    
  }
  if(dsp_entry_point ==0)
  {
    fprintf(stderr, "\nERROR:dnldmgr_load_image: Entry point invalid");
    return(-1);   
  }
  num_cores = 1;
  core_num = core_id;
  /* Load boot config if required */
  if(boot_cfg != NULL)
  {
    if (IS_CORE_LOCAL_ADDR(boot_cfg->config_dsp_addr)) // It points to 'per core' address space.
    {
      if(core_id == 0xffff) 
      {
        num_cores = TOTAL_NUM_CORES_PER_CHIP;
        core_num = 0;
      }
      for(i=0;i < num_cores; i++)
      {
        address = LOCAL_TO_GLOBAL_ADDR(boot_cfg->config_dsp_addr,(core_num +i));
        ret |= pciedrv_dsp_write(dsp_id, address, boot_cfg->config_p  , boot_cfg->config_size_b);
      }
    } else
    {
      ret = pciedrv_dsp_write(dsp_id, boot_cfg->config_dsp_addr, boot_cfg->config_p  , boot_cfg->config_size_b);
    }
    if(ret != 0)
    {
      fprintf(stderr, "\nERROR:dnldmgr_load_image: Boot config write error");
      return -1;
    }
  }

#ifdef DNLDMGR_VERBOSE
  printf("Load HEX OK\n");
#endif
  if(core_id == 0xffff)
  {
    for(i=0;i < TOTAL_NUM_CORES_PER_CHIP; i++)
    {
      ret |= pciedrv_dsp_set_entry_point(dsp_id, i, dsp_entry_point);
    }
  } else
  {
    ret = pciedrv_dsp_set_entry_point(dsp_id, core_id, dsp_entry_point);
  }
  
  if (ret != 0) {
    fprintf(stderr, "\nERROR:dnldmgr_load_image_from_memory: Set entry point failed ");
    return(-1);
  }

  if((core_id == 0) || (core_id == 0xffff))
  {
    pciedrv_pcieep_set_config(dsp_id, PCIEDRV_PCIEEP_SET_INTA_SET);
    usleep(10000);
    pciedrv_pcieep_set_config(dsp_id, PCIEDRV_PCIEEP_SET_INTA_CLR);
  }
   return(0);

}
/*** nothing past this point ***/
