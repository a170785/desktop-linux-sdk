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
#include <stdint.h>

/* Dsp Manager APIs */

typedef struct _boot_cfg_t {
    uint32_t config_dsp_addr;        /* DSP address where the boot configuration
                                       will be copied */
    uint32_t config_size_b;          /* Length of configuration structure in  bytes */
    void *config_p;                  /* Pointer to config structure */
    
} boot_cfg_t;

#define FLAG_DSP_RESET  0
#define FLAG_DSP_OUTOF_RESET 1

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
  uint32_t boot_entry_point, boot_cfg_t *boot_cfg);

/**
 *  @brief Function dnldmgr_get_image() Get DSP image from file and parse image 
 *         and store information for later loading
 *  @param[in]     dsp_image_name    Dsp image name with path (any valid image hex or out file)
 *  @param[out]    dsp_image_handle  Returned image handle
 *  @param[out]     dsp_entry_point   Entry point address for Program
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dnldmgr_get_image(char *dsp_image_name,
  void **dsp_image_handle,  uint32_t *dsp_entry_point);

/**
 *  @brief Function dnldmgr_free_image() Free image associated with the handle
 *  @param[out]    dsp_image_handle  DSP Image handle
 *  @retval        none
 *  @pre  
 *  @post 
 */
void dnldmgr_free_image(void *dsp_image_handle);

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
  uint32_t dsp_entry_point, boot_cfg_t *boot_cfg);

/**
 *  @brief Function dnldmgr_get_symbol_address() Get DSP image from file
 *  @param[in]    dsp_image_handle  image handle
 *  @param[in]    symbol_string     Symbol string to find
 *  @param[out]   symbol_addr       Symbol address found in the image
 *  @retval       0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t dnldmgr_get_symbol_address(void *dsp_image_handle,  char *symbol_string, uint32_t *symbol_addr);

