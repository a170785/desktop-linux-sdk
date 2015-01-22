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
#ifndef _FILETESTDEMO_H
#define _FILETESTDEMO_H
/* DMA resources to be reserved for Host usage */
/* These are to be treated as Reserved on the DSP side */
#define START_DMA_NUM 2
#define START_PARAM_SET_NUM 0
#define NUM_DMA_CHANNELS_FOR_HOST_DSP_XFERS 4
#define NUM_PARAM_SETS 32 
/** Number of cores per chip **/
#define DEMO_CONFIG_CORES_PER_CHIP   8

/* Mailbox related definitions */
#define DEMO_MAILBOX_MAX_PAYLOAD_SIZE         116
#define DEMO_MAILBOX_DSP_TO_HOST_DEPTH   4
#define DEMO_MAILBOX_HOST_TO_DSP_DEPTH   4

#define DEMO_PER_MAILBOX_MEM_SIZE 0x800

typedef struct _demoTestMsg_t{
  uint32_t input_addr;
  uint32_t input_size;
  uint32_t output_addr;
  uint32_t output_size;
} demoTestMsg_t;

typedef struct _filetest_boot_config_t {
  uint32_t magic_number;
  uint32_t dsp_number;
  uint32_t dummy2;
  uint32_t dummy3;
} filetest_boot_config_t;


#endif
