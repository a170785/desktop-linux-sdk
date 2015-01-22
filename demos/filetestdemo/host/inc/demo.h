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
#ifndef _DEMO_H
#define _DEMO_H

//#define DEMO_VERBOSE 

/** Total number of cores per card **/

#define DEMO_FILETEST_MAX_NUM_CHIPS        16
#define DEMO_FILETEST_MAX_NUM_CORES        (DEMO_FILETEST_MAX_NUM_CHIPS*DEMO_CONFIG_CORES_PER_CHIP)

#define DEMO_FILETEST_MAX_PAYLOAD_BUFFER_SIZE  0x800000 

#define MAX_NUM_HOST_DSP_BUFFERS (64)

#define DSP_OB_MEM_REGION_SIZE_RESERVED     0

#define MAX_CONTIGUOUS_XFER_BUFFERS 2

#define MAX_DMA_TRANSACTIONS  256

/** Base address of output buffer pool (per chip) **/
#define DEMO_FILETEST_OUTPUT_POOL_BASEADDR 0x90000000
/** Base address of input buffer pool (per chip) **/
#define DEMO_FILETEST_INPUT_POOL_BASEADDR 0x98000000

#define DEMO_FILETEST_POOL_SIZE    0x8000000 

/* This parameter is used to configure the CMEM host pools for use with
   mapping to DSP Memory range; should be 1MB, 2MB, 4MB or 8MB */
#define DSP_OB_REGION_SIZE  0x400000
#define HOST_CMEM_BUFFER_SIZE DSP_OB_REGION_SIZE


#define DSP_OB_MEM_ALLOC_SIZE ((32*DSP_OB_REGION_SIZE)-DSP_OB_MEM_REGION_SIZE_RESERVED)

#define DEMO_HOST_HUGE_BUFFER_SIZE (HOST_CMEM_BUFFER_SIZE*32)


/* Test modes */
#define DEMO_MODE_MEMCPY_TEST   0
#define DEMO_MODE_DSPMAP_TEST   1
#define DEMO_MODE_DMA_TEST      2
#define DEMO_MODE_HUGE_BUF_TEST 3
#define DEMO_MODE_DSP_XFER_TEST 4


#endif
