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
#ifndef _BUF_MGR_H
#define _BUF_MGR_H
#include <stdint.h>
#include "inc/buffdesc.h"


/**
 * @brief   See ../iface/buffdesc.h for definition of buffer descriptor
 */
typedef cmem_host_buf_desc_t bufmgrDesc_t;


/**
 *  @brief  Return codes: Enumeration of values that can be returned by the bufmgr APIs.
 */
enum {
  BUF_MGR_ERROR_NONE    =  0,  /** Function call succeeded without error.     */
  BUF_MGR_ERROR_CONFIG  = -1,  /** Error in the buffer pool configuration.    */
  BUF_MGR_ERROR_MALLOC  = -2,  /** Error allocating memory for buffer pool    */
  BUF_MGR_ERROR_FREE    = -3,  /** Could not find buffer in pool to be freed. */
  BUF_MGR_ERROR_ALLOC   = -4   /** No free buffers to allocate.               */
};

/**
 *  @brief Function bufmgrCreate() Create buffer pool from the array of descriptors.
 *
 *  @param[in,out]  bufmgrHandle    Location to store handle to the newly created buffer pool.
 *  @param[in]      numDescs        Number of buffer descriptors to add to the buffer pool.
 *  @param[in]      descs           Array of buffer descriptors to add to pool. 
 *                                  Must be atleast numDescs in length.
 *
 *  @retval         See return codes.
 */
int32_t  bufmgrCreate(void **bufmgrHandle, int32_t numDescs, bufmgrDesc_t *descs);

/**
 *  @brief  Function bufmgrCreate2() Create a buffer pool from a large region of memory.
 *
 *  @param[in,out]  bufmgrHandle    Location to store handle to the newly created buffer pool.
 *  @param[in]      poolAddr        Base address of memory region used to create the buffer pool.
 *  @param[in]      poolSize        Size of the memory region in bytes used to create the buffer pool.
 *  @param[in]      bufSize         Size of the buffer elements within the pool.
 *  @param[in]      align           Alignment of the buffers in the pool.
 *
 *  @retval         See return codes.
 */
int32_t  bufmgrCreate2(void **bufmgrHandle, void *poolAddr, int32_t poolSize, int32_t bufSize, int32_t align);

/**
 *  @brief  Function bufmgrDelete()  Free all resources associated with the buffer pool.
 *
 *  @param[in,out]  bufmgrHandle    Pointer to the bufmgrHandle. Set to NULL after resources are freed. 
 */
void     bufmgrDelete(void **bufmgrHandle);

/**
 *  @brief  Function bufmgrAlloc()  Allocate buffer from pool.
 *
 *  @param[in]      bufmgrHandle    Handle to buffer pool.
 *  @param[in]      numUsers        Number of users for the allocated buffer. Effectively, the number of times 
 *                                  bufmgrFree*() must be called until the buffer can be allocated again.
 *  @param[out]     desc            Upon success, this memory will be populated by the allocated buffer descriptor.
 *
 *  @retval         See return codes.
 */
int32_t  bufmgrAlloc(void *bufmgrHandle, int32_t numUsers, bufmgrDesc_t *desc);

/**
 *  @brief  Function bufmgrFreeDesc()  Free buffer element by providing the descriptor.
 *
 *  @param[in]      bufmgrHandle    Handle to buffer pool.
 *  @param[in]      desc            Buffer descriptor to be freed.
 *
 *  @retval         See return codes.
 */
int32_t  bufmgrFreeDesc(void *bufmgrHandle, bufmgrDesc_t *desc);

/**
 *  @brief  Function bufmgrFreeUserAddr  Free buffer element by providing the userAddr field of the descriptor.
 *
 *  @param[in]      bufmgrHandle    Handle to buffer pool.
 *  @param[in]      addr            User address of the buffer to be freed.
 *
 *  @retval         See return codes.
 */
int32_t  bufmgrFreeUserAddr(void *bufmgrHandle, void *addr);


#endif /* _BUFF_MGR_H */
/* nothing past this point */

