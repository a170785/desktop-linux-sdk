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

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
//#include <stdarg.h>
#include <semaphore.h>
#include "bufmgr.h"


typedef struct {
  int32_t        depth;
  bufmgrDesc_t  *buf_array;
  uint8_t       *buf_usage;
  sem_t          buf_sem;
} bufmgr_t;


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
int32_t bufmgrCreate(void **bufmgrHandle, int32_t numBufs, bufmgrDesc_t *descs) 
{
  bufmgr_t     *bufmgr;

  bufmgr = (bufmgr_t *)malloc(sizeof(bufmgr_t));
  if ( bufmgr == NULL )
  {
//    bufmgr_log(bufmgrLogLevel,BUFMGR_LOG_ERROR,"bufmgrCreatePool() : Could not allocate memory for buffer pool. \n");
    return(BUF_MGR_ERROR_MALLOC);
  }

  bufmgr->depth      = numBufs;

  bufmgr->buf_array  = malloc(numBufs * sizeof(bufmgrDesc_t));
  if ( bufmgr->buf_array == NULL ) 
  {
    return ( BUF_MGR_ERROR_MALLOC );
  }

  memcpy( bufmgr->buf_array, descs, (numBufs * sizeof(bufmgrDesc_t)) );

  bufmgr->buf_usage = (uint8_t *)malloc(numBufs * sizeof(uint8_t));
  if ( bufmgr->buf_usage == NULL )
  {
    free(bufmgr->buf_array);
    free(bufmgr);
    return ( BUF_MGR_ERROR_MALLOC );
  }
  memset( bufmgr->buf_usage, 0, numBufs * sizeof(uint8_t));

  sem_init(&bufmgr->buf_sem, 0, 1);

  *bufmgrHandle = (void *)bufmgr;

  return ( BUF_MGR_ERROR_NONE );
}


void *bufmgrAlign(void *addr, int32_t align)
{
  return ( addr );
}

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

int32_t bufmgrCreate2(void **bufmgrHandle, void *poolAddr, int32_t poolSize, int32_t bufSize, int32_t align)
{
  bufmgrDesc_t *descs;
  int32_t num_bufs, max_num_bufs, ret;
  void *addr;

  max_num_bufs = poolSize / bufSize;

  descs = (bufmgrDesc_t *)malloc(max_num_bufs * sizeof(bufmgrDesc_t));

  num_bufs = 0;
  addr = bufmgrAlign( poolAddr, align);
  while ( (addr + bufSize) <= (poolAddr + poolSize) )
  {
    descs[num_bufs].physAddr = 0;
    descs[num_bufs].userAddr = addr;
    descs[num_bufs].length   = bufSize;

//    printf("Buffer %d : 0x%08X, length = %d \n",num_bufs,(uint32_t)addr, bufSize);

    addr += bufSize;
    addr  = bufmgrAlign( addr, align);

    num_bufs++;
  }

  ret = bufmgrCreate( bufmgrHandle, num_bufs, descs);

  free(descs);

  return(ret);
}

/**
 *  @brief  Function bufmgrDelete()  Free all resources associated with the buffer pool.
 *
 *  @param[in,out]  bufmgrHandle    Pointer to the bufmgrHandle. Set to NULL after resources are freed. 
 */
void bufmgrDelete(void **bufmgrHandle)
{
  bufmgr_t *bufmgr = (bufmgr_t *)(*bufmgrHandle);

  sem_destroy(&bufmgr->buf_sem);  
  free(bufmgr->buf_usage);
  free(bufmgr->buf_array);
  free(bufmgr);

  *bufmgrHandle = NULL;
}


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

int32_t bufmgrAlloc(void *bufmgrHandle,  int32_t numUsers, bufmgrDesc_t *desc)
{
  bufmgr_t *bufmgr = (bufmgr_t *)bufmgrHandle;
  int32_t i, ret = BUF_MGR_ERROR_ALLOC;
//  uint32_t buf_id = (uint32_t)(-1);

  for ( i = 0; i < bufmgr->depth; i++)
  {
    sem_wait(&bufmgr->buf_sem);
    if(bufmgr->buf_usage[i] == 0)
    {
      bufmgr->buf_usage[i] = numUsers;
//      buf_id = i;
      memcpy( desc, &bufmgr->buf_array[i], sizeof(bufmgrDesc_t));
      sem_post(&bufmgr->buf_sem);
      ret = BUF_MGR_ERROR_NONE;
      break;
    }
     sem_post(&bufmgr->buf_sem);
  }
//  bufmgr_log(bufmgrLogLevel,BUFMGR_LOG_DEBUG,
//    "bufmgrAlloc(%08X) : ID = %X, Addr = 0x%08X \n",
//    mempool, (int32_t)buf_id, mempool->desc[buf_id].buf);

  return (ret);
}


/**
 *  @brief  Function bufmgrFreeUserAddr  Free buffer element by providing the userAddr field of the descriptor.
 *
 *  @param[in]      bufmgrHandle    Handle to buffer pool.
 *  @param[in]      addr            User address of the buffer to be freed.
 *
 *  @retval         See return codes.
 */


int32_t bufmgrFreeUserAddr(void *bufmgrHandle, void *addr)
{
  bufmgr_t *bufmgr = (bufmgr_t *)bufmgrHandle;
  bufmgrDesc_t *desc;
  int32_t  i, ret = BUF_MGR_ERROR_FREE;

  for ( i = 0; i < bufmgr->depth; i++)
  {
    desc = &bufmgr->buf_array[i];
    if ( ((uint8_t *)addr >= desc->userAddr) && ((uint8_t *)addr < (desc->userAddr + desc->length)) )
    {
      sem_wait(&bufmgr->buf_sem);
      bufmgr->buf_usage[i]--;
      sem_post(&bufmgr->buf_sem);

      ret = BUF_MGR_ERROR_NONE;

      break;
    }
  }

//  bufmgr_log(bufmgrLogLevel,BUFMGR_LOG_DEBUG,"bufmgrFree() : %X, ret = %d \n",buf_id,ret);

  return (ret);
}


/**
 *  @brief  Function bufmgrFreeDesc()  Free buffer element by providing the descriptor.
 *
 *  @param[in]      bufmgrHandle    Handle to buffer pool.
 *  @param[in]      desc            Buffer descriptor to be freed.
 *
 *  @retval         See return codes.
 */

int32_t bufmgrFreeDesc(void *bufmgrHandle, bufmgrDesc_t *desc)
{
  bufmgr_t *bufmgr = (bufmgr_t *)bufmgrHandle;
  int32_t  i, ret = BUF_MGR_ERROR_FREE;

#if 0
  for ( i = 0; i < bufmgr->depth; i++)
  {
    if ( memcmp( desc, &bufmgr->buf_array[i], sizeof(bufmgrDesc_t)) == 0 ) 
    {
      sem_wait(&bufmgr->buf_sem);
      bufmgr->buf_usage[i]--;
      sem_post(&bufmgr->buf_sem);

      ret = BUF_MGR_ERROR_NONE;

      break;
    }
  }
#else
  ret = bufmgrFreeUserAddr(bufmgrHandle, desc->userAddr);
#endif

//  bufmgr_log(bufmgrLogLevel,BUFMGR_LOG_DEBUG,"bufmgrFree() : %X, ret = %d \n",buf_id,ret);

  return (ret);
}

/* nothing past this point */

