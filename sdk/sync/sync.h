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

#ifndef _SYNC_H
#define _SYNC_H
#include <stdint.h>

enum {
  SYNC_LOCK_IN_USE            =  1,  /** Sync lock in use                     */
  SYNC_LOCK_ERROR_NONE        =  0,  /** Function was a success.              */

  SYNC_LOCK_ERROR_HANDLE      = -1,  /** Invalid handle was passed to the 
                                      *  funciton.                            */

  SYNC_LOCK_ERROR_USER_ID     = -2,  /** Invalid user ID was passed to the 
                                      *  function.                            */

  SYNC_LOCK_ERROR_DBL_LOCK    = -3,  /** Lock is attempted to be acquired by 
                                      *  a user ID that already holds the 
                                      *  lock.                                */

  SYNC_LOCK_ERROR_DBL_UNLOCK  = -4   /** User ID attempted to release a lock 
                                      *  when it did not hold the lock.       */
};

/**
 *  @brief Get the size requirements of a single lock.
 *
 *  @param[in]   num_users    Maximum number of users which will attempt to 
 *                            obtain the lock.
 *
 *  @return      Size of memory in bytes which is required for the lock.
 */
int32_t syncLockGetSizes(int32_t num_users);

/**
 *  @brief Initializes the memory used for a lock.
 *
 *  @param[in]   lock         Pointer to the memory to be used for the lock.
 *
 *  @param[in]   num_users    Maximum number of users which will attempt to 
 *                            obtain the lock.
 *
 *  @return      Handle to the lock.
 */
void *syncLockInit(void *lock, int32_t num_users);

/**
 *  @brief Check if any of the users currently have acquired the lock.
 *
 *  @param[in]   lock          Lock handle.
 *
 *  @return      0 if the lock is available, non-zero if the lock is in use by 
 *               a user.
 */
int32_t syncLockCheck(void *lock);

/**
 *  @brief Acquires the lock. If another user has the lock, this function will 
 *         block until the lock has been released.
 *
 *  @param[in]   lock         Lock handle.
 *
 *  @param[in]   user_id      ID of the user attempting to obtain the lock.
 *
 *  @return      0 if the lock has been obtained. Non-zero if an error was 
 *               encountered.
 */
int32_t syncLockAcquire(void *lock, int32_t user_id);


/**
 *  @brief Release the lock so that another user may acquire it.
 *
 *  @param[in]   lock         Lock handle.
 *
 *  @param[in]   user_id      ID of the user releasing the lock.
 *
 *  @return      0 if the lock was released successfully, non-zero if an error 
 *               was encountered.
 */
int32_t syncLockRelease(void *lock, int32_t user_id);



enum {
  SYNC_BARR_ERROR_NONE        =  0,  /** Function call was a success.         */

  SYNC_BARR_ERROR_HANDLE      = -1,  /** Invalid handle passed to the 
                                      *  funciton.                            */

  SYNC_BARR_ERROR_USER_ID     = -2,  /** Invalid user ID passed to the 
                                      *  funciton.                            */

  SYNC_BARR_ERROR_CORRUPTION  = -3   /** Instance data shows that the user ID
                                      *  is already waiting at the barrier 
                                      *  even though the function had just 
                                      *  been entered. This can commonly occur 
                                      *  when the same barrier is used 
                                      *  consecutively.                       */
};


/**
 *  @brief Get the size requirements of a single barrier.
 *
 *  @param[in]   num_users    Number of users which will be required to enter 
 *                            the barrier before an may proceed past the i
 *                            barrier.
 *
 *  @return      Size of memory in bytes which is required for the barrier.
 */
int32_t syncBarrGetSizes(int32_t num_users);


/**
 *  @brief Initializes the memory used for a barrier.
 *
 *  @param[in]   lock         Pointer to the memory to be used for the barrier.
 *
 *  @param[in]   num_users    Number of users which will be required to enter 
 *                            the barrier before an may proceed past the i
 *                            barrier.
 *
 *  @return      Handle to the barrier.
 */
void *syncBarrInit(void *barr, int32_t num_users);

/**
 *  @brief Function call which will block until all other users have also 
 *         called it. NOTE: Limitation: This cannot be called consecutively with the same
 *         barrier instance due to race conditions that can cause corruption.
 *
 *  @param[in]   barr         Barrier handle.
 *
 *  @param[in]   user_id      ID of the user entering the barrier.
 *
 *  @return      0 if barrier is reached by all users, or error code
 */
int32_t syncBarrWait(void *barr, int32_t user_id);


#endif /* _SYNC_H */
/* nothing past this point */

