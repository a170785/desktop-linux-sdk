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

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "sync.h"



typedef struct {
  volatile int32_t entering;
  volatile int32_t number;
} syncLockUser_t;

typedef struct {
  int32_t          num_users;
  syncLockUser_t   users[1];
} syncLock_t;


/**
 *  @brief Get the size requirements of a single lock.
 *
 *  @param[in]   num_users    Maximum number of users which will attempt to 
 *                            obtain the lock.
 *
 *  @return      Size of memory in bytes which is required for the lock.
 */
int32_t syncLockGetSizes(int32_t num_users)
{
  return ( sizeof(syncLock_t) +  (num_users - 1) * sizeof(syncLockUser_t));
}


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
void *syncLockInit(void *lock, int32_t num_users)
{
  syncLock_t     *inst   = (syncLock_t *)lock;
  syncLockUser_t *users  = &inst->users[0];

  if ( lock == NULL || num_users < 0 )
    return (NULL);

  inst->num_users = num_users;

  memset( users, 0, num_users*sizeof(syncLockUser_t));

  return ((void *)inst);
}


/**
 *  @brief Check if any of the users currently have acquired the lock.
 *
 *  @param[in]   lock          Lock handle.
 *
 *  @return      0 if the lock is available, non-zero if the lock is in use by 
 *               a user.
 */
int32_t syncLockCheck(void *lock)
{
  syncLock_t     *inst   = (syncLock_t *)lock;
  syncLockUser_t *users  = &inst->users[0];
  int32_t         i, ret = SYNC_LOCK_ERROR_NONE;

  if ( inst == NULL )
    ret = SYNC_LOCK_ERROR_HANDLE;
  else
  {
    for ( i = 0; i < inst->num_users; i++)
    {
      if ( users[i].entering == 1 || users[i].number > 0 )
      {
        ret = SYNC_LOCK_IN_USE;
        return ( ret );
      }
    }
  }

  return ( ret );
}


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
/* Based on the Lamport's Bakery algorithm */
int32_t syncLockAcquire(void *lock, int32_t user_id)
{
  syncLock_t     *inst   = (syncLock_t *)lock;
  syncLockUser_t *users  = &inst->users[0];
  int32_t         i, ret = SYNC_LOCK_ERROR_NONE;

  if ( inst == NULL )
    ret = SYNC_LOCK_ERROR_HANDLE;
  else if ( ( user_id < 0) || (user_id >= inst->num_users) )
    ret = SYNC_LOCK_ERROR_USER_ID;
  else if ( ( users[user_id].number > 0 ) || (users[user_id].entering == 1))
    ret = SYNC_LOCK_ERROR_DBL_LOCK;
  else 
  {
    /* Take a number */
    users[user_id].entering = 1;

    /* number[i] = 1 + max(number[1:N]) */
    users[user_id].number = 0;
    for ( i = 0; i < inst->num_users; i++)
    {
      if ( users[user_id].number < users[i].number )
      {
        users[user_id].number = users[i].number;
      }
    }
    users[user_id].number++;

    users[user_id].entering = 0;

    /* Wait for the number to be "called"... (No one to call you, so go around to everyone to see if its your turn.) */
    for ( i = 0; i < inst->num_users; i++ )
    {
      /* Wait for the other guy to get a number */
      while ( users[i].entering);

      /* Wait for the other guy to go if their turn is before you */
      while ( ( users[i].number != 0) &&                           // This guy is waiting...
              ( ( users[i].number < users[user_id].number ) ||     // They have a better number...
                ( ( users[i].number == users[user_id].number ) &&  // They have the same number...
                  ( i < user_id )                                  // They are more important than you.
                )
              )
            );
    }
  }
  return ( ret );
}


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
int32_t syncLockRelease(void *lock, int32_t user_id)
{
  syncLock_t      *inst   = (syncLock_t *)lock;
  syncLockUser_t  *users  = &inst->users[0];
  int32_t          ret    = SYNC_LOCK_ERROR_NONE;

  if ( inst == NULL )
    ret = SYNC_LOCK_ERROR_HANDLE;
  else if ( ( user_id < 0) || (user_id >= inst->num_users) )
    ret = SYNC_LOCK_ERROR_USER_ID;
  else if ( users[user_id].number == 0 )
    ret = SYNC_LOCK_ERROR_DBL_UNLOCK;
  else
    users[user_id].number = 0;

  return ( ret );
}

/* nothing past this point */

