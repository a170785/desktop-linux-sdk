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
  volatile int32_t sw_barr;         /** Marked when a user enters a barrier. */
  volatile int32_t sw_barr_passed;  /** Marked when a user leaves a barrier. */
} syncBarrUser_t;

typedef struct {
  int32_t         num_users;        /** Total number of users. This is also 
                                     *  the number of users required to meet 
                                     *  at a barrier before any may proceed.  */
  syncBarrUser_t  users[1];         /** First element in the array of users.  */
} syncBarr_t;


/**
 *  @brief Get the size requirements of a single barrier.
 *
 *  @param[in]   num_users    Number of users which will be required to enter 
 *                            the barrier before an may proceed past the i
 *                            barrier.
 *
 *  @return      Size of memory in bytes which is required for the barrier.
 */
int32_t syncBarrGetSizes(int32_t num_users)
{
  return ( sizeof(syncBarr_t) + (num_users - 1) * sizeof(syncBarrUser_t) );
}


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
void *syncBarrInit(void *barr, int32_t num_users)
{
  syncBarr_t      *inst   = (syncBarr_t *)barr;
  syncBarrUser_t  *users  = &inst->users[0];

  if ( barr == NULL || num_users <= 0 )
    return ( NULL );

  inst->num_users =  num_users;

  memset( users, 0, num_users * sizeof(syncBarrUser_t) );

  return ( (void *)inst);
}


/**
 *  @brief Function call which will block until all other users have also 
 *         called it.
 *
 *  @param[in]   barr         Barrier handle.
 *
 *  @param[in]   user_id      ID of the user entering the barrier.
 *
 */
int32_t  syncBarrWait(void *barr, int32_t user_id)
{
  syncBarr_t      *inst   = (syncBarr_t *)barr;
  syncBarrUser_t  *users  = &inst->users[0];
  int32_t          ret    = SYNC_BARR_ERROR_NONE;
  int32_t          i, tot;

  if ( inst == NULL )
    ret = SYNC_BARR_ERROR_HANDLE;
  else if ( (user_id < 0) || (user_id >= inst->num_users) )
    ret = SYNC_BARR_ERROR_USER_ID;
  else if ( (users[user_id].sw_barr != 0) || (users[user_id].sw_barr_passed != 0) )
    ret = SYNC_BARR_ERROR_CORRUPTION;
  else
  {
    users[user_id].sw_barr = 1;

    for (;;)
    {
      tot = 0;

      for ( i = 0; i < inst->num_users; i++)
        tot += users[i].sw_barr;

      if ( inst->num_users <= tot )
      {
        users[user_id].sw_barr_passed = 1;

        if ( user_id == 0 )
        {
          for (;;)
          {
            tot = 0;

            for ( i = 0; i < inst->num_users; i++)
              tot += users[i].sw_barr_passed;

            if ( inst->num_users <= tot )
            {
              memset ( users, 0, inst->num_users * sizeof(syncBarrUser_t));
              break;
            }
          }
        }
        break;
      }
    }
  }
  return ( ret );
}

/* nothing past this point */

