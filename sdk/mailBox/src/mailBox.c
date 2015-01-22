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



#include "mailBoxLoc.h"
#include "mailBox.h"
#include <string.h>
#include <stdio.h>

/* Mailbox context */
mailBoxContext_t mailBoxContext;

/**
 *  @brief Function mailBoxPutHeader() Write mailbox header from local buffer
 *  @param[in]     inst             Mailbox instance
 *  @param[in]     mailboxHeader    Mailbox Header pointer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxPutHeader(mailBoxInst_t *inst, mailBoxHeader_t *mailboxHeader)
{
    int32_t dsp_id;
    int32_t ret_val;
    mailBoxHeader_t *mailBox_addr = (mailBoxHeader_t *)(intptr_t)inst->mem_start_addr;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    ret_val = dsp_memory_write(dsp_id, (uint32_t)(intptr_t)mailBox_addr,(void *)mailboxHeader, sizeof(mailBoxHeader_t));

    return(ret_val);
}

/**
 *  @brief Function mailBoxGetHeader() Read mailbox header to local buffer
 *  @param[in]     inst             Mailbox instance
 *  @param[in]     mailboxHeader    Mailbox Header pointer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxGetHeader(mailBoxInst_t *inst, mailBoxHeader_t *mailboxHeader)
{
    int32_t dsp_id;
    int32_t ret_val;
    mailBoxHeader_t *mailBox_addr = (mailBoxHeader_t *)(intptr_t)inst->mem_start_addr;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    ret_val = dsp_memory_read(dsp_id, (uint32_t)(intptr_t)mailBox_addr,(void *)mailboxHeader, sizeof(mailBoxHeader_t));

    return(ret_val);
}
/**
 *  @brief Function mailBoxPutWriteIndex() Write write index into mailbox header
 *  @param[in]     inst             Mailbox instance
 *  @param[in]     writeIndex       Write index pointer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxPutWriteIndex(mailBoxInst_t *inst, uint32_t *writeIndex)
{
    int32_t dsp_id;
    mailBox_t *mailBox_addr = (mailBox_t *)(intptr_t)inst->mem_start_addr;
    int32_t ret_val;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
     /* Write back write index */
    ret_val = dsp_memory_write(dsp_id, (uint32_t)(intptr_t)(&mailBox_addr->mailboxheader.writeIndex), (void *)writeIndex, sizeof(uint32_t));

    return(ret_val);
 }
/**
 *  @brief Function mailBoxGetSlotHeader() Get slot header from mailbox
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     slotIndex          Slot index
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxGetSlotHeader(mailBoxInst_t *inst, uint32_t slotIndex, mailBoxSlotHeader_t *mailboxSlotHeader)
{
    int32_t dsp_id;
    int32_t ret_val;
    mailBox_t *mailBox_addr = (mailBox_t *)(intptr_t)inst->mem_start_addr;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    ret_val = dsp_memory_read(dsp_id, (((uint32_t)(intptr_t)&mailBox_addr->slots[0])
        +(slotIndex*inst->slot_size)),
        (void *)mailboxSlotHeader, sizeof(mailBoxSlotHeader_t));

    return(ret_val);
}

/**
 *  @brief Function mailBoxPutSlotHeader() Put slot header to mailbox
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     slotIndex          Slot index
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxPutSlotHeader(mailBoxInst_t *inst, uint32_t slotIndex,
  mailBoxSlotHeader_t *mailboxSlotHeader)
{
    int32_t dsp_id;
    int32_t ret_val;
    mailBox_t *mailBox_addr = (mailBox_t *)(intptr_t)inst->mem_start_addr;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    ret_val = dsp_memory_write(dsp_id, (uint32_t)(intptr_t)(((uint8_t *)&mailBox_addr->slots[0])
        +(slotIndex*inst->slot_size)),
        (void *)mailboxSlotHeader, sizeof(mailBoxSlotHeader_t));

    return(ret_val);
}
/**
 *  @brief Function mailBoxGetPayload() Get mailbox payload from slot
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     slotIndex          Slot index
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @param[in]     buf                Payload buffer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxGetPayload(mailBoxInst_t *inst, uint32_t slotIndex, mailBoxSlotHeader_t *mailboxSlotHeader,
    uint8_t *buf)
{
    int32_t dsp_id;
    int32_t ret_val=0;
    mailBox_t *mailBox_addr = (mailBox_t *)(intptr_t)inst->mem_start_addr;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    ret_val = dsp_memory_read(dsp_id, (((uint32_t)(intptr_t)&mailBox_addr->slots[0])
        +(slotIndex*inst->slot_size)+sizeof(mailBoxSlotHeader_t)),
        buf, mailboxSlotHeader->payloadSize);

    return(ret_val);
}

/**
 *  @brief Function mailBoxPutPayload() Put mailbox payload into slot
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     slotIndex          Slot index
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @param[in]     buf                Payload buffer
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBoxPutPayload(mailBoxInst_t *inst, uint32_t slotIndex, mailBoxSlotHeader_t *mailboxSlotHeader,
    uint8_t *buf)
{
    int32_t dsp_id;
    int32_t ret_val;
    mailBox_t *mailBox_addr = (mailBox_t *)(intptr_t)inst->mem_start_addr;
    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    ret_val = dsp_memory_write(dsp_id, (((uint32_t)(intptr_t)&mailBox_addr->slots[0])
        +(slotIndex*inst->slot_size)+sizeof(mailBoxSlotHeader_t)),
        buf, mailboxSlotHeader->payloadSize);

    return(ret_val);
}

/**
 *  @brief Function mailBox_allocSlot() Allocate slot for writing mail
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @retval        slot index, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBox_allocSlot(mailBoxInst_t *inst, mailBoxSlotHeader_t *mailBoxSlotHeader)
{
    int32_t    slotIdx;
    uint32_t slot_owner;
    mailBoxHeader_t mailBoxHeader;
    int32_t ret_val;

    slot_owner = MAILBOX_SLOT_OWNER_REMOTE;
#ifdef MAILBOX_VERBOSE
    printf("\t\t\t\t\tmailBox_allocSlot() \n");
#endif
    ret_val = mailBoxGetHeader(inst, &mailBoxHeader);
    if(ret_val)
       return(ret_val);

    slotIdx = mailBoxHeader.writeIndex;
    /* Check if the next mailbox is free */
      ret_val = mailBoxGetSlotHeader(inst, slotIdx, mailBoxSlotHeader);
      if(ret_val)
        return(MAILBOX_ERR_FAIL);
      if(mailBoxSlotHeader->owner == slot_owner)
        return(MAILBOX_ERR_MAIL_BOX_FULL);

#ifdef MAILBOX_VERBOSE
    printf("\t\t\t\t\t\twriteIndex: %2d, readIndex: %2d  (%08X | DSP:%d, CORE:%d) \n",mailBoxHeader.writeIndex, \
        mailBoxHeader.readIndex, ,MAILBOX_ID_2_REMOTE_DSP_ID(mailBox_id),MAILBOX_ID_2_CORE_ID(mailBox_id));
#endif
    return(slotIdx);
}
/**
 *  @brief Function mailBox_freeSlot() Free slot
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     mailboxSlotHeader  previously read Slot header pointer
 *  @param[in]     slotIndex          Slot index
 *  @retval        0 for success, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBox_freeSlot(mailBoxInst_t *inst, mailBoxSlotHeader_t *mailBoxSlotHeaderShadow, int32_t slotIdx)
{
    uint32_t slot_owner;
    mailBoxHeader_t mailBoxHeadershadow;
    mailBoxSlotHeader_t *mailBoxSlotHeaderP;
    int32_t ret_val;
    uint32_t dsp_id;
    mailBox_t *mailBox_addr = (mailBox_t *)(intptr_t)inst->mem_start_addr;

    slot_owner = MAILBOX_SLOT_OWNER_LOCAL;
    mailBoxHeadershadow.readIndex = slotIdx;
    inst->readCounter++;
    mailBoxHeadershadow.readIndex++;
    if(mailBoxHeadershadow.readIndex >= inst->depth)
      mailBoxHeadershadow.readIndex=0;
//    mailBoxSlotHeader->trans_id = 0;
    mailBoxSlotHeaderShadow->owner = slot_owner;

    if(inst->mem_location == MAILBOX_MEMORY_LOCATION_REMOTE)
    {
      dsp_id = MAILBOX_NODE_ID_2_DSP_ID(inst->remote_node_id);
    } else
    {
      /* Indicates local copy */
      dsp_id = 0xffff;
    }
    /* Write back read counter */
    ret_val = dsp_memory_write(dsp_id, (uint32_t)(intptr_t)(&mailBox_addr->mailboxheader.readIndex),
        &mailBoxHeadershadow.readIndex, sizeof(uint32_t));
    if(ret_val)
      return(ret_val);
    mailBoxSlotHeaderP = (mailBoxSlotHeader_t *)(((uint8_t *)(&mailBox_addr->slots[0]))+(slotIdx*inst->slot_size));
    /* Free mailbox slot */
    ret_val = dsp_memory_write(dsp_id,
        (uint32_t)(intptr_t)(&mailBoxSlotHeaderP->owner),
        &slot_owner,sizeof(uint32_t));

    return(ret_val);
}
/**
 *  @brief Function mailBox_recvSlot() Get the mail slot to be read next
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @retval        slot index, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBox_recvSlot(mailBoxInst_t *inst, mailBoxSlotHeader_t *mailBoxSlotHeader)
{
    int32_t slotIdx;
    uint32_t slot_owner;
    mailBoxHeader_t mailBoxHeader;

    slot_owner = MAILBOX_SLOT_OWNER_LOCAL;

#ifdef MAILBOX_VERBOSE
    printf("\t\t\t\t\tmailBox_recvSlot(0x%08x) \n", mailBox_id);
#endif
    if(mailBoxGetHeader(inst, &mailBoxHeader) != 0)
       return(MAILBOX_READ_ERROR);
    slotIdx = mailBoxHeader.readIndex;

    mailBoxGetSlotHeader(inst, slotIdx, mailBoxSlotHeader);
    if(mailBoxSlotHeader->owner == slot_owner)
    {
      return(MAILB0X_ERR_EMPTY);
    }

#ifdef MAILBOX_VERBOSE
    printf("\t\t\t\t\t\twriteCount: %2d, readCount: %2d  (%08X | DSP:%d, CORE:%d) \n",mailbox->writeIndex, \
           mailbox->readIndex, mailBox_id,MAILBOX_ID_2_REMOTE_DSP_ID(mailBox_id),MAILBOX_ID_2_CORE_ID(mailBox_id));
#endif

    return(slotIdx);
}
/**
 *  @brief Function mailBox_sendSlot() Put slot into maibox
 *  @param[in]     inst               Mailbox instance
 *  @param[in]     mailboxSlotHeader  Slot header pointer
 *  @param[in]     buf                Payload buffer
 *  @param[in]     slotIdx            Slot index
 *  @retval        slot index, -1 for failure
 *  @pre
 *  @post
 */
int32_t mailBox_sendSlot(mailBoxInst_t *inst, mailBoxSlotHeader_t *mailBoxSlotHeader,
  uint8_t *buf, int32_t slotIdx)
{
    uint32_t slot_owner;
    mailBoxHeader_t mailBoxHeader;

    slot_owner = MAILBOX_SLOT_OWNER_REMOTE;

    mailBoxHeader.writeIndex= slotIdx;
    inst->writeCounter++;
    mailBoxHeader.writeIndex++;
    if(mailBoxHeader.writeIndex >= inst->depth)
      mailBoxHeader.writeIndex=0;
    mailBoxSlotHeader->owner = slot_owner;
    mailBoxPutPayload(inst, slotIdx, mailBoxSlotHeader, buf);
    mailBoxPutSlotHeader(inst, slotIdx, mailBoxSlotHeader);
    mailBoxPutWriteIndex(inst,  &mailBoxHeader.writeIndex);
//    mailBoxPutSend(inst, mailbox, slotIdx);
    mailBoxNotify(inst);
    return 0;
}

/******************************************************************************/
/*  Public APIs                                                               */
/******************************************************************************/

/**
 *  @brief Function mailBox_get_size() Get size needed for Mailbox instance
 *  @retval        size in bytes needed for mailbox instance
 *  @pre
 *  @post
 */
uint32_t mailBox_get_alloc_size(void)
{
  return(sizeof(mailBoxInst_t));
}


/**
 *  @brief Function mailBox_get_mem_size() Get size needed for Mailbox memory
 *  @param[in]     max_payload_size   Maximum size of a mailBox message.
 *  @param[in]     mailBox_depth      Maximum number of messages that the 
 *                                    mailBox can hold at one time.
 *  @retval        size in bytes needed for mailbox memory
 *  @pre
 *  @post
 */
uint32_t mailBox_get_mem_size(uint32_t max_payload_size, uint32_t mailBox_depth)
{
    uint32_t slot_size, mem_size;

    slot_size = sizeof(mailBoxSlotHeader_t)+ max_payload_size;
    /* Adjust for alignment */
    slot_size = ((slot_size + (sizeof(uint32_t)-1))/sizeof(uint32_t)) * sizeof(uint32_t);

    mem_size = (slot_size * mailBox_depth) + sizeof(mailBoxHeader_t);

    return ( mem_size );
}

/**
 *  @brief Function mailBox_create() Creates a mailBox
 *  @param[out]    mailBoxHandle  Returned mailbox handle pointer
 *  @param[in]     remote_node_id    Node id of remote node
 *  @param[in]     mem_location   memory location local or remote
 *  @param[in]     direction      send or receive
 *  @param[in]     mailBox_config MailBox configuration
 *  @retval        0 for success, -1 for failure
 *  @pre   mailBox init should be executed before callint mailBox_create
 *  @post 
 */
int32_t mailBox_create(void *mailBoxHandle, int32_t remote_node_id,
  uint32_t mem_location, uint32_t direction, mailBox_config_t *mailBox_config)
{
    int32_t  i;
    uint32_t slot_owner, max_slots;
    mailBoxInst_t *mailBoxInst = (mailBoxInst_t *)mailBoxHandle;
    uint32_t slot_size;
    mailBoxHeader_t mailboxheader;
    mailBoxSlotHeader_t mailboxslotheader;
    int32_t ret_val;

   if(mailBoxInst == NULL)
     return(-1);

   memset(mailBoxHandle, 0, sizeof(mailBoxInst_t));
   /* Check sizes of structure */
    slot_size = sizeof(mailBoxSlotHeader_t)+ mailBox_config->max_payload_size;
    /* Adjust for alignment */
    slot_size = ((slot_size + (sizeof(uint32_t)-1))/sizeof(uint32_t)) * sizeof(uint32_t);
    mailBoxInst->depth = (mailBox_config->mem_size - sizeof(mailBoxHeader_t))/slot_size;
    mailBoxInst->mem_size = mailBox_config->mem_size;
    /* Store variables to mailBox inst */
    mailBoxInst->slot_size = slot_size;
    mailBoxInst->mem_start_addr = mailBox_config->mem_start_addr;
    mailBoxInst->max_payload_size  = mailBox_config->max_payload_size;
    
    mailBoxInst->remote_node_id = remote_node_id;
    mailBoxInst->mem_location = mem_location;
    mailBoxInst->direction = direction;

#ifdef MAILBOX_VERBOSE
    printf("mailBox_create(): remote_node = 0x%08X\n",remote_node);
#endif
    /* zero headers */
    memset(&mailboxheader, 0, sizeof(mailBoxHeader_t) );
    memset(&mailboxslotheader, 0, sizeof(mailBoxSlotHeader_t) );

    if (direction == MAILBOX_DIRECTION_RECEIVE) {

      slot_owner = MAILBOX_SLOT_OWNER_LOCAL;

      max_slots = mailBoxInst->depth;

      for ( i=0; i<max_slots; i++) {
        mailboxslotheader.owner = slot_owner;
        ret_val = mailBoxPutSlotHeader(mailBoxInst, i, &mailboxslotheader);
        if(ret_val)
          return(ret_val);
      }
      mailboxheader.owner_code = slot_owner;
      ret_val = mailBoxPutHeader(mailBoxInst, &mailboxheader);
      if(ret_val)
        return(ret_val);
    }
    return 0;
}

/**
 *  @brief Function mailBox_open() Opens a mailBox; This is a blocking call, wait till the remote is ready
 *  @param[in]     mailBoxHandle  mailBox Handle
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t mailBox_open(void *mailBoxHandle)
{
    mailBoxHeader_t mailboxHeader;
    
    if ( mailBoxHandle == NULL)
        return -1; 

    do {
        if(mailBoxGetHeader((mailBoxInst_t *)mailBoxHandle, &mailboxHeader) != 0)
          return(-1);
    } while(mailboxHeader.owner_code != MAILBOX_SLOT_OWNER_LOCAL);
    return (0);
}

/**
 *  @brief Function mailBox_write() Writes into a mailBox to deliver to remote
 *  @param[in]     mailBoxHandle  mailBox Handle
 *  @param[in]     *buf          Mailbox Payload buffer pointer
 *  @param[in]     size          Mailbox Payload buffer size
 *  @param[in]     trans_id      transaction ID for the mail
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t mailBox_write (void *mailBoxHandle, uint8_t *buf, uint32_t size, uint32_t trans_id)
{
    int32_t slotIdx;
    mailBoxInst_t *mailBoxInst = (mailBoxInst_t *)mailBoxHandle;
    mailBoxSlotHeader_t mailBoxSlotHeader;
    
    if(size > mailBoxInst->max_payload_size)
        return -1;

    slotIdx = mailBox_allocSlot(mailBoxInst, &mailBoxSlotHeader);
    if(slotIdx < 0)
      return(slotIdx);

    mailBoxSlotHeader.payloadSize = size;
    mailBoxSlotHeader.trans_id = trans_id;

    mailBox_sendSlot(mailBoxHandle, &mailBoxSlotHeader, buf, slotIdx);

    return 0;
}

/**
 *  @brief Function mailBox_read() Reads from a mailBox. This is a blocking call
 *  @param[in]     mailBox_id    Unique ID of the mailBox
 *  @param[in]     *buf          Mailbox Payload buffer pointer
 *  @param[in]     *size         Mailbox Payload buffer size
 *  @param[in]     *trans_id     transaction ID for the mail
 *  @retval        0 for success, -1 for failure
 *  @pre  
 *  @post 
 */
int32_t mailBox_read (void *mailBoxHandle, uint8_t *buf, uint32_t *size, uint32_t *trans_id)
{
    int32_t slotIdx;
    mailBoxSlotHeader_t mailBoxSlotHeader;
    mailBoxInst_t *mailBoxInst= (mailBoxInst_t *)mailBoxHandle;
    int32_t ret_val;

    slotIdx = mailBox_recvSlot(mailBoxInst, &mailBoxSlotHeader); /* blocking call */

    if(slotIdx < 0)
    {
      return(slotIdx);
    }

    /* Get payload from slot */
    mailBoxGetPayload(mailBoxInst, slotIdx, &mailBoxSlotHeader, buf);
    *size = mailBoxSlotHeader.payloadSize;
    *trans_id = mailBoxSlotHeader.trans_id;
    /* Free the slot read */
    ret_val = mailBox_freeSlot(mailBoxHandle,&mailBoxSlotHeader,slotIdx);
    if(ret_val)
      return(MAILBOX_ERR_FAIL);

    return 0;
}


/**
 *  @brief Function mailBox_query() Polls mailBoxes for any available messages to read. Non-blocking
 *  @param[in]     mailBoxHandle  mailBox Handle
 *  @retval        Number of messages in mailbox; negative error on failure
 *  @pre  
 *  @post 
 */
int32_t mailBox_query (void *mailBoxHandle)
{

    uint32_t    mailBoxCnt;
    mailBoxInst_t *mailBoxInst = (mailBoxInst_t *)mailBoxHandle;
    mailBoxHeader_t mailBoxHeader;
    uint32_t read_index, write_index;
    mailBoxSlotHeader_t mailboxSlotHeader;
    int32_t ret_val;

    if(mailBoxGetHeader(mailBoxInst, &mailBoxHeader)!= 0)
      return(-1);

    read_index = mailBoxHeader.readIndex;
    write_index = mailBoxHeader.writeIndex;
    if(read_index > write_index)
    {
      mailBoxCnt = (write_index-read_index)+mailBoxInst->depth;
    } else
    {
      mailBoxCnt = (write_index - read_index);
    }

    /* If this actually is the mailbox full case */
    if(!mailBoxCnt)
    {
      uint32_t prev_read_slot_idx;

      prev_read_slot_idx = (mailBoxHeader.readIndex == 0) ?  (mailBoxInst->depth - 1): (mailBoxHeader.readIndex - 1);

      /* Check the slot at read index */
      ret_val = mailBoxGetSlotHeader(mailBoxInst, prev_read_slot_idx, &mailboxSlotHeader);
      if(ret_val)
        return(MAILBOX_ERR_FAIL);

      if(mailboxSlotHeader.owner==MAILBOX_SLOT_OWNER_REMOTE)
        mailBoxCnt = mailBoxInst->depth;
    }

    return mailBoxCnt;
}

/* nothing past this point */
