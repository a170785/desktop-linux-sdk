/******************************************************************************
 * Copyright (c) 2012 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/

/* Memory Map 1 - the default */

-heap  0x1000
-stack 0x1000

MEMORY
{

    L2SRAM_BOOTCFG    (RWX) : org = 0x800000, len = 0x100
    L2SRAM    (RWX) : org = 0x0800100, len = 0x7FF00
    MSMCSRAM1 (RWX) : org = 0xc000000, len = 0x100
    MSMCSRAM (RWX) : org = 0xc000100, len = 0x3fff00

    DDR3                         : org = 0x80000000, len = 0xF000000
    DDR3_MAILBOX_RESERVED (RW)   : org = 0x8F000000, len = 0x8000
    DDR3_PCIE_RESERVED1 (RW)     : org = 0x90000000,len = 0x8000000
    DDR3_PCIE_RESERVED2 (RW)     : org = 0x98000000,len = 0x8000000

}

  
SECTIONS
{
    .text:_c_int00     >       MSMCSRAM1
    .text: load >> MSMCSRAM
    .boot_config > L2SRAM_BOOTCFG , type NOINIT
    platform_lib       >       L2SRAM
    .process_buf       >       DDR3
      
    .dsp2hostmailbox   >       DDR3_MAILBOX_RESERVED
    .host2dspmailbox   >       DDR3_MAILBOX_RESERVED

    .buf_debug         >       DDR3

    .stack: load > L2SRAM
    GROUP: load > L2SRAM
    {
        .bss:
        .neardata:
        .rodata:
    }
    .cinit: load > MSMCSRAM
    .pinit: load >> L2SRAM
    .init_array: load > L2SRAM
    .const: load >> MSMCSRAM
    .data: load >> L2SRAM
    .fardata: load >> L2SRAM
    .switch: load >> MSMCSRAM
    .sysmem: load > L2SRAM
    .far: load >> L2SRAM
    .cio: load >> L2SRAM

 
}
