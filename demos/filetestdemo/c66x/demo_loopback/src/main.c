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



#include <stdio.h>
#include <stdint.h>
#include <ti/platform/platform.h>
#include <ti/csl/csl_bootcfgAux.h>
#include "mailBox.h"


extern void demo_loopback_test(void);
/*This should be abstracted to high level functions through platform library */
extern volatile unsigned int cregister DNUM;
#define DEVICE_REG32_W(x,y)   *(volatile uint32_t *)(x)=(y)
#define DEVICE_REG32_R(x)    (*(volatile uint32_t *)(x))
#define IPCGR(x)            (0x02620240 + x*4)

void demo_disableCache()
{
    volatile unsigned int *MAR = (volatile unsigned int *)0x1848000;

    /* A value of 0x0 indicates non-cachable and non-prefetchable  */
    /* 0x8F000000 - 0xBfffffff  non-cachable, non-prefetchable */
   MAR[143] = 0x0;
   MAR[144] = 0x0;
   MAR[145] = 0x0;
   MAR[146] = 0x0;
   MAR[147] = 0x0;
   MAR[148] = 0x0;
   MAR[149] = 0x0;
   MAR[150] = 0x0;
   MAR[151] = 0x0;
   MAR[152] = 0x0;
   MAR[153] = 0x0;
   MAR[154] = 0x0;
   MAR[155] = 0x0;
   MAR[156] = 0x0;
   MAR[157] = 0x0;
   MAR[158] = 0x0;
   MAR[159] = 0x0;
   MAR[160] = 0x0;
   MAR[161] = 0x0;
   MAR[162] = 0x0;
   MAR[163] = 0x0;
   MAR[164] = 0x0;
   MAR[165] = 0x0;
   MAR[166] = 0x0;
   MAR[167] = 0x0;
   MAR[168] = 0x0;
   MAR[169] = 0x0;
   MAR[170] = 0x0;
   MAR[171] = 0x0;
   MAR[172] = 0x0;
   MAR[173] = 0x0;
   MAR[174] = 0x0;
   MAR[175] = 0x0;
   MAR[176] = 0x0;
   MAR[177] = 0x0;
   MAR[178] = 0x0;
   MAR[179] = 0x0;
   MAR[180] = 0x0;
   MAR[181] = 0x0;
   MAR[182] = 0x0;
   MAR[183] = 0x0;
   MAR[184] = 0x0;
   MAR[185] = 0x0;
   MAR[186] = 0x0;
   MAR[187] = 0x0;
   MAR[188] = 0x0;
   MAR[189] = 0x0;
   MAR[190] = 0x0;
   MAR[191] = 0x0;

}

void main(void)
{
	platform_init_flags flags;
	platform_init_config config;
    int core;

    demo_disableCache();

	if( DNUM == 0)
	{

		/* Platform initialization */
		flags.pll 	= 0x0;
		flags.ddr 	= 0x0;
		flags.tcsl 	= 0x1;
		flags.phy = 0x0;
		flags.ecc 	= 0x0;
		config.pllm  = 0;

		platform_init(&flags, &config);

 /* TODO: This portion needs to be abstracted into high level functions available in platform lib */
		CSL_BootCfgUnlockKicker();

		/* wake up the other cores */
		for (core = 1; core < 8; core++)
		{
		  /* IPC interrupt other cores */
		  DEVICE_REG32_W(IPCGR(core), 1);
		  platform_delay(1000);
		}

	}

	/* Call demo loopback test*/
	demo_loopback_test();


}


