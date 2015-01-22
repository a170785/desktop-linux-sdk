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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ti/csl/csl_cacheAux.h"
#include "ti/csl/csl_xmcAux.h"

#pragma PERSISTENT(boot_entry_location,".boot_entry_location")
volatile uint32_t boot_entry_location;

#define BOOT_MAGIC_ADDR 0x87FFFC	/* boot address in the last word of L2 memory */

#define BOOTROM_RESET_VECTOR_LOCATION 0x20b0fc00

#define DEVICE_INTCTL_BASE  0x1800000
/* The mux registers are indexed from 1, not 0 */
#define INTCTL_REG_MUX(x)   (0x104 + (4*((x)-1)))

#define DEVICE_REG32_W(x,y)   *(volatile uint32_t *)(x)=(y)
#define DEVICE_REG32_R(x)    (*(volatile uint32_t *)(x))

/************************************************************************************
 * Generic bit extraction macros
 ************************************************************************************/
#define BOOTBITMASK(x,y)      (   (   (  ((uint32_t)1 << (((uint32_t)x)-((uint32_t)y)+(uint32_t)1) ) - (uint32_t)1 )   )   <<  ((uint32_t)y)   )
#define BOOT_READ_BITFIELD(z,x,y)   (((uint32_t)z) & BOOTBITMASK(x,y)) >> (y)
#define BOOT_SET_BITFIELD(z,f,x,y)  (((uint32_t)z) & ~BOOTBITMASK(x,y)) | ( (((uint32_t)f) << (y)) & BOOTBITMASK(x,y) )

#define DEVICE_INT_IPC              91

#define DEVICE_CACHE_BASE          0x1840000
#define CACHE_REG_L2WBIV   0x5004
#define CACHE_REG_L1PINV   0x5028
#define CACHE_REG_L1DWBIV  0x5044
/* Timeout values */
#define CACHE_FLUSH_MAX_WAIT    1000


extern volatile unsigned int cregister ICR;
extern volatile unsigned int cregister IFR;
extern volatile unsigned int cregister ISTP;
extern volatile unsigned int cregister CSR;
extern volatile unsigned int cregister IER;
extern volatile unsigned int cregister DNUM;

#define TI667X_IRQ_EOI			0x21800050			/* End of Interrupt Register */
#define LEGACY_A_IRQ_STATUS_RAW	0x21800180			/* Raw Interrupt Status Register */
#define LEGACY_A_IRQ_STATUS		0x21800184			/* Interrupt Enabled Status Register */

#define IPCGR(x)            (0x02620240 + x*4)
#define IPCAR(x)            (0x02620280 + x*4)

void wait_and_start(void)
{
	void   (*entry)();

//	*((unsigned int *)BOOT_MAGIC_ADDR) = 0;
	while(*((unsigned int *)BOOT_MAGIC_ADDR) == 0);
	entry = (void (*)())(*((unsigned int *)BOOT_MAGIC_ADDR));
	(*entry)();
}


void inline hwWbInvL1DInline(void)
{
  DEVICE_REG32_W (DEVICE_CACHE_BASE + CACHE_REG_L1DWBIV, 1);
}
/***************************************************************************************
 * FUNCTION PURPOSE: Setup a mux value
 ***************************************************************************************
 * DESCRIPTION: Configures one of the 12 interrupt mux values. The original value
 *              is lost.
 ***************************************************************************************/
void hwIntctlRoute (uint32_t vector, uint32_t eventNum)
{
	uint32_t muxp;
	uint32_t muxv;
	uint32_t base;

  if (vector > 3 && vector < 8)
    muxp = 1;
  else if (vector < 12)
    muxp = 2;
  else if (vector < 16)
    muxp = 3;
  else
    return;  /* Invalid vector */

  /* Which of the four events in each register (0-3) is determined by the the
   * two lsbs of the vector number. The least significant bit of the mux
   * valud (0, 8, 16, or 24) is found. */
  base = (vector & 0x3) * 8;

  /* Read the active mux, overwrite the event num with the desired value */
  muxv = DEVICE_REG32_R (DEVICE_INTCTL_BASE + INTCTL_REG_MUX(muxp));
  muxv = BOOT_SET_BITFIELD (muxv, eventNum, base+7, base);
  DEVICE_REG32_W (DEVICE_INTCTL_BASE + INTCTL_REG_MUX(muxp), muxv);

} /* hwIntctlRoute */

/**************************************************************************************
 * FUNCTION PURPOSE: Idle
 **************************************************************************************
 * DESCRIPTION: The IPC interrupt is routed to the core, and idle is executed
 **************************************************************************************/
void idle_till_wakeup (void)
{

	uint32_t      csrReg;


  /* Clear any pending interrupts and route the IPC interrupt to vector 4 */
  ICR = 0x0013;
  hwIntctlRoute (4, DEVICE_INT_IPC);

  IER = (1 << 4) | 3;

  /* Point the ISTP to the ROM vector table */
  ISTP = (uint32_t)BOOTROM_RESET_VECTOR_LOCATION;


  /* Write back/invalidate L1D. Use inline since the magic address
   * may live on the same cache line as the current stack */
  hwWbInvL1DInline();


  /* Globally enable interrupts */
  csrReg = CSR | 1;
  CSR    = csrReg;

  asm(" NOP 4 ");
  asm(" IDLE ");
  asm(" NOP 4 ");

  /* On wakeup disable interrupts and restore the system state */
  csrReg = CSR & ~1;
  CSR    = csrReg;
  IER &= (~(1 << 4));
  ICR = 0x0013;



} /* nysh_sleep */

void wait_for_interrupt()
{

	if(DNUM==0)
	{
    	/* clear PCIe interrupt A */
    	*((unsigned int *)LEGACY_A_IRQ_STATUS) = 0x1;
    	*((unsigned int *)TI667X_IRQ_EOI) = 0x0;	/* end of INTA, event number=0 */

    	while(*((unsigned int *)LEGACY_A_IRQ_STATUS_RAW) != 0x1);

       	/* clear PCIe interrupt A */
       	*((unsigned int *)LEGACY_A_IRQ_STATUS) = 0x1;
       	*((unsigned int *)TI667X_IRQ_EOI) = 0x0;	/* end of INTA, event number=0 */

	}
	else
	{
        idle_till_wakeup();
	}
}

/* Function to reset MPAX setting to default */
void resetMPAXSettings(void)
{
  CSL_XMC_XMPAXL    mpaxl;
  CSL_XMC_XMPAXH    mpaxh;
  int index=0;

  /* MPAXH0 Reset values Base Addr + seg size ( 2 GB ) */
  mpaxh.bAddr     = 0;
  mpaxh.segSize   = 0x1E;  /* 2 GB */

  /* replacement addr + perm*/
  mpaxl.rAddr     = 0;
  mpaxl.sr        = 1;
  mpaxl.sw        = 1;
  mpaxl.sx        = 1;
  mpaxl.ur        = 1;
  mpaxl.uw        = 1;
  mpaxl.ux        = 1;

  /* set the xmpax for index0 */
  CSL_XMC_setXMPAXH(0, &mpaxh);
  CSL_XMC_setXMPAXL(0, &mpaxl);   

  /* base addr + seg size (2 GB) */
  mpaxh.bAddr     = (0x80000000 >> 12);
  mpaxh.segSize   = 0x1E;    

  /* replacement addr + perm*/
  mpaxl.rAddr     = 0x800000;
 
  /* set the xmpax for index2 */
  CSL_XMC_setXMPAXH(1, &mpaxh);
  CSL_XMC_setXMPAXL(1, &mpaxl);

  /* base addr + seg size (4 GB) */
  mpaxh.bAddr     = (0x21000000 >> 12);
  mpaxh.segSize   = 0xb;

  /* replacement addr + perm*/
  mpaxl.rAddr     = 0x100000;
  mpaxl.sr        = 1;
  mpaxl.sw        = 1;
  mpaxl.sx        = 1;
  mpaxl.ur        = 1;
  mpaxl.uw        = 0;
  mpaxl.ux        = 0;

  /* set the xmpax for index2 */
  CSL_XMC_setXMPAXH(2, &mpaxh);
  CSL_XMC_setXMPAXL(2, &mpaxl); 

  /* base addr + seg size */
  mpaxh.bAddr     = 0;
  mpaxh.segSize   = 0;

  /* replacement addr + perm*/
  mpaxl.rAddr     = 0;
  mpaxl.sr        = 0;
  mpaxl.sw        = 0;
  mpaxl.sx        = 0;
  mpaxl.ur        = 0;
  mpaxl.uw        = 0;
  mpaxl.ux        = 0;
   
  for(index = 3; index < 8; index++) { 
    /* set the xmpax for index2 */
    CSL_XMC_setXMPAXH(index, &mpaxh);
    CSL_XMC_setXMPAXL(index, &mpaxl);    
  }
 
}

void resetMARSettings(void)
{
  volatile unsigned int *MAR = (volatile unsigned int *)0x1848000;
  int i;  

  /*  A value of LS bit: 1 indicates cacheable, 0 non-cachable ; Bit 3 : prefetchable*/
  /* 0x10000000 - 0xffffffff  to default values 0x8  */
  for(i = 16; i < 256; i++)
  {
    MAR[i] = 0x8;
  }
}
void main(void)
{
  /* configure L2 with 0 cache at boot */
  CACHE_setL2Size(CACHE_0KCACHE);

  /* reset Mpax settings */
  resetMPAXSettings();

  /* reset MAR settings */
  resetMARSettings();

  *((unsigned int *)BOOT_MAGIC_ADDR) = 0;

  /* Wait for IPC interrupt */
  wait_for_interrupt();

  /* wait for code download from PCIe driver and jump to entry point */
  wait_and_start();

}
