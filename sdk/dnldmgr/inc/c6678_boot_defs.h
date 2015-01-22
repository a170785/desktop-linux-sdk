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
#ifndef _C6678_BOOT_DEFS_H
#define _C6678_BOOT_DEFS_H

#define TOTAL_NUM_CORES_PER_CHIP    8
/* PSC registers */
#define PSC_BASE_ADDRESS             0x02350000
#define PTCMD                        0x120
#define PTSTAT                       0x128
#define PDSTAT(n)                    (0x200 + (4 * (n)))
#define PDCTL(n)                     (0x300 + (4 * (n)))
#define MDSTAT(n)                    (0x800 + (4 * (n)))
#define MDCTL(n)                     (0xA00 + (4 * (n))) 

#define PSC_SWRSTDISABLE             0x0
#define PSC_MODSTATE_SYNCRST         0x1 
#define PSC_MODSTATE_DISABLE         0x2
#define PSC_ENABLE                   0x3

#define LOC_RST_ASSERT               0x0
#define LOC_RST_DEASSERT             0x1

#define PSC_PD_OFF		     0x0
#define PSC_PD_ON		     0x1

/* Chip level registers */
#define CHIP_LEVEL_BASE_ADDRESS      0x02620000
#define KICK0                        0x38    
#define KICK1                        0x3C
#define KICK0_UNLOCK                 0x83E70B13
#define KICK1_UNLOCK                 0x95A4F1E0 
#define KICK_LOCK                    0x0
#define DSP_BOOT_ADDR(n)             (0x040 + (4 * (n)))
#define IPCGR(n)                     (0x240 + (4 * (n)))

/* Power domains definitions */
#define PD0         0     // Power Domain-0
#define PD1         1     // Power Domain-1
#define PD2         2     // Power Domain-2
#define PD3         3     // Power Domain-3
#define PD4         4     // Power Domain-4
#define PD5         5     // Power Domain-5
#define PD6         6     // Power Domain-6
#define PD7         7     // Power Domain-7
#define PD8         8     // Power Domain-8
#define PD9         9     // Power Domain-9
#define PD10        10    // Power Domain-10
#define PD11        11    // Power Domain-11
#define PD12        12    // Power Domain-12
#define PD13        13    // Power Domain-13
#define PD14        14    // Power Domain-14
#define PD15        15    // Power Domain-15
#define PD16        16    // Power Domain-16
#define PD17        17    // Power Domain-17

/* Modules on power domain 0 */
#define LPSC_MODRST0     0
#define LPSC_EMIF16_SPI  3  
#define LPSC_TSIP        4

/* Modules on power domain 1 */
#define LPSC_DEBUG       5
#define LPSC_TETB_TRC    6

/* Modules on power domain 2 */
#define LPSC_PA          7  
#define LPSC_SGMII       8  
#define LPSC_SA          9  

/* Modules on power domain 3 */
#define LPSC_PCIE        10

/* Modules on power domain 4 */
#define LPSC_SRIO        11

/* Modules on power domain 5 */
#define LPSC_HYPER       12

/* Modules on power domain 6 */
#define LPSC_RESERV      13

/* Modules on power domain 7 */
#define LPSC_MSMCRAM     14

/* Modules on power domain 8 */
#define LPSC_C0_TIM0     15

/* Modules on power domain 9 */
#define LPSC_C1_TIM1     16

/* Modules on power domain 10 */
#define LPSC_C2_TIM2     17

/* Modules on power domain 11 */
#define LPSC_C3_TIM3     18

/* Modules on power domain 12 */
#define LPSC_C4_TIM4     19

/* Modules on power domain 13 */
#define LPSC_C5_TIM5     20

/* Modules on power domain 14 */
#define LPSC_C6_TIM6     21

/* Modules on power domain 15 */
#define LPSC_C7_TIM7     22


/* Register addresses for PA and SA reset */
#define CSL_PA_SS_CFG_REGS     (0x02000000)
#define CSL_PA_SS_CFG_CP_ACE_CFG_REGS (0x020C0000)

#define PDSP_CONTROL_OFFSET(n)         (0x1000 + (0x100 * (n)))
#define PDSP_PKT_ID_SOFT_RESET_OFFSET  (0x404)
#define PDSP_LUT2_SOFT_RESET_OFFSET    (0x504)
#define PDSP_STATS_SOFT_RESET_OFFSET   (0x6004)
#define PDSP_TIMER_CNTRL_REG_OFFSET(n) (0x3000 + (0x100 * (n)))

#endif /*_C6678_BOOT_DEFS_H */
