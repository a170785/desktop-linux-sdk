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

#ifndef _C6678_PCIE_DEFS_H
#define _C6678_PCIE_DEFS_H

/*
* These are the default PCI IDs for TI816X device. Update these as applicable
* or add Subsystem IDs and corresponding check in the code if you have multiple
* TI816X based PCIe cards.
*/
#define TI667X_PCI_VENDOR_ID               0x104c   /* TI */
#define TI667X_PCI_DEVICE_ID               0xb005   /* C6678 */
#define TI667X_PCI_PID                     0x4e301100 /* PCI PID */


#define TI667X_EP_L2SRAM_BASE       0x00800000      /* C6678 core0 L2 memory */
#define TI667X_EP_MSMCSRAM_BASE     0x0C000000      /* C6678 Multicore Shared Memory */
#define TI667X_EP_DDR3_BASE         0x80000000      /* C6678 DDR3 memory */
#define TI667X_EP_BOOTFLAG_ADDR     0x1087FFFC      /* entry point address in L2 memory */
#define TI667X_EP_VCONSOLE_ADDR     0x00800800      /* the virtual console addr in the shared memory */
#define TI667X_EP_CHIP_CFG_BASE     0x2000000

#define TI667X_EP_PCIE_BASE         0x21800000      /* C6678 PCIE base address */
#define TI667X_EP_MSI_IRQ_OFFSET    0x54            /* Endpoint MSI Interrupt request Register */
#define TI667X_EP_IRQ_SET_OFFSET    0x64            /* Endpoint Interrupt Request Set Register */
#define TI667X_EP_IRQ_CLR_OFFSET    0x68            /* Endpoint Interrupt Request Clear Register */
#define TI667X_EP_IRQ_STATUS_OFFSET 0x6C            /* Endpoint Interrupt Stauts Register */
#define TI667X_EP_MSI0_SET_OFFSET   0x100           /* PCIE MSI interrupt 0 SET */
#define TI667X_EP_MSI0_CLR_OFFSET   0x104           /* PCIE MSI interrupt 0 CLR */
#define TI667X_EP_INTA_SET_OFFSET   0x180           /* PCIE legacy interrupt A SET */
#define TI667X_EP_INTA_CLR_OFFSET   0x184           /* PCIE legacy interrupt A CLR */
#define TI667X_EP_INTB_SET_OFFSET   0x190           /* PCIE legacy interrupt B SET */
#define TI667X_EP_INTB_CLR_OFFSET   0x194           /* PCIE legacy interrupt B CLR */
#define TI667X_EP_PCIE_MAX_OFFSET   0x200           /* Maximum offset accessed in PCIE config space */

#define TI667X_EP_BOOTFLAG_OFFSET   0x0007FFFC      /* the last address in the respective local L2 */

#define TI667X_EP_L2SRAM_MAX_SIZE       0x00080000  /* L2RAM size */
#define TI667X_EP_MSMCSRAM_MAX_SIZE     0x00400000  /* MSMC RAM size */
#define TI667X_EP_DDR3_MAX_SIZE         0x40000000  /* DDR3 RAM size */
#define TI667X_EP_CHIP_CFG_MAX_SIZE     0x1000000    /* Chip config maximum size */

#define TI667X_EP_MAP_OFFSET_ALIGN      0x000000FF  /* Mapping alignment */
#define TI667X_EP_MAP_ALIGN             0xFFFFFF00  /* Mapping alignment */
#define TI667X_EP_L2MAP_OFFSET_ALIGN    0x000FFFFF  /* L2 Mapping alignment */
#define TI667X_EP_L2MAP_ALIGN           0xFFF00000  /* L2 Mapping alignment */

#define CMD_STATUS          0x004
#define CFG_SETUP           0x008
#define OB_SIZE             0x030
#define PRIORITY            0x03C
#define IB_BAR(x)           (0x300 + (0x10 * x))
#define IB_START_LO(x)      (0x304 + (0x10 * x))
#define IB_START_HI(x)      (0x308 + (0x10 * x))
#define IB_OFFSET(x)        (0x30c + (0x10 * x))
#define OB_OFFSET_INDEX(x)  (0x200 + (0x08 * x))
#define OB_OFFSET_HI(x)     (0x204 + (0x08 * x))

/* Application command register values */
#define DBI_CS2_EN_VAL      (1UL<< 5)
#define IB_XLAT_EN_VAL      (1UL<< 2)
#define OB_XLAT_EN_VAL      (1UL<< 1)

#define OB_SIZE_1MB         0
#define OB_SIZE_2MB         1
#define OB_SIZE_4MB         2
#define OB_SIZE_8MB         3

#define OB_REGISTERS_RESERVED_FOR_CONFIG_SPACE 4
#define TOTAL_OB_REGIONS 32

/* EDMA registers */
#define EDMA_TPCC0_BASE_ADDRESS      0x02700000
#define DCHMAP                       0x0100  
#define DMAQNUM0                     0x0240  
#define ESR                          0x1010 
#define EESR                         0x1030                 
#define IESR                         0x1060
#define IPR                          0x1068 
#define ICR                          0x1070

#define PARAM_SET_BASE               0x4000

#define PARAM_SET_OPT	     0x0000
#define PARAM_SET_SRC        0x0004
#define PARAM_SET_A_B_CNT    0x0008
#define PARAM_SET_DST            0x000C
#define PARAM_SET_SRC_DST_BIDX   0x0010
#define PARAM_SET_LINK_BCNTRLD   0x0014
#define PARAM_SET_SRC_DST_CIDX   0x0018
#define PARAM_SET_CCNT           0x001C

#define PARAM_SET_OFFSET(set_no, set_location) (PARAM_SET_BASE + (set_no*0x20) +set_location)

#define PCIE_DATA                    0x60000000  
/* Payload size in bytes over PCIE link. PCIe module supports 
outbound payload size of 128 bytes and inbound payload size of 256 bytes */
#define PCIE_TRANSFER_SIZE           0x80               

#endif /* _C6678_PCIE_DEFS_Hi */
