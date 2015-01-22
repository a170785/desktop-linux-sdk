/******************************************************************************
 * Copyright (c) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
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

/******************************************************************************
 *
 * File	Name:       dspc8681.c
 *
 * Description: This contains   TMS320C6678 specific functions.
 * 
 ******************************************************************************/
 
/************************
 * Include Files
 ************************/
#include "platform_internal.h"

static void pll_delay(uint32_t ix)
{
    while (ix--) {
        asm("   NOP");
    }
}

CSL_Status CorePllcHwSetup (
    PllcHwSetup          *hwSetup
)
{
    CSL_Status       status = CSL_SOK;
    volatile uint32_t i, loopCount;
    uint32_t ctl,secctl, pllstatus;

    /* Unlock the Boot Config */
    CSL_BootCfgUnlockKicker();

    /*In PLLCTL, write PLLENSRC = 0 (enable PLLEN bit)*/    
    /* Program pllen=0 (pll bypass), pllrst=1 (reset pll), pllsrc = 0 */

    ctl         = PLLCTL_REG;
    ctl        &= ~(0x00000031);
    PLLCTL_REG = ctl;

    /* Enable secondary controller pll bypass */
    secctl     = SECCTL_REG;
    secctl     |= 1 << 23;
    SECCTL_REG = secctl;


/* PLL WORK AROUND FOR THE SI BUG */
    /* Reset the PLL, wait at least 5 us, release reset */
    ctl = ctl | 2;
    PLLCTL_REG = ctl;

    loopCount = 50000;
    while (loopCount--) {
        asm("   NOP");
    }

    ctl = ctl & ~2;
    PLLCTL_REG = ctl;

    loopCount = 50000;
    while (loopCount--) {
        asm("   NOP");
    }

    
    /* Reset the PLL */
      ctl = ctl | (1 << 3);
      PLLCTL_REG = ctl;
/* WORK AROUND ENDS */

    /* Enable the pll divider */
    secctl = SECCTL_REG;
    secctl |= (1 << 16);
    secctl |= (1 << 19);
    SECCTL_REG = secctl;

    MAINPLLCTL0_REG  = hwSetup->preDiv & 0x3f; /* PLLD[5:0] */
    MAINPLLCTL0_REG |= (((hwSetup->pllM) >> 6) & 0x7f) << 12; /* PLLM[12:6] */
    PLLM_REG   = hwSetup->pllM & 0x3f; /* PLLM[5:0] */

    MAINPLLCTL0_REG      |= (((hwSetup->pllM+1)>>1)-1) << 24;  /* BWADJ bits */

    /* PLL Advisory 9 fix */
    MAINPLLCTL1_REG      |= 1 << 6;

    /*If necessary program PLLDIV1n.  Note that you must aplly the GO operation
     to change these dividers to new ratios*/
     if (hwSetup->divEnable & PLLC_DIVEN_PLLDIV2) {
        PLLDIV2_REG = (hwSetup->pllDiv2) | 0x00008000;        
     }
     if (hwSetup->divEnable & PLLC_DIVEN_PLLDIV5) {
        PLLDIV5_REG = (hwSetup->pllDiv5) | 0x00008000;
     }
     if (hwSetup->divEnable & PLLC_DIVEN_PLLDIV8) {
        PLLDIV8_REG = (hwSetup->pllDiv8) | 0x00008000;
     }

    /*Set GOSET bit in PLLCMD to initiate the GO operation to change the divide
    values and align the SYSCLKs as programmed */
    PLLCMD_REG |= 0x00000001;

    loopCount = 2000;
    while (loopCount--) {
        asm("   NOP");
    }

    /* set pllrst to 0 to deassert pll reset */
    ctl = ctl & ~(1<<3);
    PLLCTL_REG = ctl;


    /* wait for the pll to lock, but don't trap if lock is never read */
    for (i = 0; i < 100; i++)  {
        loopCount = 2000;
        while (loopCount--) {
            asm("   NOP");
        }
      pllstatus = PLLSTAT_REG;
      if ( (pllstatus & 0x1) != 0 )
        break;
    }

    /* Clear the secondary controller bypass bit */
    secctl = secctl & ~(1<<23);
    SECCTL_REG = secctl;
    

    /* Set pllen to 1 to enable pll mode */
    ctl = ctl | 0x1;
    PLLCTL_REG = ctl;

    loopCount = 8000;
    while (loopCount--) {
        asm("   NOP");
    }
    
    return status;
}

CSL_Status CorePllcGetHwSetup (
        PllcHwSetup             *hwSetup
        )
{
    CSL_Status       status   = CSL_SOK;
    volatile uint32_t i, loopCount;

    /* Unlock the Boot Config */
    CSL_BootCfgUnlockKicker();

    hwSetup->divEnable = 0;

    hwSetup->pllM       = (PLLM_REG & 0x3F);
    hwSetup->preDiv     = PREDIV_REG;
    hwSetup->pllDiv2    = PLLDIV2_REG ;
    hwSetup->pllDiv5    = PLLDIV5_REG;
    hwSetup->pllDiv8    = PLLDIV8_REG;

    /* wait for the GOSTAT, but don't trap if lock is never read */
    for (i = 0; i < 100; i++) {
        pll_delay(300);
        if ( (PLL1_STAT & 0x00000001) == 0 ) {
            break;
        }
    }
    if (i == 100) {
        return CSL_ESYS_FAIL;
    }

    return status;
}

CSL_Status 
SetPaPllConfig
(
 void
 ) 
{
    uint32_t passclksel = (DEVSTAT_REG & PASSCLKSEL_MASK);
    uint32_t papllctl0val = PAPLLCTL0_REG;
    uint32_t obsclkval = OBSCLCTL_REG;
    uint32_t pa_pllm = PLLM_PASS;
    uint32_t pa_plld = PLLD_PASS;
    uint32_t temp;
    int pass_freq;
    int pass_freM,pass_freD;

    if (passclksel == 0) 
    {
        IFPRINT(platform_write("SYSCLK/ALTCORECLK is the input to the PA PLL...\n"));
    }

    /* Unlock the Boot Config */
    CSL_BootCfgUnlockKicker();

    if (DNUM == 0) 
    {
        /* Usage Note 9: For optimal PLL operation, the ENSAT bit in the PLL control *
         * registers for the Main PLL, DDR3 PLL, and PA PLL should be set to 1.      *
         * The PLL initialization sequence in the boot ROM sets this bit to 0 and    *
         * could lead to non-optimal PLL operation. Software can set the bit to the  *
         * optimal value of 1 after boot                                             *
         * PAPLLCTL1_REG Bit map                                                     *
         * |31...7   |6     |5 4       |3...0      |                                 *
         * |Reserved |ENSAT |Reserved  |BWADJ[11:8]|                                 */

        PAPLLCTL1_REG |= 0x00000040;

        /* Wait for the PLL Reset time (min: 1000 ns)                                */
        /*pll_delay(1400);*/

        /* Put the PLL in Bypass Mode                                                *
         * PAPLLCTL0_REG Bit map                                                     *
         * |31...24    |23     |22...19       |18...6   |5...0 |                     *
         * |BWADJ[7:0] |BYPASS |Reserved      |PLLM     |PLLD  |                     */

        PAPLLCTL0_REG |= 0x00800000; /* Set the Bit 23 */

        /*Wait 4 cycles for the slowest of PLLOUT or reference clock source (CLKIN)*/
        /*pll_delay(4);*/

        /* In PLL Controller, reset the PLL (bit 14), set (PLLSEL bit 13) in PAPLLCTL1_REG register       */
        PAPLLCTL1_REG |= 0x00006000;

        /* Wait for the PLL Reset time (min: 1000 ns)                                */
        /*pll_delay(1400);*/

        /* Program the necessary multipliers/dividers and BW adjustments             */
        /* Set the divider values */
        PAPLLCTL0_REG &= ~(0x0000003F);
        PAPLLCTL0_REG |= (pa_plld & 0x0000003F);

        /* Set the Multipler values */
        PAPLLCTL0_REG &= ~(0x0007FFC0);
        PAPLLCTL0_REG |= ((pa_pllm << 6) & 0x0007FFC0 );

        /* Set the BWADJ */
        temp = (pa_pllm >> 1) ;
        PAPLLCTL0_REG &= ~(0xFF000000); 
        PAPLLCTL0_REG |= ((temp << 24) & 0xFF000000);
        PAPLLCTL1_REG &= ~(0x0000000F);
        PAPLLCTL1_REG |= ((temp >> 8) & 0x0000000F);

        /*Wait for PLL to lock min 5 micro seconds*/
        pll_delay(7000);

        /*In PAPLLCTL1_REG, write PLLRST = 0 to bring PLL out of reset */
        PAPLLCTL1_REG &= ~(0x00004000);

        /*Wait for PLL to lock min 50 micro seconds*/
        pll_delay(70000);

        /* Put the PLL in PLL Mode                                                   *
         * PAPLLCTL0_REG Bit map                                                     *
         * |31...24    |23     |22...19       |18...6   |5...0 |                     *
         * |BWADJ[7:0] |BYPASS |Reserved      |PLLM     |PLLD  |                     */
        PAPLLCTL0_REG &= ~(0x00800000); /* ReSet the Bit 23 */

        papllctl0val = PAPLLCTL0_REG;

        /* Tells the multiplier value for the PA PLL */
        pa_pllm = (((papllctl0val & PA_PLL_CLKF_MASK) >> 6) + 1);
        IFPRINT(platform_write("PA PLL programmable multiplier = %d\n", pa_pllm));

        /* Tells the divider value for the PA PLL */
        pa_plld = (((papllctl0val & PA_PLL_CLKR_MASK) >> 0) +1);
        IFPRINT(platform_write("PA PLL programmable divider = %d\n", pa_plld));

        // Compute the real pass clk freq (*100)
        pass_freq = (12288 * (pa_pllm) / (pa_plld) / (1));

        // Displayed frequency in MHz
        pass_freM = pass_freq / 100;

        // passclk freq first decimal if freq expressed in MHz
        pass_freD = ((pass_freq - pass_freM * 100) + 5) / 10;

        // Add roundup unit to MHz displayed and reajust decimal value if necessary...
        if (pass_freD > 9)
        {
            pass_freD = pass_freD - 10;
            pass_freM = pass_freM + 1;
        }

        IFPRINT(platform_write("PLL3 Setup for PASSCLK @ %d.%d MHz... \n", pass_freM, pass_freD ));
        IFPRINT(platform_write("PLL3 Setup... Done.\n" ));

        return CSL_SOK;

    }
    else
    {
        IFPRINT(platform_write("DSP core #%d cannot set PA PLL \n",,2,,,DNUM));
        return CSL_ESYS_FAIL;
    }

}

/***************************************************************************************
 * FUNCTION PURPOSE: Power up all the power domains
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void PowerUpDomains (void)
{
    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any 
     * PASS device register access. This not required for the simulator. */

    /* Set PASS Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_ALWAYSON);

    /* Enable the clocks for PASS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_EMIF4F, PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_EMIF25_SPI,  PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_ALWAYSON);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_ALWAYSON));


    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any 
     * PASS device register access. This not required for the simulator. */

    /* Set PASS Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_PASS);

    /* Enable the clocks for PASS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_PKTPROC, PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_CPGMAC,  PSC_MODSTATE_ENABLE);
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_Crypto,  PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_PASS);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_PASS));
}

CSL_Status SetDDR3PllConfig(void) 
{

    uint32_t ddr3pllctl0val = DDR3PLLCTL0_REG;
    uint32_t obsclkval = OBSCLCTL_REG;
    uint32_t ddr3_pllm = PLLM_DDR3;
    uint32_t ddr3_plld = PLLD_DDR3;
    uint32_t temp;

    /* Unlock the Boot Config */
    CSL_BootCfgUnlockKicker();

    if (DNUM == 0) 
    {
        /* Usage Note 9: For optimal PLL operation, the ENSAT bit in the PLL control *
         * registers for the Main PLL, DDR3 PLL, and PA PLL should be set to 1.      *
         * The PLL initialization sequence in the boot ROM sets this bit to 0 and    *
         * could lead to non-optimal PLL operation. Software can set the bit to the  *
         * optimal value of 1 after boot                                             *
         * DDR3PLLCTL1_REG Bit map                                                     *
         * |31...7   |6     |5 4       |3...0      |                                 *
         * |Reserved |ENSAT |Reserved  |BWADJ[11:8]|                                 */

        DDR3PLLCTL1_REG |= 0x00000040;

        /* Wait for the PLL Reset time (min: 1000 ns)                                */
        /*pll_delay(1400);*/

        /* Put the PLL in Bypass Mode                                                *
         * DDR3PLLCTL0_REG Bit map                                                     *
         * |31...24    |23     |22...19       |18...6   |5...0 |                     *
         * |BWADJ[7:0] |BYPASS |Reserved      |PLLM     |PLLD  |                     */

        DDR3PLLCTL0_REG |= 0x00800000; /* Set the Bit 23 */

        /*Wait 4 cycles for the slowest of PLLOUT or reference clock source (CLKIN)*/
        /*pll_delay(4);*/

        /* In PLL Controller, reset the PLL (bit 13) in DDR3PLLCTL1_REG register       */
        DDR3PLLCTL1_REG |= 0x00002000;

        /* Wait for the PLL Reset time (min: 1000 ns)                                */
        /*pll_delay(1400);*/

        /* Program the necessary multipliers/dividers and BW adjustments             */
        /* Set the divider values */
        DDR3PLLCTL0_REG &= ~(0x0000003F);
        DDR3PLLCTL0_REG |= (ddr3_plld & 0x0000003F);

        /* Set the Multipler values */
        DDR3PLLCTL0_REG &= ~(0x0007FFC0);
        DDR3PLLCTL0_REG |= ((ddr3_pllm << 6) & 0x0007FFC0 );

        /* Set the BWADJ */
        temp = (ddr3_pllm >> 1);
        DDR3PLLCTL0_REG &= ~(0xFF000000); 
        DDR3PLLCTL0_REG |= ((temp << 24) & 0xFF000000);
        DDR3PLLCTL1_REG &= ~(0x0000000F);
        DDR3PLLCTL1_REG |= ((temp >> 8) & 0x0000000F);

        /*Wait for PLL to lock min 5 micro seconds*/
        pll_delay(7000);

        /*In DDR3PLLCTL1_REG, write PLLRST = 0 to bring PLL out of reset */
        DDR3PLLCTL1_REG &= ~(0x00002000);

        /*Wait for PLL to lock min 50 micro seconds*/
        pll_delay(70000);

        /* Put the PLL in PLL Mode                                                   *
         * DDR3PLLCTL0_REG Bit map                                                   *
         * |31...24    |23     |22...19       |18...6   |5...0 |                     *
         * |BWADJ[7:0] |BYPASS |Reserved      |PLLM     |PLLD  |                     */
        DDR3PLLCTL0_REG &= ~(0x00800000); /* ReSet the Bit 23 */

        ddr3pllctl0val = DDR3PLLCTL0_REG;

        /* Tells the multiplier value for the DDR3 PLL */
        ddr3_pllm = (((ddr3pllctl0val & 0x0007FFC0) >> 6) + 1);
        IFPRINT(platform_write("PA PLL programmable multiplier = %d\n", ddr3_pllm));

        /* Tells the divider value for the DDR3 PLL */
        ddr3_plld = (((ddr3pllctl0val & 0x0000003F) >> 0) +1);
        IFPRINT(platform_write("PA PLL programmable divider = %d\n", ddr3_plld));

        IFPRINT(platform_write("PLL2 Setup... Done.\n" ));

        return CSL_SOK;

    }
    else
    {
        IFPRINT(platform_write("DSP core #%d cannot set DDR3 PLL \n",,2,,,DNUM));
        return CSL_ESYS_FAIL;
    }
}


/*--------------------------------------------------------------*/
/* xmc_setup()                                                  */
/* XMC MPAX register setting to access DDR3 config space        */
/*--------------------------------------------------------------*/
void xmc_setup()
{  
    /* mapping for ddr emif registers XMPAX*2 */
    CSL_XMC_XMPAXL    mpaxl;
    CSL_XMC_XMPAXH    mpaxh;

    /* base addr + seg size (64KB)*/    //"1B"-->"B" by xj */
    mpaxh.bAddr     = (0x2100000B >> 12);
    mpaxh.segSize   = (0x2100000B & 0x0000001F);

    /* replacement addr + perm*/
    mpaxl.rAddr     = 0x100000;
    mpaxl.sr        = 1;
    mpaxl.sw        = 1;
    mpaxl.sx        = 1;
    mpaxl.ur        = 1;
    mpaxl.uw        = 1;
    mpaxl.ux        = 1;

    /* set the xmpax for index2 */
    CSL_XMC_setXMPAXH(2, &mpaxh);
    CSL_XMC_setXMPAXL(2, &mpaxl);    
}

/* Set the desired DDR3 configuration -- assumes 66.67 MHz DDR3 clock input */
CSL_Status DDR3Init(uint8_t SDramType, uint8_t *DDR_param_addr) 
{

    CSL_Status status = CSL_SOK;    
    volatile unsigned int loopCount;
    uint32_t ddr3config, ddrPhyCtrl;
    uint8_t ddrPHYReadLatency;
    EMIF4F_TIMING1_CONFIG sdram_tim1;
    EMIF4F_TIMING2_CONFIG sdram_tim2;
    EMIF4F_TIMING3_CONFIG sdram_tim3;
    EMIF4F_OUTPUT_IMP_CONFIG    zqcfg;
    EMIF4F_PWR_MGMT_CONFIG      pwrmgmtcfg;
    EMIF4F_SDRAM_CONFIG         sdramcfg;
    SDRAM_DDR_parameter *external_sdram_param = (SDRAM_DDR_parameter *)DDR_param_addr;

    CSL_BootCfgUnlockKicker();        

    /* Wait for PLL to lock = min 500 ref clock cycles. 
       With refclk = 100MHz, = 5000 ns = 5us */
    platform_delaycycles(50000);

    /**************** 3.3 Leveling Register Configuration ********************/
    CSL_BootCfgGetDDRConfig(0, &ddr3config);
    ddr3config &= ~(0x007FE000);  // clear ctrl_slave_ratio field
    CSL_BootCfgSetDDRConfig(0, ddr3config);

    CSL_BootCfgGetDDRConfig(0, &ddr3config);
    ddr3config |= 0x00200000;     // set ctrl_slave_ratio to 0x100
    CSL_BootCfgSetDDRConfig(0, ddr3config);

    CSL_BootCfgGetDDRConfig(12, &ddr3config);
    ddr3config |= 0x08000000;    // Set invert_clkout = 1
    CSL_BootCfgSetDDRConfig(12, ddr3config);

    CSL_BootCfgGetDDRConfig(0, &ddr3config);
    ddr3config |= 0xF;            // set dll_lock_diff to 15
    CSL_BootCfgSetDDRConfig(0, ddr3config);

    CSL_BootCfgGetDDRConfig(23, &ddr3config);
    ddr3config |= 0x00000200;    // See section 4.2.1, set for partial automatic levelling
    CSL_BootCfgSetDDRConfig(23, ddr3config);

    /**************** 3.3 Partial Automatic Leveling ********************/
    ddr3config = 0x0000005E    ; CSL_BootCfgSetDDRConfig(2,  ddr3config);
    ddr3config = 0x0000005E;  CSL_BootCfgSetDDRConfig(3,  ddr3config);
    ddr3config = 0x0000005E;  CSL_BootCfgSetDDRConfig(4,  ddr3config);
    ddr3config = 0x00000051;  CSL_BootCfgSetDDRConfig(5,  ddr3config);
    ddr3config = 0x00000038;  CSL_BootCfgSetDDRConfig(6,  ddr3config);
    ddr3config = 0x0000003A;  CSL_BootCfgSetDDRConfig(7,  ddr3config);
    ddr3config = 0x00000024;  CSL_BootCfgSetDDRConfig(8,  ddr3config);
    ddr3config = 0x00000020;  CSL_BootCfgSetDDRConfig(9,  ddr3config);
    ddr3config = 0x00000044;  CSL_BootCfgSetDDRConfig(10, ddr3config);

    ddr3config = 0x000000DD;  CSL_BootCfgSetDDRConfig(14,  ddr3config);
    ddr3config = 0x000000DD;  CSL_BootCfgSetDDRConfig(15,  ddr3config);
    ddr3config = 0x000000BE;  CSL_BootCfgSetDDRConfig(16,  ddr3config);
    ddr3config = 0x000000CA;  CSL_BootCfgSetDDRConfig(17,  ddr3config);
    ddr3config = 0x000000A9;  CSL_BootCfgSetDDRConfig(18,  ddr3config);
    ddr3config = 0x000000A7;  CSL_BootCfgSetDDRConfig(19,  ddr3config);
    ddr3config = 0x0000009E;  CSL_BootCfgSetDDRConfig(20,  ddr3config);
    ddr3config = 0x000000A1;  CSL_BootCfgSetDDRConfig(21,  ddr3config);
    ddr3config = 0x000000BA;  CSL_BootCfgSetDDRConfig(22,  ddr3config);

    /*Do a PHY reset. Toggle DDR_PHY_CTRL_1 bit 15 0->1->0 */
    CSL_EMIF4F_GetPhyControl(&ddrPhyCtrl, &ddrPHYReadLatency);
    ddrPhyCtrl &= ~(0x00008000);
    CSL_EMIF4F_SetPhyControl(ddrPhyCtrl,  ddrPHYReadLatency);

    CSL_EMIF4F_GetPhyControl(&ddrPhyCtrl, &ddrPHYReadLatency);
    ddrPhyCtrl |= (0x00008000);
    CSL_EMIF4F_SetPhyControl(ddrPhyCtrl,  ddrPHYReadLatency);

    CSL_EMIF4F_GetPhyControl(&ddrPhyCtrl, &ddrPHYReadLatency);
    ddrPhyCtrl &= ~(0x00008000);
    CSL_EMIF4F_SetPhyControl(ddrPhyCtrl,  ddrPHYReadLatency);

    /***************** 3.4 Basic Controller and DRAM configuration ************/
    /* enable configuration */
    /*    hEmif->SDRAM_REF_CTRL    = 0x00005162; */
    CSL_EMIF4F_EnableInitRefresh();
    CSL_EMIF4F_SetRefreshRate(0x5162);

    if(SDramType == EXTERNAL_CONFIG)
    {
        SDRAM_TIM_1_REG = external_sdram_param->SDRAM_TIM1;
        SDRAM_TIM_2_REG = external_sdram_param->SDRAM_TIM2;
        SDRAM_TIM_3_REG = external_sdram_param->SDRAM_TIM3;
    }
    else
    {
        /*    hEmif->SDRAM_TIM_1   = 0x1113783C; */

        sdram_tim1.t_wtr    = 4;
        sdram_tim1.t_rrd    = 7;
        sdram_tim1.t_rc     = 0x20;
        sdram_tim1.t_ras    = 0x17;
        sdram_tim1.t_wr     = 9;
        sdram_tim1.t_rcd    = 8;
        sdram_tim1.t_rp     = 8;
        CSL_EMIF4F_SetTiming1Config(&sdram_tim1);

        if(SDramType == SAMSUNG_2G_BIT)
        {
            /*    hEmif->SDRAM_TIM_2   = 0x30717FE3; */
            sdram_tim2.t_cke    = 3;
            sdram_tim2.t_rtp    = 4;
            sdram_tim2.t_xsrd   = 0x1FF;
            sdram_tim2.t_xsnr   = 0x071;
            sdram_tim2.t_xp     = 3;
            sdram_tim2.t_odt    = 0;
            CSL_EMIF4F_SetTiming2Config (&sdram_tim2);

            /*    hEmif->SDRAM_TIM_3   = 0x559F86AF; */
            sdram_tim3.t_rasMax     = 0xF;
            sdram_tim3.t_rfc        = 0x06A;
            sdram_tim3.t_tdqsckmax  = 0;
            sdram_tim3.zq_zqcs      = 0x3F;
            sdram_tim3.t_ckesr      = 4;
            sdram_tim3.t_csta       = 0x5;
            sdram_tim3.t_pdll_ul    = 0x5;
            CSL_EMIF4F_SetTiming3Config (&sdram_tim3);    
        }
        else	/* Micron 4G bit*/
        {
            /*    hEmif->SDRAM_TIM_2   = 0x307A7FE3; */
            sdram_tim2.t_cke    = 3;
            sdram_tim2.t_rtp    = 4;
            sdram_tim2.t_xsrd   = 0x1FF;
            sdram_tim2.t_xsnr   = 0x07A;
            sdram_tim2.t_xp     = 3;
            sdram_tim2.t_odt    = 0;
            CSL_EMIF4F_SetTiming2Config (&sdram_tim2);

            /*    hEmif->SDRAM_TIM_3   = 0x559F873F; */
            sdram_tim3.t_rasMax     = 0xF;
            sdram_tim3.t_rfc        = 0x073;
            sdram_tim3.t_tdqsckmax  = 0;
            sdram_tim3.zq_zqcs      = 0x3F;
            sdram_tim3.t_ckesr      = 4;
            sdram_tim3.t_csta       = 0x5;
            sdram_tim3.t_pdll_ul    = 0x5;
            CSL_EMIF4F_SetTiming3Config (&sdram_tim3);    
        }
    }
	
    /*    hEmif->DDR_PHY_CTRL_1   = 0x0010010F; */
    ddrPHYReadLatency   = 0x0F;
    ddrPhyCtrl          = (0x08008);
    CSL_EMIF4F_SetPhyControl(ddrPhyCtrl,  ddrPHYReadLatency);

    /*    hEmif->ZQ_CONFIG        = 0x70073214; */
    zqcfg.zqRefInterval     = 0x3214;
    zqcfg.zqZQCLMult        = 3;
    zqcfg.zqZQCLInterval    = 1;
    zqcfg.zqSFEXITEn        = 1;
    zqcfg.zqDualCSEn        = 1;
    zqcfg.zqCS0En           = 1;
    zqcfg.zqCS1En           = 0;
    CSL_EMIF4F_SetOutputImpedanceConfig(&zqcfg);

    /*    hEmif->PWR_MGMT_CTRL    = 0x0; */
    pwrmgmtcfg.csTime           = 0;
    pwrmgmtcfg.srTime           = 0;
    pwrmgmtcfg.lpMode           = 0;
    pwrmgmtcfg.dpdEnable        = 0;
    pwrmgmtcfg.pdTime           = 0;
    CSL_EMIF4F_SetPowerMgmtConfig  (&pwrmgmtcfg);

    if(SDramType == EXTERNAL_CONFIG)
    {
        SDRAM_CFG_REG = external_sdram_param->SDRAM_CFG;
    }
    else
    {
        /* New value with DYN_ODT disabled and SDRAM_DRIVE = RZQ/7 */ 
        /*    hEmif->SDRAM_CONFIG     = 0x63062A32; */
        CSL_EMIF4F_GetSDRAMConfig (&sdramcfg);
        sdramcfg.pageSize           = 2;
        sdramcfg.eBank              = 0;
        sdramcfg.iBank              = 3;
        if(SDramType == SAMSUNG_2G_BIT)
            sdramcfg.rowSize            = 4;
        else	/* Micron 4G bit */
            sdramcfg.rowSize            = 5;
        sdramcfg.CASLatency         = 10;
        sdramcfg.narrowMode         = 0;
        sdramcfg.CASWriteLat        = 2;
        sdramcfg.SDRAMDrive         = 1;
        sdramcfg.disableDLL         = 0;
        sdramcfg.dynODT             = 0;
        sdramcfg.ddrDDQS            = 0;
        sdramcfg.ddrTerm            = 3;
        sdramcfg.iBankPos           = 0;
        sdramcfg.type               = 3;
        CSL_EMIF4F_SetSDRAMConfig (&sdramcfg);
    }
    pll_delay(840336); /*Wait 600us for HW init to complete*/

    /* Refresh rate = (7.8*666MHz] */
    /*    hEmif->SDRAM_REF_CTRL   = 0x00001450;     */
    CSL_EMIF4F_SetRefreshRate(0x00001450);

    /***************** 4.2.1 Partial automatic leveling ************/
    /*    hEmif->RDWR_LVL_RMP_CTRL      =  0x80000000; */
    CSL_EMIF4F_SetLevelingRampControlInfo(1, 0, 0, 0, 0);

    /* Trigger full leveling - This ignores read DQS leveling result and uses ratio forced value */
    /*    hEmif->RDWR_LVL_CTRL          =  0x80000000; */
    CSL_EMIF4F_SetLevelingControlInfo(1, 0, 0, 0, 0);

    /************************************************************
      Wait for min 1048576 DDR clock cycles for leveling to complete 
      = 1048576 * 1.5ns = 1572864ns = 1.57ms.
      Actual time = ~10-15 ms 
     **************************************************************/
    pll_delay(4201680); //Wait 3ms for leveling to complete

    return (status);

}

