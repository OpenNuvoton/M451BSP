/**************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "spi_transfer.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HIRC
#define PLL_CLOCK           71884800

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = __HIRC; // TIMER_GetModuleClock(timer);
    uint32_t u32Cmpr = 0UL, u32Prescale = 0UL;

    /* Fastest possible timer working freq is (u32Clk / 2). While cmpr = 2, prescaler = 0. */
    if(u32Freq > (u32Clk / 2UL))
    {
        u32Cmpr = 2UL;
    }
    else
    {
        u32Cmpr = u32Clk / u32Freq;
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */

        if(u32Prescale > 0UL)
        {
            u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
        }
    }

    timer->CTL = u32Mode | u32Prescale;
    timer->CMP = u32Cmpr;
    return (u32Clk / (u32Cmpr * (u32Prescale + 1UL)));
}

void TIMER3_Init(void)
{
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR3CKEN_Msk;
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR3SEL_Msk)) | CLK_CLKSEL1_TMR3SEL_HIRC;
    // Set timer frequency to 3HZ
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 3);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER3);
    // Start Timer 3
    TIMER_Start(TIMER3);
}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        if(--u32TimeOutCnt == 0) return -1;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;;

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        if(--u32TimeOutCnt == 0) return -1;

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Select PCLK1 as the clock source of SPI1 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK1;
    /* Enable SPI1 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Setup SPI1 multi-function pins */
    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE10MFP_Msk | SYS_GPE_MFPH_PE11MFP_Msk | SYS_GPE_MFPH_PE12MFP_Msk | SYS_GPE_MFPH_PE13MFP_Msk);
    SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE10MFP_SPI1_MISO | SYS_GPE_MFPH_PE11MFP_SPI1_MOSI | SYS_GPE_MFPH_PE12MFP_SPI1_SS | SYS_GPE_MFPH_PE13MFP_SPI1_CLK);
    /* Enable SPI1 clock pin (PE13) schmitt trigger */
    PE->SMTEN |= GPIO_SMTEN_SMTEN13_Msk;

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t cmd_buff[16];

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    if( SYS_Init() < 0 ) goto _APROM;
    SPI_Init();
    GPIO_Init();
    TIMER3_Init();

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    while(1)
    {
        if(bSpiDataReady == 1)
        {
            goto _ISP;
        }

        if(TIMER3->INTSTS & TIMER_INTSTS_TIF_Msk)
        {
            goto _APROM;
        }
    }

_ISP:
    while(1)
    {
        if(bSpiDataReady == 1)
        {
            memcpy(cmd_buff, spi_rcvbuf, 64);
            bSpiDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
