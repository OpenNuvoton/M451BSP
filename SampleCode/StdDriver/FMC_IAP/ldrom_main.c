/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock and I2C controller */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}




int main()
{
    /* Unlock protected register */
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n\n");
    printf("M451 FMC IAP Sample Code [LDROM code]\n");

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\nTo branch to APROM...\n");

    printf("\n\nChange VECMAP and branch to LDROM...\n");
    UART_WAIT_TX_EMPTY(UART0);

    /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
    __set_PRIMASK(1);

    /* Change VECMAP for booting to APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);

    /* Lock protected Register */
    SYS_LockReg();

    /* Software reset to boot to APROM */
    NVIC_SystemReset();

    while(1);
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
