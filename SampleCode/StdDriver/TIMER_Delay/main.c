/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show how to use timer0 to create various delay time.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2 */
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32DelayTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------+\n");
    printf("|    Timer Delay API Sample Code    |\n");
    printf("+-----------------------------------+\n\n");

    printf("# This sample code is using Timer1 to check Timer0 TIMER_Delay API delay time is reasonable or not.\n");
    printf("# Delay time includes 100 ms, 200 ms, 300 ms, 400 ms and 500 ms.\n\n");

    /* Start Timer1 to measure delay period of TIMER_Delay API is reasonable or not */
    TIMER1->CTL = TIMER_PERIODIC_MODE | (12 - 1);
    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_Start(TIMER1);

    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_Delay(TIMER0, 100000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-1 is %d ms .... ", u32DelayTime);
    if(u32DelayTime == 100)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_Delay(TIMER0, 200000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-2 is %d ms .... ", u32DelayTime);
    if(u32DelayTime == 200)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_Delay(TIMER0, 300000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-3 is %d ms .... ", u32DelayTime);
    if(u32DelayTime == 300)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_Delay(TIMER0, 400000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-4 is %d ms .... ", u32DelayTime);
    if(u32DelayTime == 400)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_Delay(TIMER0, 500000);
    u32DelayTime = TIMER_GetCounter(TIMER1) / 1000;
    printf("    Check DelayTime-5 is %d ms .... ", u32DelayTime);
    if(u32DelayTime == 500)
        printf("PASS.\n");
    else
        printf("FAIL.\n");

    printf("\n*** Check TIMER_Delay API delay time done ***\n");

    while(1);
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
