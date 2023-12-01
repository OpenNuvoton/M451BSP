/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/09/02 10:04a $
 * @brief    Implement timer2 event counter function to count the external input event.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/*  Generate Event Counter Source by specify GPIO pin                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void GenerateEventCounterSource(uint32_t u32Port, uint32_t u32Pin, uint32_t u32Counts)
{
    while(u32Counts--)
    {
        GPIO_PIN_DATA(u32Port, u32Pin) = 1;
        GPIO_PIN_DATA(u32Port, u32Pin) = 0;
    }
}

/**
 * @brief       Timer2 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer2 default IRQ, declared in startup_M451Series.s.
 */
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
    }
}

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
    CLK_EnableModuleClock(TMR2_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PD multi-function pin for Timer2 event counter pin */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD3MFP_T2);
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
    volatile uint32_t u32InitCount;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------------+\n");
    printf("|    Timer2 Event Counter Input Sample Code    |\n");
    printf("+----------------------------------------------+\n\n");

    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HCLK      \n");
    printf("    - Continuous counting mode  \n");
    printf("    - Interrupt enable          \n");
    printf("    - Event counter mode enable \n");
    printf("    - Compared value is 56789   \n");
    printf("# Connect PD.4 pin to event counter pin T2(PD.3) and pull PD.4 High/Low to generate T2 event input source.\n\n");

    /* Configure PD.4 as GPIO output pin and pull initial pin status to Low */
    PD->MODE = 0xFFFFFDFF;
    PD4 = 0;

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Clear Timer2 interrupt counts to 0 */
    g_au32TMRINTCount[2] = 0;

    /* Configure Timer2 settings and for event counter application */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 56789);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableInt(TIMER2);

    /* Start Timer2 counting */
    TIMER_Start(TIMER2);

    /* To check if counter value of Timer2 should be 0 while event counter mode is enabled */
    if(TIMER_GetCounter(TIMER2) != 0)
    {
        printf("Default counter value is not 0. (%d)\n", TIMER_GetCounter(TIMER2));
        goto lexit;
    }

    printf("Start to check Timer2 counter value ......\n\n");

    /* To generate one counter event from PD.4 to T2 pin */
    GenerateEventCounterSource(3, 4, 1);

    /* To check if counter value of Timer2 should be 1 */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(TIMER_GetCounter(TIMER2) == 0)
        if(--u32TimeOutCnt == 0) break;
    if(TIMER_GetCounter(TIMER2) != 1)
    {
        printf("Get unexpected counter value. (%d)\n", TIMER_GetCounter(TIMER2));
        goto lexit;
    }

    /* To generate remains counts to T2 pin */
    GenerateEventCounterSource(3, 4, (56789 - 1));

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(1)
    {
        if(g_au32TMRINTCount[2] == 1)
        {
            printf("# Timer2 interrupt event occurred.\n");
            break;
        }

        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for Timer2 interrupt time-out!\n");
            goto lexit;
        }
    }

    printf("# Get Timer2 event counter value is %d .... ", TIMER_GetCounter(TIMER2));
    if(TIMER_GetCounter(TIMER2) == 56789)
    {
        printf("PASS.\n");
    }
    else
    {
        printf("FAIL.\n");
    }

lexit:

    /* Stop Timer2 counting */
    TIMER_Close(TIMER2);

    while(1);
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
