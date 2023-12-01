/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:04a $
 * @brief    Get the current RTC data/time per tick.
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
volatile uint32_t g_u32RTCTickINT;


/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_M451Series.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();

        g_u32RTCTickINT++;

        PB8 ^= 1;
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

    /* Enable HXT and LXT-32KHz */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2 */
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(RTC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));

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
    S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
    uint32_t u32Sec;
    uint8_t u8IsNewDateTime = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    RTC Date/Time and Tick Sample Code   |\n");
    printf("+-----------------------------------------+\n\n");

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Open RTC and start counting */
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 15;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 30;
    sWriteRTC.u32Second     = 30;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
    if( RTC_Open(&sWriteRTC) < 0 )
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    /* Enable RTC tick interrupt, one RTC tick is 1/4 second */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_4_SEC);

    printf("# Showing RTC date/time on UART0.\n\n");
    printf("1.) Use PB.8 to check tick period time is 1/4 second or not.\n");
    printf("2.) Show RTC date/time and change date/time after 5 seconds:\n");

    /* Use PB.8 to check tick period time */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE8_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE8_Pos);

    u32Sec = 0;
    g_u32RTCTickINT = 0;
    while(1)
    {
        if(g_u32RTCTickINT == 4)
        {
            g_u32RTCTickINT = 0;

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);
            if(u8IsNewDateTime == 0)
            {
                printf("    %d/%02d/%02d %02d:%02d:%02d\n",
                       sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);
            }
            else
            {
                printf("    %d/%02d/%02d %02d:%02d:%02d\r",
                       sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);
            }

            if(u32Sec == sReadRTC.u32Second)
            {
                printf("\nRTC tick period time is incorrect.\n");
                goto lexit;
            }

            u32Sec = sReadRTC.u32Second;

            if(u8IsNewDateTime == 0)
            {
                if(u32Sec == (sWriteRTC.u32Second + 5))
                {
                    printf("\n");
                    printf("3.) Update new date/time to 2014/05/18 11:12:13.\n");

                    u8IsNewDateTime = 1;
                    RTC_SetDate(2014, 5, 18, RTC_SUNDAY);
                    RTC_SetTime(11, 12, 13, RTC_CLOCK_24, RTC_AM);
                }
            }
        }
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
