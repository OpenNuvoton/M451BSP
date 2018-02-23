/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 8 $
 * $Date: 15/09/02 10:03a $
 * @brief    Get the current RTC data/time per tick.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCON_SETTING      CLK_PLLCTL_72MHz_HXT
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
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT and LXT-32KHz */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));
    while(!(CLK->STATUS & CLK_STATUS_LXTSTB_Msk));

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_RTCCKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

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
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Sec, u32CurSec;;

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

    /* Initial RTC and stay in normal state */
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;
        while(RTC->INIT != RTC_INIT_ACTIVE_Msk);
    }

    /* Setting RTC current date/time and enable tick interrupt function */
    RTC_WaitAccessEnable();
    RTC->CLKFMT  = RTC_CLOCK_24;
    RTC->WEEKDAY = RTC_THURSDAY;
    RTC->CAL     = 0x00140515;         /* Date: 2014/05/15 */
    RTC->TIME    = 0x00153030;         /* Time: 15:30:30 */
    RTC->INTEN   = RTC_INTEN_TICKIEN_Msk;
    RTC->TICK    = RTC_TICK_1_4_SEC;   /* One RTC tick is 1/4 second */

    printf("# Showing RTC date/time on UART0.\n\n");
    printf("1.) Use PB.8 to check tick period time is 1/4 second or not.\n");
    printf("2.) Current RTC date/time is:\n");

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
            printf("    20%02x/%02x/%02x %02x:%02x:%02x\r",
                   (RTC->CAL >> RTC_CAL_YEAR_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_MON_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_DAY_Pos) & 0xFF,
                   (RTC->TIME >> RTC_TIME_HR_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_MIN_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_SEC_Pos) & 0xFF);

            /* Check RTC tick period time is reasonable or not */
            u32CurSec = (((RTC->TIME & RTC_TIME_TENSEC_Msk) >> RTC_TIME_TENSEC_Pos) * 10) + (RTC->TIME & RTC_TIME_SEC_Msk);
            if(u32Sec == u32CurSec)
            {
                printf("\nRTC tick period time is incorrect.\n");
                while(1);
            }

            u32Sec = u32CurSec;
        }
    }
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
