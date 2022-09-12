/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:03a $
 * @brief    Show how to access RTC spare registers.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCON_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000


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
    uint32_t i, u32ReadData, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    RTC Spare Register R/W Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    /* Initial RTC and stay in normal state */
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;
        u32TimeOutCnt = RTC_TIMEOUT;
        while(RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("RTC initial fail!\n");
                goto lexit;
            }
        }
    }

    /* Enable Spare registers access function */
    RTC_WaitAccessEnable();
    RTC->SPRCTL = RTC_SPRCTL_SPRRWEN_Msk;

    /* Wait for spare register ready */
    u32TimeOutCnt = RTC_TIMEOUT;
    while(!(RTC->SPRCTL & RTC_SPRCTL_SPRRWRDY_Msk))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for RTC spare register ready time-out!\n");
            goto lexit;
        }
    }

    for(i = 0; i < 20; i++)
    {
        printf("Write Spare register-%02d to 0x%08X ... ", i, (0x5A5A0000 + i));
        RTC_WaitAccessEnable();
        RTC_WRITE_SPARE_REGISTER(i, 0x5A5A0000 + i);

        /* Wait for spare register ready */
        u32TimeOutCnt = RTC_TIMEOUT;
        while(!(RTC->SPRCTL & RTC_SPRCTL_SPRRWRDY_Msk))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for RTC spare register ready time-out!\n");
                goto lexit;
            }
        }

        printf("DONE.\n");
    }

    printf("\n");
    for(i = 0; i < 20; i++)
    {
        printf("Read Spare register-%02d value is ", i);
        RTC_WaitAccessEnable();
        u32ReadData = RTC_READ_SPARE_REGISTER(i);
        if(u32ReadData == (0x5A5A0000 + i))
        {
            printf("0x%08X ... PASS.\n", u32ReadData);
        }
        else
        {
            printf("0x%08X ... FAIL.\n", u32ReadData);
            goto lexit;
        }

        /* Wait for spare register ready */
        u32TimeOutCnt = RTC_TIMEOUT;
        while(!(RTC->SPRCTL & RTC_SPRCTL_SPRRWRDY_Msk))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for RTC spare register ready time-out!\n");
                goto lexit;
            }
        }
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
