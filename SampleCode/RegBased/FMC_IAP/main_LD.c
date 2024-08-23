/**************************************************************************//**
 * @file     main_LD.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:03a $
 * @brief    Show how to call LDROM functions from APROM.
 *           The code in APROM will look up the table at 0x100E00 to get the address of function of LDROM and call the function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

#define KEY_ADDR            0x20000FFC  /* The location of signature */
#define SIGNATURE           0x21557899  /* The signature word is used by AP code to check if simple LD is finished */

#define CONFIG0_TEST_CODE   0x0F9000FF

#define FUN_TBL_BASE        0x00100E00

int32_t IAP_Func0(int32_t n);
int32_t IAP_Func1(int32_t n);
int32_t IAP_Func2(int32_t n);
int32_t IAP_Func3(int32_t n);

#if defined ( __ICCARM__ )
# pragma location = "FunTblSection" /* The location of FunTblSection is defined in FMC_IAP_LD.icf file. */
__root const uint32_t g_funcTable[4] =
{
    (uint32_t)IAP_Func0, (uint32_t)IAP_Func1, (uint32_t)IAP_Func2, (uint32_t)IAP_Func3
} ;
#elif defined(__ARMCC_VERSION)
const uint32_t * __attribute__((section(".ARM.__at_0x00100E00"))) g_funcTable[4] =
{
    (uint32_t *)IAP_Func0, (uint32_t *)IAP_Func1, (uint32_t *)IAP_Func2, (uint32_t *)IAP_Func3
};
#else
const uint32_t __attribute__((section (".IAPFunTable"))) g_funcTable[4] =
{
    (uint32_t)IAP_Func0, (uint32_t)IAP_Func1, (uint32_t)IAP_Func2, (uint32_t)IAP_Func3
};
#endif


void SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
}

void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init(void)
{
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

int32_t IAP_Func0(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP0! #%d\n", i);
    }

    return n;
#endif
}

int32_t IAP_Func1(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP1! #%d\n", i);
    }

    return n;
#endif
}
int32_t IAP_Func2(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP2! #%d\n", i);
    }

    return n;
#endif
}
int32_t IAP_Func3(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP3! #%d\n", i);
    }

    return n;
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if defined(__GNUC_LD_IAP__)||defined ( __ICCARM__ )
    int32_t i;
#endif

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined(__GNUC_LD_IAP__)

    // Delay 3 seconds
    for(i = 0; i < 30; i++)
    {
        SysTickDelay(10000);
    }

    while(SYS->PDID)__WFI();
#else

    /* Init UART0 for printf */
    UART0_Init();

    /*
        This is a simple sample code for LDROM in new IAP mode.
        The base address is 0x100000.
        The base address for function table is defined by FUN_TBL_BASE.
    */
#if defined ( __ICCARM__ )
    printf("+------------------------------------------------------------------+\n");
    printf("|    M451 Flash Memory Controller Driver Sample Code for LDROM     |\n");
    printf("+------------------------------------------------------------------+\n");

    printf("\nCPU @ %dHz\n\n", SystemCoreClock);

    // Delay 3 seconds
    for(i = 0; i < 30; i++)
    {
        printf(".");
        SysTickDelay(10000);
    }
    printf("\n");

    printf("Function table @ 0x%08x\n", (uint32_t)g_funcTable);
#endif

    while(SYS->PDID)__WFI();
#endif
}
