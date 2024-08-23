/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:03a $
 * @brief    Show how to read/program embedded flash by ISP function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

int32_t g_FMC_i32ErrCode;

void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;

    int32_t i;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for clock ready */
    i = 22000000; // For timeout
    while(i-- > 0)
    {
        if((CLK->STATUS & (CLK_STATUS_PLLSTB_Msk | CLK_STATUS_HXTSTB_Msk)) ==
                (CLK_STATUS_PLLSTB_Msk | CLK_STATUS_HXTSTB_Msk))
            break;
    }

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

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
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main()
{
    char *cBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t u32CBS;
    uint32_t u32Data, u32RData;
    uint32_t u32Addr;
    uint32_t u32Cfg0, u32Cfg1;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Initial UART */
    UART0_Init();

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|          M451 FMC Sample Code          |\n");
    printf("+----------------------------------------+\n");

    /* Enable FMC ISP functions */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_LDUEN_Msk | FMC_ISPCTL_CFGUEN_Msk;

    /* Check current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    printf("  Current Boot Mode is ....................... [%s]\n", cBootMode[u32CBS]);

    /* Show the user configuration */
    u32Cfg0 = FMC_Read(FMC_CONFIG_BASE);
    u32Cfg1 = FMC_Read(FMC_CONFIG_BASE + 4);
    printf("  CFG0 ....................................... [0x%08x]\n", u32Cfg0);
    printf("  CFG1 ....................................... [0x%08x]\n", u32Cfg1);

    /* Read UID */
    printf("  UID[31: 0] ................................. [0x%08x]\n", FMC_ReadUID(0));
    printf("  UID[63:32] ................................. [0x%08x]\n", FMC_ReadUID(1));
    printf("  UID[95:64] ................................. [0x%08x]\n", FMC_ReadUID(2));

    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;

    /* Erase Demo */
    FMC_Erase(u32Addr);

    /* Write Demo */
    u32Data = 0x12345678;
    FMC_Write(u32Addr, u32Data);

    /* Read Demo */
    u32RData = FMC_Read(u32Addr);

    printf("  Write %08x to 0x%08x ............... ", u32Addr, u32Data);
    if(u32Data == u32RData)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(SYS->PDID);
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
