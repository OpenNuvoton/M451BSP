/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 10 $
 * $Date: 15/09/02 10:04a $
 * @brief
 *           Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK Only)
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000

#define APROM_TEST_BASE             0x3000
#define DATA_FLASH_TEST_BASE        0x10000
#define DATA_FLASH_TEST_END         0x20000
#define TEST_PATTERN                0x5A5A5A5A



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}


int main()
{
    uint32_t u32Data, u32RData;
    uint32_t u32Addr;
    uint32_t i;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Initial UART */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
       By setting scatter loading file (scatter.scf),
       RO code is placed to 0x10000000 ~ 0x10003fff with RW is placed to 0x20004000 ~ 0x20007fff.
    */

    /* Enable FMC ISP functions */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* The ROM address for erase/write/read demo */
    u32Addr = 0x4000;
    FMC_Erase(u32Addr); /* Erase page */
    for(i = 0; i < 0x100; i += 4)
    {

        /* Write Demo */
        u32Data = i + 0x12345678;
        FMC_Write(u32Addr + i, u32Data);

        if((i & 0xf) == 0)
            printf(".");

        /* Read Demo */
        u32RData = FMC_Read(u32Addr + i);

        if(u32Data != u32RData)
        {
            printf("[Read/Write FAIL]\n");
            break;
        }
    }
    /* Disable FMC ISP function */
    FMC->ISPCTL &=  ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
