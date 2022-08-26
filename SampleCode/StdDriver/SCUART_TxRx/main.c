/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show smartcard UART mode by connecting PA.0 and PA.1 pins.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

uint8_t au8TxBuf[] = "Hello World!";
#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    UART_WRITE(UART0, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.

    return;
}

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

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable SC0 module clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Select SC0 module clock source */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PLL, CLK_CLKDIV1_SC0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set GPA multi-function pins for SC UART mode */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK | SYS_GPA_MFPL_PA1MFP_SC0_DAT);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("This sample code demos smartcard interface UART mode\n");
    printf("Please connect SC0 CLK pin(PA.0) with SC0 I/O pin(PA.1)\n");
    printf("Hit any key to continue\n");
    getchar();

    // Open smartcard interface 0 in UART mode.
    SCUART_Open(SC0, 115200);
    // Enable receive interrupt
    SCUART_ENABLE_INT(SC0, SC_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /* Write data to SC UART interface */
    SCUART_Write(SC0, au8TxBuf, sizeof(au8TxBuf));

    while(1);
}
