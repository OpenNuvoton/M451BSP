/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 15/09/02 10:04a $
 * @brief    Read the smartcard ATR from smartcard 0 interface.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "sclib.h"

#define PLL_CLOCK       72000000

void _SCLIB_StartColdReset(uint32_t num);

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    // Please don't remove any of the function calls below
    if(SCLIB_CheckCDEvent(0))
        return; // Card insert/remove event occurred, no need to check other event...
    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

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

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
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

    /* Enable SC0 module clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Select SC0 module clock source */
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HXT, CLK_CLKDIV1_SC0(3));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA.0 ~ PA.3 and PB.2 for SC0 interface */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk |
                       SYS_GPA_MFPL_PA1MFP_Msk |
                       SYS_GPA_MFPL_PA2MFP_Msk |
                       SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB2MFP_Msk;
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK |
                      SYS_GPA_MFPL_PA1MFP_SC0_DAT |
                      SYS_GPA_MFPL_PA2MFP_SC0_RST |
                      SYS_GPA_MFPL_PA3MFP_SC0_PWR);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB2MFP_SC0_CD;

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{

    SCLIB_CARD_INFO_T s_info;
    int retval, i;

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

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                       Smart Card Driver Sample Code                    |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will get smart card ATR data via SC0 port.\n");
    printf("  I/O configuration:\n");
    printf("  SC0CLK <--> smart card slot clock pin\n");
    printf("  SC0DAT <--> smart card slot data pin\n");
    printf("  SC0PWR <--> smart card slot power pin\n");
    printf("  SC0RST <--> smart card slot reset pin\n");
    printf("  SC0CD  <--> smart card slot card detect pin\n");
    printf("\nThis sample code reads ATR from smartcard\n");

    // Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    // Wait 'til card insert
    while(SC_IsCardInserted(SC0) == FALSE);
    // Activate slot 0
    retval = SCLIB_Activate(0, FALSE);

    if(retval == SCLIB_SUCCESS)
    {
        SCLIB_GetCardInfo(0, &s_info);
        printf("\nATR: ");
        for(i = 0; i < s_info.ATR_Len; i++)
            printf("%x ", s_info.ATR_Buf[i]);
        printf("\n");
    }
    else
        printf("Smartcard activate failed\n");

    while(1);
}
