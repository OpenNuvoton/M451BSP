/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to set GPIO pin mode and use pin data input/output control.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"
#include "NuEdu-Basic01.h"

#define PLL_CLOCK       72000000

uint32_t u32LEDEanble;

void IrDA_Code_Exe(uint8_t* IR_CODE1)
{
    if((IR_CODE1[0] == 0x00) & (IR_CODE1[1] == 0xFF))
    {
        if((IR_CODE1[2] == 0x10) & (IR_CODE1[3] == 0xEF))
        {
            Write_LED_Bar(++u32LEDEanble);
        }
        else if((IR_CODE1[2] == 0x14) & (IR_CODE1[3] == 0xEB))
        {
            Write_LED_Bar(--u32LEDEanble);
        }
    }
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

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Get Core Clock Frequency      */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.6) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);


}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t au8IR_CODE[4];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    IrDA_NEC_TxRx_Init();
    printf("+-----------------------------------------+\n");
    printf("|             IrDA NEC Sample Code      |\n");
    printf("+-----------------------------------------+\n");

    au8IR_CODE[0] = 0x00;
    au8IR_CODE[1] = ~au8IR_CODE[0];

    while(1)
    {
        /* Detect Key status */
        if(Get_Key_Input() == 0x01)
        {
            au8IR_CODE[2] = 0x10;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
        if(Get_Key_Input() == 0x02)
        {
            au8IR_CODE[2] = 0x14;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
    }

}
