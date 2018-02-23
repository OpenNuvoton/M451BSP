
/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    NuEdu-SDK-M451 ACMP Sample Code.
						 Rotating the knob to change the variable resistance.
						 ACMP compare the value of variable resistance and (VCC)/2 .
						 If ACMP result is 1, the LED bar will turn on; otherwise, the LED bar will turn off.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01.h"

#define PLL_CLOCK       72000000

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
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
	
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

	  /* Init UART0 for printf */
    UART0_Init();
	
    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|    NuEdu-SDK-M451 ACMP Sample Code    |\n");
    printf("+---------------------------------------+\n");
		printf("\n");

		//Open ACMP1
		Open_ACMP();
		//Initial LED
		Initial_LED();

		while(1)
		{
			if(Get_ACMP())
				Write_LED_Bar(7);
			else
				Write_LED_Bar(0);
		}
}
/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/


