/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/09/02 10:03a $
 * @brief    NuEdu Basic01 UART printf Sample Code
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>

#include "M451Series.h"
#include "NuEdu-Basic01.h"

void SendChar_ToUART(int ch);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple printf() function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void printf_UART(uint8_t *str, ...);
void printInteger(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15, j;

    *(print_buf + i) = '\0';
    j = u32Temp >> 31;
    if(j)
        u32Temp = ~u32Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + u32Temp % 10;
        u32Temp = u32Temp / 10;
    }
    while(u32Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    printf_UART(print_buf + i);
}
void printHex(uint32_t u32Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15;
    uint32_t temp;

    *(print_buf + i) = '\0';
    do
    {
        i--;
        temp = u32Temp % 16;
        if(temp < 10)
            *(print_buf + i) = '0' + temp;
        else
            *(print_buf + i) = 'a' + (temp - 10) ;
        u32Temp = u32Temp / 16;
    }
    while(u32Temp != 0);
    printf_UART(print_buf + i);
}
void printf_UART(uint8_t *str, ...)
{
    va_list args;
    va_start(args, str);
    while(*str != '\0')
    {
        if(*str == '%')
        {
            str++;
            if(*str == '\0') return;
            if(*str == 'd')
            {
                str++;
                printInteger(va_arg(args, int));
            }
            else if(*str == 'x')
            {
                str++;
                printHex(va_arg(args, int));
            }
        }
        SendChar_ToUART(*str++);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main()
{
    uint32_t u32Key, i = 0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Key and LED GPIO type */
    GPIO_SetMode(PD, BIT3, GPIO_MODE_INPUT);
    Initial_Key_Input();
    Initial_LED();

    /* Init UART to 115200-8n1 for print message */

    UART_Open(UART0, 115200);

    printf("+-----------------------------------------+\n");
    printf("|    M451 UART Sample Code      |\n");
    printf("+-----------------------------------------+\n");

    while(1)
    {
        /* Detect Key status */
        u32Key = Get_Key_Input();
        if(PD3 == 0)
        {
            LED_On(i);
            printf("+----------------------------------+\n");
            printf("|    Standare printf function:%d   |\n", i++);
            printf("+----------------------------------+\n");
        }
        if(u32Key == 0x1)
        {
            LED_On(i);
            printf("+------------------------------+\n");
            printf("|  Simple printf function:%d   |\n", i--);
            printf("+------------------------------+\n");
        }
    }
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
