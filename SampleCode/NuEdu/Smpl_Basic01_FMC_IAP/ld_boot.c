/**************************************************************************//**
 * @file     ld_boot.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/09/10 6:02p $
 * @brief    FMC VECMAP sample program (LDROM code) for M451 series MCU
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include "M451Series.h"
#include "map.h"
#include "NuEdu-Basic01.h"
#define PLL_CLOCK           72000000

#ifdef __ARMCC_VERSION
__asm __INLINE __set_SP(uint32_t _sp)
{
    MSR MSP, r0
    BX lr
}
#endif

void SendChar_ToUART(int ch);
/*---------------------------------------------------------------------------------------------------------*/
/*  Simple printf() function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void printInteger(uint32_t u32Temp);
void printHex(uint32_t u32Temp);
void printf_UART(uint8_t *str,...)
{
		va_list args;
		va_start( args, str );
    while (*str != '\0')
    {
			if(*str == '%')
			{
				str++;
				if (*str == '\0') return;
				if( *str == 'd' )
				{
					str++;
					printInteger(va_arg( args, int ));
				}
				if( *str == 'x' )
				{
					str++;
					printHex(va_arg( args, int ));
				}
			}
        SendChar_ToUART(*str++);
    }
}

void printInteger(uint32_t u32Temp)
{
	uint8_t print_buf[16];
	uint32_t i=15;
	
	*(print_buf+i) = '\0';
		do
    {
			i--;
      *(print_buf+i) = '0'+u32Temp%10;
			u32Temp = u32Temp /10;
    }while (u32Temp != 0);
		printf_UART(print_buf+i);
}

void printHex(uint32_t u32Temp)
{
	uint8_t print_buf[16];
	uint32_t i=15;
	uint8_t hextemp;
	
	*(print_buf+i) = '\0';
		do
    {
			i--;
			hextemp = u32Temp%16;
			if(hextemp > 10)
				*(print_buf+i) = 'A'+(hextemp - 10);
      else
				*(print_buf+i) = '0'+hextemp;
			u32Temp = u32Temp /16;
    }while (u32Temp != 0);
		printf_UART(print_buf+i);
}

__INLINE void BranchTo(uint32_t u32Address)
{
    FUNC_PTR        *func;    
    FMC_SetVectorPageAddr(u32Address);
    func =  (FUNC_PTR *)(*(uint32_t *)(u32Address+4));
    printf_UART("branch to address 0x%x\n", (int)func);
    printf_UART("\nChange VECMAP and branch to user application...\n");
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));
    __set_SP(*(uint32_t *)u32Address);
    func();		    
}
void SYS_Init(void)
{
   /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();
	
    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
		  
    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

		/* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
		
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD(PD.6) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
		
    /* Lock protected registers */
    SYS_LockReg();
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
int32_t main (void)
{
    int             cbs;
    uint32_t        au32Config[2];
    uint32_t        au32Version[2];

    volatile int    loop;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    FMC_ReadConfig(au32Config, 2);
    cbs = (au32Config[0] >> 6) & 0x3;
    printf_UART("Config0=0x%x, Config1=0x%x, CBS=%d\n", au32Config[0], au32Config[1], cbs);
    printf_UART("Boot loader code on LDROM\n");

    au32Version[0] = FMC_Read(USER_AP0_ENTRY+0x1000);
    au32Version[1] = FMC_Read(USER_AP1_ENTRY+0x1000);

    printf_UART("Version for AP0:0x%x, AP1:0x%x\n",au32Version[0],au32Version[1]);
//     printf_UART("Version for AP1:0x%x\n",au32Version[1]);	
//     printf_UART("Boot Selection\n");
    if((au32Version[0]>=au32Version[1])&(au32Version[0]!=0xFFFFFFFF))
    {
        printf_UART("Jump to AP0\n");	
        BranchTo(USER_AP0_ENTRY);
    }else if(au32Version[1]!=0xFFFFFFFF)
    {
        printf_UART("Jump to AP1\n");	
        BranchTo(USER_AP1_ENTRY);
    }    
    if((au32Version[0]<=au32Version[1])&(au32Version[1]!=0xFFFFFFFF))
    {
        printf_UART("Jump to AP1\n");	
        BranchTo(USER_AP1_ENTRY);
    }else if(au32Version[0]!=0xFFFFFFFF)
    {
        printf_UART("Jump to AP0\n");	
        BranchTo(USER_AP0_ENTRY);
    } 
    printf_UART("No program on APROM\n");    
    while(1);
}
