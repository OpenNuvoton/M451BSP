/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to update chip flash data through RS485 interface
 *           between chip RS485 and ISP Tool.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip RS485 and assign update file
 *           of Flash.
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "uart_transfer.h"


#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HIRC
#define PLL_CLOCK       71884880

#define nRTSPin                 (PB8)
#define REVEIVE_MODE            (0)
#define TRANSMIT_MODE           (1)

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    
    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
    
    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    
    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()
    
    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;
    
    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    
    /* Set PB multi-function pins for UART1 RXD(PB.2), TXD(PB.3) and nRTS(PB.8) */
    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE8_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE8_Pos);
    nRTSPin = REVEIVE_MODE;       
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    
    /* Init UART to 115200-8n1 */    
    UART_Init();
    
    /* Enable FMC ISP */    
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;	
    
    /* Get APROM size, data flash size and address */      
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
        
    /* Set Systick time-out for 300ms */         
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;   /* Use CPU clock */

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (1) {
        
        /* Wait for CMD_CONNECT command */ 
        if ((bufhead >= 4) || (bUartDataReady == TRUE)) {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT) {
                break;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        /* Systick time-out, then go to APROM */
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

    /* Prase command from master and send response back */      
    while (1) {
        
        if (bUartDataReady == TRUE) {            
            bUartDataReady = FALSE;;        /* Reset UART data ready flag */     
            ParseCmd(uart_rcvbuf, 64);      /* Parse command from master */  
            NVIC_DisableIRQ(UART1_IRQn);   /* Disable NVIC */
            nRTSPin = TRANSMIT_MODE;        /* Control RTS in transmit mode */
            PutString();                    /* Send response to master */

            /* Wait for data transmission is finished */
            while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);  

            nRTSPin = REVEIVE_MODE;         /* Control RTS in reveive mode */
            NVIC_EnableIRQ(UART1_IRQn);    /* Enable NVIC */      
        }
        
    }

_APROM:
    
    /* Reset system and boot from APROM */    
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); /* Clear reset status flag */
    FMC->ISPCTL &=  ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
