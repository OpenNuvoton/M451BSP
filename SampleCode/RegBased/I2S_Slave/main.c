/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 11 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Configure SPI1 as I2S Slave mode and demonstrate how I2S works in Slave mode.
 *           This sample code needs to work with I2S_Master sample code.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"

#define PLLCON_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

uint32_t g_u32TxValue;
uint32_t g_u32DataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32RxValue1, u32RxValue2;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for print message */
    UART0_Init();

    printf("+----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (slave mode)           |\n");
    printf("+----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0xAA00AA01, 0xAA02AA03, ..., 0xAAFEAAFF, wraparound\n");
    printf("  The I/O connection for I2S1 (SPI1):\n");
    printf("      I2S1_LRCLK (PA4)\n      I2S1_BCLK(PA7)\n");
    printf("      I2S1_DI (PA6)\n      I2S1_DO (PA5)\n\n");
    printf("  NOTE: Connect with a I2S master.\n");
    printf("        This sample code will transmit a TX value 50000 times, and then change to the next TX value.\n");
    printf("        When TX value or the received value changes, the new TX value or the current TX value and the new received value will be printed.\n");
    printf("  Press any key to start ...");
    getchar();
    printf("\n");

    /* Slave mode, 16-bit word width, stereo mode, I2S format. */
    SPI1->I2SCTL = I2S_MODE_SLAVE | I2S_DATABIT_16 | I2S_STEREO | I2S_FORMAT_I2S;
    /* Set TX and RX FIFO threshold to middle value */
    SPI1->FIFOCTL = I2S_FIFO_TX_LEVEL_WORD_2 | I2S_FIFO_RX_LEVEL_WORD_2;

    /* I2S peripheral clock rate is equal to PCLK1 clock rate. */
    SPI1->I2SCLK = 0;

    /* Enable I2S */
    SPI1->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;
    NVIC_EnableIRQ(SPI1_IRQn);

    /* Initiate data counter */
    g_u32DataCount = 0;
    /* Initiate TX value and RX value */
    g_u32TxValue = 0xAA00AA01;
    u32RxValue1 = 0;
    u32RxValue2 = 0;
    /* Enable TX threshold level interrupt */
    SPI1->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;

    /* Enable I2S TX function to transmit data */
    SPI1->I2SCTL |= SPI_I2SCTL_TXEN_Msk;
    /* Enable I2S RX function to receive data */
    SPI1->I2SCTL |= SPI_I2SCTL_RXEN_Msk;

    printf("Start I2S ...\nTX value: 0x%X\n", g_u32TxValue);

    while(1)
    {
        /* Check RX FIFO empty flag */
        if((SPI1->I2SSTS & SPI_I2SSTS_RXEMPTY_Msk) == 0)
        {
            /* Read RX FIFO */
            u32RxValue2 = SPI1->RX;
            /* If received value changes, print the current TX value and the new received value. */
            if(u32RxValue1 != u32RxValue2)
            {
                u32RxValue1 = u32RxValue2;
                printf("TX value: 0x%X;  RX value: 0x%X\n", g_u32TxValue, u32RxValue1);
            }
        }
        if(g_u32DataCount >= 50000)
        {
            g_u32TxValue = 0xAA00AA00 | ((g_u32TxValue + 0x00020002) & 0x00FF00FF); /* g_u32TxValue: 0xAA00AA01, 0xAA02AA03, ..., 0xAAFEAAFF */
            printf("TX value: 0x%X\n", g_u32TxValue);
            g_u32DataCount = 0;
        }
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Select HXT as the clock source of UART */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HXT;
    /* Select PCLK1 as the clock source of SPI1 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK1;

    /* Enable peripheral clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI1CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Configure SPI1 related multi-function pins. */
    /* GPA[7:4] : SPI1_CLK (I2S1_BCLK), SPI1_MISO (I2S1_DI), SPI1_MOSI (I2S1_DO), SPI1_SS (I2S1_LRCLK). */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_SPI1_SS | SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA7MFP_SPI1_CLK);

}

void UART0_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
}

void SPI1_IRQHandler()
{
    /* Write 2 TX values to TX FIFO */
    SPI1->TX = g_u32TxValue;
    SPI1->TX = g_u32TxValue;
    g_u32DataCount += 2;
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
