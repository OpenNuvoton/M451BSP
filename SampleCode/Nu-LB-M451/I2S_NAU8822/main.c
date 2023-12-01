/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Configure SPI1 as I2S Master mode and demonstrate how I2S works in Master mode.
 *           This sample code needs to work with I2S_Slave.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000

extern const uint16_t APU_SOURCE[12218];
uint32_t g_u32TxValue;
uint32_t g_u32DataCount;

/* Function prototype declaration */
void SYS_Init(void);
void NAU8822_Setup(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);
    /* Init UART0 to 115200-8n1 for printing messages */
    UART_Open(UART0, 115200);

    /* Init NAU8822 to turn on speaker ouput function*/
    NAU8822_Setup();

    /* Slave mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
    I2S_Open(SPI1, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(SPI1, 12000000);

    /* Start I2S play iteration */
    I2S_EnableInt(SPI1, I2S_FIFO_TXTH_INT_MASK);
    NVIC_EnableIRQ(SPI1_IRQn);
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Configure PLL */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, PLL_CLOCK);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Select PCLK1 as the clock source of SPI1 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK1, MODULE_NoMsk);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for SPI1 I2SMCLK, UART0 RXD and TXD and for I2C0 SCL and SDA*/
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_SPI1_I2SMCLK | SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD | SYS_GPD_MFPL_PD4MFP_I2C0_SDA | SYS_GPD_MFPL_PD5MFP_I2C0_SCL);

    /* Configure SPI1 related multi-function pins. */
    /* GPA[7:4] : SPI1_CLK (I2S1_BCLK), SPI1_MISO (I2S1_DI), SPI1_MOSI (I2S1_DO), SPI1_SS (I2S1_LRCLK). */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_SPI1_SS | SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA7MFP_SPI1_CLK);

}

uint32_t u32I2Sindex = 0;
void SPI1_IRQHandler()
{
    uint32_t u32I2SIntFlag;
    u32I2SIntFlag = SPI1->I2SSTS;
    if(u32I2SIntFlag & SPI_I2SSTS_TXTHIF_Msk)
    {
        /* Fill 2 word data when it is TX threshold interrupt */
        I2S_WRITE_TX_FIFO(SPI1, APU_SOURCE[u32I2Sindex++]);
        if(u32I2Sindex > 12218) u32I2Sindex = 0;
        I2S_WRITE_TX_FIFO(SPI1, APU_SOURCE[u32I2Sindex++]);
        if(u32I2Sindex > 12218) u32I2Sindex = 0;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{
    /* Send START */
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);
    if(I2C_GET_STATUS(I2C0) != 0x08)
        return 1;

    /* Send device address */
    I2C_SET_DATA(I2C0, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);
    if(I2C_GET_STATUS(I2C0) != 0x18)
        return 1;


    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);
    if(I2C_GET_STATUS(I2C0) != 0x28)
        return 1;


    /* Send data */
    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);
    if(I2C_GET_STATUS(I2C0) != 0x28)
        return 1;


    /* Send STOP */
    I2C_STOP(I2C0);
    return 0;

}

void NAU8822_Setup(void)
{
    /* Open I2C0 and set clock to 100k for NAU8822*/
    I2C_Open(I2C0, 100000);

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */

    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */

}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
