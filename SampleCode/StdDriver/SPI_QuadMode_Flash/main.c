/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Access SPI flash using SPI quad mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Basic <1=> Higher
#define SlewRateMode    0
// *** <<< end of configuration section >>> ***

#define PLL_CLOCK       72000000

#define TEST_NUMBER     1   /* page numbers */
#define TEST_LENGTH     256 /* length */

#define SPI_FLASH_PORT  SPI0

static uint8_t s_au8SrcArray[TEST_LENGTH];
static uint8_t s_au8DestArray[TEST_LENGTH];

void D2D3_SwitchToNormalMode(void);
void D2D3_SwitchToQuadMode(void);
uint32_t SpiFlash_ReadJedecID(void);
void SpiFlash_ChipErase(void);
uint8_t SpiFlash_ReadStatusReg(void);
uint8_t SpiFlash_ReadStatusReg2(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2);
int32_t SpiFlash_WaitReady(void);
void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void spiFlash_EnableQEBit(void);
void spiFlash_DisableQEBit(void);
void SpiFlash_QuadFastRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SYS_Init(void);

void D2D3_SwitchToNormalMode(void)
{
    SYS->GPB_MFPL =  SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT1, GPIO_MODE_OUTPUT);
    PB4 = 1;
    PB5 = 1;
}

void D2D3_SwitchToQuadMode(void)
{
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_SPI0_MOSI1 | SYS_GPB_MFPL_PB1MFP_SPI0_MISO1;
}

__STATIC_INLINE void wait_SPI_IS_BUSY(SPI_T *qspi)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while(SPI_IS_BUSY(qspi))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            break;
        }
    }
}

uint32_t SpiFlash_ReadJedecID(void)
{
    uint8_t u8RxData[4], u8IDCnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x9F, Read JEDEC ID
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x9F);

    // receive 32-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while(!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt++] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);

    return (uint32_t)((u8RxData[1] << 16) | (u8RxData[2] << 8) | u8RxData[3]);
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    uint8_t u8Val;

    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    u8Val = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    u8Val = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);

    return u8Val;
}

uint8_t SpiFlash_ReadStatusReg2(void)
{
    uint8_t u8Val;

    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x35, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x35);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    u8Val = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    u8Val = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);

    return u8Val;
}

void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value1);
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value2);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

int32_t SpiFlash_WaitReady(void)
{
    volatile uint8_t u8ReturnValue;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    do
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            return -1;
        }

        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    }
    while(u8ReturnValue != 0); // check the BUSY bit

    return 0;
}

void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress       & 0xFF);

    // write data
    while(1)
    {
        if(!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);
            if(u32Cnt > 255) break;
        }
    }

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void spiFlash_EnableQEBit(void)
{
    uint8_t u8Status1 = SpiFlash_ReadStatusReg();
    uint8_t u8Status2 = SpiFlash_ReadStatusReg2();

    u8Status2 |= 0x2;
    SpiFlash_WriteStatusReg(u8Status1, u8Status2);
    SpiFlash_WaitReady();
}

void spiFlash_DisableQEBit(void)
{
    uint8_t u8Status1 = SpiFlash_ReadStatusReg();
    uint8_t u8Status2 = SpiFlash_ReadStatusReg2();

    u8Status2 &= ~0x2;
    SpiFlash_WriteStatusReg(u8Status1, u8Status2);
    SpiFlash_WaitReady();
}

void SpiFlash_QuadFastRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt;

    // enable quad mode
    spiFlash_EnableQEBit();

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // Command: 0xEB, Fast Read quad data
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xEB);
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // enable SPI quad IO mode and set direction to input
    D2D3_SwitchToQuadMode();
    SPI_ENABLE_QUAD_OUTPUT_MODE(SPI_FLASH_PORT);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress       & 0xFF);

    // dummy byte
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    wait_SPI_IS_BUSY(SPI_FLASH_PORT);
    SPI_ENABLE_QUAD_INPUT_MODE(SPI_FLASH_PORT);

    // clear RX buffer
    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // read data
    for(u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
        wait_SPI_IS_BUSY(SPI_FLASH_PORT);
        u8DataBuffer[u32Cnt] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_DISABLE_QUAD_MODE(SPI_FLASH_PORT);
    D2D3_SwitchToNormalMode();

    // disable quad mode
    spiFlash_DisableQEBit();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set SPI0 multi-function pins */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk |
                       SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_SPI0_MOSI1 | SYS_GPB_MFPL_PB1MFP_SPI0_MISO1 | SYS_GPB_MFPL_PB2MFP_SPI0_CLK |
                      SYS_GPB_MFPL_PB3MFP_SPI0_MISO0 | SYS_GPB_MFPL_PB4MFP_SPI0_SS | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0);

#if (SlewRateMode == 0)
    /* Enable SPI0 I/O basic slew rate */
    PB->SLEWCTL &= ~(GPIO_SLEWCTL_HSREN0_Msk | GPIO_SLEWCTL_HSREN1_Msk | GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk |
                     GPIO_SLEWCTL_HSREN4_Msk | GPIO_SLEWCTL_HSREN5_Msk);
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O higher slew rate */
    PB->SLEWCTL |= (GPIO_SLEWCTL_HSREN0_Msk | GPIO_SLEWCTL_HSREN1_Msk | GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk |
                    GPIO_SLEWCTL_HSREN4_Msk | GPIO_SLEWCTL_HSREN5_Msk);
#endif
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint32_t u32ID;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    D2D3_SwitchToNormalMode();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);

    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_FLASH_PORT);

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                  SPI Quad Mode with Flash Sample Code                  |\n");
    printf("+------------------------------------------------------------------------+\n");

    u32ID = SpiFlash_ReadJedecID();

    if(u32ID == 0xEF4014)
        printf("Flash found: W25Q80 ...\n");
    else if(u32ID == 0xEF4015)
        printf("Flash found: W25Q16 ...\n");
    else if(u32ID == 0xEF4016)
        printf("Flash found: W25Q32 ...\n");
    else if(u32ID == 0xEF4017)
        printf("Flash found: W25Q64 ...\n");
    else if(u32ID == 0xEF4018)
        printf("Flash found: W25Q128 ...\n");
    else
    {
        printf("Wrong ID, 0x%X\n", u32ID);
        goto lexit;
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    if(SpiFlash_WaitReady() < 0) goto lexit;

    printf("[OK]\n");

    /* init source data buffer */
    for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8SrcArray[u32ByteCount] = (uint8_t)u32ByteCount;
    }

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, s_au8SrcArray);
        if(SpiFlash_WaitReady() < 0) goto lexit;
        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8DestArray[u32ByteCount] = 0;
    }

    printf("Quad Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        SpiFlash_QuadFastRead(u32FlashAddress, s_au8DestArray);
        u32FlashAddress += 0x100;

        for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
        {
            if(s_au8DestArray[u32ByteCount] != s_au8SrcArray[u32ByteCount])
                u32Error++;
        }
    }

    if(u32Error == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

lexit:

    while(1);
}
