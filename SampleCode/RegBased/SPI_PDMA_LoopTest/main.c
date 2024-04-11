/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate SPI data transfer with PDMA.
 *           SPI0 will be configured as Master mode and SPI1 will be configured as Slave mode.
 *           Both TX PDMA function and RX PDMA function will be enabled.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Basic <1=> Higher
#define SlewRateMode    0
// *** <<< end of configuration section >>> ***

#define SPI_MASTER_TX_DMA_CH 0
#define SPI_MASTER_RX_DMA_CH 1
#define SPI_SLAVE_TX_DMA_CH  2
#define SPI_SLAVE_RX_DMA_CH  3

#define TEST_COUNT      64

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void SPI_Init(void);
void SpiLoopTest_WithPDMA(void);

/* Global variable declaration */
uint32_t g_au32MasterToSlaveTestPattern[TEST_COUNT];
uint32_t g_au32SlaveToMasterTestPattern[TEST_COUNT];
uint32_t g_au32MasterRxBuffer[TEST_COUNT];
uint32_t g_au32SlaveRxBuffer[TEST_COUNT];

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  SPI + PDMA Sample Code                      |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0/SPI1 loopback:\n");
    printf("    SPI0_SS  (PB4) <--> SPI1_SS(PA4)\n    SPI0_CLK(PB2)  <--> SPI1_CLK(PA7)\n");
    printf("    SPI0_MISO(PB3) <--> SPI1_MISO(PA6)\n    SPI0_MOSI(PB5) <--> SPI1_MOSI(PA5)\n\n");
    printf("Please connect SPI0 with SPI1, and press any key to start transmission ...");
    getchar();
    printf("\n");

    SpiLoopTest_WithPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 and SPI1 peripheral clock */
    CLK->APBCLK0 &= (~(CLK_APBCLK0_SPI0CKEN_Msk | CLK_APBCLK0_SPI1CKEN_Msk));
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Select HXT as the clock source of HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Select HXT as the clock source of UART module */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;
    /* Select PCLK as the clock source of SPI0 and SPI1 */
    CLK->CLKSEL2 &= ~(CLK_CLKSEL2_SPI0SEL_Msk | CLK_CLKSEL2_SPI1SEL_Msk);
    CLK->CLKSEL2 |= (CLK_CLKSEL2_SPI0SEL_PCLK0 | CLK_CLKSEL2_SPI1SEL_PCLK1);

    /* Enable UART0, SPI0 and SPI1 peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI0CKEN_Msk | CLK_APBCLK0_SPI1CKEN_Msk);
    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Configure SPI0 related multi-function pins. GPB[5:2] : SPI0_MOSI0, SPI0_SS, SPI0_MISO0, SPI0_CLK. */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_SPI0_CLK | SYS_GPB_MFPL_PB3MFP_SPI0_MISO0 | SYS_GPB_MFPL_PB4MFP_SPI0_SS | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0);

    /* Configure SPI1 related multi-function pins. GPA[7:4] : SPI1_CLK, SPI1_MISO, SPI1_MOSI, SPI1_SS. */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_SPI1_SS | SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA7MFP_SPI1_CLK);

#if (SlewRateMode == 0)
    /* Enable SPI0 I/O basic slew rate */
    PB->SLEWCTL &= ~(GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk | GPIO_SLEWCTL_HSREN4_Msk | GPIO_SLEWCTL_HSREN5_Msk);

    /* Enable SPI1 I/O basic slew rate */
    PA->SLEWCTL &= ~(GPIO_SLEWCTL_HSREN4_Msk | GPIO_SLEWCTL_HSREN5_Msk | GPIO_SLEWCTL_HSREN6_Msk | GPIO_SLEWCTL_HSREN7_Msk);
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O higher slew rate */
    PB->SLEWCTL |= (GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk | GPIO_SLEWCTL_HSREN4_Msk | GPIO_SLEWCTL_HSREN5_Msk);

    /* Enable SPI1 I/O higher slew rate */
    PA->SLEWCTL |= (GPIO_SLEWCTL_HSREN4_Msk | GPIO_SLEWCTL_HSREN5_Msk | GPIO_SLEWCTL_HSREN6_Msk | GPIO_SLEWCTL_HSREN7_Msk);
#endif
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 */
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;
    /* Set SPI0 clock divider. SPI clock rate = PCLK / (9+1) */
    SPI0->CLKDIV = (SPI0->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (9 << SPI_CLKDIV_DIVIDER_Pos);
    /* Configure SPI0 as a master, clock idle low, TX on falling clock edge, RX on rising edge and 32-bit transaction. */
    SPI0->CTL = SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;

    /* Configure SPI1 */
    /* Configure SPI1's slave select signal as a low level active device. */
    SPI1->SSCTL = 0;
    /* Set SPI1 clock divider. SPI clock rate = PCLK / (1+1) */
    SPI1->CLKDIV = (SPI1->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (1 << SPI_CLKDIV_DIVIDER_Pos);
    /* Configure SPI1 as a slave, clock idle low, TX on falling clock edge, RX on rising edge and 32-bit transaction */
    SPI1->CTL = SPI_CTL_SLAVE_Msk | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
}

void SpiLoopTest_WithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;


    printf("\nSPI0/1 Loop test with PDMA ");

    /* Source data initiation */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = 0x55000000 | (u32DataCount + 1);
        g_au32SlaveToMasterTestPattern[u32DataCount] = 0xAA000000 | (u32DataCount + 1);
    }

    /* SPI master PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << SPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128   | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].SA = (uint32_t)g_au32MasterToSlaveTestPattern;
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].DA = (uint32_t)&SPI0->TX;
#if(SPI_MASTER_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x1Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                      ((0 + 1) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#elif(SPI_MASTER_TX_DMA_CH<=7)
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & (~(0x1Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                      ((0 + 1) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#else
    PDMA->REQSEL8_11 = (PDMA->REQSEL8_11 & (~(0x1Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                       ((0 + 1) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#endif

    /* SPI master PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << SPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].SA = (uint32_t)&SPI0->RX;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].DA = (uint32_t)g_au32MasterRxBuffer;
#if(SPI_MASTER_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x1Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                      ((0 + 0x11) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#elif(SPI_MASTER_RX_DMA_CH<=7)
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & (~(0x1Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                      ((0 + 0x11) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#else
    PDMA->REQSEL8_11 = (PDMA->REQSEL8_11 & (~(0x1Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                       ((0 + 0x11) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#endif

    /* SPI slave PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << SPI_SLAVE_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].SA = (uint32_t)&SPI1->RX;
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].DA = (uint32_t)g_au32SlaveRxBuffer;
#if(SPI_SLAVE_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x1Ful << (8 * (SPI_SLAVE_RX_DMA_CH % 4))))) |
                      ((1 + 0x11) << (8 * (SPI_SLAVE_RX_DMA_CH % 4))); /* SPIx_RX */
#elif(SPI_SLAVE_RX_DMA_CH<=7)
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & (~(0x1Ful << (8 * (SPI_SLAVE_RX_DMA_CH % 4))))) |
                      ((1 + 0x11) << (8 * (SPI_SLAVE_RX_DMA_CH % 4))); /* SPIx_RX */
#else
    PDMA->REQSEL8_11 = (PDMA->REQSEL8_11 & (~(0x1Ful << (8 * (SPI_SLAVE_RX_DMA_CH % 4))))) |
                       ((1 + 0x11) << (8 * (SPI_SLAVE_RX_DMA_CH % 4))); /* SPIx_RX */
#endif

    /* SPI slave PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << SPI_SLAVE_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].SA = (uint32_t)g_au32SlaveToMasterTestPattern;
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].DA = (uint32_t)&SPI1->TX;
#if(SPI_SLAVE_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x1Ful << (8 * (SPI_SLAVE_TX_DMA_CH % 4))))) |
                      ((1 + 1) << (8 * (SPI_SLAVE_TX_DMA_CH % 4))); /* SPIx_TX */
#elif(SPI_SLAVE_TX_DMA_CH<=7)
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & (~(0x1Ful << (8 * (SPI_SLAVE_TX_DMA_CH % 4))))) |
                      ((1 + 1) << (8 * (SPI_SLAVE_TX_DMA_CH % 4))); /* SPIx_TX */
#else
    PDMA->REQSEL8_11 = (PDMA->REQSEL8_11 & (~(0x1Ful << (8 * (SPI_SLAVE_TX_DMA_CH % 4))))) |
                       ((1 + 1) << (8 * (SPI_SLAVE_TX_DMA_CH % 4))); /* SPIx_TX */
#endif

    /* Enable SPI slave DMA function */
    SPI1->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);
    /* Enable SPI master DMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);


    i32Err = 0;
    for(u32TestCycle = 0; u32TestCycle < 10000; u32TestCycle++)
    {
        if((u32TestCycle & 0x1FF) == 0)
            putchar('.');

        while(1)
        {
            u32RegValue = PDMA->INTSTS;
            /* Check the DMA transfer done interrupt flag */
            if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                if((PDMA->TDSTS & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH))) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH)))
                {
                    /* Clear the DMA transfer done flag */
                    PDMA->TDSTS = ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH));
                    /* Disable SPI master's DMA transfer function */
                    SPI0->PDMACTL = 0;
                    /* Check the transfer data */
                    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        if(g_au32MasterToSlaveTestPattern[u32DataCount] != g_au32SlaveRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                        if(g_au32SlaveToMasterTestPattern[u32DataCount] != g_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if(u32TestCycle >= 10000)
                        break;

                    /* Source data initiation */
                    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        g_au32MasterToSlaveTestPattern[u32DataCount]++;
                        g_au32SlaveToMasterTestPattern[u32DataCount]++;
                    }
                    /* Re-trigger */
                    /* Slave PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_SLAVE_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Slave PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_SLAVE_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Enable master's DMA transfer function */
                    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);
                    break;
                }
            }
            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA->ABTSTS;
                /* Clear the target abort flag */
                PDMA->ABTSTS = u32Abort;
                i32Err = 1;
                break;
            }
            /* Check the DMA time-out interrupt flag */
            if(u32RegValue & 0x000FFF00)
            {
                /* Clear the time-out flag */
                PDMA->INTSTS = u32RegValue & 0x000FFF00;
                i32Err = 1;
                break;
            }
        }

        if(i32Err)
            break;
    }

    /* Disable PDMA clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;

    if(i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}
