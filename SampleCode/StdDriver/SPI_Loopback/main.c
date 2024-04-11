/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate one SPI Master self-loopback transfer and two SPI 4-wire/3-wire loopback transfer.
 *           SPI1 will be configured as Master mode and SPI0 will be configured as Slave mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <e> Two SPI port loopback transfer
#define TwoPortLoopback     0
//  <o> Bi-direction Interface
//  <0=> 4-wire <1=> 3-wire
#define Slave3WireMode      0
// </e>
// <o> GPIO Slew Rate Control
// <0=> Basic <1=> Higher
#define SlewRateMode        0
// *** <<< end of configuration section >>> ***

#define PLL_CLOCK           72000000

#define TEST_COUNT          64

/* Global variable declaration */
#if (!TwoPortLoopback)
static uint32_t s_au32SourceData[TEST_COUNT];
static uint32_t s_au32DestinationData[TEST_COUNT];
#else
static uint32_t s_au32MasterToSlaveTestPattern[TEST_COUNT];
static uint32_t s_au32SlaveToMasterTestPattern[TEST_COUNT];
static uint32_t s_au32MasterRxBuffer[TEST_COUNT];
static uint32_t s_au32SlaveRxBuffer[TEST_COUNT];
static volatile uint32_t s_u32MasterTxDataCount, s_u32MasterRxDataCount;
static volatile uint32_t s_u32SlaveTxDataCount, s_u32SlaveRxDataCount;
#endif

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
#if (!TwoPortLoopback)
    uint32_t u32TestCount, u32Err, u32TimeOutCnt;
#endif
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                       SPI Driver Sample Code                       |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
#if (!TwoPortLoopback)
    printf("This sample code demonstrates SPI1 self loop back data transfer.\n");
    printf(" SPI1 configuration:\n");
    printf("     Master mode; data width 32 bits.\n");
    printf(" I/O connection:\n");
    printf("     SPI1_MOSI(PA5) <--> SPI1_MISO(PA6)\n");
    printf("\nSPI1 Loopback test ");

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* Set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            s_au32SourceData[u32DataCount] = u32DataCount;
            s_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            /* Write to TX register */
            SPI_WRITE_TX(SPI1, s_au32SourceData[u32DataCount]);

            /* Check SPI1 busy status */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(SPI_IS_BUSY(SPI1))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for SPI busy flag is cleared time-out!\n");
                    u32Err = 1;
                    break;
                }
            }

            if(u32Err)
                break;

            /* Read received data */
            s_au32DestinationData[u32DataCount] = SPI_READ_RX(SPI1);
            u32DataCount++;
            if(u32DataCount == TEST_COUNT)
                break;
        }

        if(u32Err)
            break;

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if(s_au32DestinationData[u32DataCount] != s_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Reset SPI1 */
    SPI_Close(SPI1);
#else
#if (Slave3WireMode)
    printf("This sample code demonstrates Slave 3-wire mode loop back transfer.\n\n");
#else
    printf("This sample code demonstrates SPI0/SPI1 loop back transfer.\n\n");
#endif
    printf("Configure SPI0 as a slave and SPI1 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("Please connect below I/O connections for SPI0 and SPI1:\n");
#if (!Slave3WireMode)
    printf("    SPI0_SS   (PB4)   <->   SPI1_SS  (PA4)\n");
#endif
    printf("    SPI0_CLK  (PB2)   <->   SPI1_CLK (PA7)\n");
    printf("    SPI0_MISO0(PB3)   <->   SPI1_MISO(PA6)\n");
    printf("    SPI0_MOSI0(PB5)   <->   SPI1_MOSI(PA5)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        s_au32MasterToSlaveTestPattern[u32DataCount] = 0x00550000 + u32DataCount;
        s_au32SlaveToMasterTestPattern[u32DataCount] = 0x00AA0000 + u32DataCount;
        /* Clear destination buffer */
        s_au32MasterRxBuffer[u32DataCount] = 0;
        s_au32SlaveRxBuffer[u32DataCount] = 0;
    }

    s_u32MasterTxDataCount = 0;
    s_u32MasterRxDataCount = 0;
    s_u32SlaveTxDataCount = 0;
    s_u32SlaveRxDataCount = 0;
    printf("Press any key to start transmission ...\n");
    getchar();
    printf("\n");

    /* Access TX and RX FIFO */
    while((s_u32MasterRxDataCount < TEST_COUNT) || (s_u32SlaveRxDataCount < TEST_COUNT))
    {
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (s_u32MasterTxDataCount < TEST_COUNT))
            SPI_WRITE_TX(SPI1, s_au32MasterToSlaveTestPattern[s_u32MasterTxDataCount++]); /* Write to TX FIFO */
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (s_u32SlaveTxDataCount < TEST_COUNT))
            SPI_WRITE_TX(SPI0, s_au32SlaveToMasterTestPattern[s_u32SlaveTxDataCount++]); /* Write to TX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
            s_au32MasterRxBuffer[s_u32MasterRxDataCount++] = SPI_READ_RX(SPI1); /* Read RX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
            s_au32SlaveRxBuffer[s_u32SlaveRxDataCount++] = SPI_READ_RX(SPI0); /* Read RX FIFO */
    }

    /* Print the received data */
    printf("\tSPI0 Received data:\tSPI1 Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\t\t0x%X\n", u32DataCount, s_au32SlaveRxBuffer[u32DataCount], s_au32MasterRxBuffer[u32DataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset SPI0 */
    SPI_Close(SPI0);
    /* Reset SPI1 */
    SPI_Close(SPI1);
#endif

    while(1);
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

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Select PCLK1 as the clock source of SPI1 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK1, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable SPI1 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

#if (TwoPortLoopback)
    /* Set SPI0 multi-function pins */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_SPI0_CLK | SYS_GPB_MFPL_PB3MFP_SPI0_MISO0 | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0);
#if (!Slave3WireMode)
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_SPI0_SS;
#endif

#if (SlewRateMode == 0)
    /* Enable SPI0 I/O basic slew rate */
    PB->SLEWCTL &= ~(GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk | GPIO_SLEWCTL_HSREN5_Msk);
#if (!Slave3WireMode)
    PB->SLEWCTL &= ~GPIO_SLEWCTL_HSREN4_Msk;
#endif
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O higher slew rate */
    PB->SLEWCTL |= (GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk | GPIO_SLEWCTL_HSREN5_Msk);
#if (!Slave3WireMode)
    PB->SLEWCTL |= GPIO_SLEWCTL_HSREN4_Msk;
#endif
#endif
#endif

    /* Set SPI1 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA7MFP_SPI1_CLK);
#if (!Slave3WireMode)
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_SPI1_SS;
#endif

#if (SlewRateMode == 0)
    /* Enable SPI1 I/O basic slew rate */
    PA->SLEWCTL &= ~(GPIO_SLEWCTL_HSREN5_Msk | GPIO_SLEWCTL_HSREN6_Msk | GPIO_SLEWCTL_HSREN7_Msk);
#if (!Slave3WireMode)
    PA->SLEWCTL &= ~GPIO_SLEWCTL_HSREN4_Msk;
#endif
#elif (SlewRateMode == 1)
    /* Enable SPI1 I/O higher slew rate */
    PA->SLEWCTL |= (GPIO_SLEWCTL_HSREN5_Msk | GPIO_SLEWCTL_HSREN6_Msk | GPIO_SLEWCTL_HSREN7_Msk);
#if (!Slave3WireMode)
    PA->SLEWCTL |= GPIO_SLEWCTL_HSREN4_Msk;
#endif
#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI1 */
    /* Configure SPI1 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI1, SPI_SS, SPI_SS_ACTIVE_LOW);

#if (TwoPortLoopback)
    /* Configure SPI0 */
#if (Slave3WireMode)
    /* Enable slave 3-wire mode before enabling SPI controller */
    SPI_ENABLE_3WIRE_MODE(SPI0);
#endif
    /* Configure SPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI0->CTL = SPI_SLAVE | SPI_MODE_0 | SPI_CTL_SPIEN_Msk;
#endif
}
