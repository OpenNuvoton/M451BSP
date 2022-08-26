/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 11 $
 * $Date: 15/09/02 10:04a $
 * @brief
 *           Implement SPI Master loop back transfer.
 *           This sample code needs to connect SPI0_MISO0 pin and SPI0_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define TEST_COUNT             64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err, u32TimeOutCnt;

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
    printf("+------------------------------------------------------------------+\n");
    printf("|                   M451 SPI Driver Sample Code                    |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates SPI0 self loop back data transfer.\n");
    printf(" SPI0 configuration:\n");
    printf("     Master mode; data width 32 bits.\n");
    printf(" I/O connection:\n");
    printf("     PB.5 SPI0_MOSI0 <--> PB.3 SPI0_MISO0 \n");

    printf("\nSPI0 Loopback test ");

    /* set the source data and clear the destination buffer */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount]++;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            /* Write to TX register */
            SPI_WRITE_TX(SPI0, g_au32SourceData[u32DataCount]);
            /* Check SPI0 busy status */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(SPI_IS_BUSY(SPI0))
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
            g_au32DestinationData[u32DataCount] = SPI_READ_RX(SPI0);
            u32DataCount++;
            if(u32DataCount == TEST_COUNT)
                break;
        }

        if(u32Err)
            break;

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if(g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Close SPI0 */
    SPI_Close(SPI0);

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

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);

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
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_SPI0_CLK | SYS_GPB_MFPL_PB3MFP_SPI0_MISO0 | SYS_GPB_MFPL_PB4MFP_SPI0_SS | SYS_GPB_MFPL_PB5MFP_SPI0_MOSI0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

