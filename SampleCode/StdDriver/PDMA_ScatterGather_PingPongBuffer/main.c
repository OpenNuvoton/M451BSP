/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 15/09/02 10:04a $
 * @brief
 *           Use PDMA to implement Ping-Pong buffer by scatter-gather mode(memory to memory).
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000

uint32_t PDMA_TEST_COUNT = 50;
uint32_t g_au32SrcArray0[1] = {0x55555555};
uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
uint32_t g_au32DestArray[1];
uint32_t volatile g_u32IsTestOver = 0;
uint32_t volatile g_u32TransferredCount = 0;
uint32_t g_u32DMAConfig = 0;
static uint32_t s_u32TableIndex = 0;

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[2];

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M451series.s.
 */
void PDMA_IRQHandler(void)
{
    /* Check channel transfer done status */
    //if (PDMA->TDSTS == PDMA_TDSTS_TDIF5_Msk)
    {
        /* Reload PDMA description table configuration */
        DMA_DESC[s_u32TableIndex].ctl = g_u32DMAConfig;
        s_u32TableIndex ^= 1;
        g_u32TransferredCount++;

        if (g_u32TransferredCount >= PDMA_TEST_COUNT)
        {
            /* Set PDMA into stop mode by description table */
            DMA_DESC[0].ctl &= ~PDMA_DSCT_CTL_OPMODE_Msk;
            DMA_DESC[1].ctl &= ~PDMA_DSCT_CTL_OPMODE_Msk;
            g_u32IsTestOver = 1;
            PDMA_CLR_EMPTY_FLAG(PDMA_SCATSTS_TEMPTYF5_Msk);
        }
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF5_Msk);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
}

void UART0_Init(void)
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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    M451 PDMA Driver Ping-Pong Buffer Sample Code (Scatter-gather)     | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    /* Scatter-Gather description table configuration in SRAM */
    g_u32DMAConfig = (0 << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_BURST_1 | PDMA_REQ_BURST | PDMA_OP_SCATTER;
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/
    /* Note: PDMA_REQ_BURST is only supported in memory-to-memory transfer mode.
             PDMA transfer type should be set as PDMA_REQ_SINGLE in memory-to-peripheral and peripheral-to-memory transfer mode,
             then above code will be modified as follows:
             g_u32DMAConfig = (0 << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_FIX | PDMA_BURST_1 | PDMA_REQ_SINGLE | PDMA_OP_SCATTER; */
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/

    /* Description table 1 */
    DMA_DESC[0].ctl = g_u32DMAConfig;
    DMA_DESC[0].src = (uint32_t)g_au32SrcArray0; /* Ping-Pong buffer 1 */
    DMA_DESC[0].dest = (uint32_t)&g_au32DestArray[0];
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next operation table is table 2 */

    /* Description table 2 */
    DMA_DESC[1].ctl = g_u32DMAConfig;
    DMA_DESC[1].src = (uint32_t)g_au32SrcArray1; /* Ping-Pong buffer 2 */
    DMA_DESC[1].dest = (uint32_t)&g_au32DestArray[0];
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA); /* next operation table is table 1 */

    /* The operation of the two description table will transfer data to the same destination buffer form different source in sequence.
       And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until PDMA configuration doesn't be reloaded. */

    /* Open Channel 5 */
    PDMA_Open(1 << 5);

    /* Enable Scatter Gather mode and set the description table address in SRAM */
    PDMA_SetTransferMode(5, PDMA_MEM, TRUE, (uint32_t)&DMA_DESC[0]);

    /* Enable transfer done interrupt */
    PDMA_EnableInt(5, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
    g_u32IsTestOver = 0;

    /* Start PDMA operatin */
    PDMA_Trigger(5);

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            break;
        }
    }

    if(g_u32IsTestOver == 1)
    {
        g_u32IsTestOver = 0;
        printf("test done...\n");
    }

    /* Close PDMA channel */
    PDMA_Close();

    while(1);
}
