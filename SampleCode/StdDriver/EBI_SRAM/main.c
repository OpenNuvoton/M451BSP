/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 9 $
 * $Date: 15/09/02 10:04a $
 * @brief    Configure EBI interface to access BS616LV4017 (SRAM) on EBI interface.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int32_t SRAM_BS616LV4017(uint32_t u32MaxSize);
int32_t AccessEBIWithPDMA(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI AD0~7 pins on PA.0~7 */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk |
                       SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk |
                       SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk |
                       SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_EBI_AD0 | SYS_GPA_MFPL_PA1MFP_EBI_AD1 |
                     SYS_GPA_MFPL_PA2MFP_EBI_AD2 | SYS_GPA_MFPL_PA3MFP_EBI_AD3 |
                     SYS_GPA_MFPL_PA4MFP_EBI_AD4 | SYS_GPA_MFPL_PA5MFP_EBI_AD5 |
                     SYS_GPA_MFPL_PA6MFP_EBI_AD6 | SYS_GPA_MFPL_PA7MFP_EBI_AD7;

    /* EBI AD8~15 pins on PC.0~7 */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk |
                       SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk |
                       SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk |
                       SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_EBI_AD8 | SYS_GPC_MFPL_PC1MFP_EBI_AD9 |
                     SYS_GPC_MFPL_PC2MFP_EBI_AD10 | SYS_GPC_MFPL_PC3MFP_EBI_AD11 |
                     SYS_GPC_MFPL_PC4MFP_EBI_AD12 | SYS_GPC_MFPL_PC5MFP_EBI_AD13 |
                     SYS_GPC_MFPL_PC6MFP_EBI_AD14 | SYS_GPC_MFPL_PC7MFP_EBI_AD15;

    /* EBI AD16~19 pins on PD.12~15*/
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk |
                       SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_EBI_ADR16 | SYS_GPD_MFPH_PD13MFP_EBI_ADR17 |
                     SYS_GPD_MFPH_PD14MFP_EBI_ADR18 | SYS_GPD_MFPH_PD15MFP_EBI_ADR19;

    /* EBI nWR and nRD pins on PD.2 and PD.7 */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD7MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD2MFP_EBI_nWR | SYS_GPD_MFPL_PD7MFP_EBI_nRD;

    /* EBI nWRL and nWRH pins on PB.0 and PB.1 */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_EBI_nWRL | SYS_GPB_MFPL_PB1MFP_EBI_nWRH;

    /* EBI nCS0 pin on PD.8 */
    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD8MFP_Msk;
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD8MFP_EBI_nCS0;

    /* EBI ALE pin on PD.9 */
    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD9MFP_Msk;
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD9MFP_EBI_ALE;

    /* EBI MCLK pin on PD.3 */
    SYS->GPD_MFPL &= ~SYS_GPD_MFPL_PD3MFP_Msk;
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD3MFP_EBI_MCLK;
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

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(EBI_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));

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

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0 with PDMA transfer    |\n");
    printf("+--------------------------------------------------------+\n\n");

    printf("********************************************************************\n");
    printf("* Please connect BS616LV4017 SRAM to EBI bank0 before accessing !! *\n");
    printf("* EBI pins settings:                                               *\n");
    printf("*   - AD0 ~ AD7 on PA.0 ~ PA.7                                     *\n");
    printf("*   - AD8 ~ AD15 on PC.0 ~ PC.7                                    *\n");
    printf("*   - AD16 ~ AD19 on PD.12 ~ PD.15                                 *\n");
    printf("*   - nWR on PD.2                                                  *\n");
    printf("*   - nRD on PD.7                                                  *\n");
    printf("*   - nWRL on PB.0                                                 *\n");
    printf("*   - nWRH on PB.1                                                 *\n");
    printf("*   - nCS0 on PD.8                                                 *\n");
    printf("*   - ALE on PD.9                                                  *\n");
    printf("*   - MCLK on PD.3                                                 *\n");
    printf("********************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, 0, EBI_CS_ACTIVE_LOW);

    /* Start to test EBI SRAM */
    if( SRAM_BS616LV4017(512 * 1024) < 0 ) goto lexit;

    /* EBI sram with PDMA test */
    if( AccessEBIWithPDMA() < 0 ) goto lexit;

    printf("*** SRAM Test OK ***\n");

lexit:

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    while(1);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for PDMA                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 64;
uint32_t SrcArray[64];
uint32_t DestArray[64];
uint32_t volatile u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M451Series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if(status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS() & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIFn_Msk);
    }
    else if(status & 0x2)      /* done */
    {
        if(PDMA_GET_TD_STS() & 0x4)
            u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIFn_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

int32_t AccessEBIWithPDMA(void)
{
    uint32_t i;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;
    uint32_t u32TimeOutCnt = 0;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    for(i=0; i<64; i++) {
        SrcArray[i] = 0x76570000 + i;
        u32Result0 += SrcArray[i];
    }

    /* Open Channel 2 */
    PDMA_Open((1<<2));

    //burst size is 4
    PDMA_SetBurstType(2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(2, (uint32_t)SrcArray, PDMA_SAR_INC, EBI_BANK0_BASE_ADDR, PDMA_DAR_INC);
    PDMA_SetTransferMode(2, PDMA_MEM, FALSE, 0);

    PDMA_EnableInt(2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);

    u32IsTestOver = 0;
    PDMA_Trigger(2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return -1;
        }
    }
    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for(i=0; i<64; i++) {
        SrcArray[i] = 0x0;
    }

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    PDMA_SetTransferAddr(2, EBI_BANK0_BASE_ADDR, PDMA_SAR_INC, (uint32_t)SrcArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(2, PDMA_MEM, FALSE, 0);

    u32IsTestOver = 0;
    PDMA_Trigger(2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return -1;
        }
    }
    /* Transfer EBI SRAM to internal SRAM done */
    for(i=0; i<64; i++) {
        u32Result1 += SrcArray[i];
    }

    if(u32IsTestOver == 1) {
        if((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A)) {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }else {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);
            return -1;
        }
    }else {
        printf("        PDMA fail\n\n");
        return -1;
    }

    PDMA_Close();

    return 0;
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
