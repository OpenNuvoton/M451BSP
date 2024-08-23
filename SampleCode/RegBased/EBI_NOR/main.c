/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/09/02 10:03a $
 * @brief    Configure EBI interface to access MX29LV320T (NOR Flash) on EBI interface.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void NOR_MX29LV320T_RESET(uint32_t u32Bank);
extern int32_t NOR_MX29LV320T_CheckStatus(uint32_t u32DstAddr, uint16_t u16Data, uint32_t u32TimeoutMs);
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);


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

    /* EBI nCS1 pin on PB.15 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB15MFP_EBI_nCS1;

    /* EBI ALE pin on PD.9 */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD9MFP_Msk);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD9MFP_EBI_ALE;

    /* EBI MCLK pin on PD.3 */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD3MFP_EBI_MCLK;
}

void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch STCLK source to HCLK/2 and HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2 | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_EBICKEN_Msk;
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0 ~ AD7 on PA.0 ~ PA.7                                         *\n");
    printf("*   - AD8 ~ AD15 on PC.0 ~ PC.7                                        *\n");
    printf("*   - AD16 ~ AD19 on PD.12 ~ PD.15                                     *\n");
    printf("*   - nWR on PD.2                                                      *\n");
    printf("*   - nRD on PD.7                                                      *\n");
    printf("*   - nWRL on PB.0                                                     *\n");
    printf("*   - nWRH on PB.1                                                     *\n");
    printf("*   - nCS1 on PB.15                                                    *\n");
    printf("*   - ALE on PD.9                                                      *\n");
    printf("*   - MCLK on PD.3                                                     *\n");
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Enable EBI function and configure data bus width is 16-bit, MCLK is HCLK/2 and CS active level is low */
    EBI->CTL1  = (EBI_MCLKDIV_2 << EBI_CTL1_MCLKDIV_Pos) |
                 (EBI_CS_ACTIVE_LOW << EBI_CTL1_CSPOLINV_Pos) |
                 EBI_CTL1_DW16_Msk | EBI_CTL1_EN_Msk ;
    EBI->TCTL1 = 0x03003318;


    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);
    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);
    if((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");
        goto lexit;
    }
    else
    {
        printf("PASS !!!\n\n");
    }


    /* Step 2, erase chip */
    if(NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
        goto lexit;


    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;
    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        if(NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16WData);
            goto lexit;
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Program [0x%08X]:[0x%08X] !!!       \r", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16WData);
        }
    }

    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);
        if(u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16WData, u16RData);
            goto lexit;
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr, u16RData);
        }
    }
    printf(">> Program flash OK !!!                             \n\n");

lexit:

    /* Disable EBI function */
    EBI->CTL1 &= ~EBI_CTL1_EN_Msk;

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBICKEN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
