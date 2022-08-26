/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:04a $
 * @brief
 *           Monitor the conversion result of channel 2 by the digital compare function.
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"

#define PLL_CLOCK           72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 72MHz, set divider to 8, ADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Configure the GPB0 - GPB3 ADC analog input pins. */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk |
                       SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC_CH0 | SYS_GPB_MFPL_PB1MFP_EADC_CH1 |
                      SYS_GPB_MFPL_PB2MFP_EADC_CH2 | SYS_GPB_MFPL_PB3MFP_EADC_CH3);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, 0xF);

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

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|           EADC compare function (result monitor) sample code         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 6);

    /* Configure the sample module 0 for analog input channel 2 and ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, 0, EADC_ADINT0_TRIGGER, 2);

    /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    EADC_ENABLE_CMP0(EADC, 0, EADC_CMP_CMPCOND_LESS_THAN, 0x800, 0x5);

    /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1 : channel 2 is greater than or equal to 0x800; match count is 5.\n");
    EADC_ENABLE_CMP1(EADC, 0, EADC_CMP_CMPCOND_GREATER_OR_EQUAL, 0x800, 0x5);

    /* Enable sample module 0 interrupt */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);

    /* Clear the A/D ADINT3 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, (0x1 << 3));
    /* Enable ADINT3 interrupt */
    EADC_ENABLE_INT(EADC, (0x1 << 3));
    NVIC_EnableIRQ(ADC03_IRQn);

    /* Clear the EADC comparator 0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, (0x1 << 4));
    /* Enable ADC comparator 0 interrupt */
    EADC_ENABLE_CMP_INT(EADC, 0);

    /* Clear the EADC comparator 1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, (0x1 << 5));
    /* Enable ADC comparator 1 interrupt */
    EADC_ENABLE_CMP_INT(EADC, 1);

    /* Reset the EADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    EADC_START_CONV(EADC, 0x1);

    /* Wait EADC compare interrupt */
    u32TimeOutCnt = EADC_TIMEOUT;
    while((g_u32AdcCmp0IntFlag == 0) && (g_u32AdcCmp1IntFlag == 0))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC compare interrupt time-out!\n");
            return;
        }
    }

    /* Disable the sample module 0 interrupt */
    EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);

    /* Disable ADC comparator interrupt */
    EADC_DISABLE_CMP_INT(EADC, 0);
    EADC_DISABLE_CMP_INT(EADC, 1);
    /* Disable compare function */
    EADC_DISABLE_CMP0(EADC);
    EADC_DISABLE_CMP1(EADC);

    if(g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ADC03_IRQHandler(void)
{
    if(EADC_GET_INT_FLAG(EADC, (0x1 << 4)))
    {
        g_u32AdcCmp0IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, (0x1 << 4));/* Clear the A/D compare flag 0 */
    }

    if(EADC_GET_INT_FLAG(EADC, (0x1 << 5)))
    {
        g_u32AdcCmp1IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, (0x1 << 5));/* Clear the A/D compare flag 1 */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC00_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

