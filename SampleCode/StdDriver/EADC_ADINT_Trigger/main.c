/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:04a $
 * @brief    Use ADINT interrupt to do the ADC continuous scan conversion.
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
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

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
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    uint8_t  u8Option, u32SAMPLECount = 0;
    int32_t  i32ConversionData[8] = {0};
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 2 cycles of conversion result from the specified channels.\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
            EADC_SetInternalSampleTime(EADC, 6);

            /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 0);
            /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 1);
            /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 2);
            /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 3);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, 0x1);

            /* Enable the sample module 7 interrupt */
            EADC_ENABLE_INT(EADC, 0x1);//Enable sample module  A/D ADINT0 interrupt.
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (0x1 << 7));//Enable sample module 7 interrupt.
            NVIC_EnableIRQ(ADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC, (0x1 << 7));

            /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(g_u32AdcIntFlag == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC interrupt time-out!\n");
                    return;
                }
            }

            /* Reset the EADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(g_u32AdcIntFlag == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC interrupt time-out!\n");
                    return;
                }
            }

            /* Reset the EADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, (0x1 << 7));

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC, (u32SAMPLECount + 4));

            /* Wait conversion done */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(EADC_GET_DATA_VALID_FLAG(EADC, 0xF0) != 0xF0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC conversion done time-out!\n");
                    return;
                }
            }

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC, u32SAMPLECount);

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 8; g_u32COVNUMFlag++)
                printf("Conversion result of channel %d: 0x%X (%d)\n", (g_u32COVNUMFlag % 4), i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);

        }
        else if(u8Option == '2')
        {
            /* Set the ADC internal sampling time, input mode as differential and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_DIFFERENTIAL);
            EADC_SetInternalSampleTime(EADC, 6);

            /* Configure the sample module 4 for analog input channel 0 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 0);
            /* Configure the sample module 5 for analog input channel 1 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 1);
            /* Configure the sample module 6 for analog input channel 2 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 2);
            /* Configure the sample module 7 for analog input channel 3 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 3);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, 0x1);

            /* Enable the sample module 7 interrupt */
            EADC_ENABLE_INT(EADC, 0x1);//Enable sample module A/D ADINT0 interrupt.
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (0x1 << 7));//Enable sample module 7 interrupt.
            NVIC_EnableIRQ(ADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC, (0x1 << 7));

            /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(g_u32AdcIntFlag == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC interrupt time-out!\n");
                    return;
                }
            }

            /* Reset the EADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(g_u32AdcIntFlag == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC interrupt time-out!\n");
                    return;
                }
            }

            /* Reset the EADC interrupt indicator */
            g_u32AdcIntFlag = 0;

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, (0x1 << 7));

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC, (u32SAMPLECount + 4));

            /* Wait conversion done */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(EADC_GET_DATA_VALID_FLAG(EADC, 0xF0) != 0xF0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC conversion done time-out!\n");
                    return;
                }
            }

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC, u32SAMPLECount);

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 8; g_u32COVNUMFlag++)
                printf("Conversion result of channel %d: 0x%X (%d)\n", (g_u32COVNUMFlag % 4), i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);

        }
        else
            return ;

    }
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, 0x1);      /* Clear the A/D ADINT0 interrupt flag */
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

