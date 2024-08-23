/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/09/02 10:03a $
 * @brief    Use ADINT interrupt to do the ADC continuous scan conversion.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
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
	uint32_t u32TimeOutCnt;


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update system core clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UARTDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_UART(1);

    /* Enable EADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_EADCCKEN_Msk;

    /* EADC clock source is 72MHz, set divider to 8, ADC clock is 72/8 MHz */
    CLK->CLKDIV0  = (CLK->CLKDIV0 & ~CLK_CLKDIV0_EADCDIV_Msk) | (((8) - 1) << CLK_CLKDIV0_EADCDIV_Pos);

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
            EADC->CTL = (EADC_CTL_SMPTSEL6 | EADC_CTL_DIFFEN_SINGLE_END | EADC_CTL_ADCEN_Msk);

            /* Configure the sample module 4 for analog input channel 0 and enable ADINT0 trigger source */
            EADC->SCTL[4] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(0);
            /* Configure the sample module 5 for analog input channel 1 and enable ADINT0 trigger source */
            EADC->SCTL[5] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(1);
            /* Configure the sample module 6 for analog input channel 2 and enable ADINT0 trigger source */
            EADC->SCTL[6] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(2);
            /* Configure the sample module 7 for analog input channel 3 and enable ADINT0 trigger source */
            EADC->SCTL[7] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(3);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC->STATUS2 = EADC_STATUS2_ADIF0_Msk;

            /* Enable the sample module 7 interrupt */
            EADC->CTL |= EADC_CTL_ADCIEN0_Msk;  //Enable sample module A/D ADINT0 interrupt.
            EADC->INTSRC[0] = 0x1ul << 7;       //Enable sample module 7 interrupt.
            NVIC_EnableIRQ(ADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC->SWTRG |= (0x1 << 7);

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
            EADC->INTSRC[0] &= ~(0x1 << 7); //Disable sample module 7 interrupt.

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = (EADC->DAT[u32SAMPLECount + 4] & EADC_DAT_RESULT_Msk);

            /* Wait conversion done */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(EADC->STATUS0 != 0xF0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC conversion done time-out!\n");
                    return;
                }
            }

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = (EADC->DAT[u32SAMPLECount] & EADC_DAT_RESULT_Msk);

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 8; g_u32COVNUMFlag++)
                printf("Conversion result of channel %d: 0x%X (%d)\n", (g_u32COVNUMFlag % 4), i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);

        }
        else if(u8Option == '2')
        {
            /* Set the ADC internal sampling time, input mode as differential and enable the A/D converter */
            EADC->CTL = (EADC_CTL_SMPTSEL6 | EADC_CTL_DIFFEN_DIFFERENTIAL | EADC_CTL_ADCEN_Msk);

            /* Configure the sample module 4 for analog input channel 0 and enable ADINT0 trigger source */
            EADC->SCTL[4] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(0);
            /* Configure the sample module 5 for analog input channel 1 and enable ADINT0 trigger source */
            EADC->SCTL[5] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(1);
            /* Configure the sample module 6 for analog input channel 2 and enable ADINT0 trigger source */
            EADC->SCTL[6] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(2);
            /* Configure the sample module 7 for analog input channel 3 and enable ADINT0 trigger source */
            EADC->SCTL[7] = EADC_ADINT0_TRIGGER | EADC_SCTL_CHSEL(3);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC->STATUS2 = EADC_STATUS2_ADIF0_Msk;

            /* Enable the sample module 7 interrupt */
            EADC->CTL |= EADC_CTL_ADCIEN0_Msk;  //Enable sample module A/D ADINT0 interrupt.
            EADC->INTSRC[0] = 0x1ul << 7;       //Enable sample module 7 interrupt.
            NVIC_EnableIRQ(ADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC->SWTRG |= (0x1 << 7);

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
            EADC->INTSRC[0] &= ~(0x1 << 7); //Disable sample module 7 interrupt.

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = (EADC->DAT[u32SAMPLECount + 4] & EADC_DAT_RESULT_Msk);

            /* Wait conversion done */
            u32TimeOutCnt = EADC_TIMEOUT;
            while(EADC->STATUS0 != 0xF0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC conversion done time-out!\n");
                    return;
                }
            }

            /* Get the conversion result of the sample module */
            for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
                i32ConversionData[u32SAMPLECount] = (EADC->DAT[u32SAMPLECount] & EADC_DAT_RESULT_Msk);

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
    EADC->STATUS2 = EADC_STATUS2_ADIF0_Msk;      /* Clear the A/D ADINT0 interrupt flag */
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
    SYS->IPRST2 |= SYS_IPRST1_EADCRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST1_EADCRST_Msk ;

    /* Disable EADC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_EADCCKEN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC00_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}
