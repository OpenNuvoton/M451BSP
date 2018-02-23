/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to trigger ADC by PWM.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update system core clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UARTDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_UART(1);

    /* Select PWM0 module clock source as PLL */
    CLK->CLKSEL2 &= ~CLK_CLKSEL2_PWM0SEL_Msk;
    CLK->CLKSEL2 |= CLK_CLKSEL2_PWM0SEL_PLL;

    /* EADC clock source is 72MHz, set divider to 8, ADC clock is 72/8 MHz */
    CLK->CLKDIV0  = (CLK->CLKDIV0 & ~CLK_CLKDIV0_EADCDIV_Msk) | (((8) - 1) << CLK_CLKDIV0_EADCDIV_Pos);

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable PWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

    /* Enable EADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_EADCCKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Configure the GPB0 - GPB3 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk |
                       SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC_CH0 | SYS_GPB_MFPL_PB1MFP_EADC_CH1 |
                      SYS_GPB_MFPL_PB2MFP_EADC_CH2 | SYS_GPB_MFPL_PB3MFP_EADC_CH3);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, 0xF);

    /* Set PC multi-function pins for PWMA Channel0 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_PWM0_CH0;

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

void PWM0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PWM0                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 0);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPEn_Msk;

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, 108);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, 216);

    /* Enable PWM_CH0 period point trigger EADC */
    PWM0->EADCTS0 |= (PWM_EADCTS0_TRGEN0_Msk | PWM_TRIGGER_ADC_EVEN_PERIOD_POINT) ;

    /* Set waveform generation */
    PWM0->WGCTL0 = 0xAAA;//PWM zero point and period point output High.
    PWM0->WGCTL1 = 0x555;//PWM compare down point and  compare up point output Low.

    /* Enable output of all PWM0 channels */
    PWM0->POEN |= PWM_POEN_POENn_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                       PWM trigger mode test                          |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
            EADC->CTL = (EADC_CTL_SMPTSEL6 | EADC_CTL_DIFFEN_SINGLE_END | EADC_CTL_ADCEN_Msk);
            /* Configure the sample module 0 for analog input channel 2 and enable PWM0 trigger source */
            EADC->SCTL[0] = EADC_PWM0TG0_TRIGGER | EADC_SCTL_CHSEL(2);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC->STATUS2 = EADC_STATUS2_ADIF0_Msk;

            /* Enable the sample module 0 interrupt */
            EADC->CTL |= EADC_CTL_ADCIEN0_Msk;  //Enable sample module A/D ADINT0 interrupt.
            EADC->INTSRC[0] = 0x1;              //Enable sample module 0 interrupt.
            NVIC_EnableIRQ(ADC00_IRQn);

            printf("Conversion result of channel 2:\n");

            /* Reset the EADC indicator and enable PWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            PWM0->CNTEN |= 0x1 << PWM_CNTEN_CNTENn_Pos;

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = (EADC->DAT[0] & EADC_DAT_RESULT_Msk);

                if(g_u32COVNUMFlag > 6)
                    break;
            }

            /* Disable PWM0 channel 0 counter */
            PWM0->CNTEN &= ~(0x1 << PWM_CNTEN_CNTENn_Pos);

            /* Disable the ADINT0 interrupt */
            EADC->CTL &= ~EADC_CTL_ADCIEN0_Msk;

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);

        }
        else if(u8Option == '2')
        {

            /* Set the ADC internal sampling time, input mode as differential and enable the A/D converter */
            EADC->CTL = (EADC_CTL_SMPTSEL6 | EADC_CTL_DIFFEN_DIFFERENTIAL | EADC_CTL_ADCEN_Msk);
            /* Configure the sample module 0 for analog input channel 2 and software trigger source.*/
            EADC->SCTL[0] = EADC_PWM0TG0_TRIGGER | EADC_SCTL_CHSEL(2);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC->STATUS2 = EADC_STATUS2_ADIF0_Msk;

            /* Enable the sample module 0 interrupt */
            EADC->CTL |= EADC_CTL_ADCIEN0_Msk;  //Enable sample module A/D ADINT0 interrupt.
            EADC->INTSRC[0] = 0x1;              //Enable sample module 0 interrupt.
            NVIC_EnableIRQ(ADC00_IRQn);

            printf("Conversion result of channel 2:\n");

            /* Reset the EADC indicator and enable PWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            PWM0->CNTEN |= 0x1 << PWM_CNTEN_CNTENn_Pos;

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = (EADC->DAT[0] & EADC_DAT_RESULT_Msk);

                if(g_u32COVNUMFlag > 6)
                    break;
            }

            /* Disable PWM0 channel 0 counter */
            PWM0->CNTEN &= ~(0x1 << PWM_CNTEN_CNTENn_Pos);

            /* Disable the ADINT0 interrupt */
            EADC->CTL &= ~EADC_CTL_ADCIEN0_Msk;

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);

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
    g_u32COVNUMFlag++;
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

    /* Init PWM for EADC */
    PWM0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS->IPRST2 |= SYS_IPRST1_EADCRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST1_EADCRST_Msk ;

    /* Reset PWM0 module */
    SYS->IPRST2 |= SYS_IPRST2_PWM0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PWM0RST_Msk;

    /* Disable EADC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_EADCCKEN_Msk;

    /* Disable PWM0 IP clock */
    CLK->APBCLK1 &= ~CLK_APBCLK1_PWM0CKEN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC00_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

