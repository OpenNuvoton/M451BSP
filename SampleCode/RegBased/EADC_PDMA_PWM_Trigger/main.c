/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to trigger EADC by PWM and transfer conversion data by PDMA.
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
volatile uint32_t g_u32IsTestOver = 0;
int16_t  g_i32ConversionData[6] = {0};
uint32_t g_u32SampleModuleNum = 0;

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

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

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

    /* Set PC multi-function pins for PWM0 Channel0 */
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
    /* Reset PWM0 module */
    SYS->IPRST2 |= SYS_IPRST2_PWM0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PWM0RST_Msk;

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

void PDMA_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PDMA                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset PDMA module */
    SYS->IPRST0 |=  SYS_IPRST0_PDMARST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

    /* Configure PDMA peripheral mode form EADC to memory */
    /* Open Channel 2 */
    PDMA->CHCTL |= 0x4;
    PDMA->DSCT[2].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);

    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA->DSCT[2].CTL |= ((0x1 << PDMA_DSCT_CTL_TXWIDTH_Pos) | ((6 - 1) << PDMA_DSCT_CTL_TXCNT_Pos));

    /* Set source address as EADC data register(no increment) and destination address as g_i32ConversionData array(increment) */
    PDMA->DSCT[2].SA = (uint32_t)&EADC->DAT[g_u32SampleModuleNum];
    PDMA->DSCT[2].DA = (uint32_t)g_i32ConversionData;
    PDMA->DSCT[2].CTL &= ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk);
    PDMA->DSCT[2].CTL |= ((0x3 << PDMA_DSCT_CTL_SAINC_Pos) | (0x2 << PDMA_DSCT_CTL_DAINC_Pos));

    /* Select PDMA request source as ADC RX */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk) | (0x9 << PDMA_REQSEL0_3_REQSRC2_Pos);

    /* Select PDMA operation mode as basic mode */
    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | 0x1;

    /* Set PDMA as single request type for EADC */
    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~(PDMA_DSCT_CTL_TXTYPE_Msk)) | (0x1 << PDMA_DSCT_CTL_TXTYPE_Pos);
    
    PDMA->INTEN |= (1 << 2);
    NVIC_EnableIRQ(PDMA_IRQn);
}

void ReloadPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk)) | ((6 - 1) << PDMA_DSCT_CTL_TXCNT_Pos);

    /* Select PDMA operation mode as basic mode */
    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | 0x1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    uint8_t  u8Option;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|     PWM trigger mode and transfer conversion data by PDMA test       |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while(1)
    {
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();

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

            /* Enable PDMA transfer */
            EADC->CTL |= EADC_CTL_PDMAEN_Msk;

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM0->CNTEN |= 0x1 << PWM_CNTEN_CNTENn_Pos;

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable PWM0 channel 0 counter */
            PWM0->CNTEN &= ~(0x1 << PWM_CNTEN_CNTENn_Pos);

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);

        }
        else if(u8Option == '2')
        {

            /* Set the ADC internal sampling time, input mode as differential and enable the A/D converter */
            EADC->CTL = (EADC_CTL_SMPTSEL6 | EADC_CTL_DIFFEN_DIFFERENTIAL | EADC_CTL_ADCEN_Msk);
            /* Configure the sample module 0 for analog input channel 2 and software trigger source.*/
            EADC->SCTL[0] = EADC_PWM0TG0_TRIGGER | EADC_SCTL_CHSEL(2);

            /* Enable PDMA transfer */
            EADC->CTL |= EADC_CTL_PDMAEN_Msk;

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM0->CNTEN |= 0x1 << PWM_CNTEN_CNTENn_Pos;

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable PWM0 channel 0 counter */
            PWM0->CNTEN &= ~(0x1 << PWM_CNTEN_CNTENn_Pos);

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);

        }
        else
            return ;

        EADC->CTL &= ~EADC_CTL_ADCEN_Msk;
        /* Reset EADC module */
        SYS->IPRST1 |= SYS_IPRST1_EADCRST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_EADCRST_Msk;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void ADC00_IRQHandler(void)
{

}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS() & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS() & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
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
    /* Init PWM for EADC */
    PWM0_Init();

    /* Init PDMA for EADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Reset EADC module */
    SYS->IPRST1 |= SYS_IPRST1_EADCRST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_EADCRST_Msk;

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_EADCCKEN_Msk;

    /* Disable PWM0 IP clock */
    CLK->APBCLK1 &= ~CLK_APBCLK1_PWM0CKEN_Msk;

    /* Disable PDMA IP clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

