/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to PDMA and trigger DAC by Timer.
 * @note
 * Copyright (C) 2015~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
const uint16_t sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                         4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                         3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                         639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                         238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                        };

static uint32_t index = 0;
const uint32_t array_size = sizeof(sine) / sizeof(uint16_t);

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void DAC_FunctionTest(void);


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

    /* Select timer 0 module clock source as HXT */
    CLK->CLKSEL1 &= ~(CLK_CLKSEL1_TMR0SEL_Msk);
//                      CLK_CLKSEL1_TMR1SEL_Msk |
//                      CLK_CLKSEL1_TMR2SEL_Msk |
//                      CLK_CLKSEL1_TMR3SEL_Msk);
    CLK->CLKSEL1 |= (CLK_CLKSEL1_TMR0SEL_HXT);
//                     CLK_CLKSEL1_TMR1SEL_HXT |
//                     CLK_CLKSEL1_TMR2SEL_HXT |
//                     CLK_CLKSEL1_TMR3SEL_HXT);

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable DAC module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_DACCKEN_Msk;

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;
    
    /* Enable Timer 0 module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_TMR0CKEN_Msk);
//                     CLK_APBCLK0_TMR1CKEN_Msk |
//                     CLK_APBCLK0_TMR2CKEN_Msk |
//                     CLK_APBCLK0_TMR3CKEN_Msk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PB multi-function pins for DAC voltage output */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_DAC ;

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

void TIMER0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init TIMER0                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set timer0 periodic time-out period is 10us if timer clock is 12 MHz */
    TIMER0->CMP = 120;

    /* Start timer counter in periodic mode and enable timer interrupt trigger DAC */
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGDAC_Msk;

}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_FunctionTest(void)
{
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                  DAC Timer trigger with PDMA test                    |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");    
    printf("This sample code use PDMA transfer sine wave table to DAC(PB.0) output.\n");
        
    /* Reset DAC module */
    SYS->IPRST2 |= SYS_IPRST2_DACRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST2_DACRST_Msk ;

    /* CH0 source request from DAC */
    PDMA->REQSEL0_3 = 0x08; //CH0_SEl=DAC
    
    PDMA->DSCT[0].CTL=((array_size)<<16)//Transfer Count
                       |(0x1<<12)//Transfer Width Selection(01=>16bit)
                       |(0x3<<10)//Destination Address Increment(11=>No increment (fixed address))
                       |(0x0<<8)//Source Address Increment(00=>Increment)
                       |(0x0<<7)//Table Interrupt Disable
                       |(0x0<<4)//Burst Size(128 Transfers)
                       |(0x1<<2)//Transfer Type(1 = Single transfer type)
                       |(0x1<<0);//PDMA Operation Mode Selection(01=>Basic Mode)
    
    /* Set source address */    
    PDMA->DSCT[0].SA=(uint32_t)&sine[index];
    
    /* Set destination address */      
    PDMA->DSCT[0].DA=(uint32_t)&DAC->DAT;

    /* Enable PDMA channel 0 */
    PDMA->CHCTL=0x1;     
    
    /* Set the timer 0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC->CTL = DAC_TIMER0_TRIGGER | DAC_CTL_TRGEN_Msk | DAC_CTL_DMAEN_Msk | DAC_CTL_DACEN_Msk;

    /* When DAC controller APB clock speed is 72MHz and DAC conversion settling time is 8us,
       the selected SETTLET value must be greater than 0x241.  */
    DAC->TCTL = 0x250;

    /* Set DAC 12-bit holding data */
    DAC->DAT = sine[index];

    /* Clear the DAC conversion complete finish flag for safe */
    DAC->STATUS = DAC_STATUS_FINISH_Msk;

    /* Enable the DAC interrupt.  */
    DAC->CTL |= DAC_CTL_DACIEN_Msk;
    NVIC_EnableIRQ(DAC_IRQn);

    printf("\nHit any key to quit!\n");

    /* Enable Timer0 counting to start D/A conversion */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk; //Enable timer Counting.

    while(1)
    {
        if (PDMA->TDSTS == 0x1)
        {
            /* Re-Set transfer count and basic operation mode */
            PDMA->DSCT[0].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk);
            PDMA->DSCT[0].CTL |= (PDMA_OP_BASIC | ((array_size - 1) << PDMA_DSCT_CTL_TXCNT_Pos));        

            /* Clear CH0 transfer done flag */
            PDMA->TDSTS = 0x01;               
        }    
        
        if((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0)
            continue;
        else
        {
            /* Stop Timer0 Counting */
            TIMER0->CTL &= ~TIMER_CTL_CNTEN_Msk; 
            break;
        }
    }

    return;
}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_IRQHandler(void)
{

    if(DAC->STATUS & DAC_STATUS_FINISH_Msk)
    {
        DAC->STATUS = DAC_STATUS_FINISH_Msk;
    }
    return;
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

    /* Init TIMER0 for DAC */
    TIMER0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);
    
    /* DAC function test */
    DAC_FunctionTest();

    /* Disable External Interrupt */
    NVIC_DisableIRQ(DAC_IRQn);

    /* Reset PDMA module */
    SYS->IPRST0 |= SYS_IPRST0_PDMARST_Msk ;
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk ;
    
    /* Reset DAC module */
    SYS->IPRST2 |= SYS_IPRST2_DACRST_Msk ;
    SYS->IPRST2 &= ~SYS_IPRST2_DACRST_Msk ;

    /* Reset Timer0 module */
    SYS->IPRST1 |= SYS_IPRST1_TMR0RST_Msk ;
    SYS->IPRST1 &= ~SYS_IPRST1_TMR0RST_Msk ;
    
    /* Disable PDMA module clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;
    
    /* Disable Timer0 IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_TMR0CKEN_Msk;

    /* Disable DAC IP clock */
    CLK->APBCLK1 &= ~CLK_APBCLK1_DACCKEN_Msk;
    
    printf("Stop DAC output and exit DAC sample code\n");

    while(1);

}
