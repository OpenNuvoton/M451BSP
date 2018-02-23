/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 2 $
 * $Date: 15/09/02 10:04a $
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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
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

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);
    
    /* Select timer 0 module clock source as HXT */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

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
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void TIMER0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init TIMER0                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set timer0 periodic time-out period is 265us if timer clock is 12 MHz */
    TIMER_SET_CMP_VALUE(TIMER0, 3174);

    /* Start timer counter in periodic mode and enable timer interrupt trigger DAC */
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_TRGDAC_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void DACFunctionTest(void)
{
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                  DAC Timer trigger with PDMA test                    |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");    
    printf("This sample code use PDMA transfer sine wave table and set DAC(PB.0) output sine wave.\n");
    
    
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);
    
    /* Open Channel 0 */
    PDMA_Open(0x1);    
    
    /* Set transfer data width, and tranfer count */    
    PDMA_SetTransferCnt(0, PDMA_WIDTH_16, array_size);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferAddr(0, (uint32_t)&sine[index], PDMA_SAR_INC, (uint32_t)&DAC->DAT, PDMA_DAR_FIX);

    /* Select channel 0 request source from DAC */    
    PDMA_SetTransferMode(0, PDMA_DAC_TX, FALSE, 0);

    /* Set transfer type and burst size */
    PDMA_SetBurstType(0, PDMA_REQ_SINGLE, PDMA_BURST_128);
    
    /* Set the timer 0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC, 0, DAC_TIMER0_TRIGGER);

    /* The DAC conversion settling time is 8us */
    DAC_SetDelayTime(DAC, 8);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC, 0);

    /* Enable the PDMA Mode */    
    DAC_ENABLE_PDMA(DAC);
    
    /* Enable the DAC interrupt.  */
    DAC_ENABLE_INT(DAC,0);
    NVIC_EnableIRQ(DAC_IRQn);

    printf("\nHit any key to quit!\n");
    
    /* Enable Timer0 counting to start D/A conversion */
    TIMER_Start(TIMER0);

    while(1)
    {
        if (PDMA_GET_TD_STS() == 0x1)
        {
            /* Re-Set transfer count and basic operation mode */            
            PDMA_SetTransferCnt(0, PDMA_WIDTH_16, array_size);
            PDMA_SetTransferMode(0, PDMA_DAC_TX, FALSE, 0);
            
            /* Clear PDMA channel 0 transfer done flag */            
            PDMA_CLR_TD_FLAG(0x1);
        }
        
        if((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0)
            continue;
        else
        {
            /* Stop Timer0 Counting */            
            TIMER_Stop(TIMER0);
            break;
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void DAC_IRQHandler(void)
{
    if(DAC_GET_INT_FLAG(DAC, 0))
    {
        DAC_CLR_INT_FLAG(DAC, 0);
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

    /* Init TIMER0 trigger for DAC */
    TIMER0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* DAC function test */
    DACFunctionTest();

    /* Disable External Interrupt */
    NVIC_DisableIRQ(DAC_IRQn);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);
    
    /* Reset DAC module */
    SYS_ResetModule(DAC_RST);

    /* Reset timer0 module */
    SYS_ResetModule(TMR0_RST);

    /* Disable PDMA IP clock */
    CLK_DisableModuleClock(PDMA_MODULE);
    
    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

    /* Disable DAC IP clock */
    CLK_DisableModuleClock(DAC_MODULE);

    printf("Stop DAC output and exit DAC sample code\n");

    while(1);
    
}
