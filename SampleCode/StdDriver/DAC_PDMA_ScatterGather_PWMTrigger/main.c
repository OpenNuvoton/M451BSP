/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 2 $
 * $Date: 15/09/02 10:04a $
 * @brief    Demonstrate how to use PDMA scatter gather mode and trigger DAC by PWM.
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
/* The positive duty table of Sine wave */
const uint16_t sinePos[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                         4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                         3102, 2921, 2732, 2536, 2335, 2132
                        };
/* The negative duty table of Sine wave */
const uint16_t sineNeg[] = {1927, 1724, 1523, 1328, 1141,  962,  794, 639,  497,  371,  262,  171,   99,
                         45,   12,    0,    7,   35,   84,  151, 238,  343,  465,  602,  754,  919, 1095,
                         1281, 1475, 1674, 1876
                        };

static uint32_t index = 0;
const uint32_t array_Pos_size = sizeof(sinePos) / sizeof(uint16_t);
const uint32_t array_Neg_size = sizeof(sineNeg) / sizeof(uint16_t);

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[2];
uint32_t gu32DMAConfig = 0;
uint32_t gu32DMAConfigPos = 0;
uint32_t gu32DMAConfigNeg = 0;

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

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);
    
    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PB multi-function pins for DAC voltage output */
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB0MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_DAC;

    /* Set PC multi-function pins for PWMA Channel0 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_PWM0_CH0;

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

void PWM0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PWM0                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PWM0 Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 100);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPEn_Msk;

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, 360);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, 720);

    /* PWM period point trigger DAC enable */
    PWM_EnableDACTrigger(PWM0, 0, PWM_TRIGGER_DAC_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, 0x1, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of all PWM0 channels */
    PWM_EnableOutput(PWM0, 0x1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* DAC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void DACFunctionTest(void)
{
    static uint32_t u32TableIndex = 0;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|        DAC PWM trigger with PDMA scatter gather mode test            |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");    
    printf("This sample code use PDMA scatter gather mode transfer sine wave table to DAC(PB.0) output.\n");
    
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    gu32DMAConfigPos = ((array_Pos_size - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    gu32DMAConfigNeg = ((array_Neg_size - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_16 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    
    DMA_DESC[0].ctl = gu32DMAConfigPos;
    DMA_DESC[0].src = (uint32_t)&sinePos[index];
    DMA_DESC[0].dest = (uint32_t)&DAC->DAT;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA);

    DMA_DESC[1].ctl = gu32DMAConfigNeg;
    DMA_DESC[1].src = (uint32_t)&sineNeg[index];
    DMA_DESC[1].dest = (uint32_t)&DAC->DAT;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);
    
    /* Open PWM Channel 0 */
    PDMA_Open(0x1);    
    
    /* Select channel 0 request source from DAC and enable PDMA scatter-gather mode */
    PDMA_SetTransferMode(0, PDMA_DAC_TX, TRUE, (uint32_t)&DMA_DESC[0]);

    /* Set the PWM0 trigger,enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC, 0, DAC_PWM0_TRIGGER);

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
    
    /* Enable PWM0 chann0 to start D/A conversion */
    PWM_Start(PWM0, 0x1); 

    /* initial gu32DMAConfig value as gu32DMAConfigPos */
    gu32DMAConfig = gu32DMAConfigPos;

    while(1)
    {
        if (PDMA_GET_TD_STS() == 0x1)
        {
            /* Re-Set transfer count and basic operation mode */            
            DMA_DESC[u32TableIndex].ctl = gu32DMAConfig;
            u32TableIndex ^= 1;
            if (u32TableIndex == 0)
            {
                gu32DMAConfig = gu32DMAConfigPos;
            }
            else
            {
                gu32DMAConfig = gu32DMAConfigNeg;
            }
            
            /* Clear PDMA channel 0 transfer done flag */            
            PDMA_CLR_TD_FLAG(0x1);
        }
        
        if((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0)
            continue;
        else
        {
            /* Stop PWM0 channel 0 Counting */            
            PWM_Stop(PWM0, 0x1);
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
        /* Clear the DAC conversion complete finish flag */
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

    /* Init PWM0 trigger for DAC */
    PWM0_Init();

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
    SYS_ResetModule(PWM0_RST);

    /* Disable PDMA IP clock */
    CLK_DisableModuleClock(PDMA_MODULE);
    
    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(PWM0_MODULE);

    /* Disable DAC IP clock */
    CLK_DisableModuleClock(DAC_MODULE);

    printf("Stop DAC output and exit DAC sample code\n");
    
    while(1);

}

