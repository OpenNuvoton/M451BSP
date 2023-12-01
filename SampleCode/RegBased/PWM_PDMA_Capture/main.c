/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    Capture the PWM1 Channel 0 waveform by PWM1 Channel 2, and use PDMA to transfer captured data.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLLCTL_SETTING  CLK_PLLCTL_144MHz_HXT

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint16_t g_u32Count[4];
volatile uint32_t g_u32IsTestOver = 0;

/**
 * @brief       PWM1 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM1 interrupt event
 */
void PWM1P1_IRQHandler(void)
{

}

/**
 * @brief       PDMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PDMA interrupt event
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS() & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS() & PDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* g_u32Count[4] : Keep the internal counter value when input signal rising / falling   */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
//void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
int32_t CalPeriodTime()
{
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    g_u32IsTestOver = 0;
    /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(g_u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA interrupt time-out!\n");
            return (-1);
        }
    }

    u16RisingTime = g_u32Count[1];

    u16FallingTime = g_u32Count[0];

    u16HighPeriod = g_u32Count[1] - g_u32Count[2];

    u16LowPeriod = 0x10000 - g_u32Count[1];

    u16TotalPeriod = 0x10000 - g_u32Count[2];

    printf("\nPWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 7199) || (u16HighPeriod > 7201) || (u16LowPeriod < 16799) || (u16LowPeriod > 16801) || (u16TotalPeriod < 23999) || (u16TotalPeriod > 24001))
    {
        printf("Capture Test Fail!!\n");
        return (-1);
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable PWM1 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM1SEL_Msk) | CLK_CLKSEL2_PWM1SEL_PCLK1;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM1SEL_Msk) | CLK_CLKSEL2_PWM1SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_HXT;

    /* Reset PWM1 module */
    SYS->IPRST2 |= SYS_IPRST2_PWM1RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PWM1RST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PC multi-function pins for PWM1 Channel 0 and 2 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC6MFP_PWM1_CH0;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk));
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC11MFP_PWM1_CH2;
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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("PWM1 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM1SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM1 channel 2 to capture the signal from PWM1 channel 0.\n");
    printf("  And the captured data is transferred by PDMA channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    PWM1 channel 2(PC.11) <--> PWM1 channel 0(PC.6)\n\n");
    printf("Use PWM1 Channel 2(PC.11) to capture the PWM1 Channel 0(PC.6) Waveform\n");

    while(1)
    {
        printf("\n\nPress any key to start PWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 Channel 0 as PWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 1500Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency = PLL = 72000000
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                   = 72000000/2/1500 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 23999
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 7199
           Prescale value is 1 : prescaler= 2
        */

        /*Set counter as down count*/
        PWM1->CTL1 = (PWM1->CTL1 & ~PWM_CTL1_CNTTYPE0_Msk) | 0x1;

        /*Set PWM Timer clock prescaler*/
        PWM_SET_PRESCALER(PWM1, 0, 1); // Divided by 2

        /*Set PWM Timer duty*/
        PWM_SET_CMR(PWM1, 0, 7199);

        /*Set PWM Timer period*/
        PWM_SET_CNR(PWM1, 0, 23999);

        /* Set waveform generation */
        PWM1->WGCTL0 = 0x00010000;
        PWM1->WGCTL1 = 0x00020000;

        /* Enable PWM Output path for PWM1 channel 0 */
        PWM1->POEN |= PWM_CH_0_MASK;

        /* Enable Timer for PWM1 channel 0 */
        PWM1->CNTEN |= PWM_CH_0_MASK;

        /*--------------------------------------------------------------------------------------*/
        /* Configure PDMA peripheral mode form PWM to memory                                    */
        /*--------------------------------------------------------------------------------------*/
        /* Open Channel 0 */
        PDMA->CHCTL |= 0x1;
        PDMA->DSCT[0].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);

        /* transfer width is half word(16 bit) and transfer count is 4 */
        PDMA->DSCT[0].CTL |= ((0x1 << PDMA_DSCT_CTL_TXWIDTH_Pos) | ((4 - 1) << PDMA_DSCT_CTL_TXCNT_Pos));

        /* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_u32Count array(increment) */
        PDMA->DSCT[0].SA = (uint32_t)&PWM1->PDMACAP2_3;
        PDMA->DSCT[0].DA = (uint32_t)&g_u32Count[0];
        PDMA->DSCT[0].CTL &= ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk);
        PDMA->DSCT[0].CTL |= ((0x3 << PDMA_DSCT_CTL_SAINC_Pos) | (0x2 << PDMA_DSCT_CTL_DAINC_Pos));

        /* Select PDMA request source as PWM RX(PWM1 channel 2 should be PWM1 pair 2) */
        PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Msk) | (0xF << PDMA_REQSEL0_3_REQSRC0_Pos);

        /* Select PDMA operation mode as basic mode */
        PDMA->DSCT[0].CTL = (PDMA->DSCT[0].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | 0x1;

        /* Set PDMA as single request type for PWM */
        PDMA->DSCT[0].CTL = (PDMA->DSCT[0].CTL & ~(PDMA_DSCT_CTL_TXTYPE_Msk)) | (0x1 << PDMA_DSCT_CTL_TXTYPE_Pos);

        PDMA->INTEN |= (1 << 0);
        NVIC_EnableIRQ(PDMA_IRQn);

        /* Enable PDMA for PWM1 channel 2 capture function, and set capture order as falling first */
        PWM1->PDMACTL &= ~(PWM_PDMACTL_CHSEL2_3_Msk | PWM_PDMACTL_CAPORD2_3_Msk);

        /* Select capture mode as both rising and falling to do PDMA transfer */
        PWM1->PDMACTL |= (PWM_PDMACTL_CAPMOD2_3_Msk | PWM_PDMACTL_CHEN2_3_Msk);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 channel 2 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 1500Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 72000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 72000000/2/1500 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */

        /*Set counter as down count*/
        PWM1->CTL1 = (PWM1->CTL1 & ~PWM_CTL1_CNTTYPE2_Msk) | (0x1 << PWM_CTL1_CNTTYPE2_Pos);

        /*Set PWM1 channel 2 Timer clock prescaler*/
        PWM_SET_PRESCALER(PWM1, 2, 1); // Divided by 2

        /*Set PWM1 channel 2 Timer period*/
        PWM_SET_CNR(PWM1, 2, 0xFFFF);

        /* Enable capture function */
        PWM1->CAPCTL |= PWM_CAPCTL_CAPEN2_Msk;

        /* Enable falling capture reload */
        PWM1->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

        // Start
        PWM1->CNTEN |= PWM_CNTEN_CNTEN2_Msk;

        /* Wait until PWM1 channel 2 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CNT[2]) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 2 Timer start time-out!\n");
                goto lexit;
            }
        }

        /* Enable capture input path for PWM1 channel 2 */
        PWM1->CAPINEN |= PWM_CAPINEN_CAPINEN2_Msk;

        /* Capture the Input Waveform Data */
        //CalPeriodTime(PWM1, 2);
        if( CalPeriodTime() < 0 ) goto lexit;
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set PWM1 channel 0 loaded value as 0 */
        PWM1->PERIOD[0] = 0;

        /* Wait until PWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CNT[0] & PWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 0 Timer stop time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for PWM1 channel 0 */
        PWM1->CNTEN &= ~PWM_CNTEN_CNTEN0_Msk;

        /* Disable PWM Output path for PWM1 channel 0 */
        PWM1->POEN &= ~PWM_CH_0_MASK;

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 2 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable PDMA NVIC */
        NVIC_DisableIRQ(PDMA_IRQn);

        /* Set loaded value as 0 for PWM1 channel 2 */
        PWM1->PERIOD[2] = 0;

        /* Wait until PWM1 channel 2 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CNT[2] & PWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 2 current counter reach to 0 time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for PWM1 channel 2 */
        PWM1->CNTEN &= ~PWM_CNTEN_CNTEN2_Msk;

        /* Disable Capture Function and Capture Input path for  PWM1 channel 2*/
        PWM1->CAPCTL &= ~PWM_CAPCTL_CAPEN2_Msk;
        PWM1->CAPINEN &= ~PWM_CAPINEN_CAPINEN2_Msk;

        /* Clear Capture Interrupt flag for PWM1 channel 2 */
        PWM1->CAPIF = PWM_CAPIF_CFLIF2_Msk;

        PDMA->CHCTL = 0;
    }

lexit:

    while(1);
}
