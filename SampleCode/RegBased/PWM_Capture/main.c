/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 15/09/02 10:03a $
 * @brief    Capture the PWM1 Channel 0 waveform by PWM1 Channel 2.
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
    uint32_t u32CapIntFlag;
    u32CapIntFlag = PWM1->CAPIF;

    if(u32CapIntFlag & PWM_CAPIF_CFLIF2_Msk)
    {
        PWM1->CAPIF = PWM_CAPIF_CFLIF2_Msk;
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
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
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    PWM1->CAPIF = PWM_CAPIF_CFLIF2_Msk;

    /* Wait for Capture Falling Indicator */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((PWM1->CAPIF & PWM_CAPIF_CFLIF2_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B) */
    PWM1->CAPIF = PWM_CAPIF_CFLIF2_Msk;

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CAPIF & PWM_CAPIF_CFLIF2_Msk) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        PWM1->CAPIF = PWM_CAPIF_CFLIF2_Msk | PWM_CAPIF_CRLIF2_Msk;

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_FALLING_DATA(PWM1, 2);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CAPIF & PWM_CAPIF_CRLIF2_Msk) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        PWM1->CAPIF = PWM_CAPIF_CRLIF2_Msk;

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_RISING_DATA(PWM1, 2);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeriod = u32Count[1] - u32Count[2];

    u16LowPeriod = 0x10000 - u32Count[1];

    u16TotalPeriod = 0x10000 - u32Count[2];

    printf("\nPWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 7199) || (u16HighPeriod > 7201) || (u16LowPeriod < 16799) || (u16LowPeriod > 16801) || (u16TotalPeriod < 23999) || (u16TotalPeriod > 24001))
    {
        printf("Capture Test Fail!!\n");
        return -1;
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
}

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
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;
    u32TimeOutCnt = __HIRC;
	while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable PWM1 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM1CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM1SEL_Msk) | CLK_CLKSEL2_PWM1SEL_PCLK1;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM1SEL_Msk) | CLK_CLKSEL2_PWM1SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

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
    printf("  This sample code will use PWM1 channel 2 to capture\n  the signal from PWM1 channel 0.\n");
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

        /* Enable capture falling edge interrupt for PWM1 channel 2 */
        PWM1->CAPIEN |= PWM_CAPIEN_CAPFIEN2_Msk;

        /* Enable capture function */
        PWM1->CAPCTL |= PWM_CAPCTL_CAPEN2_Msk;

        /* Enable falling capture reload */
        PWM1->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

        NVIC_EnableIRQ(PWM1P1_IRQn);

        // Start
        PWM1->CNTEN |= PWM_CNTEN_CNTEN2_Msk;

        /* Wait until PWM1 channel 2 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CNT[2]) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 2 Timer start to count time-out!\n");
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
                printf("Wait for PWM1 channel 0 Timer Stop time-out!\n");
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

        /* Disable PWM1 NVIC */
        NVIC_DisableIRQ(PWM1P1_IRQn);

        /* Set loaded value as 0 for PWM1 channel 2 */
        PWM1->PERIOD[2] = 0;

        /* Wait until PWM1 channel 2 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock;  /* 1 second time-out */
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

        /* Disable Capture Function and Capture Input path for PWM1 channel 2 */
        PWM1->CAPCTL &= ~PWM_CAPCTL_CAPEN2_Msk;
        PWM1->CAPINEN &= ~PWM_CAPINEN_CAPINEN2_Msk;

        /* Clear Capture Interrupt flag for PWM1 channel 2 */
        PWM1->CAPIF = PWM_CAPIF_CFLIF2_Msk;

    }

lexit:

    while(1);
}
