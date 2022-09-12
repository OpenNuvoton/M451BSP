/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/02 10:04a $
 * @brief    Capture the PWM1 Channel 0 waveform by PWM1 Channel 2, and use PDMA to transfer captured data.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       144000000

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
int32_t CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
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
            return -1;
        }
    }

    u16RisingTime = g_u32Count[1];

    u16FallingTime = g_u32Count[0];

    u16HighPeriod = g_u32Count[1] - g_u32Count[2];

    u16LowPeriod = 0x10000 - g_u32Count[1];

    u16TotalPeriod = 0x10000 - g_u32Count[2];

    printf("\nPWM generate: \nHigh Period=17279 ~ 17281, Low Period=40319 ~ 40321, Total Period=57599 ~ 57601\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 17279) || (u16HighPeriod > 17281) || (u16LowPeriod < 40319) || (u16LowPeriod > 40321) || (u16TotalPeriod < 57599) || (u16TotalPeriod > 57601))
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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Enable PWM1 module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, NULL);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Reset PWM1 module */
    SYS_ResetModule(PWM1_RST);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

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
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency = PLL = 72000000
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                   = 72000000/5/250 = 57600
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 57599
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 17279
           Prescale value is 4 : prescaler= 5
        */

        /* Set PWM1 channel 0 output configuration */
        PWM_ConfigOutputChannel(PWM1, 0, 250, 30);

        /* Enable PWM Output path for PWM1 channel 0 */
        PWM_EnableOutput(PWM1, PWM_CH_0_MASK);

        /* Enable Timer for PWM1 channel 0 */
        PWM_Start(PWM1, PWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Configure PDMA peripheral mode form PWM to memory                                    */
        /*--------------------------------------------------------------------------------------*/
        /* Open Channel 0 */
        PDMA_Open(0x1);

        /* Transfer width is half word(16 bit) and transfer count is 4 */
        PDMA_SetTransferCnt(0, PDMA_WIDTH_16, 4);

        /* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_u32Count array(increment) */
        PDMA_SetTransferAddr(0, (uint32_t)&PWM1->PDMACAP2_3, PDMA_SAR_FIX, (uint32_t)&g_u32Count[0], PDMA_DAR_INC);

        /* Select PDMA request source as PWM RX(PWM1 channel 2 should be PWM1 pair 2) */
        PDMA_SetTransferMode(0, PDMA_PWM1_P2_RX, FALSE, 0);

        /* Set PDMA as single request type for PWM */
        PDMA_SetBurstType(0, PDMA_REQ_SINGLE, PDMA_BURST_4);

        PDMA_EnableInt(0, PDMA_INT_TRANS_DONE);
        NVIC_EnableIRQ(PDMA_IRQn);

        /* Enable PDMA for PWM1 channel 2 capture function, and set capture order as falling first, */
        /* And select capture mode as both rising and falling to do PDMA transfer. */
        PWM_EnablePDMA(PWM1, 2, FALSE, PWM_CAPTURE_PDMA_RISING_FALLING_LATCH);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 channel 2 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 72000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 72000000/5/250 = 57600
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1/Capture clock source frequency/prescaler
           69.4ns = 1/72000000/5
        */

        /* Set PWM1 channel 2 capture configuration */
        PWM_ConfigCaptureChannel(PWM1, 2, 69, 0);

        /* Enable Timer for PWM1 channel 2 */
        PWM_Start(PWM1, PWM_CH_2_MASK);

        /* Enable Capture Function for PWM1 channel 2 */
        PWM_EnableCapture(PWM1, PWM_CH_2_MASK);

        /* Enable falling capture reload */
        PWM1->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

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

        /* Capture the Input Waveform Data */
        if( CalPeriodTime(PWM1, 2) < 0 ) goto lexit;
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set PWM1 channel 0 loaded value as 0 */
        PWM_Stop(PWM1, PWM_CH_0_MASK);

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
        PWM_ForceStop(PWM1, PWM_CH_0_MASK);

        /* Disable PWM Output path for PWM1 channel 0 */
        PWM_DisableOutput(PWM1, PWM_CH_0_MASK);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 2 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for PWM1 channel 2 */
        PWM_Stop(PWM1, PWM_CH_2_MASK);

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
        PWM_ForceStop(PWM1, PWM_CH_2_MASK);

        /* Disable Capture Function and Capture Input path for PWM1 channel 2 */
        PWM_DisableCapture(PWM1, PWM_CH_2_MASK);

        /* Clear Capture Interrupt flag for PWM1 channel 2 */
        PWM_ClearCaptureIntFlag(PWM1, 2, PWM_CAPTURE_INT_FALLING_LATCH);

        /* Disable PDMA NVIC */
        NVIC_DisableIRQ(PDMA_IRQn);

        PDMA_Close();
    }

lexit:

    while(1);
}
