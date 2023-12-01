/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 15/09/02 10:03a $
 * @brief
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
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0P0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
            PWM0->POEN |= PWM_POEN_POENn_Msk;
        else
            PWM0->POEN &= ~PWM_POEN_POENn_Msk;
        out ^= 1;
        cnt = 0;
    }
    // Clear channel 0 period interrupt flag
    PWM0->INTSTS0 = PWM_INTSTS0_PIFn_Msk;
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

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable PWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PCLK0;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_HXT;

    /* Reset PWM0 module */
    SYS->IPRST2 |= SYS_IPRST2_PWM0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PWM0RST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PC multi-function pins for PWM0 Channel 0~3 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_PWM0_CH0;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC1MFP_PWM0_CH1;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC2MFP_PWM0_CH2;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC3MFP_PWM0_CH3;
//     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk));
//     SYS->GPC_MFPL |= SYS_GPC_MFPL_PC4MFP_PWM0_CH4;
//     SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk));
//     SYS->GPC_MFPL |= SYS_GPC_MFPL_PC5MFP_PWM0_CH5;

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
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output PWM0 channel 0~3 with different\n");
    printf("  frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PC.0), PWM0_CH1(PC.1), PWM0_CH2(PC.2), PWM0_CH3(PC.3)\n");

    // PWM0 channel0 frequency is 600Hz, duty 30%,
    /* Assume PWM output frequency is 100Hz and duty ratio is 30%, user can calculate PWM settings by follows.
       duty ratio = (CMR+1)/(CNR+1)
       cycle time = CNR+1
       High level = CMR+1
       PWM clock source frequency = PLL = 72000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 72000000/2/600 = 60000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 59999
       duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30% ==> CMR = (CNR+1)*0.3-1 = 60000*30/100-1
       CMR = 17999
       Prescale value is 1 : prescaler= 2
    */

    /*Set Pwm mode as complementary mode*/
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /*Set PWM Timer clock prescaler*/
    PWM_SET_PRESCALER(PWM0, 0, 1); // Divided by 2

    /*Set PWM Timer duty*/
    PWM_SET_CMR(PWM0, 0, 17999);
    PWM_SET_CMR(PWM0, 1, 17999);

    /*Set PWM Timer period*/
    PWM_SET_CNR(PWM0, 0, 59999);

    /* Set waveform generation */
    PWM0->WGCTL0 = 0xAAA;
    PWM0->WGCTL1 = 0x555;

    /* enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL0_1 = 400;
    PWM0->DTCTL0_1 |= PWM_DTCTL0_1_DTEN_Msk;
    SYS_LockReg();

    // PWM0 channel2 frequency is 1800Hz, duty 50%
    /* Assume PWM output frequency is 1800Hz and duty ratio is 50%, user can calculate PWM settings by follows.
       duty ratio = (CMR+1)/(CNR+1)
       cycle time = CNR+1
       High level = CMR+1
       PWM clock source frequency = PLL = 72000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 72000000/2/1800 = 20000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 19999
       duty ratio = 50% ==> (CMR+1)/(CNR+1) = 50% ==> CMR = (CNR+1)*0.5-1 = 20000*50/100-1
       CMR = 9999
       Prescale value is 1 : prescaler= 2
    */

    /*Set PWM0 Timer clock prescaler*/
    PWM_SET_PRESCALER(PWM0, 2, 1); // Divided by 2

    /*Set PWM0 Timer duty*/
    PWM_SET_CMR(PWM0, 2, 9999);
    PWM_SET_CMR(PWM0, 3, 9999);

    /*Set PWM0 Timer period*/
    PWM_SET_CNR(PWM0, 2, 19999);

    /* enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL2_3 = 200;
    PWM0->DTCTL2_3 |= PWM_DTCTL2_3_DTEN_Msk;
    SYS_LockReg();

    // Enable output of all PWM0 channels
    PWM0->POEN |= PWM_POEN_POENn_Msk;

    // Enable PWM0 channel 0 period interrupt, use channel 0 to measure time.
    PWM0->INTEN0 = (PWM0->INTEN0 & ~PWM_INTEN0_PIENn_Msk) | PWM_INTEN0_PIENn_Msk;
    NVIC_EnableIRQ(PWM0P0_IRQn);

    // Start
    PWM0->CNTEN = 0xF;

    while(1);

}
