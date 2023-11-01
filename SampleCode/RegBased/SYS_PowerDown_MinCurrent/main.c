/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"


#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HIRC


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       1

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


void PowerDownFunction(void);
void GPB_IRQHandler(void);
void LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    /* Set the processor is deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled and Power-down entry condition */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWTCPU_Msk);

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M451Series.s.
 */
void GPB_IRQHandler(void)
{
    /* To check if PB.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = PB->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

void LvrSetting(void)
{
    if(SET_LVR == 0)
    {
        /* Disable LVR */
        SYS_DISABLE_LVR();
        CLK_SysTickDelay(200);
    }
    else
    {
        /* Enable LVR */
        SYS_ENABLE_LVR();
        CLK_SysTickDelay(200);
    }
}

void PorSetting(void)
{
    if(SET_POR == 0)
    {
        /* Disable POR */
        SYS_DISABLE_POR();
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK->PWRCTL &= ~CLK_PWRCTL_LIRCEN_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LIRCSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK->PWRCTL |= CLK_PWRCTL_LIRCEN_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (CLK->STATUS & CLK_STATUS_LIRCSTB_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LXT == 0)
    {
        /* Disable LXT and wait for LXT stable flag is cleared */
        CLK->PWRCTL &= ~CLK_PWRCTL_LXTEN_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LXTSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LXT and wait for LXT stable flag is set */
        CLK->PWRCTL |= CLK_PWRCTL_LXTEN_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (CLK->STATUS & CLK_STATUS_LXTSTB_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set core clock from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.3 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Enter to Power-Down                                           |\n");
    printf("|  6. Wait for PB.3 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /*
        To measure Power-down current:
        On NuTiny-EVB-M451 V1.3 board, remove components, e.g. Nu-Link-Me, R4 and R5.
    */

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFPL = 0;
    SYS->GPA_MFPH = 0;
    SYS->GPB_MFPL = 0;
    SYS->GPB_MFPH = 0;
    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;
    SYS->GPD_MFPL = SYS_GPD_MFPL_PD1MFP_UART0_TXD;
    SYS->GPD_MFPH = 0;
    SYS->GPE_MFPL = 0;
    SYS->GPE_MFPH = 0;
    SYS->GPF_MFPL = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode. They are default output high. */
    PA->MODE = 0xFFFFFFFF;
    PB->MODE = 0xFFFFFFFF;
    PC->MODE = 0xFFFFFFFF;
    PD->MODE = 0xFFFFFFFF;
    PE->MODE = 0xFFFFFFFF;
    PF->MODE = 0xFFFFFFFF;

    /* Unlock protected registers for Power-down setting */
    SYS_UnlockReg();

    /* LVR setting */
    LvrSetting();

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if( LircSetting() < 0 ) goto lexit;

    /* LXT setting */
    if( LxtSetting() < 0 ) goto lexit;

    /* Wake-up source configuration */
    /* Configure PB.3 as Quasi mode and enable interrupt by falling edge trigger */
    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE3_Msk)) | (GPIO_MODE_QUASI << GPIO_MODE_MODE3_Pos);
    PB->INTTYPE |= (GPIO_INTTYPE_EDGE << GPIO_INTTYPE_TYPE3_Pos);
    PB->INTEN |= GPIO_INTEN_FLIEN3_Msk;
    NVIC_EnableIRQ(GPB_IRQn);

    /* Enter to Power-down mode */
    printf("Enter to Power-Down ......\n");
    PowerDownFunction();

    /* Waiting for PB.3 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while(1);
}
