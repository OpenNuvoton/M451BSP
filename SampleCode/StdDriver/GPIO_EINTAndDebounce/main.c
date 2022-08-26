/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/09/02 10:04a $
 * @brief    Show the usage of GPIO external interrupt function and de-bounce function.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"


#define PLL_CLOCK       72000000


/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0 default IRQ, declared in startup_M451Series.s.
 */
void EINT0_IRQHandler(void)
{

    /* To check if PA.0 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT0))
    {
        GPIO_CLR_INT_FLAG(PA, BIT0);
        printf("PA.0 EINT0 occurred.\n");
    }

    /* To check if PD.2 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PD, BIT2))
    {
        GPIO_CLR_INT_FLAG(PD, BIT2);
        printf("PD.2 EINT0 occurred.\n");
    }

}

/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1 default IRQ, declared in startup_M451Series.s.
 */
void EINT1_IRQHandler(void)
{

    /* To check if PB.0 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT0))
    {
        GPIO_CLR_INT_FLAG(PB, BIT0);
        printf("PB.0 EINT1 occurred.\n");
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA multi-function pin for EINT0(PA.0) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_INT0;

    /* Set PD multi-function pin for EINT0(PD.2) */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD2MFP_Msk)) | SYS_GPD_MFPL_PD2MFP_INT0;

    /* Set PB multi-function pin for EINT1(PB.0) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_INT1;

}

void UART0_Init(void)
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
    printf("+------------------------------------------------------------+\n");
    printf("|    GPIO EINT0/EINT1 Interrupt and De-bounce Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("EINT0(PA.0 and PD.2) and EINT1(PB.0) are used to test interrupt\n");

    /* Configure PA.0 as EINT0 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 0, GPIO_INT_FALLING);

    /* Configure PD.2 as EINT0 pin and enable interrupt by rising edge trigger */
    GPIO_SetMode(PD, BIT2, GPIO_MODE_INPUT);
    GPIO_EnableInt(PD, 2, GPIO_INT_RISING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PB.0 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 0, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT0);
    GPIO_ENABLE_DEBOUNCE(PB, BIT0);
    GPIO_ENABLE_DEBOUNCE(PD, BIT2);

    /* Waiting for interrupts */
    while(1);
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
