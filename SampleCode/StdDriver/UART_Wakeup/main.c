/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 8 $
 * $Date: 15/09/02 10:05a $
 * @brief    Show how to wake up system form Power-down mode by UART interrupt.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter to Power-down mode */
    CLK_PowerDown();

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

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PB multi-function pins for UART1 RXD(PB.2), TXD(PB.3) and nCTS(PB.4) */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB4MFP_UART1_nCTS);

    /* Set PB multi-function pins for UART1 nRTS(PB.8) */
    SYS->GPB_MFPH &= (~SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB8MFP_UART1_nRTS;

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

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 110);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
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

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART Power-down and Wake-up sample function */
    UART_PowerDownWakeUpTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_WKIF_Msk))  /* UART wake-up interrupt flag */
    {
        if(UART_GET_INT_FLAG(UART1, UART_INTSTS_DATWKIF_Msk))         /* UART data wake-up interrupt flag */
        {
            printf("UART data wake-up interrupt happen.\n");
            UART_ClearIntFlag(UART1, UART_INTSTS_DATWKIF_Msk);    /* Clear UART data wake-up interrupt flag */
        }

        if(UART_GET_INT_FLAG(UART1, UART_INTSTS_CTSWKIF_Msk))   /* UART CTS wake-up interrupt flag */
        {
            printf("UART nCTS wake-up interrupt happen.\n");
            UART_ClearIntFlag(UART1, UART_INTSTS_CTSWKIF_Msk);   /* Clear UART CTS wake-up interrupt flag */
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{

    /* Enable UART data wake-up interrupt */
    UART_EnableInt(UART1, UART_INTEN_WKDATIEN_Msk);
    NVIC_EnableIRQ(UART1_IRQn);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 110bps to UART1 to wake-up system.\n");

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_UnlockReg();

    /* Wait to receive wake-up data */
    while(!UART_IS_RX_READY(UART1));
    printf("The first wake-up data is 0x%x.\n", UART1->DAT);

    /* Disable UART Data Wake-up Interrupt */
    UART_DisableInt(UART1, UART_INTEN_WKDATIEN_Msk);
    NVIC_DisableIRQ(UART1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{

    /* Clear MODEM interrupt before CTS wake-up interrupt */
    UART_ClearIntFlag(UART1, UART_INTSTS_MODEMINT_Msk);

    /* Enable UART CTS wake-up interrupt */
    UART_EnableInt(UART1, UART_INTEN_WKCTSIEN_Msk);
    NVIC_EnableIRQ(UART1_IRQn);

    printf("System enter to Power-down mode.\n");
    printf("Toggle nCTS of UART1 to wake-up system.\n");

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_UnlockReg();

    /* Clear MODEM interrupt after CTS wake-up interrupt */
    UART_ClearIntFlag(UART1, UART_INTSTS_MODEMINT_Msk);

    /* Disable UART CTS Wake-up Interrupt */
    UART_DisableInt(UART1, UART_INTEN_WKCTSIEN_Msk);
    NVIC_DisableIRQ(UART1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Data wake-up test                                     |\n");
    printf("| [2] nCTS wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
        case '1':
            UART_DataWakeUp();
            break;
        case '2':
            UART_CTSWakeUp();
            break;
        default:
            break;
    }

    printf("\nUART Sample Program End.\n");
}
