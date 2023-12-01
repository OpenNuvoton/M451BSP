/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 11 $
 * $Date: 17/01/25 9:26a $
 * @brief
 *           Transmit and receive data in UART RS485 mode.
 *           This sample code needs to work with UART_RS485_Master.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"


#define PLL_CLOCK           72000000


#define IS_USE_RS485NMM     1      //1:Select NMM_Mode , 0:Select AAD_Mode
#define MATCH_ADDRSS1       0xC0
#define MATCH_ADDRSS2       0xA2
#define UNMATCH_ADDRSS1     0xB1
#define UNMATCH_ADDRSS2     0xD3


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void RS485_HANDLE(void);
void RS485_9bitModeSlave(void);
void RS485_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    RS485_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE()
{
    volatile uint32_t addr = 0;
    volatile uint32_t regRX = 0xFF;
    volatile uint32_t u32IntSts = UART1->INTSTS;;

    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RLSINT_Msk) && UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk))      /* RLS INT & RDA INT */
    {
        if(UART_RS485_GET_ADDR_FLAG(UART1))         /* RS485 address byte detect flag */
        {
            addr = UART_READ(UART1);
            UART_RS485_CLEAR_ADDR_FLAG(UART1);      /* Clear RS485 address byte detect flag */
            printf("\nAddr=0x%x,Get:", addr);

#if (IS_USE_RS485NMM ==1) //RS485_NMM
            /* if address match, enable RX to receive data, otherwise to disable RX. */
            /* In NMM mode,user can decide multi-address filter. In AAD mode,only one address can set */
            if((addr == MATCH_ADDRSS1) || (addr == MATCH_ADDRSS2))
            {
                UART1->FIFO &= ~UART_FIFO_RXOFF_Msk;   /* Enable RS485 RX */
            }
            else
            {
                printf("\n");
                UART1->FIFO |= UART_FIFO_RXOFF_Msk;    /* Disable RS485 RX */
                UART1->FIFO |= UART_FIFO_RXRST_Msk;    /* Clear data from RX FIFO */
            }
#endif
        }
    }
    else if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk) || UART_GET_INT_FLAG(UART1, UART_INTSTS_RXTOINT_Msk))       /* Rx Ready or Time-out INT*/
    {
        /* Handle received data */
        printf("%d,", UART1->DAT);
    }

    else if(UART_GET_INT_FLAG(UART1, UART_INTSTS_BUFERRINT_Msk))       /* Buffer Error INT */
    {
        printf("\nBuffer Error...\n");
        UART_ClearIntFlag(UART1, UART_INTSTS_BUFERRINT_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test  (IS_USE_RS485NMM: 0:AAD  1:NMM)                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave()
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Set Data Format, only need parity enable whatever parity ODD/EVEN */
    UART_SetLine_Config(UART1, 0, UART_WORD_LEN_8, UART_PARITY_EVEN, UART_STOP_BIT_1);

    /* Set RTS pin active level as high level active */
    UART1->MODEM = (UART1->MODEM & (~UART_MODEM_RTSACTLV_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

#if(IS_USE_RS485NMM == 1)
    printf("+-----------------------------------------------------------+\n");
    printf("|    Normal Multidrop Operation Mode                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Only Address %x and %x,data can receive                   |\n", MATCH_ADDRSS1, MATCH_ADDRSS2);
    printf("+-----------------------------------------------------------+\n");

    /* Set Receiver disabled before set RS485-NMM mode */
    UART1->FIFO |= UART_FIFO_RXOFF_Msk;

    /* Set RS485-NMM Mode */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485NMM_Msk | UART_ALTCTL_RS485AUD_Msk, 0);

    /* Set RS485 address detection enable */
    UART1->ALTCTL |= UART_ALTCTL_ADDRDEN_Msk;

#else
    printf("Auto Address Match Operation Mode\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("|    Auto Address Match Operation Mode                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|Only Address %x,data can receive                           |\n", MATCH_ADDRSS1);
    printf("+-----------------------------------------------------------+\n");

    /* Set RS485-AAD Mode and address match is 0xC0 */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485AAD_Msk | UART_ALTCTL_RS485AUD_Msk, MATCH_ADDRSS1);

    /* Set RS485 address detection enable */
    UART1->ALTCTL |= UART_ALTCTL_ADDRDEN_Msk;

#endif

    /* Enable RDA\RLS\Time-out Interrupt */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    /* Enable UART1 interrupt */
    NVIC_EnableIRQ(UART1_IRQn);

    printf("Ready to receive data...(Press any key to stop test)\n");
    GetChar();

    /* Flush FIFO */
    while(UART_GET_RX_EMPTY(UART1) == 0)
    {
        UART_READ(UART1);
        if(--u32TimeOutCnt == 0) break;
    }

    /* Disable RDA/RLS/RTO interrupt */
    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    /* Set UART Function */
    UART_Open(UART1, 115200);

    printf("\n\nEnd test\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RS485 Function Test IO Setting                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PB.3)        UART1_RXD(PB.2)--|Slave| |\n");
    printf("| |      |--UART1_nRTS(PB.8)      UART1_nRTS(PB.8)--|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");


    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.
        Master:
            1.Set AUD mode and HW will control RTS pin. RTSACTLV is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UART_LINE = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UART_LINE = 0x3B)
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. RTSACTLV is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match address match value.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check if RS485 address byte detect flag is set and read RX FIFO data to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until RXOFF is disabled.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match, clear RXOFF bit to receive data byte.
              If the ADDRESS is not match, set RXOFF bit to avoid data byte stored in FIFO.

        Note: User can measure transmitted data waveform on TXD and RXD pin.
              RTS pin is used for RS485 transceiver to control transmission direction.
              RTS pin is low in idle state. When master is sending data, RTS pin will be pull high.
              The connection to RS485 transceiver is as following figure for reference.
               __________     ___________      ___________      __________
              |          |   |           |    |           |    |          |
              |Master    |   |RS485      |    |RS485      |    |Slave     |
              | UART_TXD |---|Transceiver|<==>|Transceiver|----| UART_RXD |
              | UART_nRTS|---|           |    |           |----| UART_nRTS|
              |__________|   |___________|    |___________|    |__________|
    */

    RS485_9bitModeSlave();

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
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PB multi-function pins for UART1 RXD(PB.2) and TXD(PB.3) */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    /* Set PB multi-function pins for UART1 nRTS(PB.8) */
    SYS->GPB_MFPH &= (~SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB8MFP_UART1_nRTS;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 module */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
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

    /* Init UART1 for testing */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART RS485 sample slave function */
    RS485_FunctionTest();

    while(1);

}
