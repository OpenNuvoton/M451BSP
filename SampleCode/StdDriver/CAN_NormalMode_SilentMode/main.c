/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Use CAN Silent mode to listen to CAN bus communication test in Normal mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK       48000000

/*---------------------------------------------------------------------------*/
/* Function Declare                                                          */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);
extern uint32_t CAN_GetCANBitRate(CAN_T *tCAN);
extern void CAN_EnterTestMode(CAN_T *tCAN, uint8_t u8TestMask);

/*---------------------------------------------------------------------------------------------------------*/
/* Reset message interface parameters                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ResetIF(CAN_T *tCAN, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCAN->IF[u8IF_Num].CREQ     = 0x0;          // set bit15 for sending
    tCAN->IF[u8IF_Num].CMASK    = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk | CAN_IF_CMASK_ARB_Msk |
                                  CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;
    tCAN->IF[u8IF_Num].MASK1    = 0x0;          // useless in silent mode
    tCAN->IF[u8IF_Num].MASK2    = 0x0;          // useless in silent mode
    tCAN->IF[u8IF_Num].ARB1     = 0x0;          // ID15~0
    tCAN->IF[u8IF_Num].ARB2     = CAN_IF_ARB2_MSGVAL_Msk;
    tCAN->IF[u8IF_Num].MCON     = CAN_IF_MCON_UMASK_Msk | CAN_IF_MCON_RXIE_Msk;
    tCAN->IF[u8IF_Num].DAT_A1   = 0x0;          // data0,1
    tCAN->IF[u8IF_Num].DAT_A2   = 0x0;          // data2,3
    tCAN->IF[u8IF_Num].DAT_B1   = 0x0;          // data4,5
    tCAN->IF[u8IF_Num].DAT_B2   = 0x0;          // data6,7
}

/*---------------------------------------------------------------------------*/
/* Show Message Function                                                     */
/*---------------------------------------------------------------------------*/
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    /* Show the message information */
    printf("Read ID=0x%X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    for(i = 0; i < Msg->DLC; i++)
        printf("%X,", Msg->Data[i]);
    printf("\n\n");
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

    /* Enable CAN module clock */
    CLK_EnableModuleClock(CAN0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA multi-function pins for CAN0 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA1MFP_CAN0_TXD | SYS_GPA_MFPL_PA0MFP_CAN0_RXD);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init CAN                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_SilentMode_Init(CAN_T *tCAN)
{
    printf("CAN monitoring baud rate(bps): %d\n\n", CAN_GetCANBitRate(tCAN));

    /* Enable the Silent Mode */
    CAN_EnterTestMode(tCAN, CAN_TEST_SILENT_Msk);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable CAN Clock and Reset it                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_STOP(void)
{
    /* Disable CAN0 Clock and Reset it */
    SYS_ResetModule(CAN0_RST);
    CLK_DisableModuleClock(CAN0_MODULE);
}

/*----------------------------------------------------------------------------*/
/* Some description about how to create test environment                      */
/*----------------------------------------------------------------------------*/
void Note_Configure(void)
{
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                    CAN Normal + Silent Mode Sample Code                     |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                              |\n");
    printf("|    The sample code needs three boards. Use CAN Silent mode to listen to     |\n");
    printf("|    CAN Bus communication test. The sample code must be set up on the node   |\n");
    printf("|    of CAN communication transmission. Users can use the sample codes        |\n");
    printf("|    ' CAN_NormalMode_Transmit ' and ' CAN_NormalMode_Receive ' as the CAN    |\n");
    printf("|    communication transmission network.                                      |\n");
    printf("|    Note: Users need to confirm whether the transmission rate matches.       |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                Pin Configure                                |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                                         CAN_TXD(Any board)  |\n");
    printf("|                                CANBUS                   CAN_RXD(Any board)  |\n");
    printf("|                                  ||    CAN_H   |-----------|                |\n");
    printf("|                                  || <--------->|           |<------         |\n");
    printf("|                                  ||            |    CAN    |CAN_TX          |\n");
    printf("|      CAN_TXD                     ||    CAN_L   |Transceiver|                |\n");
    printf("|      CAN_RXD                     || <--------->|           |------>         |\n");
    printf("|         |-----------|   CAN_H    ||            |           |CAN_RX          |\n");
    printf("|  ------>|           |<---------> ||            |-----------|                |\n");
    printf("|   CAN_TX|   CAN     |            ||                                         |\n");
    printf("|         |Transceiver|            ||                     CAN_TXD(Any board)  |\n");
    printf("|  <------|           |   CAN_L    ||                     CAN_RXD(Any board)  |\n");
    printf("|   CAN_RX|           |<---------> ||    CAN_H   |-----------|                |\n");
    printf("|         |-----------|            || <--------->|           |<------         |\n");
    printf("|                                  ||            |    CAN    |CAN_TX          |\n");
    printf("|                                  ||    CAN_L   |Transceiver|                |\n");
    printf("|                                  || <--------->|           |------>         |\n");
    printf("|                                  ||            |           |CAN_RX          |\n");
    printf("|                                  ||            |-----------|                |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");
}

/*----------------------------------------------------------------------------*/
/* Check the real baud-rate                                                   */
/*----------------------------------------------------------------------------*/
void BaudRateCheck(uint32_t u32BaudRate, uint32_t u32RealBaudRate)
{
    /* Get Core Clock Frequency */
    SystemCoreClockUpdate();

    if(u32BaudRate != u32RealBaudRate)
    {
        printf("Set CAN baud-rate is fail\n");
        printf("Real baud-rate value(bps): %d\n", u32RealBaudRate);
        printf("CAN baud-rate calculation equation as below:\n");
        printf("CAN baud-rate(bps) = Fin/(BPR+1)*(Tseg1+Tseg2+3)\n");
        printf("where: Fin: System clock freq.(Hz)\n");
        printf("       BRP: The baud rate prescale. It is composed of BRP (CAN_BTIME[5:0]) and BRPE (CAN_BRPE[3:0]).\n");
        printf("       Tseg1: Time Segment before the sample point. You can set tseg1 (CAN_BTIME[11:8]).\n");
        printf("       Tseg2: Time Segment after the sample point. You can set tseg2 (CAN_BTIME[14:12]).\n");

        if(SystemCoreClock % u32BaudRate != 0)
            printf("\nThe BPR does not calculate, the Fin must be a multiple of the CAN baud-rate.\n");

        else
            printf("\nThe BPR does not calculate, the (Fin/(CAN baud-rate)) must be a multiple of the (Tseg1+Tseg1+3).\n");
    }
    else
        printf("\nReal baud-rate value(bps): %d\n", u32RealBaudRate);
}

/*----------------------------------------------------------------------------*/
/* Set the CAN speed                                                          */
/*----------------------------------------------------------------------------*/
void SelectCANSpeed(CAN_T *tCAN)
{
    uint32_t unItem, BaudRate = 0, RealBaudRate = 0;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  800Kbps\n");
    printf("[2]  500Kbps\n");
    printf("[3]  250Kbps\n");
    printf("[4]  125Kbps\n");
    printf("[5]  100Kbps\n");
    printf("[6]   50Kbps\n");

    unItem = GetChar();
    printf("%c\n", unItem);

    /* Set target baud-rate and operation mode */
    switch(unItem)
    {
        case '0':
            BaudRate = 1000000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;

        case '1':
            BaudRate = 800000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;

        case '2':
            BaudRate = 500000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;

        case '3':
            BaudRate = 250000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;

        case '4':
            BaudRate = 125000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;

        case '5':
            BaudRate = 100000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;

        case '6':
            BaudRate = 50000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_NORMAL_MODE);
            break;
    }

    /* Check the real baud-rate is OK */
    BaudRateCheck(BaudRate, RealBaudRate);
}

/*----------------------------------------------------------------------------*/
/* Receive Rx Msg by Normal Mode Function (With Message RAM)                  */
/*----------------------------------------------------------------------------*/
void CAN_NormalMode_SetRxMsg(CAN_T *tCAN)
{
    uint32_t i;

    for(i = 1; i <= 32; i++)
    {
        tCAN->IF[0].MASK1 = 0;
        tCAN->IF[0].MASK2 = 0;
        tCAN->IF[0].ARB1 = 0;
        tCAN->IF[0].ARB2 = CAN_IF_ARB2_MSGVAL_Msk;

        tCAN->IF[0].MCON |= CAN_IF_MCON_UMASK_Msk | CAN_IF_MCON_RXIE_Msk;
        if(i == 32)
            tCAN->IF[0].MCON |= CAN_IF_MCON_EOB_Msk;
        else
            tCAN->IF[0].MCON &= (~CAN_IF_MCON_EOB_Msk);

        tCAN->IF[0].CMASK = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk | CAN_IF_CMASK_ARB_Msk |
                            CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;

        tCAN->IF[0].CREQ = i;

        while(tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    STR_CANMSG_T rrMsg[3];
    uint8_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Some description about how to create test environment */
    Note_Configure();

    /* Configuring the Bit Timing */
    SelectCANSpeed(CAN0);

    /* CAN interface initialization */
    CAN_SilentMode_Init(CAN0);

    CAN_NormalMode_SetRxMsg(CAN0);

    for(i = 0; i < 3; i++)
    {
        CAN_ResetIF(CAN0, 0);
        while(CAN_Receive(CAN0, 0, &rrMsg[i]) == FALSE);
    }
    for(i = 0; i < 3; i++)
        CAN_ShowMsg(&rrMsg[i]);

    /* Disable CAN */
    CAN_Close(CAN0);

    /* Disable CAN Clock and Reset it */
    CAN_STOP();

    while(1);
}
