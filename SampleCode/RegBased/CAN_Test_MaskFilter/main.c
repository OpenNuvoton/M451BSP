/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 7 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Use message object No.1 to send message objects (ID=0x700~0x70F).
 *           This sample code needs to work with CAN_Set_MaskFilter.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"


#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);
void CANShowMsg(STR_CANMSG_T* Msg);

/*---------------------------------------------------------------------------*/
/*  Callback function                                                        */
/*---------------------------------------------------------------------------*/

/* Declare a CAN message structure */
STR_CANMSG_T rrMsg;

/*---------------------------------------------------------------------------------------------------------*/
/* Clear interrupt pending bit                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void ClrIntPendingGBit(CAN_T *tCAN, uint8_t u32MsgNum)
{
    uint32_t u32MsgIfNum = 0;
    uint32_t u32IFBusyCount = 0;

    while(u32IFBusyCount < 0x10000000)
    {
        if((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) == 0)
        {
            u32MsgIfNum = 0;
            break;
        }
        else if((tCAN->IF[1].CREQ  & CAN_IF_CREQ_BUSY_Msk) == 0)
        {
            u32MsgIfNum = 1;
            break;
        }

        u32IFBusyCount++;
    }

    tCAN->IF[u32MsgIfNum].CMASK = CAN_IF_CMASK_CLRINTPND_Msk | CAN_IF_CMASK_TXRQSTNEWDAT_Msk;
    tCAN->IF[u32MsgIfNum].CREQ = 1 + u32MsgNum;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Gets the message                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t ReadMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg)
{
    uint32_t u32TimeOutCnt;

    if(!CAN_GET_NEW_DATA_IN_BIT(tCAN, u8MsgObj))
    {
        return FALSE;
    }

    tCAN->STATUS &= (~CAN_STATUS_RXOK_Msk);

    /* read the message contents */
    tCAN->IF[1].CMASK = CAN_IF_CMASK_MASK_Msk
                        | CAN_IF_CMASK_ARB_Msk
                        | CAN_IF_CMASK_CONTROL_Msk
                        | CAN_IF_CMASK_CLRINTPND_Msk
                        | CAN_IF_CMASK_TXRQSTNEWDAT_Msk
                        | CAN_IF_CMASK_DATAA_Msk
                        | CAN_IF_CMASK_DATAB_Msk;

    tCAN->IF[1].CREQ = 1 + u8MsgObj;

    /* Wait */
    u32TimeOutCnt = CAN_TIMEOUT;
    while(tCAN->IF[1].CREQ & CAN_IF_CREQ_BUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for CAN_IF1_CREQ register busy flag is cleared time-out!\n");
            return FALSE;
        }
    }

    if((tCAN->IF[1].ARB2 & CAN_IF_ARB2_XTD_Msk) == 0)
    {
        /* standard ID */
        pCanMsg->IdType = CAN_STD_ID;
        pCanMsg->Id     = (tCAN->IF[1].ARB2 & CAN_IF_ARB2_ID_Msk) >> 2;
    }
    else
    {
        /* extended ID */
        pCanMsg->IdType = CAN_EXT_ID;
        pCanMsg->Id  = (((tCAN->IF[1].ARB2) & 0x1FFF) << 16) | tCAN->IF[1].ARB1;
    }

    pCanMsg->DLC     = tCAN->IF[1].MCON & CAN_IF_MCON_DLC_Msk;
    pCanMsg->Data[0] = tCAN->IF[1].DAT_A1 & CAN_IF_DAT_A1_DATA0_Msk;
    pCanMsg->Data[1] = (tCAN->IF[1].DAT_A1 & CAN_IF_DAT_A1_DATA1_Msk) >> CAN_IF_DAT_A1_DATA1_Pos;
    pCanMsg->Data[2] = tCAN->IF[1].DAT_A2 & CAN_IF_DAT_A2_DATA2_Msk;
    pCanMsg->Data[3] = (tCAN->IF[1].DAT_A2 & CAN_IF_DAT_A2_DATA3_Msk) >> CAN_IF_DAT_A2_DATA3_Pos;
    pCanMsg->Data[4] = tCAN->IF[1].DAT_B1 & CAN_IF_DAT_B1_DATA4_Msk;
    pCanMsg->Data[5] = (tCAN->IF[1].DAT_B1 & CAN_IF_DAT_B1_DATA5_Msk) >> CAN_IF_DAT_B1_DATA5_Pos;
    pCanMsg->Data[6] = tCAN->IF[1].DAT_B2 & CAN_IF_DAT_B2_DATA6_Msk;
    pCanMsg->Data[7] = (tCAN->IF[1].DAT_B2 & CAN_IF_DAT_B2_DATA7_Msk) >> CAN_IF_DAT_B2_DATA7_Pos;

    return TRUE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR == 1)
    {
        printf("Msg-0 INT and Callback\n");
        ReadMsgObj(tCAN, 0, &rrMsg);
        CANShowMsg(&rrMsg);
    }
    if(u32IIDR == 5 + 1)
    {
        printf("Msg-5 INT and Callback \n");
        ReadMsgObj(tCAN, 5, &rrMsg);
        CANShowMsg(&rrMsg);
    }
    if(u32IIDR == 31 + 1)
    {
        printf("Msg-31 INT and Callback \n");
        ReadMsgObj(tCAN, 31, &rrMsg);
        CANShowMsg(&rrMsg);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;
        }
        else if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;
        }
        else if((CAN0->ERR & CAN_ERR_TEC_Msk) != 0)
        {
            printf("Transmit error!\n") ;
        }
        else if((CAN0->ERR & CAN_ERR_REC_Msk) != 0)
        {
            printf("Receive error!\n") ;
        }
    }
    else if(u8IIDRstatus != 0)
    {
        MsgInterrupt(CAN0, u8IIDRstatus);

        ClrIntPendingGBit(CAN0, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;    /* Write '0' to clear */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Reset message interface parameters                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void ResetIF(CAN_T *tCAN, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCAN->IF[u8IF_Num].CREQ     = 0x0;          // set bit15 for sending
    tCAN->IF[u8IF_Num].CMASK    = 0x0;
    tCAN->IF[u8IF_Num].MASK1    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].MASK2    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].ARB1     = 0x0;          // ID15~0
    tCAN->IF[u8IF_Num].ARB2     = 0x0;          // MsgVal, eXt, xmt, ID28~16
    tCAN->IF[u8IF_Num].MCON     = 0x0;          // DLC
    tCAN->IF[u8IF_Num].DAT_A1   = 0x0;          // data0,1
    tCAN->IF[u8IF_Num].DAT_A2   = 0x0;          // data2,3
    tCAN->IF[u8IF_Num].DAT_B1   = 0x0;          // data4,5
    tCAN->IF[u8IF_Num].DAT_B2   = 0x0;          // data6,7
}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CANShowMsg(STR_CANMSG_T* Msg)
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
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UARTDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_UART(1);

    /* Enable CAN0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA multi-function pins for CANTX0 and CANRX0 */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA1MFP_CAN0_TXD | SYS_GPA_MFPL_PA0MFP_CAN0_RXD;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*----------------------------------------------------------------------------*/
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void NoteConfigure()
{
    printf("\n\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  About CAN sample code configure                                         |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN         |\n");
    printf("|   Before execute it, please check description as below                   |\n");
    printf("|                                                                          |\n");
    printf("|   1.CAN_TX and CAN_RX should be connected to your CAN transceiver        |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS               |\n");
    printf("|   3.Check the terminal resistor of bus is connected                      |\n");
    printf("|   4.Using UART0 as print message port                                    |\n");
    printf("|                                                                          |\n");
    printf("|  |--------|       |-----------|   CANBUS  |-----------|       |--------| |\n");
    printf("|  |        |------>|           |<--------->|           |<------|        | |\n");
    printf("|  |        |CAN_TX |    CAN    |   CAN_H   |   CAN     |CAN_TX |        | |\n");
    printf("|  |  M451  |       |Transceiver|           |Transceiver|       |  M451  | |\n");
    printf("|  |        |<------|           |<--------->|           |------>|        | |\n");
    printf("|  |        |CAN_RX |           |   CAN_L   |           |CAN_RX |        | |\n");
    printf("|  |--------|       |-----------|           |-----------|       |--------| |\n");
    printf("|  |                                                            |          |\n");
    printf("|  |                                                            |          |\n");
    printf("|  V                                                            V          |\n");
    printf("| UART0                                                         UART0      |\n");
    printf("|(print message)                                          (print message)  |\n");
    printf("+--------------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Test Menu                                                                 */
/*----------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                                                                   |\n");
    printf("|     Test Mask Filter                                              |\n");
    printf("|     (At first, another module board should be set to              |\n");
    printf("|     [CAN_Set_MaskFilter] waiting for receiving data)              |\n");
    printf("|                                                                   |\n");
    printf("+-------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Get current bit rate                                                      */
/*----------------------------------------------------------------------------*/
uint32_t GetCANBitRate(CAN_T  *tCAN)
{
    uint8_t u8Tseg1, u8Tseg2;
    uint32_t u32Bpr;

    u8Tseg1 = (tCAN->BTIME & CAN_BTIME_TSEG1_Msk) >> CAN_BTIME_TSEG1_Pos;
    u8Tseg2 = (tCAN->BTIME & CAN_BTIME_TSEG2_Msk) >> CAN_BTIME_TSEG2_Pos;
    u32Bpr  = (tCAN->BTIME & CAN_BTIME_BRP_Msk) | (tCAN->BRPE << 6);

    return (SystemCoreClock / (u32Bpr + 1) / (u8Tseg1 + u8Tseg2 + 3));
}

/*----------------------------------------------------------------------------*/
/*  Set target baud-rate and Basic mode                                       */
/*----------------------------------------------------------------------------*/
uint32_t CANInit(CAN_T *tCAN, uint32_t u32BaudRate)
{
    uint8_t u8Tseg1, u8Tseg2;
    uint32_t u32Brp;
    uint32_t u32Value;
    uint32_t u32TimeOutCnt;

    /* Set the CAN to enter initialization mode and enable access bit timing register */
    tCAN->CON |= CAN_CON_INIT_Msk;
    tCAN->CON |= CAN_CON_CCE_Msk;

    SystemCoreClockUpdate();

    u8Tseg1 = 2;
    u8Tseg2 = 1;

    u32Value = SystemCoreClock / u32BaudRate;

    while(1)
    {
        if(((u32Value % (u8Tseg1 + u8Tseg2 + 3)) == 0))
            break;
        if(u8Tseg1 < 7)
            u8Tseg2++;

        if((u32Value % (u8Tseg1 + u8Tseg2 + 3)) == 0)
            break;
        if(u8Tseg1 < 15)
            u8Tseg1++;
        else
        {
            u8Tseg1 = 2;
            u8Tseg2 = 1;
            break;
        }
    }

    u32Brp  = SystemCoreClock / (u32BaudRate) / (u8Tseg1 + u8Tseg2 + 3) - 1;

    u32Value = ((uint32_t)u8Tseg2 << CAN_BTIME_TSEG2_Pos) | ((uint32_t)u8Tseg1 << CAN_BTIME_TSEG1_Pos) |
               (u32Brp & CAN_BTIME_BRP_Msk) | (tCAN->BTIME & CAN_BTIME_SJW_Msk);
    tCAN->BTIME = u32Value;
    tCAN->BRPE     = (u32Brp >> 6) & 0x0F;

    /* Set the CAN to leave initialization mode */
    tCAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));

    /* Check INIT bit is released */
    u32TimeOutCnt = CAN_TIMEOUT;
    while(tCAN->CON & CAN_CON_INIT_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for CAN to leave initialization mode time-out!\n");
            return 0;
        }
    }

    return (GetCANBitRate(tCAN));

}

/*----------------------------------------------------------------------------*/
/*  Check the real baud-rate                                                  */
/*----------------------------------------------------------------------------*/
void BaudRateCheck(uint32_t u32BaudRate, uint32_t u32RealBaudRate)
{
    /* Get Core Clock Frequency */
    SystemCoreClockUpdate();

    if(u32BaudRate != u32RealBaudRate)
    {
        printf("\nSet CAN baud-rate is fail\n");
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
/*  Set the CAN speed                                                         */
/*----------------------------------------------------------------------------*/
void SelectCANSpeed(CAN_T  *tCAN)
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

    switch(unItem)
    {
        case '0':
            BaudRate = 1000000;
            RealBaudRate = CANInit(tCAN,  BaudRate);//Set target baud-rate and operation mode.
            break;

        case '1':
            BaudRate = 800000;
            RealBaudRate = CANInit(tCAN,  BaudRate);
            break;

        case '2':
            BaudRate = 500000;
            RealBaudRate = CANInit(tCAN,  BaudRate);
            break;

        case '3':
            BaudRate = 250000;
            RealBaudRate = CANInit(tCAN,  BaudRate);
            break;

        case '4':
            BaudRate = 125000;
            RealBaudRate = CANInit(tCAN,  BaudRate);
            break;

        case '5':
            BaudRate = 100000;
            RealBaudRate = CANInit(tCAN,  BaudRate);
            break;

        case '6':
            BaudRate = 50000;
            RealBaudRate = CANInit(tCAN,  BaudRate);
            break;
    }

    /* Check the real baud-rate is OK */
    BaudRateCheck(BaudRate, RealBaudRate);

}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable CAN                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void StopCAN(void)
{
    /* Disable CAN0 Clock and Reset it */
    SYS->IPRST2 |= SYS_IPRST1_CAN0RST_Msk;          /* CAN0 controller reset */
    SYS->IPRST2 &= ~SYS_IPRST1_CAN0RST_Msk;         /* CAN0 controller normal operation */
    CLK->APBCLK0 &= ~CLK_APBCLK0_CAN0CKEN_Msk;      /* CAN0 clock disabled */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Check if SmartCard slot is presented                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t GetFreeIF(CAN_T *tCAN)
{
    /* Check Read/write action has finished */
    if((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) == 0)
        return 0;
    else if((tCAN->IF[1].CREQ  & CAN_IF_CREQ_BUSY_Msk) == 0)
        return 1;
    else
        return 2;
}

/*----------------------------------------------------------------------------*/
/*  The function is used to configure a transmit object                       */
/*----------------------------------------------------------------------------*/
int32_t SetTxMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg)
{
    /* Set the Message Buffer Register */
    uint8_t u8MsgIfNum = 0;

    /* Check Free Interface for configure */
    if((u8MsgIfNum = GetFreeIF(tCAN)) == 2)
    {
        return FALSE;
    }

    /* Update the contents needed for transmission*/
    tCAN->IF[u8MsgIfNum].CMASK  = CAN_IF_CMASK_WRRD_Msk      /* Transfer data from the selected Message Buffer Registers to the Message Object addressed */
                                  | CAN_IF_CMASK_MASK_Msk      /* Transfer Identifier Mask + MDir + MXtd to Message Object. */
                                  | CAN_IF_CMASK_ARB_Msk       /* Transfer Identifier + Dir + Xtd + MsgVal to Message Object */
                                  | CAN_IF_CMASK_CONTROL_Msk   /* Transfer Control Bits to Message Object */
                                  | CAN_IF_CMASK_DATAA_Msk     /* Transfer Data Bytes [3:0] to Message Object */
                                  | CAN_IF_CMASK_DATAB_Msk;    /* Transfer Data Bytes [7:4] to Message Object */

    if(pCanMsg->IdType == CAN_STD_ID)
    {
        /* Set the Standard ID(11-bit), Message Direction and Message Valid for IFn Arbitration Register */
        tCAN->IF[u8MsgIfNum].ARB1 = 0;
        tCAN->IF[u8MsgIfNum].ARB2 = (((pCanMsg->Id) & 0x7FF) << 2) | CAN_IF_ARB2_DIR_Msk | CAN_IF_ARB2_MSGVAL_Msk;
    }
    else
    {
        /* Set the Extended ID(29-bit), Message Direction, Extended Identifier and Message Valid for IFn Arbitration Register */
        tCAN->IF[u8MsgIfNum].ARB1 = (pCanMsg->Id) & 0xFFFF;
        tCAN->IF[u8MsgIfNum].ARB2 = ((pCanMsg->Id) & 0x1FFF0000) >> 16 | CAN_IF_ARB2_DIR_Msk | CAN_IF_ARB2_XTD_Msk | CAN_IF_ARB2_MSGVAL_Msk;
    }

    if(pCanMsg->FrameType)
        tCAN->IF[u8MsgIfNum].ARB2 |= CAN_IF_ARB2_DIR_Msk;       /* The Message Object is transmitted as a Data Frame */
    else
        tCAN->IF[u8MsgIfNum].ARB2 &= (~CAN_IF_ARB2_DIR_Msk);    /* The Remote Frame with the identifier of this Message Object is transmitted */

    /* Set the data for IFn Data Register */
    tCAN->IF[u8MsgIfNum].DAT_A1 = ((uint16_t)pCanMsg->Data[1] << 8) | pCanMsg->Data[0];
    tCAN->IF[u8MsgIfNum].DAT_A2 = ((uint16_t)pCanMsg->Data[3] << 8) | pCanMsg->Data[2];
    tCAN->IF[u8MsgIfNum].DAT_B1 = ((uint16_t)pCanMsg->Data[5] << 8) | pCanMsg->Data[4];
    tCAN->IF[u8MsgIfNum].DAT_B2 = ((uint16_t)pCanMsg->Data[7] << 8) | pCanMsg->Data[6];

    /* Update the contents needed for transmission*/
    tCAN->IF[u8MsgIfNum].MCON  =  CAN_IF_MCON_NEWDAT_Msk | pCanMsg->DLC | CAN_IF_MCON_TXIE_Msk | CAN_IF_MCON_EOB_Msk;
    /* Set the Message Object in the Message RAM is selected for data transfer */
    tCAN->IF[u8MsgIfNum].CREQ  = 1 + u8MsgObj;

    return TRUE;
}

/*----------------------------------------------------------------------------*/
/*  Set transmit request bit                                                  */
/*----------------------------------------------------------------------------*/
uint32_t SetTxRqst(CAN_T *tCAN, uint8_t u8MsgObj)
{
    /* CAN message structure*/
    STR_CANMSG_T rMsg;

    /* Gets the message */
    ReadMsgObj(tCAN, u8MsgObj, &rMsg);

    /* Access Transmission Request Bit */
    tCAN->IF[0].CMASK  = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_TXRQSTNEWDAT_Msk ;

    /* Set the Message Object in the Message RAM is selected for data transfer */
    tCAN->IF[0].CREQ  = 1 + u8MsgObj;
    return TRUE;
}

/*----------------------------------------------------------------------------*/
/*  Test Mask Filter Function                                                 */
/*----------------------------------------------------------------------------*/
void TestMaskFilter(CAN_T *tCAN)
{
    int32_t i32id = 0;

    /* Declare a CAN message structure */
    STR_CANMSG_T tMsg;

    printf("Use Message Object No.1 to send ID=0x700 ~0x70F\n");
    printf("Enter any key to send\n ");
    GetChar();

    /* Enable CAN interrupt */
    tCAN->CON |= (CAN_CON_IE_Msk | CAN_CON_SIE_Msk);

    /* Install CAN call back functions */
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    /* Enable corresponding NVIC of CAN */
    NVIC_EnableIRQ(CAN0_IRQn);

    while(1)
    {
        /* Send a 11-bit Standard Identifier message */
        tMsg.FrameType = CAN_DATA_FRAME;    /* Set the Frame Type */
        tMsg.IdType   = CAN_STD_ID;     /* Set the Identifier Type */
        tMsg.Id       = 0x700 + i32id;  /* Set the Message Identifier  */
        tMsg.DLC      = 0;                          /* Set the Data Length Code  */

        /* Call CAN_SetTxMsgObj() only Configure Msg RAM */
        if(SetTxMsgObj(tCAN, MSG(1), &tMsg) < 0)
        {
            printf("Set Tx Msg Object failed\n");
            return;
        }

        /* Set transmit request bit */
        SetTxRqst(tCAN, MSG(1));

//        /* Check the message has been successfully transmitted */
//        while((tCAN->STATUS & CAN_STATUS_TXOK_Msk) == 0);
//        /* Clear TXOK flag */
//        tCAN->STATUS &= ~CAN_STATUS_TXOK_Msk;

        /* Generate the Delay Time by Systick */
        CLK_SysTickDelay(100000);

        i32id++;
        if(i32id >= 0x10)
            break;

    }
    printf("\nCheck the receive host received data\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    CAN_T *tCAN;
    tCAN = (CAN_T *) CAN0;
    //tCAN = (CAN_T *) CAN1;

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

//     /* Set GPB12 to control CAN transceiver for Nuvoton board */
//     PB12 = 0;

    /* Some description about how to create test environment */
    NoteConfigure();

    /* Configuring the Bit Timing */
    SelectCANSpeed(tCAN);

    /* Test Menu */
    TestItem();

    printf("Test Mask Filter\n\n");
    printf("Please confirm [CAN_Set_MaskFilter] of receiver is selected.\n");
    printf("Press any key to continue ...\n\n");
    GetChar();

    /* Use Message Object No.1 to send ID=0x700~0x70F */
    TestMaskFilter(tCAN);

    /* Disable CAN */
    StopCAN();

    while(1);

}
