/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Implement transmit/receive message in Normal mode of CAN.
 *           This sample code needs to work with another Nu-LB-M451 board.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "M451Series.h"


/*---------------------------------------------------------------------------*/
/*  TFT LCD Functions                                                        */
/*---------------------------------------------------------------------------*/
#define SPI_LCD_PORT    SPI2

#define ILI9341_RESET   PB15
#define ILI9341_DC      PB11
#define ILI9341_LED     PB5

extern uint8_t Font8x16[];

#define White           0xFFFF
#define Black           0x0000
#define Blue            0x001F
#define Blue2           0x051F
#define Red             0xF800
#define Magenta         0xF81F
#define Green           0x07E0
#define Cyan            0x7FFF
#define Yellow          0xFFE0

uint8_t LCD_ReadReg(uint8_t u8Comm)
{
    SPI_ClearRxFIFO(SPI_LCD_PORT);

    ILI9341_DC = 0;

    SPI_WRITE_TX(SPI_LCD_PORT, u8Comm);
    SPI_WRITE_TX(SPI_LCD_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_LCD_PORT));

    SPI_READ_RX(SPI_LCD_PORT);

    return (SPI_READ_RX(SPI_LCD_PORT));

}

void LCD_WriteCommand(uint8_t u8Comm)
{
    ILI9341_DC = 0;

    SPI_WRITE_TX(SPI_LCD_PORT, u8Comm);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_LCD_PORT));

}

void LCD_WriteData(uint8_t u8Data)
{
    ILI9341_DC = 1;

    SPI_WRITE_TX(SPI_LCD_PORT, u8Data);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_LCD_PORT));

}

void ILI9341_LCD_SetAddress(uint32_t x1, uint32_t x2, uint32_t y1, uint32_t y2)
{
    if(x1 >= 240)
        x1 = 239;
    if(x2 >= 240)
        x2 = 239;
    if(y1 >= 320)
        y1 = 319;
    if(y2 >= 320)
        y2 = 319;

    LCD_WriteCommand(0x2a);
    LCD_WriteData(x1 >> 8);
    LCD_WriteData(x1);
    LCD_WriteData(x2 >> 8);
    LCD_WriteData(x2);

    LCD_WriteCommand(0x2b);
    LCD_WriteData(y1 >> 8);
    LCD_WriteData(y1);
    LCD_WriteData(y2 >> 8);
    LCD_WriteData(y2);

}

void ILI9341_LCD_PutChar8x16(uint16_t x, uint16_t y, uint8_t c, uint32_t fColor, uint32_t bColor)
{
    uint32_t i, j;
    for(i = 0; i < 16; i++)
    {
        uint8_t m = Font8x16[c * 16 + i];
        ILI9341_LCD_SetAddress(x + i, x + i, y, y + 7);
        LCD_WriteCommand(0x2c);

        for(j = 0; j < 8; j++)
        {
            if((m & 0x01) == 0x01)
            {
                LCD_WriteData(fColor >> 8);
                LCD_WriteData(fColor);
            }
            else
            {
                LCD_WriteData(bColor >> 8);
                LCD_WriteData(bColor);
            }
            m >>= 1;
        }
    }

}

void ILI9341_LCD_PutString(uint16_t x, uint16_t y, char *s, uint32_t fColor, uint32_t bColor)
{
    uint8_t l = 0;
    while(*s)
    {
        if(*s < 0x80)
        {
            ILI9341_LCD_PutChar8x16(x, 312 - y - l * 8, *s, fColor, bColor);
            s++;
            l++;
        }
    }
}

void ILI9341_LCD_Init(void)
{
    /* Configure DC/RESET/LED pins */
    ILI9341_DC = 0;
    ILI9341_RESET = 0;
    ILI9341_LED = 0;

    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT11, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);

    /* Configure LCD */
    ILI9341_DC = 1;

    ILI9341_RESET = 0;
    TIMER_Delay(TIMER0, 20000);

    ILI9341_RESET = 1;
    TIMER_Delay(TIMER0, 40000);

    LCD_WriteCommand(0xCB);
    LCD_WriteData(0x39);
    LCD_WriteData(0x2C);
    LCD_WriteData(0x00);
    LCD_WriteData(0x34);
    LCD_WriteData(0x02);

    LCD_WriteCommand(0xCF);
    LCD_WriteData(0x00);
    LCD_WriteData(0xC1);
    LCD_WriteData(0x30);

    LCD_WriteCommand(0xE8);
    LCD_WriteData(0x85);
    LCD_WriteData(0x00);
    LCD_WriteData(0x78);

    LCD_WriteCommand(0xEA);
    LCD_WriteData(0x00);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0xED);
    LCD_WriteData(0x64);
    LCD_WriteData(0x03);
    LCD_WriteData(0x12);
    LCD_WriteData(0x81);

    LCD_WriteCommand(0xF7);
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xC0);
    LCD_WriteData(0x23);

    LCD_WriteCommand(0xC1);
    LCD_WriteData(0x10);

    LCD_WriteCommand(0xC5);
    LCD_WriteData(0x3e);
    LCD_WriteData(0x28);

    LCD_WriteCommand(0xC7);
    LCD_WriteData(0x86);

    LCD_WriteCommand(0x36);
    LCD_WriteData(0x48);

    LCD_WriteCommand(0x3A);
    LCD_WriteData(0x55);

    LCD_WriteCommand(0xB1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x18);

    LCD_WriteCommand(0xB6);
    LCD_WriteData(0x08);
    LCD_WriteData(0x82);
    LCD_WriteData(0x27);

    LCD_WriteCommand(0xF2);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0x26);
    LCD_WriteData(0x01);

    LCD_WriteCommand(0xE0);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x31);
    LCD_WriteData(0x2B);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x08);
    LCD_WriteData(0x4E);
    LCD_WriteData(0xF1);
    LCD_WriteData(0x37);
    LCD_WriteData(0x07);
    LCD_WriteData(0x10);
    LCD_WriteData(0x03);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x09);
    LCD_WriteData(0x00);

    LCD_WriteCommand(0xE1);
    LCD_WriteData(0x00);
    LCD_WriteData(0x0E);
    LCD_WriteData(0x14);
    LCD_WriteData(0x03);
    LCD_WriteData(0x11);
    LCD_WriteData(0x07);
    LCD_WriteData(0x31);
    LCD_WriteData(0xC1);
    LCD_WriteData(0x48);
    LCD_WriteData(0x08);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x31);
    LCD_WriteData(0x36);
    LCD_WriteData(0x0F);

    LCD_WriteCommand(0x11);
    TIMER_Delay(TIMER0, 60000);

    LCD_WriteCommand(0x29);    //Display on

    ILI9341_LED = 1;

}

/*---------------------------------------------------------------------------*/
/*  CAN Functions                                                            */
/*---------------------------------------------------------------------------*/
/* Declare a CAN message structure */
CAN_T *tCAN0 = (CAN_T *)CAN0;

/* Define the FIFO depth for message objects */
#define CAN_FIFO_DEPTH      8

/* Define the message RAM ID uses for Tx */
#define TX_MSG_OBJ_ID           31

/* */
#define MAX_FIFO_MESSAGES   40

typedef struct
{
    uint8_t Head;
    uint8_t Tail;
    STR_CANMSG_T Msg[MAX_FIFO_MESSAGES];
} MSG_BUF_T;

typedef MSG_BUF_T *PMSG_BUF_T;

MSG_BUF_T rrMsg0 = {0}; /* Receive FIFO buffer for CAN0 */

uint8_t ResetCAN0 = FALSE;  /* It will be TRUE if bus off, reset CAN to start again */

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);

void CAN_ShowMsg(STR_CANMSG_T* Msg);
void GetMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg);
void SendMsgObj(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg);
void SetMsgObj_for_Rx(CAN_T *tCAN);
void SendMsgObj_to_Tx(CAN_T *tCAN);

uint32_t CountsInFifo(PMSG_BUF_T pMsgBuf);
uint8_t PutToFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg);
uint8_t GetFromFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg);

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)         /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;       /* Clear RxOK status*/

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;       /* Clear TxOK status*/

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n");
            ResetCAN0 = TRUE;
        }
        else if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n");
            ResetCAN0 = TRUE;
        }
        else if((CAN0->ERR & CAN_ERR_TEC_Msk) != 0)
        {
            printf("Transmit error!\n");
        }
        else if((CAN0->ERR & CAN_ERR_REC_Msk) != 0)
        {
            printf("Receive error!\n");
        }
    }
    else if((u8IIDRstatus >= 0x1) || (u8IIDRstatus <= 0x20))
    {
        STR_CANMSG_T rrMsg;

        uint32_t i;
        uint32_t u32NewDataReg;

        u32NewDataReg = tCAN0->NDAT1 & ((1 << CAN_FIFO_DEPTH) - 1);

        if(u32NewDataReg)
        {
            for(i = 0; (u32NewDataReg != 0) && (i < CAN_FIFO_DEPTH); i++)
            {
                if(u32NewDataReg & 1)
                {
                    GetMsgObj(tCAN0, i, &rrMsg);
                    CAN_ShowMsg(&rrMsg);
                    PutToFifo(&rrMsg0, &rrMsg);
                }
                u32NewDataReg >>= 1;
            }
        }
        else
        {
            CAN_CLR_INT_PENDING_BIT(CAN0, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
        }
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
void CAN_ResetIF(CAN_T *tCAN, uint8_t u8IF_Num)
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
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    /* Show the message information */
    printf("\nRead ID=0x%X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    ILI9341_LCD_PutString(32, 0, "==> Receive Message In!", Red, Yellow);
    for(i = 0; i < Msg->DLC; i++)
        printf("%X,", Msg->Data[i]);

    printf("\n");

}

/*---------------------------------------------------------------------------*/
/*  CAN Message FIFO Buffer Functions                                        */
/*---------------------------------------------------------------------------*/
void ResetFifo(PMSG_BUF_T pMsgBuf)
{
    pMsgBuf->Head = 0;
    pMsgBuf->Tail = 0;

}

/* Return numbers of messages in FIFO */
uint32_t CountsInFifo(PMSG_BUF_T pMsgBuf)
{
    uint8_t h, t;

    /* This method can safely get the difference without disable interrupt */
    h = pMsgBuf->Head;
    t = pMsgBuf->Tail;

    return (h >= t) ? (h - t) : (h + MAX_FIFO_MESSAGES - t);

}

uint8_t PutToFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg)
{
    if(CountsInFifo(pMsgBuf) >= MAX_FIFO_MESSAGES - 1)
        return FALSE;

    memcpy(&(pMsgBuf->Msg[pMsgBuf->Head]), pMsg, sizeof(STR_CANMSG_T));

    if(++(pMsgBuf->Head) >= MAX_FIFO_MESSAGES)
        pMsgBuf->Head = 0;

    return TRUE;

}

uint8_t GetFromFifo(PMSG_BUF_T pMsgBuf, STR_CANMSG_T *pMsg)
{
    if(CountsInFifo(pMsgBuf) == 0)
        return FALSE;

    memcpy(pMsg, &(pMsgBuf->Msg[pMsgBuf->Tail]), sizeof(STR_CANMSG_T));

    if(++(pMsgBuf->Tail) >= MAX_FIFO_MESSAGES)
        pMsgBuf->Tail = 0;

    return TRUE;

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_72MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);
    CLK_EnableModuleClock(CAN0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PLL, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD, TXD and */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* SPI2: GPD12=SS, GPD15=CLK, GPD14=MISO, GPD13=MOSI */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD12MFP_SPI2_SS | SYS_GPD_MFPH_PD13MFP_SPI2_MOSI | SYS_GPD_MFPH_PD14MFP_SPI2_MISO | SYS_GPD_MFPH_PD15MFP_SPI2_CLK);

    /* Set PA multi-function pins for CANTX0, CANRX0 */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_CAN0_TXD | SYS_GPA_MFPH_PA13MFP_CAN0_RXD);

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
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
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|   About CAN sample code configure                                          |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN           |\n");
    printf("|   Before execute it, please check description as below                     |\n");
    printf("|                                                                            |\n");
    printf("|   1.CAN0_TX and CAN0_RX should be connected to your CAN transceiver        |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS                 |\n");
    printf("|   3.Check the terminal resistor of bus is connected                        |\n");
    printf("|   4.Using UART0 as print message port                                      |\n");
    printf("|                                                                            |\n");
    printf("|  |--------|        |-----------|   CANBUS  |-----------|        |--------| |\n");
    printf("|  |        |------->|           |<--------->|           |<-------|        | |\n");
    printf("|  |        |CAN0_TX |    CAN    |   CAN_H   |   CAN     |CAN0_TX |        | |\n");
    printf("|  |  M451  |        |Transceiver|           |Transceiver|        |  M451  | |\n");
    printf("|  |        |<-------|           |<--------->|           |------->|        | |\n");
    printf("|  |        |CAN0_RX |           |   CAN_L   |           |CAN0_RX |        | |\n");
    printf("|  |--------|        |-----------|           |-----------|        |--------| |\n");
    printf("|  |                                                              |          |\n");
    printf("|  |                                                              |          |\n");
    printf("|  V                                                              V          |\n");
    printf("| UART0                                                           UART0      |\n");
    printf("|(print message)                                             (print message) |\n");
    printf("+----------------------------------------------------------------------------+\n");

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
        printf("       Tseg2: Time Segment ater the sample point. You can set tseg2 (CAN_BTIME[14:12]).\n");

        if(SystemCoreClock % u32BaudRate != 0)
            printf("\nThe BPR does not calculate, the Fin must be a multiple of the CAN baud-rate.\n");
        else
            printf("\nThe BPR does not calculate, the (Fin/(CAN baud-rate)) must be a multiple of the (Tseg1+Tseg1+3).\n");
    }
    else
        printf("\nReal baud-rate value(bps): %d\n", u32RealBaudRate);

}

/*----------------------------------------------------------------------------*/
/*  Init CAN baud-rate                                                        */
/*----------------------------------------------------------------------------*/
uint32_t InitCAN(CAN_T *tCAN, uint32_t u32BaudRate)
{
    uint8_t u8Tseg1, u8Tseg2;
    uint32_t u32Brp;
    uint32_t u32Value;

    // CAN do Initialization
    tCAN ->CON |= CAN_CON_INIT_Msk | CAN_CON_CCE_Msk;

    // Update system clock for CAN
    SystemCoreClockUpdate();
    u32Value = SystemCoreClock;

    // Set BTIME and BRPE registers
    u8Tseg1 = 3;
    u8Tseg2 = 2;
    while(1)
    {
        if(((u32Value % (u8Tseg1 + u8Tseg2 + 3)) == 0) | (u8Tseg1 >= 15))
            break;

        u8Tseg1++;
        if((u32Value % (u8Tseg1 + u8Tseg2 + 3)) == 0)
            break;

        if(u8Tseg2 < 7)
            u8Tseg2++;
    }
    u32Brp  = u32Value / (u32BaudRate) / (u8Tseg1 + u8Tseg2 + 3) - 1;

    u32Value = ((uint32_t)u8Tseg2 << CAN_BTIME_TSEG2_Pos) |
               ((uint32_t)u8Tseg1 << CAN_BTIME_TSEG1_Pos) |
               (u32Brp & CAN_BTIME_BRP_Msk) |
               (CAN0->BTIME & CAN_BTIME_SJW_Msk);
    tCAN->BTIME = u32Value;
    tCAN->BRPE = (u32Brp >> 6) & 0x0F;

    // CAN Initialization finished
    tCAN->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
    while(tCAN->CON & CAN_CON_INIT_Msk);

    // Enable CAN interrupt
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk);
    NVIC_SetPriority(CAN0_IRQn, 1);
    NVIC_EnableIRQ(CAN0_IRQn);

    // Return baudrate
    return (SystemCoreClock / (u8Tseg1 + u8Tseg2 + 3) / (u32Brp + 1));

}

/*----------------------------------------------------------------------------*/
/*  Set the CAN speed                                                         */
/*----------------------------------------------------------------------------*/
void SetCANSpeed(CAN_T *tCAN)
{
    uint32_t BaudRate = 0, RealBaudRate = 0;

    /* Set CAN baud rate to 125kbps */
    BaudRate = 125000;
    RealBaudRate = InitCAN(tCAN,  BaudRate);

    /* Check the real baud-rate is OK */
    BaudRateCheck(BaudRate, RealBaudRate);

}


/*----------------------------------------------------------------------------*/
/*  Test Menu                                                                 */
/*----------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                                                                   |\n");
    printf("|     1. Master: Transmit messages from CAN0                        |\n");
    printf("|     2. Slave: Receive messages thru CAN0                          |\n");
    printf("|     Please select 1 (Master) or 2 (Slave).                        |\n");
    printf("|                                                                   |\n");
    printf("+-------------------------------------------------------------------+\n");

}

/*----------------------------------------------------------------------------*/
/*  Set Rx Message Object                                                     */
/*----------------------------------------------------------------------------*/
void SetRxMsgObj(CAN_T *tCAN, uint32_t u32MsgNum, uint8_t u8FifoLast)
{
    tCAN->IF[0].CMASK = CAN_IF_CMASK_WRRD_Msk | CAN_IF_CMASK_MASK_Msk | CAN_IF_CMASK_ARB_Msk |
                        CAN_IF_CMASK_CONTROL_Msk | CAN_IF_CMASK_DATAA_Msk | CAN_IF_CMASK_DATAB_Msk;

    tCAN->IF[0].ARB1 = 0;
    tCAN->IF[0].ARB2 = CAN_IF_ARB2_MSGVAL_Msk;
    tCAN->IF[0].MASK1 = 0;
    tCAN->IF[0].MASK2 = 0;

    tCAN->IF[0].MCON = CAN_IF_MCON_UMASK_Msk | CAN_IF_MCON_RXIE_Msk |
                       ((u8FifoLast) ? CAN_IF_MCON_EOB_Msk : 0);

    tCAN->IF[0].DAT_A1  = 0;
    tCAN->IF[0].DAT_A2  = 0;
    tCAN->IF[0].DAT_B1  = 0;
    tCAN->IF[0].DAT_B2  = 0;

    tCAN->IF[0].CREQ = 1 + u32MsgNum;

    while((tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk));

}

/*----------------------------------------------------------------------------*/
/*  Get Message Object                                                        */
/*----------------------------------------------------------------------------*/
void GetMsgObj(CAN_T *tCAN, uint8_t u8MsgObj, STR_CANMSG_T* pCanMsg)
{
    tCAN->STATUS &= (~CAN_STATUS_RXOK_Msk);

    /* read the message contents*/
    tCAN->IF[0].CMASK = CAN_IF_CMASK_MASK_Msk
                        | CAN_IF_CMASK_ARB_Msk
                        | CAN_IF_CMASK_CONTROL_Msk
                        | CAN_IF_CMASK_CLRINTPND_Msk
                        | CAN_IF_CMASK_TXRQSTNEWDAT_Msk
                        | CAN_IF_CMASK_DATAA_Msk
                        | CAN_IF_CMASK_DATAB_Msk;

    tCAN->IF[0].CREQ = 1 + u8MsgObj;

    /*Wait*/
    while(tCAN->IF[0].CREQ & CAN_IF_CREQ_BUSY_Msk) {};

    if((tCAN->IF[0].ARB2 & CAN_IF_ARB2_XTD_Msk) == 0)
    {
        /* standard ID*/
        pCanMsg->IdType = CAN_STD_ID;
        pCanMsg->Id     = (tCAN->IF[0].ARB2 & CAN_IF_ARB2_ID_Msk) >> 2;
    }
    else
    {
        /* extended ID*/
        pCanMsg->IdType = CAN_EXT_ID;
        pCanMsg->Id  = (((tCAN->IF[0].ARB2) & 0x1FFF) << 16) | tCAN->IF[0].ARB1;
    }

    pCanMsg->FrameType = !((tCAN->IF[0].ARB2 & CAN_IF_ARB2_DIR_Msk) >> CAN_IF_ARB2_DIR_Pos);
    pCanMsg->DLC     = tCAN->IF[0].MCON & CAN_IF_MCON_DLC_Msk;
    pCanMsg->Data[0] = tCAN->IF[0].DAT_A1 & CAN_IF_DAT_A1_DATA0_Msk;
    pCanMsg->Data[1] = (tCAN->IF[0].DAT_A1 & CAN_IF_DAT_A1_DATA1_Msk) >> CAN_IF_DAT_A1_DATA1_Pos;
    pCanMsg->Data[2] = tCAN->IF[0].DAT_A2 & CAN_IF_DAT_A2_DATA2_Msk;
    pCanMsg->Data[3] = (tCAN->IF[0].DAT_A2 & CAN_IF_DAT_A2_DATA3_Msk) >> CAN_IF_DAT_A2_DATA3_Pos;
    pCanMsg->Data[4] = tCAN->IF[0].DAT_B1 & CAN_IF_DAT_B1_DATA4_Msk;
    pCanMsg->Data[5] = (tCAN->IF[0].DAT_B1 & CAN_IF_DAT_B1_DATA5_Msk) >> CAN_IF_DAT_B1_DATA5_Pos;
    pCanMsg->Data[6] = tCAN->IF[0].DAT_B2 & CAN_IF_DAT_B2_DATA6_Msk;
    pCanMsg->Data[7] = (tCAN->IF[0].DAT_B2 & CAN_IF_DAT_B2_DATA7_Msk) >> CAN_IF_DAT_B2_DATA7_Pos;

}

/*----------------------------------------------------------------------------*/
/*  Set Message Object for Rx                                                 */
/*----------------------------------------------------------------------------*/
void SetMsgObj_for_Rx(CAN_T *tCAN)
{
    /* Use several message objects as FIFO buffer to receive CAN messages */
    int i;

    for(i = 0; i < CAN_FIFO_DEPTH - 1; i++)
        SetRxMsgObj(tCAN, i, 0);

    SetRxMsgObj(tCAN, i, 1);

}

/*----------------------------------------------------------------------------*/
/*  Send Message Object                                                       */
/*----------------------------------------------------------------------------*/
void SendMsgObj(CAN_T *tCAN, uint32_t u32MsgNum, STR_CANMSG_T* pCanMsg)
{
    if(u32MsgNum < 16)
        while(tCAN->TXREQ1 & (1 << u32MsgNum));
    else
        while(tCAN->TXREQ2 & (1 << (u32MsgNum - 16)));

    /* update the contents needed for transmission*/
    tCAN->IF[1].CMASK = 0xF7; /* 0xF3;  CAN_CMASK_WRRD_Msk | CAN_CMASK_MASK_Msk | CAN_CMASK_ARB_Msk
                                    | CAN_CMASK_CONTROL_Msk | CAN_CMASK_DATAA_Msk  | CAN_CMASK_DATAB_Msk ; */

    if(pCanMsg->IdType == CAN_STD_ID)
    {
        /* standard ID*/
        tCAN->IF[1].ARB1 = 0;
        tCAN->IF[1].ARB2 = (((pCanMsg->Id) & 0x7FF) << 2) | CAN_IF_ARB2_DIR_Msk | CAN_IF_ARB2_MSGVAL_Msk;
    }
    else
    {
        /* extended ID*/
        tCAN->IF[1].ARB1 = (pCanMsg->Id) & 0xFFFF;
        tCAN->IF[1].ARB2 = ((pCanMsg->Id) & 0x1FFF0000) >> 16 | CAN_IF_ARB2_DIR_Msk
                           | CAN_IF_ARB2_XTD_Msk | CAN_IF_ARB2_MSGVAL_Msk;
    }

    if(pCanMsg->FrameType)
        tCAN->IF[1].ARB2 |=   CAN_IF_ARB2_DIR_Msk;
    else
        tCAN->IF[1].ARB2 &= (~CAN_IF_ARB2_DIR_Msk);

    tCAN->IF[1].DAT_A1 = ((uint16_t)pCanMsg->Data[1] << 8) | pCanMsg->Data[0];
    tCAN->IF[1].DAT_A2 = ((uint16_t)pCanMsg->Data[3] << 8) | pCanMsg->Data[2];
    tCAN->IF[1].DAT_B1 = ((uint16_t)pCanMsg->Data[5] << 8) | pCanMsg->Data[4];
    tCAN->IF[1].DAT_B2 = ((uint16_t)pCanMsg->Data[7] << 8) | pCanMsg->Data[6];

    tCAN->IF[1].MCON   =  CAN_IF_MCON_NEWDAT_Msk | pCanMsg->DLC | CAN_IF_MCON_TXIE_Msk | CAN_IF_MCON_EOB_Msk;
    tCAN->IF[1].CREQ   = 1 + u32MsgNum;

    /* Wait */
    while(tCAN->IF[1].CREQ & CAN_IF_CREQ_BUSY_Msk) {};

}

/*----------------------------------------------------------------------------*/
/*  Send Message Object to Tx                                                 */
/*----------------------------------------------------------------------------*/
void SendMsgObj_to_Tx(CAN_T *tCAN)
{
    STR_CANMSG_T tMsg;

    /* Send a 29-bit Extended Identifier message */
    tMsg.FrameType = CAN_DATA_FRAME;
    tMsg.IdType   = CAN_EXT_ID;
    tMsg.Id       = 0x7FF01;
    tMsg.DLC      = 4;
    tMsg.Data[0]  = 0xA1;
    tMsg.Data[1]  = 0xB2;
    tMsg.Data[2]  = 0xC3;
    tMsg.Data[3]  = 0xD4;

    SendMsgObj(tCAN, TX_MSG_OBJ_ID, &tMsg);

    printf("\nMsgObj(%d), Send EXT_ID:0x7FF01, Data[A1,B2,C3,D4] done!\n", TX_MSG_OBJ_ID);

}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Configure SPI3 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 4MHz */
    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 4000000);

    /* Configure SPI1 as a low level active device. */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Start SPI */
    SPI_ENABLE(SPI_LCD_PORT);

    /* Init LCD */
    ILI9341_LCD_Init();

    /* Show the String on the screen */
    ILI9341_LCD_PutString(0, 0, "< Nu-LB-M451 CAN Sample Code >", Red, Yellow);

    /* Enable CAN transceiver on M451 LB board */
    GPIO_SetMode(PB, BIT9,  GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);
    PB9 = 0;
    PB10 = 0;

    /* Set PA.8 as Input for key button */
    GPIO_SetMode(PA, BIT8,  GPIO_MODE_INPUT);

    /* Some description about how to create test environment */
    Note_Configure();

    /* Configuring the Bit Timing */
    SetCANSpeed(CAN0);

    /* Set Message Object for Rx */
    SetMsgObj_for_Rx(tCAN0);

    ILI9341_LCD_PutString(16, 0, "Press Key 1 to Send Message.", Red, Yellow);

    while(1)
    {
        if(PA8 == 0)
        {
            SendMsgObj_to_Tx(tCAN0);
            ILI9341_LCD_PutString(32, 0, "Send Message Out!!! ==>", Red, Yellow);
            CLK_SysTickDelay(200000);
        }
    }

}



