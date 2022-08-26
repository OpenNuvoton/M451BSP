/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/09/02 10:05a $
 * @brief    Transmit LIN header and response.
 * @note
 * Copyright (C) 2011~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"


#define PLL_CLOCK   72000000


/* CheckSum Method */
#define MODE_CLASSIC    2
#define MODE_ENHANCED   1


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile int32_t g_i32pointer;
uint8_t g_u8SendData[12] ;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void LIN_FunctionTest(void);
void LIN_FunctionTestUsingLinCtlReg(void);
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel);
void LIN_MasterTestUsingLinCtlReg(uint32_t u32id, uint32_t u32ModeSel);
void LIN_SendHeader(uint32_t u32id);
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32id, uint32_t u32HeaderSel);
void LIN_SendResponse(int32_t checkSumOption, uint32_t *pu32TxBuf);
void LIN_SendResponseWithByteCnt(int32_t checkSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt);
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel);
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);


/*---------------------------------------------------------------------------------------------------------*/
/*  Sample Code Menu                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|                LIN Sample Program                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| LIN Master function test                            - [1] |\n");
    printf("| LIN Master function test using UART_LINCTL register - [2] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

void LIN_TestItem()
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     LIN Master Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Master send header with ID = 0x30                     |\n");
    printf("| [2] Master send header and response with classic checksum |\n");
    printf("| [3] Master send header and response with enhanced checksum|\n");
    printf("|                                                           |\n");
    printf("| To measure UART1_TXD(PB.3) to check waveform ...          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTest()
{
    uint32_t u32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLine_Config(UART1, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);
        switch(u32Item)
        {
            case '1':
                LIN_SendHeader(0x30);
                break;
            case '2':
                LIN_MasterTest(0x35, MODE_CLASSIC);
                break;
            case '3':
                LIN_MasterTest(0x12, MODE_ENHANCED);
                break;
            default:
                break;
        }
    }
    while(u32Item != 27);

    UART_Close(UART1);

    printf("\nLIN Sample Code End.\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTestUsingLinCtlReg(void)
{
    uint32_t u32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLine_Config(UART1, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);
        switch(u32Item)
        {
            case '1':
                LIN_SendHeaderUsingLinCtlReg(0x30, UART_LINCTL_HSEL_BREAK_SYNC_ID);
                break;
            case '2':
                LIN_MasterTestUsingLinCtlReg(0x35, MODE_CLASSIC);
                break;
            case '3':
                LIN_MasterTestUsingLinCtlReg(0x12, MODE_ENHANCED);
                break;
            default:
                break;
        }
    }
    while(u32Item != 27);

    /* Clear header select setting */
    UART1->LINCTL &= ~UART_LINCTL_HSEL_Msk;

    UART_Close(UART1);

    printf("\nLIN Sample Code End.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel)
{
    uint32_t testPattern[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};

    /* Send ID=0x35 Header and Response TestPatten */
    LIN_SendHeader(u32id);
    LIN_SendResponse(u32ModeSel, &testPattern[0]);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTestUsingLinCtlReg(uint32_t u32id, uint32_t u32ModeSel)
{
    uint8_t au8TestPattern[9] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0}; // 8 data byte + 1 byte checksum
    uint32_t i;

    if(u32ModeSel == MODE_CLASSIC)
    {
        /* Send break+sync+ID */
        LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK_SYNC_ID);
        /* Compute checksum without ID and fill checksum value to  au8TestPattern[8] */
        au8TestPattern[8] = ComputeChksumValue(&au8TestPattern[0], 8);
        UART_Write(UART1, &au8TestPattern[0], 9);
    }
    else if(u32ModeSel == MODE_ENHANCED)
    {
        /* Send break+sync+ID and fill ID value to g_u8SendData[0] */
        LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK_SYNC);
        /* Fill test pattern to g_u8SendData[1]~ g_u8SendData[8] */
        for(i = 0; i < 8; i++)
            g_u8SendData[g_i32pointer++] = au8TestPattern[i];
        /* Compute checksum value with ID and fill checksum value to g_u8SendData[9] */
        g_u8SendData[g_i32pointer++] = ComputeChksumValue(&g_u8SendData[0], 9) ;
        UART_Write(UART1, &g_u8SendData[1], 9);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32id)
{
    uint32_t u32Res = 0, ID[6], p_Bit[2], mask = 0;

    for(mask = 0; mask < 6; mask++)
        ID[mask] = (u32id & (1 << mask)) >> mask;

    p_Bit[0] = (ID[0] + ID[1] + ID[2] + ID[4]) % 2;
    p_Bit[1] = (!((ID[1] + ID[3] + ID[4] + ID[5]) % 2));

    u32Res = u32id + (p_Bit[0] << 6) + (p_Bit[1] << 7);
    return u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value , MODE_CLASSIC:(Not Include ID)    MODE_ENHANCED:(Include ID)                    */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel)
{
    uint32_t i, CheckSum = 0;

    for(i = u32ModeSel; i <= 9; i++)
    {
        CheckSum += pu8Buf[i];
        if(CheckSum >= 256)
            CheckSum -= 255;
    }
    return (255 - CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t i, CheckSum = 0;

    for(i = 0 ; i < u32ByteCnt; i++)
    {
        CheckSum += pu8Buf[i];
        if(CheckSum >= 256)
            CheckSum -= 255;
    }
    return (uint8_t)(255 - CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendHeader(uint32_t u32id)
{
    g_i32pointer = 0 ;

    /* Set LIN operation mode, Tx mode and break field length is 12 bits */
    UART_SelectLINMode(UART1, UART_ALTCTL_LINTXEN_Msk, 11);

    g_u8SendData[g_i32pointer++] = 0x55 ;                   // SYNC Field
    g_u8SendData[g_i32pointer++] = GetParityValue(u32id);   // ID+Parity Field
    UART_Write(UART1, g_u8SendData, 2);
}

/*-------------------------------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                                        */
/*  u32HeaderSel =  UART_LINCTL_HSEL_BREAK/UART_LINCTL_HSEL_BREAK_SYNC/UART_LINCTL_HSEL_BREAK_SYNC_ID                            */
/*-------------------------------------------------------------------------------------------------------------------------------*/
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32id, uint32_t u32HeaderSel)
{
    uint32_t u32TimeOutCnt;

    g_i32pointer = 0 ;

    /* Switch back to LIN Function */
    UART1->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN 1. PID as 0x30 [UART_LINCTL_PID(0x30)]
               2. Header select as includes "break field", "sync field" and "frame ID field".[UART_LINCTL_HSEL_BREAK_SYNC_ID]
               3. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               4. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
               5. ID Parity Enable. Hardware will calculate and fill P0/P1 automatically  [UART_LINCTL_IDPEN_Msk]
    */
    if(u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC_ID)
    {
        UART1->LINCTL = UART_LINCTL_PID(u32id) | UART_LINCTL_HSEL_BREAK_SYNC_ID |
                        UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12) | UART_LINCTL_IDPEN_Msk;
        /* LIN TX Send Header Enable */
        UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
        /* Wait until break field, sync field and ID field transfer completed */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for UART LIN header transfer completed time-out!\n");
                return;
            }
        }
    }

    /* Set LIN 1. Header select as includes "break field" and "sync field".[UART_LINCTL_HSEL_BREAK_SYNC]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if(u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC)
    {
        UART1->LINCTL = UART_LINCTL_HSEL_BREAK_SYNC | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
        /* Wait until break field and sync field transfer completed */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for UART LIN header transfer completed time-out!\n");
                return;
            }
        }

        /* Send ID field, g_u8SendData[0] is ID+parity field*/
        g_u8SendData[g_i32pointer++] = GetParityValue(u32id);   // ID+Parity Field
        UART_Write(UART1, g_u8SendData, 1);
    }

    /* Set LIN 1. Header select as includes "break field".[UART_LINCTL_HSEL_BREAK]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if(u32HeaderSel == UART_LINCTL_HSEL_BREAK)
    {
        UART1->LINCTL = UART_LINCTL_HSEL_BREAK | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
        /* Wait until break field transfer completed */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for UART LIN header transfer completed time-out!\n");
                return;
            }
        }

        /* Send sync field and ID field */
        g_u8SendData[g_i32pointer++] = 0x55 ;                  // SYNC Field
        g_u8SendData[g_i32pointer++] = GetParityValue(u32id);   // ID+Parity Field
        UART_Write(UART1, g_u8SendData, 2);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponse(int32_t checkSumOption, uint32_t *pu32TxBuf)
{
    int32_t i32;

    for(i32 = 0; i32 < 8; i32++)
        g_u8SendData[g_i32pointer++] = pu32TxBuf[i32] ;

    g_u8SendData[g_i32pointer++] = GetCheckSumValue(g_u8SendData, checkSumOption) ; //CheckSum Field

    UART_Write(UART1, g_u8SendData + 2, 9);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponseWithByteCnt(int32_t checkSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt)
{
    int32_t i32;

    /* Prepare data */
    for(i32 = 0; i32 < u32ByteCnt; i32++)
        g_u8SendData[g_i32pointer++] = pu32TxBuf[i32] ;

    /* Prepare check sum */
    if(checkSumOption == MODE_CLASSIC)
        g_u8SendData[g_i32pointer++] = GetCheckSumValue(&g_u8SendData[2], u32ByteCnt) ;  //CheckSum Field
    else if(checkSumOption == MODE_ENHANCED)
        g_u8SendData[g_i32pointer++] = GetCheckSumValue(&g_u8SendData[1], (u32ByteCnt + 1)) ; //CheckSum Field

    /* Send data and check sum */
    UART_Write(UART1, g_u8SendData + 2, 9);
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
    uint32_t u32Item;

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

    /* UART sample LIN function */
    do
    {
        TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);
        switch(u32Item)
        {
            case '1':
                LIN_FunctionTest();
                break;
            case '2':
                LIN_FunctionTestUsingLinCtlReg();
                break;
            default:
                break;
        }
    }
    while(u32Item != 27);

    while(1);

}

