/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrates how to set PWM pin analog IR emitters,
             the information is sent to the PWM capture pin, analyze the data received and displayed.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"
#include "NuEdu-Basic01.h"

#define PLL_CLOCK       72000000


#define SPI_LCD_PORT  SPI2

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

extern void Initial_LB_Key_Input(void);
extern unsigned char Get_LB_Key_Input(void);

void ILI9341_LCD_PutString(uint16_t x, uint16_t y, char *s, uint32_t fColor, uint32_t bColor);

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

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PLL, 0);

    /* Get Core Clock Frequency      */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.6) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* SPI2: GPD12=SS, GPD15=CLK, GPD14=MISO, GPD13=MOSI */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD12MFP_SPI2_SS | SYS_GPD_MFPH_PD13MFP_SPI2_MOSI | SYS_GPD_MFPH_PD14MFP_SPI2_MISO | SYS_GPD_MFPH_PD15MFP_SPI2_CLK);

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
    uint8_t au8IR_CODE[4];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
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

    /* Button Init */
    Initial_LB_Key_Input();
    IrDA_NEC_TxRx_Init();

    /* Show the String on the screen */                                                         // Long= 40 , Width= 15
    ILI9341_LCD_PutString(0, 0, "******************************************", Red, Yellow);
    ILI9341_LCD_PutString(15, 0, "*Demonstrates how to set PWM pin analog*", Red, Yellow);
    ILI9341_LCD_PutString(30, 0, "*IR emitters, the information is sent  *", Red, Yellow);
    ILI9341_LCD_PutString(45, 0, "*to the PWM capture pin, analyze the   *", Red, Yellow);
    ILI9341_LCD_PutString(60, 0, "*data received and displayed.          *", Red, Yellow);
    ILI9341_LCD_PutString(75, 0, "*Please touch the KEY 1-2              *", Red, Yellow);
    ILI9341_LCD_PutString(90, 0, "******************************************", Red, Yellow);

    au8IR_CODE[0] = 0x00;
    au8IR_CODE[1] = ~au8IR_CODE[0];

    while(1)
    {
        /* Detect Key status */
        if(Get_LB_Key_Input() == 0x01)
        {
            au8IR_CODE[2] = 0x10;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
        if(Get_LB_Key_Input() == 0x02)
        {
            au8IR_CODE[2] = 0x14;
            au8IR_CODE[3] = ~au8IR_CODE[2];
            SendNEC(au8IR_CODE);
            CLK_SysTickDelay(100000);
        }
    }
}
