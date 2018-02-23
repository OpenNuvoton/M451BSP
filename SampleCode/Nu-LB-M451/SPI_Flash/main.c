/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to read from and write to SPI Flash.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"

#define PLL_CLOCK       72000000
#define TEST_NUMBER                 10 /* page numbers */
#define SPI_Flash SPI0

#define SPI_LCD_PORT  SPI2

#define ILI9341_RESET   PB15
#define ILI9341_DC      PB11
#define ILI9341_LED     PB5

extern uint8_t Font8x16[];

#define	White           0xFFFF
#define Black           0x0000
#define Blue            0x001F
#define Blue2           0x051F
#define Red             0xF800
#define Magenta         0xF81F
#define Green           0x07E0
#define Cyan            0x7FFF
#define Yellow          0xFFE0

unsigned char SrcArray[256];
unsigned char DestArray[256];

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


void ILI9341_LCD_SetAddress(uint32_t x1,uint32_t x2,uint32_t y1,uint32_t y2)
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
    LCD_WriteData(x1>>8);
    LCD_WriteData(x1);
    LCD_WriteData(x2>>8);
    LCD_WriteData(x2);

    LCD_WriteCommand(0x2b);
    LCD_WriteData(y1>>8);
    LCD_WriteData(y1);
    LCD_WriteData(y2>>8);
    LCD_WriteData(y2);
}

void ILI9341_LCD_PutChar8x16(uint16_t x, uint16_t y, uint8_t c, uint32_t fColor, uint32_t bColor)
{
	uint32_t i,j;
	for(i=0;i<16;i++){
        uint8_t m=Font8x16[c*16+i];
        ILI9341_LCD_SetAddress(x+i,x+i,y,y+7);
        LCD_WriteCommand(0x2c);        
        
		for(j=0;j<8;j++){
			if((m&0x01)==0x01){
                LCD_WriteData(fColor>>8);
				LCD_WriteData(fColor);
			}
			else{
                LCD_WriteData(bColor>>8);
				LCD_WriteData(bColor);
			}
			m>>=1;
		}
	}
}

void ILI9341_LCD_Color(uint32_t bColor)
{
		uint32_t i,j;
		ILI9341_LCD_SetAddress(0,239,0,319);
		LCD_WriteCommand(0x2c); 
		for(i=0;i<240;i++){
				for(j=0;j<320;j++){
						LCD_WriteData(bColor>>8);
						LCD_WriteData(bColor);
				}
		}
}

void ILI9341_LCD_PutString(uint16_t x, uint16_t y,uint8_t *s, uint32_t fColor, uint32_t bColor)
{
    uint8_t l=0;
    while(*s){
        if(*s<0x80){
            ILI9341_LCD_PutChar8x16(x,312-y-l*8,*s,fColor,bColor);
            s++;
            l++;
		}
	}	
}


void ILI9341_LCD_Init(void)
{
    /* Configure DC/RESET/LED pins */
    ILI9341_DC =0;
    ILI9341_RESET=0;
    ILI9341_LED=0;

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

void Initial_SPI0_GPIO(void)
{
    /* Set PE10, PE11, PE12 and PE13 for SPI0 */
    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE10MFP_Msk | SYS_GPE_MFPH_PE11MFP_Msk | SYS_GPE_MFPH_PE12MFP_Msk | SYS_GPE_MFPH_PE13MFP_Msk);
    SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE10MFP_SPI0_MISO0 | SYS_GPE_MFPH_PE11MFP_SPI0_MOSI0 | SYS_GPE_MFPH_PE12MFP_SPI0_SS | SYS_GPE_MFPH_PE13MFP_SPI0_CLK);
}

void Open_SPI_Flash(void)
{
    /* Init GPIO for SPI0 */
    Initial_SPI0_GPIO();

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Configure SPI0 as a master, SPI clock rate 2 MHz,
    clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI_Open(SPI_Flash, SPI_MASTER, SPI_MODE_0, 32, 24000000);

    /* Disable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_DisableAutoSS(SPI_Flash);
    SPI_SET_SS_HIGH(SPI_Flash);
}

// **************************************
// For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
// For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
unsigned int SpiFlash_ReadMidDid(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 8);

    // send Command: 0x90, Read Manufacturer/Device ID
    au32SourceData = 0x90;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 24);

    // send 24-bit '0', dummy
    au32SourceData = 0x0;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 16);

    SPI_ClearRxFIFO(SPI_Flash);

    // receive
    au32SourceData = 0x0;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI_Flash);

    return (au32DestinationData & 0xffff);

}

// **************************************
void SpiFlash_ChipErase(void)
{
    unsigned int au32SourceData;

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 8);

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    // send Command: 0xC7, Chip Erase
    au32SourceData = 0xc7;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    //CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

}

// **************************************
unsigned int SpiFlash_ReadStatusReg1(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    I2S_CLR_RX_FIFO(SPI_Flash);

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 16);

    // send Command: 0x05, Read status register 1
    au32SourceData = 0x0500;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI_Flash);

    return (au32DestinationData & 0xFF);

}


// **************************************
unsigned int SpiFlash_ReadStatusReg2(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    I2S_CLR_RX_FIFO(SPI_Flash);

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 16);

    // send Command: 0x35, Read status register 2
    au32SourceData = 0x3500;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI_Flash);

    return (au32DestinationData & 0xFF);
}


// **************************************
void SpiFlash_WaitReady(void)
{
    unsigned int ReturnValue;

    do
    {
        ReturnValue = SpiFlash_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue != 0); // check the BUSY bit

}

// **************************************
void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int Counter;

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 8);

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 24);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 8);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // send data to program
        au32SourceData = DataBuffer[Counter];
        SPI_WRITE_TX(SPI_Flash, au32SourceData);
        while(SPI_IS_BUSY(SPI_Flash));
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

}


// **************************************
void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;
    unsigned int Counter;

    // /CS: active
    SPI_SET_SS_LOW(SPI_Flash);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 8);

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 24);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_WRITE_TX(SPI_Flash, au32SourceData);
    while(SPI_IS_BUSY(SPI_Flash));

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI_Flash, 8);

    I2S_CLR_RX_FIFO(SPI_Flash);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // receive
        au32SourceData = 0x0;
        SPI_WRITE_TX(SPI_Flash, au32SourceData);
        while(SPI_IS_BUSY(SPI_Flash));

        // dump Rx register
        au32DestinationData = SPI_READ_RX(SPI_Flash);
        DataBuffer[Counter] = (unsigned char) au32DestinationData;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_Flash);

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

    /* Enable IP module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);

    /* Select IP module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PLL, 0);

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
    unsigned int u32ByteCount;
    unsigned int u32PageNumber;
    unsigned int u32ProgramFlashAddress = 0;
    unsigned int u32VerifyFlashAddress = 0;
    unsigned int MidDid;
    uint8_t Message[30];
    char* Message_Print;
    
    Message_Print = (char*)Message;

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
    ILI9341_LCD_Color(Red);
	
    printf("Hello World.\n");
    printf("PLL Clock = %d Hz\n", CLK_GetPLLClockFreq());
    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|       M451 Series SPI_Flash Sample Code               |\n");
    printf("+-------------------------------------------------------+\n");
		
    /* Show the String on the screen */
    ILI9341_LCD_PutString(0*16, 0,  (uint8_t *)"Hello World.", White, Red);
    sprintf(Message_Print, "PLL Clock = %d Hz", CLK_GetPLLClockFreq());
    ILI9341_LCD_PutString(1*16, 0, Message, White, Red);
    sprintf(Message_Print, "Core Clock = %d Hz", CLK_GetHCLKFreq());
    ILI9341_LCD_PutString(2*16, 0, Message, White, Red);
    ILI9341_LCD_PutString(3*16, 0, (uint8_t *)"+----------------------------+", White, Red);
    ILI9341_LCD_PutString(4*16, 0, (uint8_t *)"+ Smpl_SPI_Flash Sample Code +", White, Red);		
    ILI9341_LCD_PutString(5*16, 0, (uint8_t *)"+----------------------------+", White, Red);
		
    /* Open SPI for Serial Flash */
    Open_SPI_Flash();

    /* Read MID & DID */
    MidDid = SpiFlash_ReadMidDid();
    printf("\nMID and DID = %x", MidDid);
    sprintf(Message_Print, "MID and DID = %4X", MidDid);
    ILI9341_LCD_PutString(6*16, 0, Message, White, Red);  

    /* Erase SPI Flash */
    SpiFlash_ChipErase();
    printf("\nFlash Erasing... ");
    ILI9341_LCD_PutString(7*16, 0, (uint8_t *)"Flash Erasing... ", White, Red);
		
    /* Wait ready */
    SpiFlash_WaitReady();
    printf("Done!");
    ILI9341_LCD_PutString(7*16, 17*8, (uint8_t *)"Done!", White, Red);

    /* Fill the Source Data and clear Destination Data Buffer */
    for(u32ByteCount = 0; u32ByteCount < 256; u32ByteCount++)
    {
        SrcArray[u32ByteCount] = u32ByteCount;
        DestArray[u32ByteCount] = 0;
    }

    u32ProgramFlashAddress = 0;
    u32VerifyFlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        printf("\n\nTest Page Number = %d", u32PageNumber);
        sprintf(Message_Print, "Test Page Number = %d", u32PageNumber);
        ILI9341_LCD_PutString(9*16, 0, Message, White, Red);
        CLK_SysTickDelay(200000);

        /*=== Program SPI Flash ===*/
        printf("\n Flash Programming... ");
        ILI9341_LCD_PutString(10*16, 0, (uint8_t *)" Flash Programming... ", White, Red);

        /* Page Program */
        SpiFlash_PageProgram(SrcArray, u32ProgramFlashAddress, 256);
        SpiFlash_WaitReady();
        u32ProgramFlashAddress += 0x100;
        printf("Done!");
        ILI9341_LCD_PutString(10*16, 22*8, (uint8_t *)"Done!", White, Red);

        /*=== Read Back and Compare Data ===*/
        printf("\n Flash Verifying... ");
        ILI9341_LCD_PutString(11*16, 0, (uint8_t *)" Flash Verifying... ", White, Red);

        /* Page Read */
        SpiFlash_ReadData(DestArray, u32VerifyFlashAddress, 256);
        u32VerifyFlashAddress += 0x100;

        for(u32ByteCount = 0; u32ByteCount < 256; u32ByteCount++)
        {
            if(DestArray[u32ByteCount] != u32ByteCount)
            {
                /* Error */
                printf("SPI Flash R/W Fail!");
                ILI9341_LCD_PutString(11*16, 20*8, (uint8_t *)"SPI Flash R/W Fail!", White, Red);
                while(1);
            }
        }

        /* Clear Destination Data Buffer */
        for(u32ByteCount = 0; u32ByteCount < 256; u32ByteCount++)
            DestArray[u32ByteCount] = 0;

        printf("Done!");
        ILI9341_LCD_PutString(11*16, 20*8, (uint8_t *)"Done!", White, Red);
    }

    printf("\n\nSPI Flash Test Ok!");
    printf("\n\n");
    ILI9341_LCD_PutString(13*16, 0, (uint8_t *)"SPI Flash Test Ok!", White, Red);

    while(1);

}
