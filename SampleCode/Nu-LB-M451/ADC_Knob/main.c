/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to use EADC to measure voltage which is controled by cariable resistor.
						 And turn on the LED based on conversion.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"

#define PLL_CLOCK       72000000

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
		
    /* Set PD multi-function pins for UART0 RXD(PD.6) and TXD(PD.1) */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk);
		SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_GPIO);
		SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC2MFP_GPIO | SYS_GPC_MFPL_PC3MFP_GPIO | SYS_GPC_MFPL_PC7MFP_GPIO);
		
		/* SPI2: GPD12=SS, GPD15=CLK, GPD14=MISO, GPD13=MOSI */
		SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_SPI2_SS | SYS_GPD_MFPH_PD13MFP_SPI2_MOSI | SYS_GPD_MFPH_PD14MFP_SPI2_MISO | SYS_GPD_MFPH_PD15MFP_SPI2_CLK;
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

void GPIO_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init GPIO                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure PB.1, PC.2, PC.3 and PC.7 as Output mode */
    GPIO_SetMode(PB, BIT1, GPIO_MODE_OUTPUT);
		GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);
		GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);
		GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT);
	
		/* Trun off LED 5 ~ 8 */
		PB1 = 1;
		PC2 = 1;
		PC3 = 1;
		PC7 = 1;
}

void Open_ADC_Knob(void)
{
	  /*---------------------------------------------------------------------------------------------------------*/
    /* Init ADC                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is HCLK(72MHz), set divider to 8, ADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Configure the GPB14 for ADC analog input pins  */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB14MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_EADC_CH11;

    /* Disable the GPB14 digital input path to avoid the leakage current */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT14);

    /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 14);

    /* Configure the sample module 0 for analog input channel 11 and software trigger source */
    EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 11);
		/* Enable sample module 0 interrupt */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Enable the sample module 0 A/D ADINT0 interrupt */
    EADC_ENABLE_INT(EADC, 0x1);
}

uint32_t Get_ADC_Knob(void)
{
    uint32_t ADC_Raw_Data;
	
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Trigger sample module 0 to start A/D conversion */
    EADC_START_CONV(EADC, 0x1);

    /* Wait ADC interrupt Flag be setted */
    while(EADC_GET_INT_FLAG(EADC, 0x1) == 0);
    ADC_Raw_Data = EADC_GET_CONV_DATA(EADC, 0);

    return ADC_Raw_Data;
}

void Show_LED(uint32_t input_value)
{
		/* Turn on LED6 if Volume >= 1000 */
		(input_value>=1000)?(PC2 = 0):(PC2 = 1);
		/* Turn on LED5 if Volume >= 2000 */
		(input_value>=2000)?(PC3 = 0):(PC3 = 1);
		/* Turn on LED8 if Volume >= 3000 */
		(input_value>=3000)?(PC7 = 0):(PC7 = 1);
		/* Turn on LED7 if Volume >= 4000 */
		(input_value>=4000)?(PB1 = 0):(PB1 = 1);
}

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
    if(x1 > 240)
        x1 = 240;
    if(x2 > 240)
        x2 = 240;  
    if(y1 > 320)
        y1 = 320;  
    if(y2 > 320)
        y2 = 320;      
    
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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
		uint8_t Message[30];
		char* Message_Print;
		uint32_t Volume, i=0, j;
	
    Message_Print = (char*)Message;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init GPIO for LED control */
    GPIO_Init();		

    /* Configure SPI2 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 4MHz */
    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 4000000);

    /* Configure SPI2 as a low level active device. */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Start SPI */
    SPI_ENABLE(SPI_LCD_PORT);

    /* Init LCD */
    ILI9341_LCD_Init();
		ILI9341_LCD_Color(Red);
	
    /* Open Volume Knob Device */
    Open_ADC_Knob();
	
    while(1)
    {
				printf("Volume Knob Value: ");			
        /* Get Volume Knob Data, Volume Range: 0 ~ 4095 */
        Volume = Get_ADC_Knob();
        printf("%d\n", Volume);
				/* Turn on LED based on Volume */
				Show_LED(Volume);

			  /* Show the String on the screen */
				sprintf(Message_Print, "Volume Knob Value: %4d", Volume);
				ILI9341_LCD_PutString(16*i, 0, Message, White, Red);
				(i>=14)?(i=0):(i++);
			
				/* Delay 0.5 second */
				for(j=0;j<=1;j++)
						CLK_SysTickDelay(250000);
    }
}
