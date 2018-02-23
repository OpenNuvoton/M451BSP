/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:03a $
 * @brief    Display an string on TFT LCD panel via SPI interface.
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define CLK_PLLCTL_84MHz_HXT    (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 56) | CLK_PLLCTL_NO_4) /*!< Predefined PLLCTL setting for 72MHz PLL output with HXT(12MHz X'tal) */


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


void ILI9341_LCD_Fill(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, uint32_t bColor)
{
    uint32_t i, j;
    
    ILI9341_LCD_SetAddress(x, x1-1, y, y1-1);
    
    //for(i=0;i<x1-x;i++)
    {
        LCD_WriteCommand(0x2c);

        for(j=0;j<(x1-x)*(y1-y);j++)
        {
            LCD_WriteData(bColor >> 8);
            LCD_WriteData(bColor);
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

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_72MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);
    CLK_EnableModuleClock(EADC_MODULE);
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PLL, 0);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, NULL);

    /* EADC clock source is 72MHz, set divider to 8, ADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));
    

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk) ;
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD) ;

    /* SPI2: GPD12=SS, GPD15=CLK, GPD14=MISO, GPD13=MOSI */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD12MFP_SPI2_SS | SYS_GPD_MFPH_PD13MFP_SPI2_MOSI | SYS_GPD_MFPH_PD14MFP_SPI2_MISO | SYS_GPD_MFPH_PD15MFP_SPI2_CLK);

    /* Set PD multi-function pins for Timer0 event counter pin */
    //SYS->GPD_MFPL |= SYS_GPD_MFPL_PD2MFP_T0_EXT;
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD4MFP_T0;

    /* Configure the GPB0 - GPB3 ADC analog input pins.  */
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_EADC_CH11;

    /* Disable the digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1 << 14));

    /* PWM CH2 for LED2 */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC2MFP_PWM0_CH2;


    /* Lock protected registers */
    //SYS_LockReg();
}


volatile uint32_t g_u32CapValue = 0;
volatile uint32_t g_u32TmpCapValue[2] = {0};
volatile int32_t g_i32CapIdx = 0;
uint32_t g_u32Tmr0Ctl = 0;
void TMR0_IRQHandler(void)
{
//    if(TIMER0->EINTSTS == 1)
//    {
//        g_u32TmpCapValue[g_i32CapIdx++] = TIMER0->CAP;
//        if(g_i32CapIdx > 1)
//        {
//            g_u32CapValue = g_u32TmpCapValue[1] - g_u32TmpCapValue[0];
//            TIMER0->CMP = 0xffffff;
//            g_i32CapIdx = 0;
//        }
//        /* Clear Timer0 capture trigger interrupt flag */
//        TIMER0->EINTSTS = 1;
//    }
    
    if(TIMER0->INTSTS & 1)
    {
        // Time-out
        g_u32CapValue = 0;
        g_i32CapIdx = 0;
        TIMER0->INTSTS = 1;
    }
}


void TMR1_IRQHandler(void)
{
    g_u32CapValue = TIMER0->CNT;
    //TIMER0->CMP = 0xffff;
    TIMER0->CTL = g_u32Tmr0Ctl | TIMER_CTL_RSTCNT_Msk;        
    TIMER0->CTL = g_u32Tmr0Ctl;        
    //while(TIMER0->CNT < 3);
    
    TIMER1->INTSTS = 1;
    
}


int main(void)
{
    uint8_t buf[255] = {0};
    int32_t i;
    uint32_t u32Freq;
    uint32_t u32Data, u32Reg;
    extern void VectorRemap(void);
    uint32_t u32PwmFreq;
    uint32_t u32AdcValue = 0;
    
    VectorRemap();
    
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    // LEDS1
    GPIO_SetMode(PB, BIT6 | BIT1, GPIO_MODE_OUTPUT);
    PB6 = 1;
    
    
    /* Configure SPI3 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 4MHz */
    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 4000000);

    /* Configure SPI1 as a low level active device. */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Start SPI */
    SPI_ENABLE(SPI_LCD_PORT);

    /* Init LCD */
    ILI9341_LCD_Init();


    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_FALLING_EDGE);
    //TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);
    //TIMER_EnableInt(TIMER0);
    //TIMER_EnableCaptureInt(TIMER0);
    TIMER_Start(TIMER0);
    //NVIC_EnableIRQ(TMR0_IRQn);
    g_u32Tmr0Ctl = TIMER0->CTL;
    
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER1);
    //NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    TIMER_Start(TIMER1);

    /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 6);

    /* Configure the sample module 0 for analog input channel 11 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 11);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Enable the sample module 0 interrupt.  */
    //EADC_ENABLE_INT(EADC, 0x1);//Enable sample module A/D ADINT0 interrupt.
    //EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);//Enable sample module 0 interrupt.
    //NVIC_EnableIRQ(ADC00_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    //g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, 0x1);

    /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    //while(g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    //EADC_DISABLE_INT(EADC, 0x1);


    // PWM0 channel 0 frequency is 180000Hz, duty 50%,
    u32PwmFreq = 1000000;
    PWM_ConfigOutputChannel(PWM0, 2, u32PwmFreq, 50);

    // Enable output of PWM0 channel 0
    PWM_EnableOutput(PWM0, PWM_CH_2_MASK);

    // Enable PWM0 channel 0 period interrupt, use channel 0 to measure time.
    //PWM_EnablePeriodInt(PWM0, 0, 0);
    //NVIC_EnableIRQ(PWM0P0_IRQn);

    // Start
    PWM_Start(PWM0, PWM_CH_2_MASK);



    /* 
        PD4 for counter input. Support 0 ~ 6MHz
        PC2 for PWM output (duty cycle 50%)
        
        Variable resistor is used to control PWM output frequency.
    */
    
    ILI9341_LCD_Fill(0,0, 240, 320, Black);
    
    ILI9341_LCD_PutString(0,0,"PD4 for counter input. Support 0~14MHz",White,Black);
    ILI9341_LCD_PutString(16,0,"PC2 for PWM output (duty cycle 50%)",White,Black);
    ILI9341_LCD_PutString(32,0,"VR1 for PWM output frequency",White,Black);



    /* Show the String on the screen */
    i = 0;
    u32Data = 0;
    while(1)
    {
        
        u32Reg = EADC->DAT[0];
        if(u32Reg & EADC_DAT_VALID_Msk)
        {
            EADC_START_CONV(EADC, 0x1);
            u32Data = u32Data * 29000 + ((u32Reg & EADC_DAT_RESULT_Msk) * 3768) >> 15;
            if(u32AdcValue > u32Data)
            {
                if(u32AdcValue - u32Data >= 10)
                    u32AdcValue = u32Data;
            }
            else
            {
                if(u32Data - u32AdcValue >= 10)
                    u32AdcValue = u32Data;
            }
            
            u32Reg = (SystemCoreClock/1000) / u32AdcValue /4;
            PWM0->PERIOD[2] = u32Reg - 1;
            PWM0->CMPDAT[2] = (u32Reg >> 1) - 1;
            
        }

        sprintf((char *)buf, "Test Frequency %5d    ", u32AdcValue*4);
        ILI9341_LCD_PutString(48,0,buf,White,Black);

        
        //u32Freq = (SystemCoreClock / g_u32CapValue + 1) >> 1;
        u32Freq = g_u32CapValue;
        
        if(u32Freq >= 1000000)
            sprintf((char *)buf, "Frequency %3d.%06dMHz            ", u32Freq/1000000,u32Freq%1000000);
        else if(u32Freq >= 1000)
            sprintf((char *)buf, "Frequency %3d.%03dKHz            ", u32Freq/1000,u32Freq%1000);
        else
            sprintf((char *)buf, "Frequency %3dHz            ", u32Freq);
        
        ILI9341_LCD_PutString(64,0,buf,Green,Black);
        
        
        
        //PB1 ^= 1;
        //CLK_SysTickDelay(100000);
        //CLK_SysTickDelay(100000);
    }
    while(1);
}
