#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_RTC.h"

volatile uint32_t g_u32RTCTickINT,u32counter,u32Sec;
volatile uint8_t u8IsNewDateTime = 0;
volatile uint8_t g_u8IsRTCAlarmINT = 0;
extern int IsDebugFifoEmpty(void);
S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;


#define KEY1 PE2		//NU4.8
#define KEY2 PA8		//NU4.8
#define KEY3 PB6		//NU3.1
#define KEY4 PB7    	//NU3.2

void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();

        g_u32RTCTickINT++;

        PB8 ^= 1;
    }
	
    /* To check if RTC alarm interrupt occurred */
    if(RTC_GET_ALARM_INT_FLAG() == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG();

        g_u8IsRTCAlarmINT++;
    }
}

void Initial_Key_Input(void)
{  
	GPIO_SetMode(PE, BIT2, GPIO_MODE_INPUT);
	GPIO_SetMode(PA, BIT8, GPIO_MODE_INPUT);
	GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);
	GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);	
}

unsigned char Get_Key_Input(void)
{
	unsigned char temp=0;
	if (KEY1 == 0)		
		temp=0x1;
	if (KEY2 == 0)
		temp=0x2;	
	if (KEY3 == 0)
		temp=0x4;	
	if (KEY4 == 0)
		temp=0x8;    
	return   temp;
}

void get_key(void)
{
	if(Get_Key_Input()==0x01){
	}
	if(Get_Key_Input()==0x02){
	}
	if(Get_Key_Input()==0x04){
	}
	if(Get_Key_Input()==0x08)
	{
		Alarm_Wakeup_function();
	}
}

void Alarm_Wakeup_function(void)
{
    /* Open RTC */
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 15;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 23;
    sWriteRTC.u32Minute     = 59;
    sWriteRTC.u32Second     = 50;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
    RTC_Open(&sWriteRTC);

    /* Set RTC alarm date/time */
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 15;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 23;
    sWriteRTC.u32Minute     = 59;
    sWriteRTC.u32Second     = 55;
    RTC_SetAlarmDateAndTime(&sWriteRTC);

    /* Enable RTC alarm interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);
	//
	printf("# Set RTC current date/time: 2014/05/15 23:59:50.\n");
    printf("# Set RTC alarm date/time:   2014/05/15 23:59:55.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    printf("\nSystem enter to power-down mode ...\n");
    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);
    CLK_PowerDown();

    while(g_u8IsRTCAlarmINT == 0);

    /* Read current RTC date/time */
    RTC_GetDateAndTime(&sReadRTC);
    printf("System has been waken-up and current date/time is:\n");
    printf("    %d/%02d/%02d %02d:%02d:%02d\n",
           sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

    printf("\n\n");
    printf("# Set next RTC alarm date/time: 2014/05/16 00:00:05.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");
    RTC_SetAlarmDate(2014, 05, 16);
    RTC_SetAlarmTime(0, 0, 5, RTC_CLOCK_24, 0);

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    printf("\nSystem enter to power-down mode ...\n");
    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);
    CLK_PowerDown();

    while(g_u8IsRTCAlarmINT == 0);

    /* Read current RTC date/time */
    RTC_GetDateAndTime(&sReadRTC);
    printf("System has been waken-up and current date/time is:\n");
    printf("    %d/%02d/%02d %02d:%02d:%02d\n",
           sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

	g_u32RTCTickINT = 4;
	Time_and_Tick_function();
}

void RTC_Init(void)
{	
	Initial_Key_Input();
	/* Enable peripheral clock */
	CLK_EnableModuleClock(RTC_MODULE);
	
	printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    RTC Date/Time and Tick Sample Code   |\n");
    printf("+-----------------------------------------+\n\n");

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Open RTC and start counting */
    sWriteRTC.u32Year       = 2014;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 15;
    sWriteRTC.u32DayOfWeek  = RTC_THURSDAY;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 30;
    sWriteRTC.u32Second     = 30;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
    RTC_Open(&sWriteRTC);

    /* Enable RTC tick interrupt, one RTC tick is 1/4 second */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_4_SEC);

    printf("# Showing RTC date/time on UART0.\n\n");
    printf("1.) Use PB.8 to check tick period time is 1/4 second or not.\n");
    printf("2.) Show RTC date/time and change date/time after 5 seconds:\n");

    /* Use PB.8 to check tick period time */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE8_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE8_Pos);

    u32Sec = 0;
    g_u32RTCTickINT = 0;
}

void Time_and_Tick_function(void)
{

	    if(g_u32RTCTickINT == 4)
        {
            g_u32RTCTickINT = 0;

            /* Read current RTC date/time */
            RTC_GetDateAndTime(&sReadRTC);

            if(u8IsNewDateTime == 0)
            {
                printf("    %d/%02d/%02d %02d:%02d:%02d\n",
                       sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);
            }
            else
            {
                printf("    %d/%02d/%02d %02d:%02d:%02d\r",
                       sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);
            }

            if(u32Sec == sReadRTC.u32Second)
            {
                printf("\nRTC tick period time is incorrect.\n");
                while(1);
            }
			
			get_key(); //into function
            
			u32Sec = sReadRTC.u32Second;

            if(u8IsNewDateTime == 0)
            {
                if(u32Sec == (sWriteRTC.u32Second + 5))
                {
                    printf("\n");
                    printf("3.) Update new date/time to 2014/05/18 11:12:13.\n");

                    u8IsNewDateTime = 1;
                    RTC_SetDate(2014, 5, 18, RTC_SUNDAY);
                    RTC_SetTime(11, 12, 13, RTC_CLOCK_24, RTC_AM);
                }
            }
        }
}

