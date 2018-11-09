/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB audio class device.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "usbh_uac_nau8822.h"

static uint16_t  vol_max, vol_min, vol_res, vol_cur;

extern int kbhit(void);             /* function in retarget.c */
extern char GetChar(void);          /* function in retarget.c */
#define UAC_MIXER 0

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBH_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBH_MODULE, 0, CLK_CLKDIV0_USB(3));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PLL, 0);

    // Configure OTG function as Host-Only
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk | SYS_USBPHY_USBROLE_STD_USBH;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

#ifdef AUTO_POWER_CONTROL
    /* Below settings is use power switch IC to enable/disable USB Host power.
       Set PC.4 is VBUS_EN function pin and PC.3 VBUS_ST function pin             */
    //SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
    //SYS->GPC_MFPL |=  (SYS_GPC_MFPL_PC3MFP_USB_VBUS_ST | SYS_GPC_MFPL_PC4MFP_USB_VBUS_EN);

    /* Below settings is use power switch IC to enable/disable USB Host power.
       Set PA.2 is VBUS_EN function pin and PA.3 VBUS_ST function pin             */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_USB_VBUS_ST | SYS_GPA_MFPL_PA2MFP_USB_VBUS_EN);

    CLK->APBCLK0 |= CLK_APBCLK0_OTGCKEN_Msk;   //Enable OTG_EN clock
#else
    /* Below settings is use GPIO to enable/disable USB Host power.
       Set PC.4 output high to enable USB power                               */

    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC4MFP_Msk;
    PC->MODE = (PC->MODE & ~GPIO_MODE_MODE4_Msk) | (0x1 << GPIO_MODE_MODE4_Pos);
    PC->DOUT |= 0x10;
#endif

    /* Set PD.0, PD.1, PD.6, PD.4 and PD.5 as SPI1_I2SMCLK, UART0 TXD, UART0 RXD, I2C0_SDA and I2C0_SCL function pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD6MFP_Msk |
                    SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_SPI1_I2SMCLK | SYS_GPD_MFPL_PD1MFP_UART0_TXD | SYS_GPD_MFPL_PD6MFP_UART0_RXD |
                    SYS_GPD_MFPL_PD4MFP_I2C0_SDA | SYS_GPD_MFPL_PD5MFP_I2C0_SCL;

    /* Set I2S1 interface: I2S1_LRCLK (PA.4), I2S1_DO (PA.5), I2S1_DI (PA.6), I2S1_BCLK (PA.7) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & 0x0000FFFF) |
                    SYS_GPA_MFPL_PA4MFP_SPI1_SS | SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA7MFP_SPI1_CLK;
}

uint32_t CLK_GetUSBFreq(void)
{  
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/  
    /* USB Peripheral clock = PLL_CLOCK/USBDIV+1) */    
    return CLK_GetPLLClockFreq()/(((CLK->CLKDIV0 & CLK_CLKDIV0_USBDIV_Msk)>>CLK_CLKDIV0_USBDIV_Pos)+1);
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}

void  uac_control_example(UAC_DEV_T *uac_dev)
{
    uint16_t   val16;
    uint32_t   srate[4];
    uint8_t    val8;
    //uint8_t    data[8];
    int        i, ret;
    uint32_t   val32;
    
   	vol_max = vol_min = vol_res = 0;

    printf("\nGet channel information ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get channel number information                             */
    /*-------------------------------------------------------------*/
    ret = UAC_GetChannelNumber(uac_dev, UAC_SPEAKER);
    if(ret < 0)
        printf("    Failed to get speaker's channel number.\n");
    else
        printf("    Speaker: %d\n", ret);

    ret = UAC_GetChannelNumber(uac_dev, UAC_MICROPHONE);
    if (ret < 0)
        printf("    Failed to get microphone's channel number.\n");
    else
    {
        printf("    Microphone: %d\n", ret);
        if (ret == 1)
        	g_bMicIsMono = 1;
        else
        	g_bMicIsMono = 0;
    }

    printf("\nGet subframe bit resolution ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    ret = UAC_GetBitResolution(uac_dev, UAC_SPEAKER, &val8);
    if(ret < 0)
        printf("    Failed to get speaker's bit resoltion.\n");
    else
    {
        printf("    Speaker audio subframe size: %d bytes\n", val8);
        printf("    Speaker subframe bit resolution: %d\n", ret);
    }

    ret = UAC_GetBitResolution(uac_dev, UAC_MICROPHONE, &val8);
    if(ret < 0)
        printf("    Failed to get microphone's bit resoltion.\n");
    else
    {
        printf("    Microphone audio subframe size: %d bytes\n", val8);
        printf("    Microphone subframe bit resolution: %d\n", ret);
    }

    printf("\nGet sampling rate list ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    ret = UAC_GetSamplingRate(uac_dev, UAC_SPEAKER, (uint32_t *)&srate[0], 4, &val8);
    if(ret < 0)
        printf("    Failed to get speaker's sampling rate.\n");
    else
    {
        if(val8 == 0)
            printf("    Speaker sampling rate range: %d ~ %d Hz\n", srate[0], srate[1]);
        else
        {
            for(i = 0; i < val8; i++)
                printf("    Speaker sampling rate: %d\n", srate[i]);
        }
    }

    ret = UAC_GetSamplingRate(uac_dev, UAC_MICROPHONE, (uint32_t *)&srate[0], 4, &val8);
    if(ret < 0)
        printf("    Failed to get microphone's sampling rate.\n");
    else
    {
        if(val8 == 0)
            printf("    Microphone sampling rate range: %d ~ %d Hz\n", srate[0], srate[1]);
        else
        {
            for(i = 0; i < val8; i++)
                printf("    Microphone sampling rate: %d\n", srate[i]);
        }
    }

#if 0
    printf("\nSpeaker mute control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if(UAC_MuteControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK)
    {
        printf("    Speaker mute state is %d.\n", data[0]);
    }
    else
        printf("    Failed to get speaker mute state!\n");

	printf("\nSpeaker L(F) volume control ===>\n");
#endif

	/*--------------------------------------------------------------------------*/
	/*  Get current volume value of UAC device's speaker left channel.          */
	/*--------------------------------------------------------------------------*/
	if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
    	printf("    Speaker L(F) volume is 0x%x.\n", val16);
	else
    	printf("    Failed to get seaker L(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker L(F) minimum volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker L(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker L(F) maximum volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker L(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker left channel.             */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker L(F) volume resolution is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker L(F) volume resolution!\n");


    printf("\nSpeaker R(F) volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker R(F) volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker R(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker R(F) minimum volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker R(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker R(F) maximum volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker R(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker right channel.            */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
        printf("    Speaker R(F) volume resolution is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker R(F) volume resolution!\n");

    printf("\nSpeaker master volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker master channel.        */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_MASTER, &val16) == UAC_RET_OK)
        printf("    Speaker minimum master volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker master minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker master channel.        */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_MASTER, &val16) == UAC_RET_OK)
        printf("    Speaker maximum master volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker maximum master volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker master channel.           */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_MASTER, &val16) == UAC_RET_OK)
        printf("    Speaker master volume resolution is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker master volume resolution!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker master channel.        */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, &val16) == UAC_RET_OK)
        printf("    Speaker master volume is 0x%x.\n", val16);
    else
        printf("    Failed to get speaker master volume!\n");

    printf("\nMixer master volume control ===>\n");

#if 0
    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's microphone.         */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone mute control ===>\n");
    if(UAC_MuteControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK)
        printf("    Microphone mute state is %d.\n", data[0]);
    else
        printf("    Failed to get microphone mute state!\n");
#endif

    printf("\nMicrophone volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &vol_cur) == UAC_RET_OK)
        printf("    Microphone current volume is 0x%x.\n", vol_cur);
    else
        printf("    Failed to get microphone current volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_MIN, UAC_CH_MASTER, &vol_min) == UAC_RET_OK)
        printf("    Microphone minimum volume is 0x%x.\n", vol_min);
    else
        printf("    Failed to get microphone minimum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_MAX, UAC_CH_MASTER, &vol_max) == UAC_RET_OK)
        printf("    Microphone maximum volume is 0x%x.\n", vol_max);
    else
        printf("    Failed to get microphone maximum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get resolution of UAC device's microphone volume value.    */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_RES, UAC_CH_MASTER, &vol_res) == UAC_RET_OK)
        printf("    Microphone volume resolution is 0x%x.\n", vol_res);
    else
        printf("    Failed to get microphone volume resolution!\n");

#if 0
    /*-------------------------------------------------------------*/
    /*  Get current auto-gain setting of UAC device's microphone.  */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone automatic gain control ===>\n");
    if(UAC_AutoGainControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK)
        printf("    Microphone auto gain is %s.\n", data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get microphone auto-gain state!\n");
#endif

    printf("\nSampling rate control ===>\n");
    
    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's speaker.   */
    /*-------------------------------------------------------------*/
    if(UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &val32) == UAC_RET_OK)
        printf("    Speaker's current sampling rate is %d.\n", val32);
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's speaker.       */
    /*-------------------------------------------------------------*/
    val32 = 48000;
    if (UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &val32) != UAC_RET_OK)
        printf("    Failed to set Speaker's current sampling rate %d.\n", val32);

    if(UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &val32) == UAC_RET_OK)
        printf("    Speaker's current sampling rate is %d.\n", val32);
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's microphone.*/
    /*-------------------------------------------------------------*/
    if(UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &val32) == UAC_RET_OK)
        printf("    Microphone's current sampling rate is %d.\n", val32);
    else
        printf("    Failed to get microphone's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's microphone.    */
    /*-------------------------------------------------------------*/
    val32 = 48000;
    if (UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, &val32) != UAC_RET_OK)
        printf("    Failed to set microphone's current sampling rate!\n");
    
    if (UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &val32) == UAC_RET_OK)
        printf("    Microphone's current sampling rate is %d.\n", val32);
    else
        printf("    Failed to get microphone's current sampling rate!\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    UAC_DEV_T  *uac_dev;
    int        i, ch;
    uint16_t   val16;

    /*
        This sample code is used to demo USB Host Audio Class + NAU8822.

        ENABLE_UAC_REC 
             - This sample grabes audio input from UAC MIC and playback to NAU8822 speaker.
        ENABLE_UAC_PLAY
             - This sample grabes audio input from NAU8822 MIC and playback to UAC speaker.

        NAU8822 is connect with I2S1 (PA.4~PA.7) and controlled by I2C0 (PD.4, PD.5).
        NAU8822 clock source is also come from I2S1 (MCLK, PD.0).
    */

    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    printf("\n\n");
    printf(" System clock:   %d Hz.\n", SystemCoreClock);
    printf(" USB Host clock: %d Hz.\n", CLK_GetUSBFreq());    
    printf("+------------------------------------------------------+\n");
    printf("|      NuMicro USB Host UAC + NAU882 Sample Code       |\n");
    printf("+------------------------------------------------------+\n");

    USBH_Open();

    UAC_Init();

    /* Init I2C0 to access NAU8822 */
    I2C0_Init();

    /* Initialize NAU8822 codec */
    WAU8822_Setup();

    WAU8822_SetSamplingRate(48000);

    I2S_Open(SPI1, I2S_MODE_SLAVE, PLAY_RATE, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(SPI1, 12000000);

    /* Fill dummy data to I2S TX for start I2S iteration */
    for(i = 0; i < 4; i++)
        I2S_WRITE_TX_FIFO(SPI1, 0);

    /* Start I2S play iteration */
    I2S_EnableInt(SPI1, I2S_FIFO_TXTH_INT_MASK | I2S_FIFO_RXTH_INT_MASK);

    NVIC_EnableIRQ(SPI1_IRQn);

    /* SPI (I2S) interrupt has higher frequency then USBH interrupt.
       Therefore, we need to set SPI (I2S) with higher priority to avoid
       SPI (I2S) interrupt pending too long time when USBH interrupt happen. */
    NVIC_SetPriority(USBH_IRQn, 3);
    NVIC_SetPriority(SPI1_IRQn, 2);

    while(1)
    {
        if (USBH_ProcessHubEvents())              /* USB Host port detect polling and management */
        {
        	/*
        	 *  Has hub port event. 
        	 */
        	
        	uac_dev = UAC_GetDeviceList();
        	if (uac_dev == NULL)
                continue;

            if (uac_dev != NULL)                  /* should be newly connected UAC device        */
            {
            	/*UAC_StartDevice(uac_dev);*/
            	
    			uac_control_example(uac_dev);

				val16 = 0x8800;
        		UAC_VolumeControl(uac_dev, UAC_MIXER, UAC_SET_CUR, UAC_CH_MASTER, &val16);

    			if (UAC_InstallIsoInCbFun(uac_dev, NULL, 0xFFFF, audio_in_callback) != UAC_RET_OK)
    			{
        			printf("Failed to install audio-in callback function!\n");
        			goto err_out;
    			}

    			if (UAC_InstallIsoOutCbFun(uac_dev, audio_out_callback) != UAC_RET_OK)
    			{
        			printf("Failed to install audio-out callback function!\n");
        			goto err_out;
    			}

#ifdef ENABLE_UAC_REC
    			UAC_StartIsoInPipe(uac_dev);
#endif

#ifdef ENABLE_UAC_PLAY
    			UAC_StartIsoOutPipe(uac_dev);
#endif    			
            }
        }
        
        if (uac_dev == NULL)
        {
        	g_u8RecEn = 0;
        	g_u8PlayEn = 0;
            continue;
        }
        
    	if (!kbhit())
    	{
    		ch = GetChar();
    
    		if ((ch == '+') && (vol_cur + vol_res <= vol_max))
    		{
    			printf("+");
    			val16 = vol_cur+vol_res;
        		if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, &val16) == UAC_RET_OK)
        		{
            		printf("    Microphone set volume 0x%x success.\n", val16);
            		vol_cur = val16;
            	}
            	else
            		printf("    Failed to set microphone volume 0x%x!\n", val16);
            }
    		else if ((ch == '-') && (vol_cur - vol_res >= vol_min))
    		{
    			printf("-");
    			val16 = vol_cur-vol_res;
        		if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, &val16) == UAC_RET_OK)
        		{
            		printf("    Microphone set volume 0x%x success.\n", val16);
            		vol_cur = val16;
            	}
            	else
            		printf("    Failed to set microphone volume 0x%x!\n", val16);
            }
    		else if ((ch == '0') && (vol_cur - vol_res >= vol_min))
    		{
        		if (UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &vol_cur) == UAC_RET_OK)
            		printf("    Microphone current volume is 0x%x.\n", vol_cur);
        		else
            		printf("    Failed to get microphone current volume!\n");
    		}
            else
            {
            	/* ignore this key */
            }

        }  /* end of kbhit() */
    }

err_out:
	while (1);    
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

