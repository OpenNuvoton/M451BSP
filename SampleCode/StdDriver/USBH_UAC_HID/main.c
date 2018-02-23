/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:05a $
 * @brief
 *        A USB Host sample code to support USB Audio Class with HID composite device.
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"
#include "usbh_core.h"
#include "usbh_uac.h"
#include "usbh_hid.h"

uint8_t  au_in_buff[2048];

static volatile int  au_in_cnt, au_out_cnt;

void Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}


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

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBH_MODULE, 0, CLK_CLKDIV0_USB(3));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USB Host clock                                                                                     */
    /*---------------------------------------------------------------------------------------------------------*/

    // Configure OTG function as Host-Only
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk | SYS_USBPHY_USBROLE_STD_USBH;

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
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/**
 *  @brief  Audio-in data callback function.
 *          UAC driver notify user that audio-in data has been moved into user audio-in buffer,
 *          which is provided by user application via UAC_InstallIsoInCbFun().
 *  @param[in] dev    Audio Class device
 *  @param[in] data   Available audio-in data, which is located in user audio-in buffer.
 *  @param[in] len    Length of available audio-in data started from <data>.
 *  @return   UAC driver does not check this return value.
 */
int audio_in_callback(UAC_DEV_T *dev, uint8_t *data, int len)
{
    au_in_cnt += len;
    //printf("I %x,%x\n", (int)data & 0xffff, len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to get audio-in data ...
    // For example, memcpy(audio_record_buffer, data, len);
    // . . .

    return 0;
}


/**
 *  @brief  Audio-out data callback function.
 *          UAC driver requests user to move audio-out data into the specified address. The audio-out
 *          data will then be send to UAC device via isochronous-out pipe.
 *  @param[in] dev    Audio Class device
 *  @param[in] data   Application should move audio-out data into this buffer.
 *  @param[in] len    Maximum length of audio-out data can be moved.
 *  @return   Actual length of audio data moved.
 */
int audio_out_callback(UAC_DEV_T *dev, uint8_t *data, int len)
{
    au_out_cnt += len;
    //printf("O %x,%x\n", (int)data & 0xffff, len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to put audio-out data ...
    // For example, memcpy(data, playback_buffer, actual_len);
    //              return actual_len;
    // . . .

    return 192;   // for 48000 stero Hz
}


void  uac_control_example(UAC_DEV_T *uac_dev)
{
    uint8_t    data[8];
    uint32_t   srate[4];
    uint32_t   val32;
    uint16_t   val16;
    uint8_t    val8;
    int        i, ret;

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
    if(ret < 0)
        printf("    Failed to get microphone's channel number.\n");
    else
        printf("    Microphone: %d\n", ret);

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

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker L(F) volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get seaker L(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker L(F) minimum volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker L(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker L(F) maximum volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker L(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker left channel.             */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker L(F) volume resolution is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker L(F) volume resolution!\n");


    printf("\nSpeaker R(F) volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker R(F) volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker R(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker R(F) minimum volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker R(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker R(F) maximum volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker R(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker right channel.            */
    /*--------------------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_RIGHT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Speaker R(F) volume resolution is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get speaker R(F) volume resolution!\n");

    printf("\nMicrophone mute control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if(UAC_MuteControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK)
    {
        printf("    Microphone mute state is %d.\n", data[0]);
    }
    else
        printf("    Failed to get microphone mute state!\n");

    printf("\nMicrophone volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &val16) == UAC_RET_OK)
    {
        printf("    Microphone current volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get microphone current volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_MIN, UAC_CH_MASTER, &val16) == UAC_RET_OK)
    {
        printf("    Microphone minimum volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get microphone minimum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_MAX, UAC_CH_MASTER, &val16) == UAC_RET_OK)
    {
        printf("    Microphone maximum volume is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get microphone maximum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get resolution of UAC device's microphone volume value.    */
    /*-------------------------------------------------------------*/
    if(UAC_VolumeControl(uac_dev, UAC_MICROPHONE, UAC_GET_RES, UAC_CH_LEFT_FRONT, &val16) == UAC_RET_OK)
    {
        printf("    Microphone volume resolution is 0x%x.\n", val16);
    }
    else
        printf("    Failed to get microphone volume resolution!\n");

    printf("\nMicrophone automatic gain control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if(UAC_AutoGainControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, data) == UAC_RET_OK)
    {
        printf("    Microphone auto gain is %s.\n", data[0] ? "ON" : "OFF");
    }
    else
        printf("    Failed to get microphone auto-gain state!\n");

    printf("\nSampling rate control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's speaker.   */
    /*-------------------------------------------------------------*/
    if(UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &val32) == UAC_RET_OK)
    {
        printf("    Speaker's current sampling rate is %d.\n", val32);
    }
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's speaker.       */
    /*-------------------------------------------------------------*/
    val32 = 48000;
    if(UAC_SamplingRateControl(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &val32) == UAC_RET_OK)
    {
        printf("    Speaker's current sampling rate is %d.\n", val32);
    }
    else
        printf("    Failed to set speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's microphone.*/
    /*-------------------------------------------------------------*/
    if(UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &val32) == UAC_RET_OK)
    {
        printf("    Microphone's current sampling rate is %d.\n", val32);
    }
    else
        printf("    Failed to get microphone's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's microphone.    */
    /*-------------------------------------------------------------*/
    val32 = 48000;
    if(UAC_SamplingRateControl(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, &val32) == UAC_RET_OK)
    {
        printf("    Microphone's current sampling rate is %d.\n", val32);
    }
    else
        printf("    Failed to set microphone's current sampling rate!\n");
}


void  int_read_callback(HID_DEV_T *hdev, uint8_t *rdata, int data_len)
{
    int  i;
    printf("INT-in pipe data %d bytes received =>\n", data_len);
    for(i = 0; i < data_len; i++)
        printf("0x%02x ", rdata[i]);
    printf("\n");
}

uint32_t CLK_GetUSBFreq(void)
{  
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/  
    /* USB Peripheral clock = PLL_CLOCK/USBDIV+1) */    
    return CLK_GetPLLClockFreq()/(((CLK->CLKDIV0 & CLK_CLKDIV0_USBDIV_Msk)>>CLK_CLKDIV0_USBDIV_Pos)+1);
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    UAC_DEV_T    *uac_dev;
    HID_DEV_T    *hdev;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf(" System clock:   %d Hz.\n", SystemCoreClock);
    printf(" USB Host clock: %d Hz.\n", CLK_GetUSBFreq());    
    printf("+--------------------------------------+\n");
    printf("|                                      |\n");
    printf("|  M451 USB Host UAC sample program    |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");

    USBH_Open();

    USBH_HidInit();
    UAC_Init();

    printf("Wait until any Audio Class devices connected...\n");
    while(1)
    {
        USBH_ProcessHubEvents();             /* USB Host port detect polling and management */

        uac_dev = UAC_GetDeviceList();
        if(uac_dev != NULL)
            break;
    }

    hdev = USBH_HidGetDeviceList();
    if(hdev == NULL)
    {
        printf("HID device not found!\n");
    }
    else
    {
        printf("\nUSBH_HidStartIntReadPipe...\n");
        if(USBH_HidStartIntReadPipe(hdev, int_read_callback) == HID_RET_OK)
        {
            printf("Interrupt in transfer started...\n");
        }
    }

    uac_control_example(uac_dev);

    if(UAC_InstallIsoInCbFun(uac_dev, au_in_buff, 2048, audio_in_callback) != UAC_RET_OK)
    {
        printf("Failed to install audio-in callback function!\n");
        goto err_out;
    }

    if(UAC_InstallIsoOutCbFun(uac_dev, audio_out_callback) != UAC_RET_OK)
    {
        printf("Failed to install audio-out callback function!\n");
        goto err_out;
    }

    while(1)
    {
        au_in_cnt = 0;
        au_out_cnt = 0;

        printf("\nStart audio output stream...\n");
        UAC_StartIsoOutPipe(uac_dev);

        printf("\nStart audio input stream...\n");
        UAC_StartIsoInPipe(uac_dev);

        while(au_in_cnt < 64 * 1024);

        UAC_StopIsoInPipe(uac_dev);
        printf("64 KB bytes audio data received.\n");
        printf("Audio input stream stopped.\n");

        while(au_out_cnt < 64 * 1024) ;

        UAC_StopIsoOutPipe(uac_dev);
        printf("64 KB bytes audio data send.\n");
        printf("Audio output stream stopped.\n");

        getchar();
    }

err_out:
    printf("\nFailed!\n");
    while(1);
}


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
