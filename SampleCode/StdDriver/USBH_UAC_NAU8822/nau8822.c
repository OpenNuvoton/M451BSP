/******************************************************************************
 * @file     usbd_audio.c
 * @brief    NuMicro series USBD audio sample file
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include <stdio.h>
#include "M451Series.h"
#include "usbh_uac_nau8822.h"


/*--------------------------------------------------------------------------*/
/* Global variables  */

volatile uint8_t g_u8RecEn = 0;
volatile uint8_t g_u8PlayEn = 0;
static volatile int32_t g_i32AdjFlag = 0;    /* To indicate current I2S frequency adjustment status */


/*******************************************************************/
typedef enum
{
    E_RS_NONE,          // no resampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;


volatile int8_t   g_bMicIsMono = 0;

/* UAC audio out buffer. It's also the record PCM buffer of NAU8822. */
#ifdef __ICCARM__
#pragma data_alignment=32
uint8_t g_u8UacOutBuf[UAC_OUT_BUF_LEN];
#else
__align(32) uint8_t g_u8UacOutBuf[UAC_OUT_BUF_LEN];
#endif
volatile uint32_t g_UacOutBuff_8822RecPos = 0;   /* NAU8822 record pointer          */
volatile uint32_t g_UacOutBuff_UacPlayPos = 0;   /* UAC play data pointer           */
volatile uint32_t g_UacOutBuff_8822RecCnt = 0;   /* NAU8822 record pcm counter      */
volatile uint32_t g_UacOutBuff_UacPlayCnt = 0;   /* UAC play pcm counter            */

/* UAC audio in buffer. It's also the play PCM buffer of NAU8822. */
#ifdef __ICCARM__
#pragma data_alignment=32
uint8_t g_u8UacInBuf[UAC_IN_BUF_LEN];
#else
__align(32) uint8_t g_u8UacInBuf[UAC_IN_BUF_LEN];
#endif
volatile uint32_t g_UacInBuff_8822PlayPos = 0;   /* NAU8822 play pointer            */
volatile uint32_t g_UacInBuff_UacRecPos = 0;     /* UAC data recieving pointer      */
volatile uint32_t g_UacInBuff_8822PlayCnt = 0;   /* NAU8822 play pcm counter        */
volatile uint32_t g_UacInBuff_UacRecCnt = 0;     /* UAC data recieving pcm counter  */


void ResetAudioBuffer(void)
{
	memset(g_u8UacOutBuf, 0, sizeof(g_u8UacOutBuf));
	g_UacOutBuff_8822RecPos = 0;
	g_UacOutBuff_UacPlayPos = 0;
	g_UacOutBuff_8822RecCnt = 0;
	g_UacOutBuff_UacPlayCnt = 0;

	memset(g_u8UacInBuf, 0, sizeof(g_u8UacInBuf));
	g_UacInBuff_8822PlayPos = 0;
	g_UacInBuff_UacRecPos = 0;
	g_UacInBuff_8822PlayCnt = 0;
	g_UacInBuff_UacRecCnt = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{
    /* Send START */
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    /* Send device address */
    I2C_SET_DATA(I2C0, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send data */
    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    /* Send STOP */
    I2C_STOP(I2C0);

}

void WAU8822_Setup(void)
{
    ResetAudioBuffer();
    
    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

    I2C_WriteWAU8822(1,  0x0EF);
    I2C_WriteWAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */

#if(PLAY_RATE == 48000)
    I2C_WriteWAU8822(6,  0x14D);   /* Divide by 2, 48K */
    I2C_WriteWAU8822(7,  0x000);   /* 48K for internal filter coefficients */
#elif(PLAY_RATE == 32000)
    I2C_WriteWAU8822(6,  0x16D);   /* Divide by 3, 32K */
    I2C_WriteWAU8822(7,  0x002);   /* 32K for internal filter coefficients */
#elif(PLAY_RATE == 16000)
    I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
#else
    I2C_WriteWAU8822(6,  0x1ED);   /* Divide by 12, 8K */
    I2C_WriteWAU8822(7,  0x00A);   /* 8K for internal filter coefficients */
#endif

    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x0FF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteWAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteWAU8822(45, 0x090);   /* LAUXIN connected, and its Gain value is 0dB */
    I2C_WriteWAU8822(46, 0x190);   /* RAUXIN connected, and its Gain value is 0dB */
    I2C_WriteWAU8822(47, 0x005);   /* LAUXIN connected, and its Gain value is 0dB */
    I2C_WriteWAU8822(48, 0x005);   /* RAUXIN connected, and its Gain value is 0dB */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    I2C_WriteWAU8822(52, 0x039);   /* HP Volume */
    I2C_WriteWAU8822(53, 0x139);   /* HP Volume */

    I2C_WriteWAU8822(54, 0x140);   /* LSPKOUT Volume */
    I2C_WriteWAU8822(55, 0x140);   /* RSPKOUT Volume */

    // Headphone output control
    PD->MODE = (PD->MODE & (~0x3ul << 7 * 2)) | (0x1ul << 7 * 2);
    PD7 = 0;
}

int WAU8822_SetSamplingRate(int srate)
{
	if (srate == 48000) {
    	I2C_WriteWAU8822(6,  0x14D);   /* Divide by 2, 48K */
    	I2C_WriteWAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    }
    else if (srate == 32000) {
    	I2C_WriteWAU8822(6,  0x16D);   /* Divide by 3, 32K */
    	I2C_WriteWAU8822(7,  0x002);   /* 32K for internal filter coefficients */
    }
    else if (srate == 16000) {
    	I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    	I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    }
    else if (srate == 8000) {
    	I2C_WriteWAU8822(6,  0x1ED);   /* Divide by 12, 8K */
    	I2C_WriteWAU8822(7,  0x00A);   /* 8K for internal filter coefficients */
    }
    else {
    	printf("Not pre-defined sampling rate %d!\n", srate);
    	return -1;
	}
	return 0;
}

void SPI1_IRQHandler(void)
{
    if (SPI1->I2SSTS & SPI_I2SSTS_TXTHIF_Msk)
    {
    	if (g_u8PlayEn)
        	I2S_WRITE_TX_FIFO(SPI1, inpw(&g_u8UacInBuf[g_UacInBuff_8822PlayPos]));
        else
        	I2S_WRITE_TX_FIFO(SPI1, 0);    /* send mute PCM data to NAU8822 */

        if (!g_bMicIsMono) {        /* MIC in mono, just duplicate channel to be stereo */
        	g_UacInBuff_8822PlayPos = (g_UacInBuff_8822PlayPos+4) % UAC_IN_BUF_LEN;
        }
                
    	if (g_u8PlayEn)
        	I2S_WRITE_TX_FIFO(SPI1, inpw(&g_u8UacInBuf[g_UacInBuff_8822PlayPos]));
        else
        	I2S_WRITE_TX_FIFO(SPI1, 0);    /* send mute PCM data to NAU8822 */

        g_UacInBuff_8822PlayPos = (g_UacInBuff_8822PlayPos+4) % UAC_IN_BUF_LEN;

        g_UacInBuff_8822PlayCnt += 8;
    }

    if (SPI1->I2SSTS & SPI_I2SSTS_RXTHIF_Msk)
    {
    	*(uint32_t *)(&g_u8UacOutBuf[g_UacOutBuff_8822RecPos]) = I2S_READ_RX_FIFO(SPI1);
        g_UacOutBuff_8822RecPos = (g_UacOutBuff_8822RecPos+4) % UAC_OUT_BUF_LEN;
        
        *(uint32_t *)(&g_u8UacOutBuf[g_UacOutBuff_8822RecPos]) = I2S_READ_RX_FIFO(SPI1);
        g_UacOutBuff_8822RecPos = (g_UacOutBuff_8822RecPos+4) % UAC_OUT_BUF_LEN;
        
        g_UacOutBuff_8822RecCnt += 8;
        
    	/* 
     	 *  If UAC play is not activated, enable it once NAU8822 audio in record data 
     	 *  accumulated over a half of g_u8UacOutBuf.
     	 */
    	if ((g_u8RecEn == 0) && (g_UacOutBuff_8822RecPos >= UAC_OUT_BUF_LEN/2))
    	{
       		g_UacOutBuff_UacPlayPos = g_UacOutBuff_UacPlayCnt = 0;
       		g_u8RecEn = 1;
    	}
    }
}

void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    static uint16_t tb[3][3] = {{0x00C, 0x093, 0x0E9}, // 8.192
        {0x00E, 0x1D2, 0x1E3},  // * 1.005 = 8.233
        {0x009, 0x153, 0x1EF}
    }; // * .995 = 8.151
    static RESAMPLE_STATE_T current = E_RS_NONE;
    int i, s;

    if(r == current)
        return;
    else
        current = r;
    switch(r)
    {
        case E_RS_UP:
            s = 1;
            break;
        case E_RS_DOWN:
            s = 2;
            break;
        case E_RS_NONE:
        default:
            s = 0;
    }

    for(i = 0; i < 3; i++)
        I2C_WriteWAU8822(37 + i, tb[s][i]);
}


#if 0
void AdjFreq(void)
{
    uint32_t u32Size;
    static int32_t i32PreFlag = 0;
    static int32_t i32Cnt = 0;

    /* Only adjust the frequency when play data */
    if(g_u8PlayEn == 0)
        return;

    /* Get sample size in play buffer */
    u32Size = GetSamplesInBuf();


    if(g_i32AdjFlag == 0)
    {
        /* Check if we need to adjust the frequency when we didn't in adjusting state */
        if(u32Size > (BUF_LEN * 3 / 4))
        {
            /* USB rate > I2S rate. So we increase I2S rate here */
            AdjustCodecPll(E_RS_UP);
            g_i32AdjFlag = -1;
        }
        else if(u32Size < (BUF_LEN * 1 / 4))
        {
            /* USB rate < I2S rate. So we decrease I2S rate here */
            AdjustCodecPll(E_RS_DOWN);
            g_i32AdjFlag = 1;
        }
    }
    else
    {
        /* Check if we need to stop adjust the frequency when we are in adjusting state */
        if((g_i32AdjFlag > 0) && (u32Size > BUF_LEN / 2))
        {
            AdjustCodecPll(E_RS_NONE);
            g_i32AdjFlag = 0;
        }

        if((g_i32AdjFlag < 0) && (u32Size < BUF_LEN / 2))
        {
            AdjustCodecPll(E_RS_NONE);
            g_i32AdjFlag = 0;
        }
    }

    /* Show adjustment, buffer, volume status */
    if((i32PreFlag != g_i32AdjFlag) || (i32Cnt++ > 40000))
    {
        i32PreFlag = g_i32AdjFlag;
        printf("%d %d %d %d\n", g_i32AdjFlag, u32Size, g_usbd_PlayVolumeL, g_usbd_RecVolumeL);
        i32Cnt = 0;
    }

}
#endif

#if 0
void VolumnControl(void)
{
    static uint8_t u8PrePlayMute = 0;
    static uint8_t u8PreRecMute = 0;
    static int16_t i16PrePlayVolumeL = 0;
    static int16_t i16PrePlayVolumeR = 0;
    static int16_t i16PreRecVolumeL = 0;
    static int16_t i16PreRecVolumeR = 0;
    uint8_t IsChange = 0;
    uint32_t u32R52, u32R53;
    uint32_t u32R15, u32R16;

    /*
        g_usbd_PlayMute is used for MUTE control. 0 = not MUTE. 1 = MUTE.
        g_usbd_PlayVolumeL is volume of left channel. Range is -32768 ~ 32767
        g_usbd_PlayVolumeR is volume of right channel. Range is -32768 ~ 32767

        NAU8822 LHPGAIN (R52) = -57dB ~ +6dB. Code is 0x0 ~ 0x3F.
        NAU8822 RHPGAIN (R53) = -57dB ~ +6dB. Code is 0x0 ~ 0x3F.
        Play volume mapping to code will be (Volume >> 10)+32

        NAU8822 LADCGAIN (R15) = MUTE, -127dB ~ 0dB. Code is 0x0, 0x1 ~ 0xFF.
        NAU8822 RADCGAIN (R16) = MUTE, -127dB ~ 0dB. Code is 0x0, 0x1 ~ 0xFF.
        Record volume mapping to code will be (Volume >> 8)+128
    */

    u32R52 = 0;
    u32R53 = 0;

    /* Update MUTE and volume to u32R52/53 when MUTE changed for play */
    if(u8PrePlayMute != g_usbd_PlayMute)
    {
        u8PrePlayMute = g_usbd_PlayMute;
        u32R52 = u32R52 | (g_usbd_PlayMute << 6);
        u32R53 = u32R53 | (g_usbd_PlayMute << 6);
        i16PrePlayVolumeL = g_usbd_PlayVolumeL;
        u32R52 |= (g_usbd_PlayVolumeL >> 10) + 32;
        i16PrePlayVolumeR = g_usbd_PlayVolumeR;
        u32R53 |= (g_usbd_PlayVolumeL >> 10) + 32;
        IsChange |= 3;
    }

    /* Update left volume to u32R52 when left volume changed for play */
    if(i16PrePlayVolumeL != g_usbd_PlayVolumeL)
    {
        i16PrePlayVolumeL = g_usbd_PlayVolumeL;
        u32R52 |= (g_usbd_PlayVolumeL >> 10) + 32;
        IsChange |= 1;
    }

    /* Update right volume to u32R53 when left volume changed for play */
    if(i16PrePlayVolumeR != g_usbd_PlayVolumeR)
    {
        i16PrePlayVolumeR = g_usbd_PlayVolumeR;
        u32R53 |= (g_usbd_PlayVolumeR >> 10) + 32;
        IsChange |= 2;
    }

    u32R15 = 0;
    u32R16 = 0;

    /* Update MUTE and volume to u32R15/16 when MUTE changed for record */
    if(u8PreRecMute != g_usbd_RecMute)
    {
        u8PreRecMute = g_usbd_RecMute;

        if(!g_usbd_RecMute)
        {
            i16PreRecVolumeL = g_usbd_RecVolumeL;
            i16PreRecVolumeR = g_usbd_RecVolumeR;
            u32R15 |= (g_usbd_RecVolumeL >> 8) + 128;
            u32R16 |= (g_usbd_RecVolumeR >> 8) + 128;
        }

        IsChange |= 0xc;
    }

    /* Update left volume to u32R15 when left volume changed for record */
    if(i16PreRecVolumeL != g_usbd_RecVolumeL)
    {
        i16PreRecVolumeL = g_usbd_RecVolumeL;
        u32R15 |= (g_usbd_RecVolumeL >> 8) + 128;
        IsChange |= 4;
    }

    /* Update right volume to u32R16 when left volume changed for record */
    if(i16PreRecVolumeR != g_usbd_RecVolumeR)
    {
        i16PreRecVolumeR = g_usbd_RecVolumeR;
        u32R16 |= (g_usbd_RecVolumeR >> 8) + 128;
        IsChange |= 8;
    }

    /* Update R52, R53 when MUTE or volume changed */
    if((IsChange & 3) == 3)
    {
        /* Both channels need to be changed */
        I2C_WriteWAU8822(52, u32R52);
        I2C_WriteWAU8822(53, u32R53 | 0x100);
        IsChange ^= 3;
    }
    else if(IsChange & 1)
    {
        /* Only change left channel */
        I2C_WriteWAU8822(52, u32R52 | 0x100);
        IsChange ^= 1;
    }
    else if(IsChange & 2)
    {
        /* Only change right channel */
        I2C_WriteWAU8822(53, u32R53 | 0x100);
        IsChange ^= 2;
    }

    /* Update R15, R16 when MUTE or volume changed */
    if((IsChange & 0xc) == 0xc)
    {
        /* Both channels need to be changed */
        I2C_WriteWAU8822(15, u32R15);
        I2C_WriteWAU8822(16, u32R16 | 0x100);
        IsChange ^= 0xc;
    }
    else if(IsChange & 4)
    {
        /* Only change left channel */
        I2C_WriteWAU8822(15, u32R15 | 0x100);
        IsChange ^= 4;
    }
    else if(IsChange & 8)
    {
        /* Only change right channel */
        I2C_WriteWAU8822(16, u32R16 | 0x100);
        IsChange ^= 8;
    }

}
#endif


/**
 *  @brief  USB UAC audio-in data callback function.
 *          UAC driver deleivers an audio in data packet received from UAC device.
 *  @param[in] dev    Audio Class device
 *  @param[in] data   Audio in packet buffer
 *  @param[in] len    Length of audio in packet
 *  @return   UAC driver does not check this return value.
 */
int audio_in_callback(UAC_DEV_T *dev, uint8_t *data, int len)
{
	int   cp_len;
	
	if (g_UacInBuff_UacRecPos + len >= UAC_IN_BUF_LEN) {
		cp_len = UAC_IN_BUF_LEN - g_UacInBuff_UacRecPos;
	}
	else {
		cp_len = len;
	}
	
	memcpy(&g_u8UacInBuf[g_UacInBuff_UacRecPos], data, cp_len);
	
	g_UacInBuff_UacRecPos = (g_UacInBuff_UacRecPos + cp_len) % UAC_IN_BUF_LEN;
	g_UacInBuff_UacRecCnt += cp_len;
	len -= cp_len;
	
	if (len) {
    	memcpy(&g_u8UacInBuf[0], &data[cp_len], len);
    	g_UacInBuff_UacRecPos = len;
    	g_UacInBuff_UacRecCnt += len;
    }

    /* 
     *  If NAU8822 playback is not activated, enable it once audio in data accumulated over
     *  a half of g_u8UacInBuf.
     */
    if ((g_u8PlayEn == 0) && (g_UacInBuff_UacRecPos >= UAC_IN_BUF_LEN/2))
    {
       	/* Fill 0x0 to buffer before playing for buffer operation smooth */
    	NVIC_DisableIRQ(SPI1_IRQn);
       	g_UacInBuff_8822PlayPos = g_UacInBuff_8822PlayCnt = 0;
       	g_u8PlayEn = 1;
    	NVIC_EnableIRQ(SPI1_IRQn);
    }

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
	int  cp_len;
	
	if (g_u8RecEn == 0)    /* do not send data until NAU8822 record enough data */
		return 0;

	if (UAC_OUT_BUF_LEN - g_UacOutBuff_UacPlayPos < 192) {
		cp_len = UAC_OUT_BUF_LEN - g_UacOutBuff_UacPlayPos;
	}
	else {
	    cp_len = 192;   
	}
	
	memcpy(data, &g_u8UacOutBuf[g_UacOutBuff_UacPlayPos], cp_len);
	g_UacOutBuff_UacPlayPos = (g_UacOutBuff_UacPlayPos + cp_len) % UAC_OUT_BUF_LEN;

    if (cp_len < 192)	
    {
    	memcpy(&data[cp_len], &g_u8UacOutBuf[0], 192-cp_len);
    	g_UacOutBuff_UacPlayPos = 192-cp_len;
	}
	g_UacOutBuff_UacPlayCnt += 192;

    return 192;   // for 48000 stero Hz
}


