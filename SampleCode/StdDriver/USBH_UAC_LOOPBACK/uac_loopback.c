/******************************************************************************
 * @file     usbd_audio.c
 * @brief    M451 series USBD audio sample file
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include <stdio.h>

#include "M451Series.h"
#include "usbh_lib.h"
#include "usbh_uac.h"


#define PCM_BUF_LEN            (192*24)     /* suggest 1K at least */


/* Global variables  */
volatile uint8_t g_u8RecEn = 0;
volatile uint8_t g_u8PlayEn = 0;

volatile int8_t  g_i8MicIsMono = 0;

/* UAC audio in/out PCM buffer.  */
#ifdef __ICCARM__
#pragma data_alignment=32
uint8_t g_u8PcmBuf[PCM_BUF_LEN];
#else
uint8_t g_u8PcmBuf[PCM_BUF_LEN] __attribute__((aligned(4)));
#endif
volatile uint32_t g_u32UacRecPos = 0;       /* UAC record pointer of PCM buffer       */
volatile uint32_t g_u32UacPlayPos = 0;      /* UAC playback pointer of PCM buffer     */
volatile uint32_t g_u32UacRecCnt = 0;       /* Counter of UAC record data             */
volatile uint32_t g_u32UacPlayCnt = 0;      /* Counter UAC playback data              */


void ResetAudioLoopBack(void)
{
    memset(g_u8PcmBuf, 0, sizeof(g_u8PcmBuf));
    g_u32UacRecPos = 0;
    g_u32UacPlayPos = 0;
    g_u32UacRecCnt = 0;
    g_u32UacPlayCnt = 0;
    g_u8RecEn = 0;
    g_u8PlayEn = 0;
}


/**
 *  @brief  USB UAC audio-in data callback function.
 *          UAC driver deleivers an audio in data packet received from UAC device.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Audio in packet buffer
 *  @param[in] i8Len    Length of audio in packet
 *  @return   UAC driver does not check this return value.
 */
int audio_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    int        i8Cnt, i8CpLen;
    uint16_t   *pu16Dptr, *pu16Bptr;

    if(g_i8MicIsMono)
    {
        if(g_u32UacRecPos + i8Len * 2 >= PCM_BUF_LEN)
        {
            i8CpLen = (PCM_BUF_LEN - g_u32UacRecPos) / 2;
        }
        else
        {
            i8CpLen = i8Len;
        }

        pu16Dptr = (uint16_t *)pu8Data;
        pu16Bptr = (uint16_t *)&g_u8PcmBuf[g_u32UacRecPos];
        for(i8Cnt = 0; i8Cnt < i8CpLen; i8Cnt += 2)
        {
            *pu16Bptr++ = *pu16Dptr;                /* 16-bit PCM data                            */
            *pu16Bptr++ = *pu16Dptr++;              /* duplicate PCM data                         */
        }

        g_u32UacRecPos = (g_u32UacRecPos + i8CpLen * 2) % PCM_BUF_LEN;
        g_u32UacRecCnt += i8CpLen;
        i8Len -= i8CpLen;

        if(i8Len)
        {
            pu16Dptr = (uint16_t *)&pu8Data[i8CpLen];
            pu16Bptr = (uint16_t *)g_u8PcmBuf;
            for(i8Cnt = 0; i8Cnt < i8Len; i8Cnt += 2)
            {
                *pu16Bptr++ = *pu16Dptr;            /* 16-bit PCM data                            */
                *pu16Bptr++ = *pu16Dptr++;          /* duplicate PCM data                         */
            }
            g_u32UacRecPos = i8Len * 2;
            g_u32UacRecCnt += i8Len;
        }
    }
    else
    {
        if(g_u32UacRecPos + i8Len >= PCM_BUF_LEN)
        {
            i8CpLen = PCM_BUF_LEN - g_u32UacRecPos;
        }
        else
        {
            i8CpLen = i8Len;
        }
        memcpy(&g_u8PcmBuf[g_u32UacRecPos], pu8Data, i8CpLen);

        g_u32UacRecPos = (g_u32UacRecPos + i8CpLen) % PCM_BUF_LEN;
        g_u32UacRecCnt += i8CpLen;
        i8Len -= i8CpLen;

        if(i8Len)
        {
            memcpy(&g_u8PcmBuf[0], &pu8Data[i8CpLen], i8Len);
            g_u32UacRecPos = i8Len;
            g_u32UacRecCnt += i8Len;
        }
    }

    if((g_u8PlayEn == 0) && (g_u32UacRecPos >= PCM_BUF_LEN / 2))
    {
        g_u32UacPlayPos = g_u32UacPlayCnt = 0;
        g_u8PlayEn = 1;
    }

    return 0;
}


/**
 *  @brief  Audio-out data callback function.
 *          UAC driver requests user to move audio-out data into the specified address. The audio-out
 *          data will then be send to UAC device via isochronous-out pipe.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Application should move audio-out data into this buffer.
 *  @param[in] i8Len    Maximum length of audio-out data can be moved.
 *  @return   Actual length of audio data moved.
 */
int audio_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    int  i8CpLen;

    if(!g_u8PlayEn)
    {
        memset(pu8Data, 0, 192);
        g_u32UacPlayCnt += 192;
        return 192;
    }

    if(PCM_BUF_LEN - g_u32UacPlayPos < 192)
    {
        i8CpLen = PCM_BUF_LEN - g_u32UacPlayPos;
    }
    else
    {
        i8CpLen = 192;
    }

    memcpy(pu8Data, &g_u8PcmBuf[g_u32UacPlayPos], i8CpLen);
    g_u32UacPlayPos = (g_u32UacPlayPos + i8CpLen) % PCM_BUF_LEN;

    if(i8CpLen < 192)
    {
        memcpy(&pu8Data[i8CpLen], &g_u8PcmBuf[0], 192 - i8CpLen);
        g_u32UacPlayPos = 192 - i8CpLen;
    }
    g_u32UacPlayCnt += 192;

    return 192;   // for 48000 stero Hz
}


