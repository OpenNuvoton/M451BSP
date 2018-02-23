/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB audio header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBH_UAC_NAU8822_H__
#define __USBH_UAC_NAU8822_H__

#include "M451Series.h"
#include "usbh_core.h"
#include "usbh_uac.h"


#define ENABLE_UAC_PLAY
#define ENABLE_UAC_REC


#define UAC_IN_BUF_LEN         (192*12)     /* suggest 1K at least */
#define UAC_OUT_BUF_LEN        (192*12)     /* suggest 1K at least */

#define PLAY_RATE              48000

/*-------------------------------------------------------------*/


void WAU8822_Setup(void);
void timer_init(void);
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data);
int WAU8822_SetSamplingRate(int srate);
extern void ResetAudioBuffer(void);

extern int audio_in_callback(UAC_DEV_T *dev, uint8_t *data, int len);
extern int audio_out_callback(UAC_DEV_T *dev, uint8_t *data, int len);

extern volatile uint32_t g_UacOutBuff_8822RecPos;   /* NAU8822 record pointer          */
extern volatile uint32_t g_UacOutBuff_UacPlayPos;   /* UAC play data pointer           */
extern volatile uint32_t g_UacOutBuff_8822RecCnt;   /* NAU8822 record pcm counter      */
extern volatile uint32_t g_UacOutBuff_UacPlayCnt;   /* UAC play pcm counter            */

extern volatile uint32_t g_UacInBuff_8822PlayPos;   /* NAU8822 play pointer            */
extern volatile uint32_t g_UacInBuff_UacRecPos;     /* UAC data recieving pointer      */
extern volatile uint32_t g_UacInBuff_8822PlayCnt;   /* NAU8822 play pointer            */
extern volatile uint32_t g_UacInBuff_UacRecCnt;     /* UAC data recieving pointer      */

extern volatile int8_t   g_bMicIsMono;              /* Is the currently connected UAC MIC is mono? */
extern volatile uint8_t  g_u8RecEn;
extern volatile uint8_t  g_u8PlayEn;


#endif  /* __USBH_UAC_NAU8822_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
