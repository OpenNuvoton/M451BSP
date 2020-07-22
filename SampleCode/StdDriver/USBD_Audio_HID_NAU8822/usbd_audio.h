/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  1.0.0
 * @date     22, December, 2013
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

#include "M451Series.h"

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0xB007

#define HID_CONSUMER    0
#define HID_KEYBOARD    1
#define HID_FUNCTION    HID_CONSUMER

#define UAC_MICROPHONE  0
#define UAC_SPEAKER     1

/*!<Define Audio information */
#define PLAY_RATE       48000       /* The audo play sampling rate. It could be 8000, 16000, 32000, and 48000 */
#define PLAY_CHANNELS   2           /* Number of channels. Don't Change */

#define REC_RATE        PLAY_RATE   /* The record sampling rate. Must be the same with PLAY_RATE */
#define REC_CHANNELS    2           /* Number of channels. Don't Change */

/* USB Sync Method */
#define BY_SYNC_FRAME   1   /* 1: Sync by USB Sync frame. 0: Sync by adjust PLL frequency */

#define REC_FEATURE_UNITID      0x05
#define PLAY_FEATURE_UNITID     0x06

#define BUF_LEN     32*12
#define REC_LEN     REC_RATE / 1000
#define PLAY_LEN_FRAME	PLAY_RATE/1000
#define REC_BUFF_LEN	96*2

#define REFRESH         4       /* bRefresh rate = 2^n (0x04 ==> 16ms)  */
#define REFRESH_RATE    (2^REFRESH)
#define SYNC_RATE       (PLAY_LEN_FRAME * REFRESH_RATE)

/* Define Descriptor information */
#if(PLAY_CHANNELS == 1)
#define PLAY_CH_CFG     1
#endif
#if(PLAY_CHANNELS == 2)
#define PLAY_CH_CFG     3
#endif

#if(REC_CHANNELS == 1)
#define REC_CH_CFG     1
#endif
#if(REC_CHANNELS == 2)
#define REC_CH_CFG     3
#endif

#define PLAY_RATE_LO    (PLAY_RATE & 0xFF)
#define PLAY_RATE_MD    ((PLAY_RATE >> 8) & 0xFF)
#define PLAY_RATE_HI    ((PLAY_RATE >> 16) & 0xFF)

#define REC_RATE_LO     (REC_RATE & 0xFF)
#define REC_RATE_MD     ((REC_RATE >> 8) & 0xFF)
#define REC_RATE_HI     ((REC_RATE >> 16) & 0xFF)

/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
#define UAC_STOP_AUDIO_RECORD           0
#define UAC_START_AUDIO_RECORD          1
#define UAC_PROCESSING_AUDIO_RECORD     2
#define UAC_BUSY_AUDIO_RECORD           3

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF
/*!<Define HID Class Specific Request */
#define HID_GET_REPORT              0x01
#define HID_GET_IDLE                0x02
#define HID_GET_PROTOCOL            0x03
#define HID_SET_REPORT              0x09
#define HID_SET_IDLE                0x0A
#define HID_SET_PROTOCOL            0x0B


#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    8
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    256
#define EP3_MAX_PKT_SIZE    PLAY_RATE*PLAY_CHANNELS*2/1000+24  //8+16
#define EP4_MAX_PKT_SIZE    32
#define EP5_MAX_PKT_SIZE	3
#define EP6_MAX_PKT_SIZE	3

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         EP5_MAX_PKT_SIZE

#define EP6_BUF_BASE        (EP5_BUF_BASE + 8)
#define EP6_BUF_LEN         EP6_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM    0x02
#define ISO_OUT_EP_NUM   0x03
#define HID_IN_EP_NUM    0x04
#define ISO_IN_FB_EP_NUM	0x05
#define ISO_OUT_FB_EP_NUM	0x06

#if(HID_FUNCTION == HID_CONSUMER)

#define HID_CTRL_MUTE        0x01
#define HID_CTRL_VOLUME_INC  0x02
#define HID_CTRL_VOLUME_DEC  0x04

#define HID_CTRL_EJECT       0x08
#define HID_CTRL_PLAY        0x01
#define HID_CTRL_STOP        0x02
#define HID_CTRL_PAUSE       0x04
#define HID_CTRL_NEXT        0x08
#define HID_CTRL_PREVIOUS    0x10
#define HID_CTRL_RECORD      0x20
#define HID_CTRL_REWIND      0x40
#define HID_CTRL_FF          0x80
#endif

/*-------------------------------------------------------------*/
extern volatile uint32_t g_usbd_UsbAudioState;

void UAC_DeviceEnable(uint8_t u8Object);
void UAC_DeviceDisable(uint8_t u8Object);
void UAC_SendRecData(void);
void UAC_GetPlayData(int16_t *pi16src, int16_t i16Samples);


/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(void);

void EP2_Handler(void);
void EP3_Handler(void);
void EP4_Handler(void);
void EP5_Handler(void);
void EP6_Handler(void);

void WAU8822_Setup(void);
void timer_init(void);
void AdjFreq(void);
void VolumnControl(void);
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data);

#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
