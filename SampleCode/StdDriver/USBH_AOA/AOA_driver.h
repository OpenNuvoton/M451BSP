/******************************************************************************
 * @file     AOA_driver.h
 * @brief    M451 series USB Host AOA(Android Open Accessory) driver header file.
 * @version  0.10
 * $Date: 16/04/11 11:20a $
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBH_AOA_H_
#define __USBH_AOA_H_

#include "usbh_core.h"


/* Define the AOA vendor id and product id */
#define VID_GOOGLE                     	0x18D1
#define PID_AOAv1                      	0x2D00
#define PID_AOAv1_ADB                  	0x2D01
#define PID_AOAv2_AUDIO                	0x2D02
#define PID_AOAv2_AUDIO_ADB            	0x2D03
#define PID_AOAv2_AOA_AUDIO            	0x2D04
#define PID_AOAv2_AOA_AUDIO_ADB        	0x2D05

/*!<Define AOA Control Request */
#define AOA_REQ_GET_PROTOCOL            51
#define AOA_REQ_SEND_STRING             52
#define AOA_REQ_START                	53

#define AOA_MAX_STR_LEN                 64

/* The following AOA_STR_XXX string length must be <= AOA_MAX_STR_LEN */
#define AOA_STR0_MANUFACTURER 			"dlodlo Inc."
#define AOA_STR1_MODEL        			"VR_Model"
#define AOA_STR2_DESCRIPTION  			"dlodlo Android Accessory device"
#define AOA_STR3_VERSION      			"2.0"
#define AOA_STR4_URI          			"http://www.dlodlo.com"
#define AOA_STR5_SERIAL_NUM     		"00000452"

#define AOA_BULK_IN_BUFF_SIZE           64

typedef void (AOA_READ_FUN)(uint8_t *data_buff, int data_len);

typedef struct aoa_dev_t {
    int          connected;
    int          is_aoa;
    USB_DEV_T    *dev;
    uint16_t     vid;
    uint16_t     pid;
    int          ifnum;
    EP_INFO_T    *bulk_in;
    EP_INFO_T    *bulk_out;
    URB_T        *urb_bulk_in;
    uint8_t      buff_in[AOA_BULK_IN_BUFF_SIZE];
    AOA_READ_FUN *func_read;
}  AOADEV_T;


/*-------------------------------------------------------------*/

void AOA_Init(AOA_READ_FUN *func);
int  AOA_IsConnected(void);
int  AOA_WriteData(uint8_t *buff, int len);


#endif  /* __USBH_AOA_H_ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
