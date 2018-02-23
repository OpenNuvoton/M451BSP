/**************************************************************************//**
 * @file     AOA_driver.c
 * @version  V0.10
 * $Revision: 1 $
 * $Date: 16/04/11 11:20a $
 * @brief    M451 series MCU USB Host AOA(Android Open Accessory) driver.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "M451Series.h"
#include "usbh_core.h"
#include "AOA_driver.h"

#define AOA_DBGMSG        printf

static AOADEV_T  g_aoa_dev;

#ifdef __ICCARM__
#pragma data_alignment=16
static uint8_t aoa_buff[AOA_MAX_STR_LEN];
#pragma data_alignment=4
#else
static __align(16) uint8_t aoa_buff[AOA_MAX_STR_LEN];
#endif

static volatile int  bulk_out_done;


static void  bulk_in_irq(URB_T *urb)
{
	USB_DEV_T 	*dev = urb->dev;
	int  		pipe;
	
    if (!g_aoa_dev.connected)
    	return;
    	
    if (g_aoa_dev.func_read != NULL)
    	g_aoa_dev.func_read(urb->transfer_buffer, urb->actual_length);
    	
    /*------------------------------------------*/
    /*  Re-submit bulk in transfer              */
    /*------------------------------------------*/
    pipe = usb_rcvbulkpipe(dev, g_aoa_dev.bulk_in->bEndpointAddress);

    FILL_BULK_URB(urb, dev, pipe, g_aoa_dev.buff_in, AOA_BULK_IN_BUFF_SIZE, bulk_in_irq, NULL);

    if (USBH_SubmitUrb(urb) < 0)
	{
		AOA_DBGMSG("Failed to submit Bulk in transfer!\n");
    }
}


/*
 *  If a USB device is connected, USB core library will enable it and read descriptors
 *  from it. If the USB device vendor ID and product ID matches with LBK device,
 *  aoa_probe will be called.
 */
static int  aoa_probe(USB_DEV_T *dev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id)
{
    int         ifnum;
    int         i;
    int         pipe;
    URB_T       *urb;

    AOA_DBGMSG("aoa_probe - VID=0x%x, PID=0x%x\n", dev->descriptor.idVendor, dev->descriptor.idProduct);

    if (g_aoa_dev.connected && g_aoa_dev.is_aoa)
    {
        AOA_DBGMSG("There's an AOA device connected. The other AOA device will be ignored.\n");
        return -1;
    }
    
    g_aoa_dev.dev = dev;
    g_aoa_dev.vid = dev->descriptor.idVendor;
    g_aoa_dev.pid = dev->descriptor.idProduct;
    
    g_aoa_dev.connected = 1;
    
    if ((g_aoa_dev.vid != VID_GOOGLE) ||
        ((g_aoa_dev.pid != PID_AOAv1) && (g_aoa_dev.pid != PID_AOAv1_ADB) &&
         (g_aoa_dev.pid != PID_AOAv2_AUDIO) && (g_aoa_dev.pid != PID_AOAv2_AUDIO_ADB) &&
         (g_aoa_dev.pid != PID_AOAv2_AOA_AUDIO) && (g_aoa_dev.pid != PID_AOAv2_AOA_AUDIO_ADB)))
    {
    	// Not an AOA device or not in AOA mode.
    	g_aoa_dev.is_aoa = 0;
    	return 0;
	}

	g_aoa_dev.is_aoa = 1;

    ifnum = ifd->bInterfaceNumber;
    g_aoa_dev.ifnum = ifnum;

    for (i = 0; i < dev->ep_list_cnt; i++) 
    {
        if (dev->ep_list[i].ifnum != ifnum)
            continue;

        if ((dev->ep_list[i].bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK)
        {
           	if (dev->ep_list[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK)
               	g_aoa_dev.bulk_in = &dev->ep_list[i];
           	else
               	g_aoa_dev.bulk_out = &dev->ep_list[i];
        }
    }

    if (!g_aoa_dev.bulk_in || !g_aoa_dev.bulk_out) 
    {
        AOA_DBGMSG("Expected bulk endpoints not found!\n");
        g_aoa_dev.connected = 0;
        return -1;
    }

	/*----------------------------------------*/
	/*  Start bulk in transfer                */
	/*----------------------------------------*/

	AOA_DBGMSG("Start AOA bulk in transfer.\n");
	
    urb = USBH_AllocUrb();
    if (urb == NULL) 
    {
        AOA_DBGMSG("No free URB!\n");
        g_aoa_dev.connected = 0;
        return -1;
    }
    g_aoa_dev.urb_bulk_in = urb;

    pipe = usb_rcvbulkpipe(dev, g_aoa_dev.bulk_in->bEndpointAddress);

    FILL_BULK_URB(urb, dev, pipe, g_aoa_dev.buff_in, AOA_BULK_IN_BUFF_SIZE, bulk_in_irq, NULL);

    if (USBH_SubmitUrb(urb) < 0)
	{
		AOA_DBGMSG("Failed to submit Bulk in transfer!\n");
		g_aoa_dev.connected = 0;
        return -1;
    }

    return 0;
}


static void  aoa_disconnect(USB_DEV_T *dev)
{
    if (g_aoa_dev.urb_bulk_in) 
    {
        USBH_UnlinkUrb(g_aoa_dev.urb_bulk_in);
        USBH_FreeUrb(g_aoa_dev.urb_bulk_in);
        g_aoa_dev.urb_bulk_in = NULL;
    }

    g_aoa_dev.connected = 0;
    AOA_DBGMSG("AOA device disconnected!\n");
}


static const USB_DEV_ID_T  aoa_id_table[] = {
	{ 0, VID_GOOGLE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//	{ USB_DEVICE_ID_MATCH_PRODUCT, 0, PID_AOAv1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//	{ USB_DEVICE_ID_MATCH_PRODUCT, 0, PID_AOAv1_ADB, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//	{ USB_DEVICE_ID_MATCH_PRODUCT, 0, PID_AOAv2_AUDIO, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//	{ USB_DEVICE_ID_MATCH_PRODUCT, 0, PID_AOAv2_AUDIO_ADB, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//	{ USB_DEVICE_ID_MATCH_PRODUCT, 0, PID_AOAv2_AOA_AUDIO, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
//	{ USB_DEVICE_ID_MATCH_PRODUCT, 0, PID_AOAv2_AOA_AUDIO_ADB, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};


static USB_DRIVER_T  aoa_driver = {
    "AOA driver",
    aoa_probe,
    aoa_disconnect,
    aoa_id_table,
    NULL,                       /* suspend */
    NULL,                       /* resume */
    {NULL,NULL}                 /* driver_list */
};



/******************************************************************************************/
/*                                                                                        */
/*   Android Open Accessory driver Export functions                                       */
/*                                                                                        */
/******************************************************************************************/

static void  bulk_out_irq(URB_T *urb)
{
    bulk_out_done = 1;
}


/**
 *  @brief  This function executes Bulk-Out transfer on AOA device.
 *  @param[in] buff       Buffer holds the user data to be send to AOA device.
 *  @param[in] len        Bytes counr of data to be transferred.
 *  @retval   0           Success
 *  @retval   Otherwise   Error (error code refer to usbh_err_code.h.)
 */
int AOA_WriteData(uint8_t *buff, int len)
{
    USB_DEV_T  *dev;
    EP_INFO_T  *ep_info;
    URB_T      *urb;
    int        pipe;
    int        timeout, ret;

	if (!g_aoa_dev.connected || !g_aoa_dev.is_aoa)
        return -1;

    dev = g_aoa_dev.dev;
    ep_info = g_aoa_dev.bulk_out;

    urb = USBH_AllocUrb();
    if (urb == NULL) {
        AOA_DBGMSG("No free URB!\n");
        return -1;
    }

    pipe = usb_sndbulkpipe(dev, ep_info->bEndpointAddress);

    FILL_BULK_URB(urb, dev, pipe, buff, len, bulk_out_irq, NULL);

    bulk_out_done = 0;

    ret = USBH_SubmitUrb(urb);
    if (ret < 0) {
        AOA_DBGMSG("Failed to submit Bulk out transfer!\n");
        goto err_out;
    }

    for (timeout = 0x1000000 ; (timeout > 0) && (bulk_out_done == 0) ; timeout--);

    if (timeout <= 0) {
        USBH_UnlinkUrb(urb);
        AOA_DBGMSG("Bulk out xfer time-out!\n");
        goto err_out;
    }

    USBH_FreeUrb(urb);
    return 0;

err_out:
    USBH_FreeUrb(urb);
    return -1;
}


/**
 *  @brief  Check if an AOA device is connected. If a device is connected and not presented as 
 *          an AOA device, this function will try to start it in AOA mode.
 *  @retval   0           AOA device not found.
 *  @retval   Otherwise   An AOA device is found and connected.
 */
int  AOA_IsConnected(void)
{
    USB_DEV_T   *dev;

	dev = g_aoa_dev.dev;
	
	if ((g_aoa_dev.connected) && (g_aoa_dev.is_aoa == 0))
	{
		/*
		 *  A device connected, but is not in AOA mode.
		 *  Attempt to start in accessory mode
		 */
		 
		/*
		 *  [1] Get Protocol
		 */
    	if (USBH_SendCtrlMsg(dev, usb_rcvctrlpipe(dev, 0),
                             AOA_REQ_GET_PROTOCOL, USB_DIR_IN | USB_TYPE_VENDOR,
                             0, 0, aoa_buff, 16, HZ * 10000) < 0)
        {
        	g_aoa_dev.connected = 0;
        	return 0;
        }
        
        if (aoa_buff[0] >= 1)
        {
        	AOA_DBGMSG("Get protocl code is 0x%x.\n", aoa_buff[0]);
    	}
    	else
    	{
    		/* Not support AOA */
        	g_aoa_dev.connected = 0;
        	return 0;
    	}

		/*
		 *  [2] Send identifying string information to the device
		 */
		strcpy((char *)aoa_buff, AOA_STR0_MANUFACTURER);
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_SEND_STRING, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 0, (void *)aoa_buff, strlen((char *)aoa_buff), HZ * 10000);
		
		strcpy((char *)aoa_buff, AOA_STR1_MODEL);
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_SEND_STRING, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 1, (void *)aoa_buff, strlen((char *)aoa_buff), HZ * 10000);

		strcpy((char *)aoa_buff, AOA_STR2_DESCRIPTION);
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_SEND_STRING, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 2, (void *)aoa_buff, strlen((char *)aoa_buff), HZ * 10000);

		strcpy((char *)aoa_buff, AOA_STR3_VERSION);
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_SEND_STRING, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 3, (void *)aoa_buff, strlen((char *)aoa_buff), HZ * 10000);

		strcpy((char *)aoa_buff, AOA_STR4_URI);
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_SEND_STRING, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 4, (void *)aoa_buff, strlen((char *)aoa_buff), HZ * 10000);

		strcpy((char *)aoa_buff, AOA_STR5_SERIAL_NUM);
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_SEND_STRING, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 5, (void *)aoa_buff, strlen((char *)aoa_buff), HZ * 10000);
                            
		/*
		 *  [3] Send request to start in accessory mode
		 */
    	USBH_SendCtrlMsg(dev, usb_sndctrlpipe(dev, 0),
                            AOA_REQ_START, USB_DIR_OUT | USB_TYPE_VENDOR,
                            0, 0, NULL, 0, HZ * 10000);
        
        g_aoa_dev.connected = 0;
        return 0;
	}
	
    return g_aoa_dev.connected;
}

/**
 *  @brief  Register AOA driver to USB Host core, and install AOA read callback function.
 *  @param[in] func       The AOA read callback function.
 */
void AOA_Init(AOA_READ_FUN *func)
{
    memset((uint8_t *)&g_aoa_dev, 0, sizeof(g_aoa_dev));
    g_aoa_dev.func_read = func;
    USBH_RegisterDriver(&aoa_driver);
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

