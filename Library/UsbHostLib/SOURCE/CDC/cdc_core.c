/**************************************************************************//**
 * @file     cdc_core.c
 * @version  V1.00
 * $Revision: 14 $
 * $Date: 18/01/30 11:46a $
 * @brief    M451 MCU USB Host CDC library core
 *
 * @note
 * Copyright (C) 2018~2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "M451Series.h"
#include "usbh_core.h"

#include "usbh_cdc.h"


/** @addtogroup Component_Library Component Library
  @{
*/

/** @addtogroup USB_Host_Driver USB Host Driver
  @{
*/

/** @addtogroup USBH_CDC_Driver USB Host CDC Driver
  @{
*/

/** @addtogroup USBH_CDC_EXPORTED_FUNCTIONS USB Host CDC Driver Exported Functions
  @{
*/


/// @cond HIDDEN_SYMBOLS

#define USB_TIMEOUT             0x800000


static int usb_snd_control_msg(CDC_DEV_T *cdev, int requesttype, int request,
                               int value, int index, char *bytes, int size, int timeout)
{
    USB_DEV_T       *udev = cdev->udev;

    return USBH_SendCtrlMsg(udev, usb_sndctrlpipe(udev, 0),
                            request, requesttype, value, index, bytes, size, timeout);
}

int usb_rcv_control_msg(CDC_DEV_T *cdev, int requesttype, int request,
                        int value, int index, char *bytes, int size, int timeout)
{
    USB_DEV_T       *udev = cdev->udev;

    return USBH_SendCtrlMsg(udev, usb_rcvctrlpipe(udev, 0),
                            request, requesttype, value, index, bytes, size, timeout);
}


static EP_INFO_T *cdc_get_ep_info(CDC_DEV_T *cdev, int xfer_type, int dir)
{
	USB_DEV_T       *udev = cdev->udev;
    int   i;

    for (i = 0; i < udev->ep_list_cnt; i++) 
    {
        if ((udev->ep_list[i].ifnum != cdev->ifnum_cdc) && (udev->ep_list[i].ifnum != cdev->ifnum_data))
            continue;
                
        if ((udev->ep_list[i].bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != xfer_type)
            continue;
            
        if ((udev->ep_list[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK) == dir)
            return &udev->ep_list[i];
    }
    return NULL;
}

/// @endcond


/**
 *  @brief  GET_LINE_CODING  request
 *  @param[in]  cdev         CDC device
 *  @param[out] line_code    The currently configured line coding.
 *
 * @return   Success or not.
 * @retval   0           Success
 * @retval   Otherwise   Failed
 */
int32_t  USBH_CDC_GetLineCoding(CDC_DEV_T *cdev, LINE_CODING_T *line_code)
{
    int  len;
    
    len = usb_rcv_control_msg(cdev,
                              USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                              CDC_GET_LINE_CODING,
                              0, 
                              cdev->ifnum_cdc,
                              (char*)line_code, 7,
                              CDC_CMD_TIMEOUT);

    if (len != 7) {
        CDC_DBGMSG("GET_LINE_CODIN command failed. %d\n", len);
        return len;
    }
    return len;
}


/**
 *  @brief  SET_LINE_CODING  request
 *  @param[in]  cdev         CDC device
 *  @param[in]  line_code    The line coding configuration to be set.
 *
 * @return   Success or not.
 * @retval   0           Success
 * @retval   Otherwise   Failed
 */
int32_t  USBH_CDC_SetLineCoding(CDC_DEV_T *cdev, LINE_CODING_T *line_code)
{
    int  len;
    
    if ((line_code->stop_bits != 0) && (line_code->stop_bits != 1) &&
        (line_code->stop_bits != 2))
        return USB_ERR_INVAL;

    if (line_code->parity > 4)
        return USB_ERR_INVAL;

    if ((line_code->data_bits != 5) && (line_code->data_bits != 6) &&
        (line_code->data_bits != 7) && (line_code->data_bits != 8) &&
        (line_code->data_bits != 16))
        return USB_ERR_INVAL;

    len = usb_snd_control_msg(cdev,
                              USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                              CDC_SET_LINE_CODING,
                              0, 
                              cdev->ifnum_cdc,
                              (char*)line_code, 7,
                              CDC_CMD_TIMEOUT);

    if (len != 7) {
        CDC_DBGMSG("SET_LINE_CODIN command failed. %d\n", len);
        return len;
    }
    return len;
}

/**
 *  @brief  SET_LINE_CODING  request
 *  @param[in]  cdev             CDC device
 *  @param[in]  active_carrier   Carrier control for half duplex modems is activated or not.
 *  @param[in]  DTE_present      Indicates to DCE if DTE is present or not.
 *
 * @return   Success or not.
 * @retval   0           Success
 * @retval   Otherwise   Failed
 */
int32_t  USBH_CDC_SetControlLineState(CDC_DEV_T *cdev, int active_carrier, int DTE_present)
{
    int  len;
    uint16_t   ctrl_bitmap = 0;
    
    if (active_carrier)
        ctrl_bitmap |= 0x02;

    if (DTE_present)
        ctrl_bitmap |= 0x01;
    
    len = usb_snd_control_msg(cdev,
                              USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                              CDC_SET_CONTROL_LINE_STATE,
                              ctrl_bitmap, 
                              cdev->ifnum_cdc,
                              NULL, 0,
                              CDC_CMD_TIMEOUT);

    if (len != 0) {
        CDC_DBGMSG("SET_CONTROL_LINE_STATE command failed. %d\n", len);
        return len;
    }
    return len;
}

/// @cond HIDDEN_SYMBOLS
/*
 * CDC INT-in complete function
 */
static void  cdc_int_in_irq(URB_T *urb)
{
    CDC_DEV_T   *cdev;

    //CDC_DBGMSG("cdc_int_in_irq. %d\n", urb->actual_length);

    cdev = (CDC_DEV_T *)urb->context;

    if (urb->status) {
        CDC_DBGMSG("cdc_int_in_irq - has error: 0x%x\n", urb->status);
        return;
    }

    if (cdev->sts_func && urb->actual_length)
        cdev->sts_func(cdev, urb->transfer_buffer, urb->actual_length);
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief  Start purge the USB interrupt in transfer.
 *  @param[in] cdev       CDC device
 *  @param[in] func       The interrupt in data receiver callback function.
 *  @return   Success or not.
 * @retval   0           Success
 * @retval   Otherwise   Failed
 */
int32_t USBH_CDC_StartStatusPipe(CDC_DEV_T *cdev, CDC_CB_FUNC *func)
{
    EP_INFO_T   *ep_info;
    URB_T       *urb;
    uint32_t    pipe;
    USB_DEV_T   *udev;
    int         maxp, ret;

    if (!func || cdev->urb_sts)
        return USB_ERR_INVAL;

    udev = cdev->udev;

	ep_info = cdev->ep_sts;
	if (ep_info == NULL)
	{
    	ep_info = cdc_get_ep_info(cdev, USB_ENDPOINT_XFER_INT, USB_DIR_IN);
    	if (ep_info == NULL) 
    	{
        	CDC_DBGMSG("Interrupt-in endpoint not found in this CDC device!\n");
        	return USB_ERR_PIPE;
    	}
    }

    urb = USBH_AllocUrb();
    if (urb == NULL) {
        CDC_DBGMSG("Failed to allocated URB!\n");
        return USB_ERR_NOMEM;
    }

    pipe = usb_rcvintpipe(udev, ep_info->bEndpointAddress);
    maxp = usb_maxpacket(udev, pipe, usb_pipein(pipe));

    if (maxp > CDC_STATUS_BUFF_SIZE)
        maxp = CDC_STATUS_BUFF_SIZE;

    FILL_INT_URB(urb, udev, pipe, (char *)cdev->sts_buff, maxp, cdc_int_in_irq,
                 cdev, ep_info->bInterval);

    cdev->urb_sts = urb;
    cdev->sts_func = func;

    ret = USBH_SubmitUrb(urb);
    if (ret) {
        CDC_DBGMSG("Error - failed to submit interrupt read request (%d)", ret);
        USBH_FreeUrb(urb);
        cdev->urb_sts = NULL;
        return USB_ERR_IO;
    }
    return 0;
}

/*
 * CDC BULK-in complete function
 */
static void  cdc_bulk_in_irq(URB_T *urb)
{
    CDC_DEV_T   *cdev;

    //CDC_DBGMSG("cdc_bulk_in_irq. %d\n", urb->actual_length);

    cdev = (CDC_DEV_T *)urb->context;

    if (urb->status) {
        CDC_DBGMSG("cdc_bulk_in_irq - has error: 0x%x\n", urb->status);
        return;
    }

    if (cdev->rx_func)
        cdev->rx_func(cdev, urb->transfer_buffer, urb->actual_length);
        
    cdev->rx_busy = 0;
}

/// @endcond HIDDEN_SYMBOLS

/**
 * @brief  Start purge the USB BULK in transfer.
 *  @param[in] cdev       CDC device
 *  @param[in] func       The BULK in data receiver callback function.
 *  @return   Success or not.
 * @retval   0           Success
 * @retval   Otherwise   Failed
 */
int32_t USBH_CDC_StartRxPipe(CDC_DEV_T *cdev, CDC_CB_FUNC *func)
{
    EP_INFO_T   *ep_info;
    URB_T       *urb;
    uint32_t    pipe;
    USB_DEV_T   *udev;
    int         maxp, ret;

    if (!func)
        return USB_ERR_INVAL;
        
    if (cdev->urb_rx && cdev->rx_busy)
        return USB_ERR_BUSY;

    udev = cdev->udev;
	
	ep_info = cdev->ep_rx;
	if (ep_info == NULL)
	{
        ep_info = cdc_get_ep_info(cdev, USB_ENDPOINT_XFER_BULK, USB_DIR_IN);
        if (ep_info == NULL) 
        {
        	CDC_DBGMSG("Bulk-in endpoint not found in this CDC device!\n");
        	return USB_ERR_PIPE;
    	}
    	cdev->ep_rx = ep_info;
    }

	urb = cdev->urb_rx;
	if (urb == NULL)
	{
        urb = USBH_AllocUrb();
        if (urb == NULL) {
            CDC_DBGMSG("Failed to allocated URB!\n");
            return USB_ERR_NOMEM;
        }
        cdev->urb_rx = urb;
    }

    pipe = usb_rcvbulkpipe(udev, ep_info->bEndpointAddress);
    maxp = usb_maxpacket(udev, pipe, usb_pipein(pipe));

    if (maxp > CDC_RX_BUFF_SIZE)
        maxp = CDC_RX_BUFF_SIZE;

    FILL_BULK_URB(urb, udev, pipe, (char *)cdev->rx_buff, maxp, cdc_bulk_in_irq, cdev);

    cdev->rx_func = func;
    cdev->rx_busy = 1;

    ret = USBH_SubmitUrb(urb);
    if (ret) {
        CDC_DBGMSG("Error - failed to submit bulk read request (%d)", ret);
        USBH_FreeUrb(urb);
        cdev->urb_rx = NULL;
        return USB_ERR_IO;
    }
    return 0;
}

/*
 * CDC BULK-in complete function
 */
static volatile int  bulk_out_done; 
static void  cdc_bulk_out_irq(URB_T *urb)
{
	bulk_out_done = 1;
}

/**
 * @brief  Send a block of data.
 *  @param[in] cdev      CDC device
 *  @param[in] buff      Buffer contains the data block to be send.
 *  @param[in] buff_len  Length in byte of data to be send
 *  @return   Success or not.
 * @retval   0           Success
 * @retval   Otherwise   Failed
 */
int32_t USBH_CDC_SendData(CDC_DEV_T *cdev, uint8_t *buff, int buff_len)
{
    EP_INFO_T   *ep_info;
    URB_T       *urb;
    uint32_t    pipe;
    USB_DEV_T   *udev;
    int         t0, ret;

    udev = cdev->udev;
    
	ep_info = cdev->ep_tx;
	if (ep_info == NULL)
	{
        ep_info = cdc_get_ep_info(cdev, USB_ENDPOINT_XFER_BULK, USB_DIR_OUT);
        if (ep_info == NULL) 
        {
        	CDC_DBGMSG("Bulk-out endpoint not found in this CDC device!\n");
        	return USB_ERR_PIPE;
    	}
    	cdev->ep_tx = ep_info;
    }

    urb = USBH_AllocUrb();
    if (urb == NULL) 
    {
    	CDC_DBGMSG("Failed to allocated URB!\n");
     	return USB_ERR_NOMEM;
    }

    pipe = usb_sndbulkpipe(udev, ep_info->bEndpointAddress);

    FILL_BULK_URB(urb, udev, pipe, buff, buff_len, cdc_bulk_out_irq, cdev);

	bulk_out_done = 0;
	
    ret = USBH_SubmitUrb(urb);
    if (ret) {
        CDC_DBGMSG("Error - failed to submit bulk out request (%d)", ret);
        USBH_FreeUrb(urb);
        return USB_ERR_IO;
    }

    for (t0 = 0; t0 < USB_TIMEOUT; t0++)
    {
        if (bulk_out_done)
            break;
    }
    if (t0 >= USB_TIMEOUT)
    {
   		CDC_DBGMSG("Error - failed to send data, time-out!\n");
   		USBH_UnlinkUrb(urb);
   		USBH_FreeUrb(urb);
   		return USB_ERR_IO;
	}

    USBH_FreeUrb(urb);
    return 0;
}

/*@}*/ /* end of group USBH_CDC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group USBH_CDC_Driver */

/*@}*/ /* end of group USB_Host_Driver */

/*@}*/ /* end of group Component_Library */


/*** (C) COPYRIGHT 2018~2019 Nuvoton Technology Corp. ***/


