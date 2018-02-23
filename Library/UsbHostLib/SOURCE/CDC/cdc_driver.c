/**************************************************************************//**
 * @file     cdc_driver.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 18/01/29 11:46a $
 * @brief    M451 MCU USB Host CDC driver
 *
 * @note
 * Copyright (C) 2018~2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

extern int  cdc_config_parser(CDC_DEV_T *cdev);

static CDC_DEV_T *g_cdev_list = NULL;

static CDC_DEV_T *alloc_cdc_device(void)
{
	CDC_DEV_T  *cdev; 
	
	cdev = (CDC_DEV_T *)malloc(sizeof(CDC_DEV_T));
	if (cdev == NULL)
	    return NULL;
	    
	memset((char *)cdev, 0, sizeof(CDC_DEV_T));
	cdev->ifnum_data = -1;
    return cdev;
}

void  free_cdc_device(CDC_DEV_T *cdev)
{
	free(cdev);
}

static void add_new_cdc_device(CDC_DEV_T *cdev)
{
	if (g_cdev_list == NULL)
	{
		cdev->next = NULL;
		g_cdev_list = cdev;
	}
	else
	{
		cdev->next = g_cdev_list;
		g_cdev_list = cdev;
	}
}

static void remove_cdc_device(CDC_DEV_T *cdev)
{
	CDC_DEV_T  *p;
	
	if (g_cdev_list == cdev)
	{
		g_cdev_list = g_cdev_list->next;
		return;
	}

	p = g_cdev_list;
	while (p != NULL)
	{
		if (p->next == cdev)
		{
			p->next = cdev->next;
			return;
		}
		p = p->next;
	}
	CDC_DBGMSG("Warning! remove_cdc_device 0x%x not found!\n", (int)cdev);
}

static int  cdc_probe(USB_DEV_T *udev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id)
{
    int         ifnum;
    CDC_DEV_T   *cdev;
    int         ret;

    CDC_DBGMSG("cdc_probe - udev=0x%x\n", (int)udev);

    if (ifd->bInterfaceClass != USB_CLASS_COMM)
        return -1;      /* Not CDC class interface */

    ifnum = ifd->bInterfaceNumber;

    CDC_DBGMSG("CDC probe called for ifnum %d\n", ifnum);

    cdev = alloc_cdc_device();
    if (cdev == NULL)
        return USB_ERR_NODEV;
        
    CDC_DBGMSG("New CDC device 0x%x.\n", (int)cdev);

    cdev->ifnum_cdc = ifnum;
    cdev->udev = udev;
    cdev->next = NULL;
    cdev->bSubClassCode = ifd->bInterfaceSubClass;
    cdev->bProtocolCode = ifd->bInterfaceProtocol;
    
    ret = cdc_config_parser(cdev);
    if (ret != 0)
    {
    	CDC_DBGMSG("Parsing CDC desceiptor failed! 0x%x\n", ret);
    	free_cdc_device(cdev);
    	return -1;
	}
	
	add_new_cdc_device(cdev);

    return 0;
}


static void  cdc_disconnect(USB_DEV_T *udev)
{
    CDC_DEV_T   *cdev;

    CDC_DBGMSG("CDC device disconnected!\n");
    
    cdev = g_cdev_list;
    while (cdev != NULL)
    {
    	if (cdev->udev == udev)
    	    break;
    	cdev = cdev->next;
    }
    
    if (cdev == NULL)
    {
    	CDC_DBGMSG("cdc_disconnect - CDC device not found!\n");
    	return;
	}
    	
    if (cdev->urb_sts)
    {
        USBH_UnlinkUrb(cdev->urb_sts);
        USBH_FreeUrb(cdev->urb_sts);
        cdev->urb_sts = NULL;
    }
    if (cdev->urb_rx)
    {
        USBH_UnlinkUrb(cdev->urb_rx);
        USBH_FreeUrb(cdev->urb_rx);
        cdev->urb_rx = NULL;
    }
    
    remove_cdc_device(cdev);    
    free_cdc_device(cdev);    
}


static USB_DEV_ID_T  cdc_id_table[] = {
    USB_DEVICE_ID_MATCH_INT_CLASS,     /* match_flags */
    0, 0, 0, 0, 0, 0, 0,
    USB_CLASS_COMM,                    /* bInterfaceClass */
    0, 0, 0
};


static USB_DRIVER_T  cdc_driver = {
    "CDC driver",
    cdc_probe,
    cdc_disconnect,
    cdc_id_table,
    NULL,                       /* suspend */
    NULL,                       /* resume */
    {NULL,NULL}                 /* driver_list */
};


/// @endcond /* HIDDEN_SYMBOLS */


/**
  * @brief    Init USB Host CDC driver.
  * @return   None
  */
void USBH_CDCInit(void)
{
    g_cdev_list = NULL;
    USBH_RegisterDriver(&cdc_driver);
}


/**
 *  @brief   Get a list of currently connected USB Hid devices.
 *  @return  List of CDC devices.
 *  @retval  NULL       There's no CDC device found.
 *  @retval  Otherwise  A list of connected CDC devices.
 *
 *  The CDC devices are chained by the "next" member of CDC_DEV_T.
 */
CDC_DEV_T * USBH_CDCGetDeviceList(void)
{
    return g_cdev_list;
}


/*@}*/ /* end of group USBH_CDC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group USBH_CDC_Driver */

/*@}*/ /* end of group USB_Host_Driver */

/*@}*/ /* end of group Component_Library */


/*** (C) COPYRIGHT 2018~2019 Nuvoton Technology Corp. ***/

