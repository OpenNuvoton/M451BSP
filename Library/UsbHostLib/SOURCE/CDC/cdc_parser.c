/**************************************************************************//**
 * @file     cdc_parser.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 18/01/30 10:02a $
 * @brief    M451 MCU USB Host CDC Class driver
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

/// @cond HIDDEN_SYMBOLS


static int  cdc_parse_cs_interface(CDC_DEV_T *cdev, uint8_t *buffer, int size)
{
    USB_DESC_HDR_T  *header;
    CDC_IF_HDR_T    *cifd;
    int             parsed = 0;

    while (size > 0)
    {
        while(size >= sizeof(USB_DESC_HDR_T))
        {
            header = (USB_DESC_HDR_T *)buffer;

            if (header->bLength < 2)
            {
                CDC_DBGMSG("Invalid descriptor length of %d\n", header->bLength);
                return -1;
            }
            
            if (header->bDescriptorType != CDC_CS_INTERFACE)
                return parsed;
                
            cifd = (CDC_IF_HDR_T *)header;
            
            CDC_DBGMSG("CS_INTERFACE: 0x%x, ", cifd->bDescriptorSubtype);
            
            switch (cifd->bDescriptorSubtype)
            {
            	case CDC_DT_HDR_FUNC:
            	    CDC_DBGMSG("Header Functional\n");
            	    break;
            	case CDC_DT_CALL_MANAGE:
            	    CDC_DBGMSG("Call Management\n");
            	    break;
            	case CDC_DT_ABS_CTRL:
            	    CDC_DBGMSG("Abstract Control Management\n");
            	    break;
            	case CDC_DT_LINE_MANAGE:
            	    CDC_DBGMSG("Direct Line Management\n");
            	    break;
            	case CDC_DT_TEL_RINGER:
            	    CDC_DBGMSG("Telephone Ringer\n");
            	    break;
            	case CDC_DT_TEL_OPER_MODES:
            	    CDC_DBGMSG("Telephone Operational Modes\n");
            	    break;
            	case CDC_DT_CALL_LINE_CAP:
            	    CDC_DBGMSG("Telephone Call and Line State Reporting Capabilities\n");
            	    break;
            	case CDC_DT_UNION:
            	    CDC_DBGMSG("Union Functional\n");
            	    if (cifd->bLength >= 5)
            	    	cdev->ifnum_data = cifd->payload[1];
            	   	if (cifd->bLength >= 6)
            	   	{
            	   		CDC_DBGMSG("Union Functional length %d, not supported!\n", cifd->bLength);
            		}
            	    break;
            	case CDC_DT_COUNTRY_SEL:
            	    CDC_DBGMSG("Country Selection\n");
            	    break;
            	case CDC_DT_USB_TERMINAL:
            	    CDC_DBGMSG("USB Terminal\n");
            	    break;
            	case CDC_DT_NET_CHANNEL:
            	    CDC_DBGMSG("Network Channel Terminal\n");
            	    break;
            	case CDC_DT_PROTO_UNIT:
            	    CDC_DBGMSG("Protocol Unit\n");
            	    break;
            	case CDC_DT_EXTENT_UNIT:
            	    CDC_DBGMSG("Extension Unit\n");
            	    break;
            	case CDC_DT_MULTI_CHANNEL:
            	    CDC_DBGMSG("Multi-Channel Management\n");
            	    break;
            	case CDC_DT_CAPI_CTRL:
            	    CDC_DBGMSG("CAPI Control Management\n");
            	    break;
            	case CDC_DT_ETHERNET_FUNC:
            	    CDC_DBGMSG("Ethernet Networking Functional\n");
            	    break;
            	case CDC_DT_ATM_FUNC:
            	    CDC_DBGMSG("ATM Networking Functional\n");
            	    break;
            }

            buffer += header->bLength;
            parsed += header->bLength;
            size -= header->bLength;
        }

    }   /* end of while */
    return parsed;
}

#define MAX_CFG_DESC_SIZE   384

int  cdc_config_parser(CDC_DEV_T *cdev)
{
    USB_DEV_T       *udev = cdev->udev;
    USB_CONFIG_DESC_T *config;
    USB_DESC_HDR_T  *header;
    USB_IF_DESC_T   *ifd;
    uint8_t         *bptr, *buffer = NULL;
    int             length, size, result;

    buffer = (uint8_t *)malloc(MAX_CFG_DESC_SIZE);
    if (buffer == NULL)
        return USB_ERR_NOMEM;
        
    config = (USB_CONFIG_DESC_T *)buffer;

    /*------------------------------------------------------------------*/
    /*  Get configuration descriptor from device...                     */
    /*------------------------------------------------------------------*/

    result = USBH_GetDescriptor(udev, USB_DT_CONFIG, 0, buffer, 8);
    if(result < 8)
    {
        result = USB_ERR_INVAL;
        goto err_out;
    }

    length = config->wTotalLength;
    if(length > MAX_CFG_DESC_SIZE)
    {
        CDC_ERRMSG("The Configuration descriptor of CDC device is too large.\n");
        CDC_ERRMSG("Please enlarge MAX_CFG_DESC_SIZE in usbh_uac.h to be larger than %d.\n", length);
        result = USB_ERR_NOMEM;
        goto err_out;
    }

    /* Now that we know the length, get the whole thing */
    result = USBH_GetDescriptor(udev, USB_DT_CONFIG, 0, buffer, length);
    if(result != length)
    {
        CDC_DBGMSG("Failed to get configuration descriptor!\n");
        goto err_out;
    }

    /*------------------------------------------------------------------*/
    /*  Search the CDC interface descriptor                             */
    /*------------------------------------------------------------------*/
    bptr = buffer;
    bptr += config->bLength;
    size = config->wTotalLength - config->bLength;

    while (size >= sizeof(USB_DESC_HDR_T))
    {
        header = (USB_DESC_HDR_T *)bptr;

        if ((header->bLength > size) || (header->bLength < 2))
        {
            CDC_DBGMSG("Error - invalid descriptor length of %d\n", header->bLength);
            result = USB_ERR_INVAL;
            goto err_out;
        }

        /*
         *  Is the interface descriptor of this CDC device?
         */
        if (header->bDescriptorType == USB_DT_INTERFACE)
        {
        	ifd = (USB_IF_DESC_T *)header;
        	if (ifd->bInterfaceNumber == cdev->ifnum_cdc)
        	{
        		bptr += header->bLength;
        		size -= header->bLength;
        	    break;
        	}
        }
        bptr += header->bLength;
        size -= header->bLength;
    }   /* end of while */

    /*------------------------------------------------------------------*/
    /*  Parsing all follwoing CDC class interface descriptors           */
    /*------------------------------------------------------------------*/

    while (size >= sizeof(USB_DESC_HDR_T))
    {
        header = (USB_DESC_HDR_T *)bptr;

        if ((header->bLength > size) || (header->bLength < 2))
        {
            CDC_DBGMSG("Error - invalid descriptor length of %d\n", header->bLength);
            result = USB_ERR_INVAL;
            goto err_out;
        }

        /*
         *  Is a class interface descriptor?
         */
        if (header->bDescriptorType != CDC_CS_INTERFACE)
        	break;
        	
  		result = cdc_parse_cs_interface(cdev, bptr, size);
   		if (result < 0)
       		goto err_out;
        bptr += result;
        size -= result;
    }   /* end of while */

#if 0
    /*------------------------------------------------------------------*/
    /*  Search the CDC data interface descriptor if found               */
    /*------------------------------------------------------------------*/
    if (cdev->ifnum_data = -1)
    {
        bptr = buffer;
        bptr += config->bLength;
        size = config->wTotalLength - config->bLength;
        
        while (size >= sizeof(USB_DESC_HDR_T))
        {
            header = (USB_DESC_HDR_T *)bptr;
        
            if ((header->bLength > size) || (header->bLength < 2))
            {
                CDC_DBGMSG("Error - invalid descriptor length of %d\n", header->bLength);
                result = USB_ERR_INVAL;
                goto err_out;
            }
        
            /*
             *  Is the data interface descriptor of this CDC device?
             */
            if (header->bDescriptorType == USB_DT_INTERFACE)
            {
            	ifd = (USB_IF_DESC_T *)header;
            	if (ifd->bInterfaceNumber == cdev->ifnum_data)
            	{
            		bptr += header->bLength;
            		size -= header->bLength;
            	    break;
            	}
            }
            bptr += header->bLength;
            size -= header->bLength;
        }   /* end of while */
    }
#endif

	CDC_DBGMSG("CDC ifnum_cdc = %d\n", cdev->ifnum_cdc);
	CDC_DBGMSG("CDC ifnum_data = %d\n", cdev->ifnum_data);

	free(buffer);

    return 0;
    
err_out:
	if (buffer)
	    free(buffer);
	return result;            
}


/// @endcond HIDDEN_SYMBOLS

/*** (C) COPYRIGHT 2018~2019 Nuvoton Technology Corp. ***/

