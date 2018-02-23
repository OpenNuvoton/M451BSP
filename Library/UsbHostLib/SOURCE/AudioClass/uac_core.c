/**************************************************************************//**
 * @file     uac_driver.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 17/07/04 3:24p $
 * @brief    M451 MCU USB Host Audio Class driver
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "M451Series.h"
#include "usbh_core.h"

#include "usbh_uac.h"
#include "uac.h"


/** @addtogroup Component_Library Component Library
  @{
*/

/** @addtogroup USB_Host_Driver USB Host Driver
  @{
*/

/** @addtogroup USBH_AS_Driver USB Host Audio Class Driver
  @{
*/

/** @addtogroup USBH_AS_EXPORTED_FUNCTIONS USB Host Audio Class Driver Exported Functions
  @{
*/


/**
 *  @brief  Obtain Audio Class device's channel number.
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @return   Channel number or error code.
 *  @retval   < 0      Failed. UAC device may not present or function not supported.
 *  @retval   Otheriwse  The channle number.
 */
int32_t  UAC_GetChannelNumber(UAC_DEV_T *audev, uint8_t target)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;

    if(uac_info == NULL)
        return UAC_RET_IO_ERR;

    if(target == UAC_SPEAKER)
    {
        if(!uac_info->ft_play)
            return UAC_RET_DEV_NOT_SUPPORTED;
        return uac_info->ft_play->bNrChannels;
    }
    else
    {
        if(!uac_info->ft_rec)
            return UAC_RET_DEV_NOT_SUPPORTED;
        return uac_info->ft_rec->bNrChannels;
    }
}


/**
 *  @brief  Obtain Audio Class device subframe bit resolution..
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[out] byte_cnt  The number of bytes occupied by one audio subframe. Can be 1, 2, 3 or 4.
 *  @return   Bit resolution or error code.
 *  @retval   < 0        Failed. UAC device may not present or function not supported.
 *  @retval   Otherwise  The number of effectively used bits from the available bits in an audio subframe.
 */
int32_t  UAC_GetBitResolution(UAC_DEV_T *audev, uint8_t target, uint8_t *byte_cnt)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;

    if(uac_info == NULL)
        return UAC_RET_IO_ERR;

    if(target == UAC_SPEAKER)
    {
        if(!uac_info->ft_play)
            return UAC_RET_DEV_NOT_SUPPORTED;
        *byte_cnt = uac_info->ft_play->bSubframeSize;
        return uac_info->ft_play->bBitResolution;
    }
    else
    {
        if(!uac_info->it_microphone)
            return UAC_RET_DEV_NOT_SUPPORTED;
        *byte_cnt = uac_info->ft_rec->bSubframeSize;
        return uac_info->ft_rec->bBitResolution;
    }
}


/// @cond HIDDEN_SYMBOLS

uint32_t  srate_to_u32(uint8_t *srate)
{
    return (srate[2] << 16) | (srate[1] << 8) | srate[0];
}

/// @endcond HIDDEN_SYMBOLS


/**
 *  @brief  Get a list of sampling rate frequences supported by the UAC device.
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[out] srate_list  A word array provided by user application to hold the sampling rate list.
 *  @param[in]  max_cnt  Available number of entries of srate_list[]. Must be > 2.
 *  @param[out] type   Indicates how the sampling frequency can be programmed.
 *                     0:  Continuous sampling frequency. srate_list[0] is the lower bound
 *                          in Hz of the sampling frequency and srate_list[1] is the upper bound.
 *                     1~255:  The number of discrete sampling frequencies supported. They are
 *                             listed in srate_list[].
 *  @return   Success or not.
 *  @retval   0        Success
 *  @retval   Otherwise  Failed
 */
int32_t  UAC_GetSamplingRate(UAC_DEV_T *audev, uint8_t target, uint32_t *srate_list,
                             int max_cnt, uint8_t *type)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    AC_FT1_T    *ft;
    int         i;

    if(target == UAC_SPEAKER)
        ft = uac_info->ft_play;
    else
        ft = uac_info->ft_rec;

    if(!ft)
        return UAC_RET_DEV_NOT_SUPPORTED;

    *type = ft->bSamFreqType;

    if(*type == 0)
    {
        if(max_cnt < 2)
            return UAC_RET_OUT_OF_MEMORY;

        srate_list[0] = srate_to_u32(&ft->tSamFreq[0][0]);
        srate_list[1] = srate_to_u32(&ft->tSamFreq[1][0]);
    }
    else
    {
        for(i = 0; i < *type; i++)
            srate_list[i] = srate_to_u32(&ft->tSamFreq[i][0]);
    }
    return 0;
}


/**
 *  @brief  Set sampling rate frequency.
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *                     - \ref UAC_SET_MIN
 *                     - \ref UAC_GET_MIN
 *                     - \ref UAC_SET_MAX
 *                     - \ref UAC_GET_MAX
 *                     - \ref UAC_SET_RES
 *                     - \ref UAC_GET_RES
 *  @param[in]  srate  Sampling rate frequncy to be set or get.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otheriwse  Error occurred
 */
int32_t  UAC_SamplingRateControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint32_t *srate)
{
    USB_DEV_T   *udev = audev->udev;
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    USB_EP_DESC_T  *ep;
    int         pipe, len;
    uint8_t     bmRequestType;
    uint8_t     tSampleFreq[3];

    if(uac_info == NULL)
        return UAC_RET_IO_ERR;

    if(target == UAC_SPEAKER)
        ep = uac_info->epd_play;
    else
        ep = uac_info->epd_rec;

    if(ep == NULL)
        return UAC_RET_DEV_NOT_SUPPORTED;

	if (target == UAC_SPEAKER)
    	USBH_SetInterface(audev->udev, uac_info->ifd_play->bInterfaceNumber, uac_info->ifd_play->bAlternateSetting);
	
	if (target == UAC_MICROPHONE)
		USBH_SetInterface(audev->udev, uac_info->ifd_rec->bInterfaceNumber, uac_info->ifd_rec->bAlternateSetting);

    tSampleFreq[0] = *srate & 0xff;
    tSampleFreq[1] = (*srate >> 8) & 0xff;
    tSampleFreq[2] = (*srate >> 16) & 0xff;

    bmRequestType = USB_TYPE_CLASS | USB_RECIP_ENDPOINT;

    if(req & 0x80)
    {
        pipe = usb_rcvctrlpipe(udev, 0);
        bmRequestType |= 0x80;
    }
    else
    {
        pipe = usb_sndctrlpipe(udev, 0);
    }

    len = USBH_SendCtrlMsg(udev, pipe, req, bmRequestType,
                           (SAMPLING_FREQ_CONTROL << 8),               // wValue
                           ep->bEndpointAddress,   // wIndex
                           tSampleFreq, 3, UAC_REQ_TIMEOUT);
    if(len == 3)
    {
        *srate = srate_to_u32(tSampleFreq);
        return 0;
    }

    return UAC_RET_IO_ERR;
}



/**
 *  @brief  Control Audio Class device mute on/off.
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *  @param[in]  chn    The requested channel. It can be one of the followings:
 *                     - \ref UAC_CH_LEFT_FRONT
 *                     - \ref UAC_CH_RIGHT_FRONT
 *                     - \ref UAC_CH_CENTER_FRONT
 *                     - \ref UAC_CH_LOW_FREQ_EN
 *                     - \ref UAC_CH_LEFT_SRN
 *                     - \ref UAC_CH_RIGHT_SRN
 *                     - \ref UAC_CH_LEFT_OF_CENTER
 *                     - \ref UAC_CH_RIGHT_OF_CENTER
 *                     - \ref UAC_CH_SURROUND
 *                     - \ref UAC_CH_SIDE_LEFT
 *                     - \ref UAC_CH_SIDE_RIGHT
 *                     - \ref UAC_CH_TOP
 *  @param[in]  mute   One byte data. If the channel is muted, then the value is 1. Otherwise, it's 0.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   UAC_RET_DEV_NOT_SUPPORTED  This UAC device does not support this function.
 *  @retval   Otheriwse  Error occurred
 */
int32_t  UAC_MuteControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint16_t chn, uint8_t *mute)
{
    USB_DEV_T   *udev = audev->udev;
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    AC_FU_T     *fu;
    int         pipe, len;
    uint8_t     bmRequestType;

    if(uac_info == NULL)
        return UAC_RET_IO_ERR;

    if(target == UAC_SPEAKER)
        fu = uac_info->fu_play;
    else if(target == UAC_MICROPHONE)
        fu = uac_info->fu_rec;
    else
        return UAC_RET_INVALID;

    //if (uac_check_fu_ctrl(uac_info, target, chn, MUTE_CONTROL) != 0)
    //  return UAC_RET_DEV_NOT_SUPPORTED;

    bmRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE;

    if(req & 0x80)
    {
        pipe = usb_rcvctrlpipe(udev, 0);
        bmRequestType |= 0x80;
    }
    else
    {
        pipe = usb_sndctrlpipe(udev, 0);
    }

    len = USBH_SendCtrlMsg(udev, pipe, req, bmRequestType,
                           (MUTE_CONTROL << 8) | chn,                  // wValue
                           (fu->bUnitID << 8) | (audev->ctrl_ifnum),   // wIndex
                           mute, 1, UAC_REQ_TIMEOUT);
    if(len == 1)
        return 0;
    else
        return -1;
}


/**
 *  @brief  Audio Class device volume control.
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *                     - \ref UAC_SET_MIN
 *                     - \ref UAC_GET_MIN
 *                     - \ref UAC_SET_MAX
 *                     - \ref UAC_GET_MAX
 *                     - \ref UAC_SET_RES
 *                     - \ref UAC_GET_RES
 *                     - \ref UAC_GET_STAT
 *  @param[in]  chn    The requested channel. It can be one of the followings:
 *                     - \ref UAC_CH_LEFT_FRONT
 *                     - \ref UAC_CH_RIGHT_FRONT
 *                     - \ref UAC_CH_CENTER_FRONT
 *                     - \ref UAC_CH_LOW_FREQ_EN
 *                     - \ref UAC_CH_LEFT_SRN
 *                     - \ref UAC_CH_RIGHT_SRN
 *                     - \ref UAC_CH_LEFT_OF_CENTER
 *                     - \ref UAC_CH_RIGHT_OF_CENTER
 *                     - \ref UAC_CH_SURROUND
 *                     - \ref UAC_CH_SIDE_LEFT
 *                     - \ref UAC_CH_SIDE_RIGHT
 *                     - \ref UAC_CH_TOP
 *  @param[in]  volume   Audio Class device volume value, which is intepreted as the following:
 *                       0x7FFF:    127.9961 dB
 *                       . . .
 *                       0x0100:      1.0000 dB
 *                       . . .
 *                       0x0002:      0.0078 dB
 *                       0x0001:      0.0039 dB
 *                       0x0000:      0.0000 dB
 *                       0xFFFF:     -0.0039 dB
 *                       0xFFFE:     -0.0078 dB
 *                       . . .
 *                       0xFE00:     -1.0000 dB
 *                       . . .
 *                       0x8002:   -127.9922 dB
 *                       0x8001:   -127.9961 dB
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   UAC_RET_DEV_NOT_SUPPORTED  This UAC device does not support this function.
 *  @retval   Otheriwse  Error occurred
 */
int32_t  UAC_VolumeControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint16_t chn, uint16_t *volume)
{
    USB_DEV_T   *udev = audev->udev;
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    AC_FU_T     *fu;
    int         pipe, len;
    uint8_t     bmRequestType;

    if(uac_info == NULL)
        return UAC_RET_IO_ERR;

    if(target == UAC_SPEAKER)
        fu = uac_info->fu_play;
    else if(target == UAC_MICROPHONE)
        fu = uac_info->fu_rec;
    else
        return UAC_RET_INVALID;

    //if (uac_check_fu_ctrl(uac_info, target, chn, VOLUME_CONTROL) != 0)
    //  return UAC_RET_DEV_NOT_SUPPORTED;

    bmRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE;

    if(req & 0x80)
    {
        pipe = usb_rcvctrlpipe(udev, 0);
        bmRequestType |= 0x80;
    }
    else
    {
        pipe = usb_sndctrlpipe(udev, 0);
    }


    len = USBH_SendCtrlMsg(udev, pipe, req, bmRequestType,
                           (VOLUME_CONTROL << 8) | chn,          // wValue
                           (fu->bUnitID << 8) | (audev->ctrl_ifnum),   // wIndex
                           (uint8_t *)volume, 2, UAC_REQ_TIMEOUT);
    if(len == 2)
        return 0;
    else
        return -1;
}


/**
 *  @brief  Audio Class device automatic gain control.
 *  @param[in]  audev  UAC device
 *  @param[in]  target Select the control target.
 *                     - \ref UAC_SPEAKER
 *                     - \ref UAC_MICROPHONE
 *  @param[in]  req    Control request. This UAC driver supports the following request:
 *                     - \ref UAC_SET_CUR
 *                     - \ref UAC_GET_CUR
 *  @param[in]  chn    The requested channel. It can be one of the followings:
 *                     - \ref UAC_CH_LEFT_FRONT
 *                     - \ref UAC_CH_RIGHT_FRONT
 *                     - \ref UAC_CH_CENTER_FRONT
 *                     - \ref UAC_CH_LOW_FREQ_EN
 *                     - \ref UAC_CH_LEFT_SRN
 *                     - \ref UAC_CH_RIGHT_SRN
 *                     - \ref UAC_CH_LEFT_OF_CENTER
 *                     - \ref UAC_CH_RIGHT_OF_CENTER
 *                     - \ref UAC_CH_SURROUND
 *                     - \ref UAC_CH_SIDE_LEFT
 *                     - \ref UAC_CH_SIDE_RIGHT
 *                     - \ref UAC_CH_TOP
 *  @param[in]  mute   One byte data. If the channel's automaic gain control is on, then the value is 1. Otherwise, it's 0.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   UAC_RET_DEV_NOT_SUPPORTED  This UAC device does not support this function.
 *  @retval   Otheriwse  Error occurred
 */
int32_t  UAC_AutoGainControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint16_t chn, uint8_t *mute)
{
    USB_DEV_T   *udev = audev->udev;
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    AC_FU_T     *fu;
    int         pipe, len;
    uint8_t     bmRequestType;

    if(uac_info == NULL)
        return UAC_RET_IO_ERR;

    if(target == UAC_SPEAKER)
        fu = uac_info->fu_play;
    else if(target == UAC_MICROPHONE)
        fu = uac_info->fu_rec;
    else
        return UAC_RET_INVALID;

    //if (uac_check_fu_ctrl(uac_info, target, chn, MUTE_CONTROL) != 0)
    //  return UAC_RET_DEV_NOT_SUPPORTED;

    bmRequestType = USB_TYPE_CLASS | USB_RECIP_INTERFACE;

    if(req & 0x80)
    {
        pipe = usb_rcvctrlpipe(udev, 0);
        bmRequestType |= 0x80;
    }
    else
    {
        pipe = usb_sndctrlpipe(udev, 0);
    }

    len = USBH_SendCtrlMsg(udev, pipe, req, bmRequestType,
                           (AUTOMATIC_GAIN_CONTROL << 8) | chn,        // wValue
                           (fu->bUnitID << 8) | (audev->ctrl_ifnum),   // wIndex
                           mute, 1, UAC_REQ_TIMEOUT);
    if(len == 1)
        return 0;
    else
        return -1;
}


/// @cond HIDDEN_SYMBOLS

static void iso_in_irq(URB_T *urb)
{
    UAC_DEV_T *audev = (UAC_DEV_T *)urb->context;
    uint8_t     *buff;
    int         i, len, cp_len;
    int         status;

    /* We don't want to do anything if we are about to be removed! */
    if(!audev || !audev->udev)
        return;

    //printf("Iso in - SF=%d, EC=%d, L=%d.\n", urb->start_frame, urb->error_count, urb->actual_length);
    //printf("IN: SF=%d, L=%d\n", urb->start_frame, urb->actual_length);

    for(i = 0; i < urb->number_of_packets; i++)
    {
        len = urb->iso_frame_desc[i].actual_length;
        if(!len)
            continue;

        buff = (uint8_t *)urb->transfer_buffer + i * AU_IN_MAX_PKTSZ;
        cp_len = audev->au_in_bufsz - audev->au_in_bufidx;
        if(cp_len > len)
            cp_len = len;

        memcpy(&(audev->au_in_buff[audev->au_in_bufidx]), buff, cp_len);
        audev->au_in_func(audev, &(audev->au_in_buff[audev->au_in_bufidx]), cp_len);

        audev->au_in_bufidx += cp_len;
        len -= cp_len;

        if(len)
        {
            buff += cp_len;
            memcpy(audev->au_in_buff, buff + cp_len, len);
            audev->au_in_func(audev, audev->au_in_buff, len);
            audev->au_in_bufidx = len;
        }
    }

    if(!audev->in_streaming)
        return;

    urb->transfer_flags = USB_ISO_ASAP;
    urb->number_of_packets = ISO_FRAME_COUNT;
    urb->transfer_buffer_length = AU_IN_MAX_PKTSZ * ISO_FRAME_COUNT;
    urb->actual_length = 0;
    for(i = 0; i < ISO_FRAME_COUNT; i++)
    {
        urb->iso_frame_desc[i].status = 0;
        urb->iso_frame_desc[i].actual_length = 0;
        urb->iso_frame_desc[i].offset = i * AU_IN_MAX_PKTSZ;
        urb->iso_frame_desc[i].length = AU_IN_MAX_PKTSZ;
    }

    /* Submit URB */
    status = USBH_SubmitUrb(urb);
    if(status)
        USBAS_DBGMSG("v: USBH_SubmitUrb ret %d\n", status);
}

/// @endcond HIDDEN_SYMBOLS


/**
 *  @brief  Install isochronous-in (microphone) callback function. Received audio data from
 *          UAC device will be delivered to user application by this callback function.
 *  @param[in] audev      Audio Class device
 *  @param[in] au_in_buff Audio stream input buffer. User application prepares and announces
 *                        this buffer. UAC driver will directly move received audio data into
 *                        it. Once UAC driver moves audio data into au_in_buff, it will call
 *                        the callback to notify user.
 *  @param[in] bufsiz     Size of au_in_buff.
 *  @param[in] func       The audio in callback function.
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int32_t UAC_InstallIsoInCbFun(UAC_DEV_T *audev, uint8_t *au_in_buff, int bufsiz, UAC_CB_FUNC *func)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    USB_EP_DESC_T  *ep;

    if(!func || !uac_info)
        return UAC_RET_INVALID;

    ep = uac_info->epd_rec;
    if(!ep)
    {
        USBAS_DBGMSG("Isochronous-in endpoint not found in this device!\n");
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    audev->au_in_func = func;
    audev->au_in_buff = au_in_buff;
    audev->au_in_bufsz = bufsiz;
    audev->au_in_bufidx = 0;
    return UAC_RET_OK;
}


/**
 *  @brief  Start to receive audio data from UAC device via isochronous in pipe.
 *  @param[in] audev      Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int32_t UAC_StartIsoInPipe(UAC_DEV_T *audev)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    USB_EP_DESC_T  *ep;
    URB_T       *urb;
    int         uidx, i, ret;

    if(!uac_info || !audev->au_in_func || (audev->urbin[0]))
        return UAC_RET_INVALID;

    ep = uac_info->epd_rec;
    if(!ep)
    {
        USBAS_DBGMSG("Isochronous-in endpoint not found in this device!\n");
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    audev->au_in_bufidx = 0;

    /* Set interface alternative settings */
    if(USBH_SetInterface(audev->udev, uac_info->ifd_rec->bInterfaceNumber, uac_info->ifd_rec->bAlternateSetting) != 0)
        return UAC_RET_IO_ERR;

    for(uidx = 0; uidx < ISO_IN_URB_CNT; uidx++)
    {
        urb = USBH_AllocUrb();
        if(urb == NULL)
        {
            USBAS_DBGMSG("Failed to allocated URB!\n");
            ret = UAC_RET_OUT_OF_MEMORY;
            goto err_out;
        }

        audev->urbin[uidx] = urb;
        urb->dev = audev->udev;
        urb->context = audev;
        urb->pipe = usb_rcvisocpipe(audev->udev, ep->bEndpointAddress);
        urb->transfer_flags = USB_ISO_ASAP;
        urb->complete = iso_in_irq;
        urb->interval = ep->bInterval;
        urb->number_of_packets = ISO_FRAME_COUNT;
        urb->transfer_buffer = &(audev->iso_inbuf[uidx][0]);
        urb->transfer_buffer_length = AU_IN_MAX_PKTSZ * ISO_FRAME_COUNT;
        urb->actual_length = 0;

        for(i = 0; i < ISO_FRAME_COUNT; i++)
        {
            urb->iso_frame_desc[i].offset = i * AU_IN_MAX_PKTSZ;
            urb->iso_frame_desc[i].length = AU_IN_MAX_PKTSZ;
        }

        ret = USBH_SubmitUrb(urb);
        if(ret)
        {
            USBAS_DBGMSG("Error - failed to submit URB (%d)", ret);
            ret = UAC_RET_IO_ERR;
            goto err_out;
        }
    }
    audev->in_streaming = 1;

    return UAC_RET_OK;

err_out:
    for(i = 0; i < ISO_IN_URB_CNT; i++)
    {
        if(audev->urbin[i])
        {
            USBH_UnlinkUrb(audev->urbin[i]);
            USBH_FreeUrb(audev->urbin[i]);
            audev->urbin[i] = NULL;
        }
    }
    audev->au_in_func = NULL;
    return ret;
}


/**
 *  @brief  Stop UAC device audio in data stream.
 *  @param[in] audev      Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int32_t UAC_StopIsoInPipe(UAC_DEV_T *audev)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    int    i;

    audev->in_streaming = 0;

    /* Set interface alternative settings */
    USBH_SetInterface(audev->udev, uac_info->ifd_rec->bInterfaceNumber, 0);

    for(i = 0; i < ISO_IN_URB_CNT; i++)
    {
        if(audev->urbin[i])
        {
            USBH_UnlinkUrb(audev->urbin[i]);
            USBH_FreeUrb(audev->urbin[i]);
            audev->urbin[i] = NULL;
        }
    }
    return UAC_RET_OK;
}


/// @cond HIDDEN_SYMBOLS

static void iso_out_irq(URB_T *urb)
{
    UAC_DEV_T *audev = (UAC_DEV_T *)urb->context;
    USB_EP_DESC_T  *ep;
    int         i, status;

    /* We don't want to do anything if we are about to be removed! */
    if(!audev || !audev->udev)
        return;

    ep = ((UAC_INFO_T *)audev->priv)->epd_play;

    //printf("Iso out - SF=%d, EC=%d, L=%d.\n", urb->start_frame, urb->error_count, urb->actual_length);
    //printf("OUT: SF=%d, L=%d\n", urb->start_frame, urb->actual_length);

    if(!audev->out_streaming)
        return;

    urb->transfer_flags = USB_ISO_ASAP;
    urb->number_of_packets = ISO_FRAME_COUNT;
    urb->transfer_buffer_length = AU_OUT_MAX_PKTSZ * ISO_FRAME_COUNT;
    urb->actual_length = 0;
    for(i = 0; i < ISO_FRAME_COUNT; i++)
    {
        urb->iso_frame_desc[i].status = 0;
        urb->iso_frame_desc[i].offset = i * AU_OUT_MAX_PKTSZ;
        urb->iso_frame_desc[i].actual_length = audev->au_out_func(audev, (uint8_t *)urb->transfer_buffer + urb->iso_frame_desc[i].offset, ep->wMaxPacketSize);
        urb->iso_frame_desc[i].length = urb->iso_frame_desc[i].actual_length;
        urb->actual_length += urb->iso_frame_desc[i].actual_length;
    }

    /* Submit URB */
    status = USBH_SubmitUrb(urb);
    if(status)
        USBAS_DBGMSG("v: USBH_SubmitUrb ret %d\n", status);
}

/// @endcond HIDDEN_SYMBOLS


/**
 *  @brief  Install isochronous-out (speaker) callback function. The UAC driver will call
 *          the callback function to request one audio out packet from user application.
 *          UAC driver will then send this packet to UAC device via isochronous out pipe.
 *  @param[in] audev      Audio Class device
 *  @param[in] func       The audio out callback function.
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int32_t UAC_InstallIsoOutCbFun(UAC_DEV_T *audev, UAC_CB_FUNC *func)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    USB_EP_DESC_T  *ep;

    if(!func || !uac_info)
        return UAC_RET_INVALID;

    ep = uac_info->epd_play;
    if(!ep)
    {
        USBAS_DBGMSG("Isochronous-out endpoint not found in this device!\n");
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    audev->au_out_func = func;
    return UAC_RET_OK;
}


/**
 *  @brief  Start to send audio data to UAC device via isochronous out pipe.
 *  @param[in] audev      Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int32_t UAC_StartIsoOutPipe(UAC_DEV_T *audev)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    USB_EP_DESC_T  *ep;
    URB_T       *urb;
    int         uidx, i, ret;

    if(!uac_info || !audev->au_out_func || (audev->urbout[0]))
        return UAC_RET_INVALID;

    ep = uac_info->epd_play;
    if(!ep)
    {
        USBAS_DBGMSG("Isochronous-out endpoint not found in this device!\n");
        return UAC_RET_DEV_NOT_SUPPORTED;
    }

    /* Set interface alternative settings */
    if(USBH_SetInterface(audev->udev, uac_info->ifd_play->bInterfaceNumber, uac_info->ifd_play->bAlternateSetting) != 0)
        return UAC_RET_IO_ERR;

    for(uidx = 0; uidx < ISO_OUT_URB_CNT; uidx++)
    {
        urb = USBH_AllocUrb();
        if(urb == NULL)
        {
            USBAS_DBGMSG("Failed to allocated URB!\n");
            ret = UAC_RET_OUT_OF_MEMORY;
            goto err_out;
        }

        audev->urbout[uidx] = urb;
        urb->dev = audev->udev;
        urb->context = audev;
        urb->pipe = usb_sndisocpipe(audev->udev, ep->bEndpointAddress);
        urb->transfer_flags = USB_ISO_ASAP;
        urb->complete = iso_out_irq;
        urb->interval = ep->bInterval;
        urb->number_of_packets = ISO_FRAME_COUNT;
        urb->transfer_buffer = &(audev->iso_outbuf[uidx][0]);
        urb->transfer_buffer_length = AU_OUT_MAX_PKTSZ * ISO_FRAME_COUNT;
        urb->actual_length = 0;

        for(i = 0; i < ISO_FRAME_COUNT; i++)
        {
            urb->iso_frame_desc[i].offset = i * AU_OUT_MAX_PKTSZ;
            urb->iso_frame_desc[i].length = audev->au_out_func(audev, (uint8_t *)urb->transfer_buffer + urb->iso_frame_desc[i].offset, ep->wMaxPacketSize);
            if(urb->iso_frame_desc[i].length > ep->wMaxPacketSize)
            {
                USBAS_ERRMSG("Audio-out packet buffer overrun!\n");
                return -1;
            }
            urb->iso_frame_desc[i].actual_length = urb->iso_frame_desc[i].length;
        }

        ret = USBH_SubmitUrb(urb);
        if(ret)
        {
            USBAS_DBGMSG("Error - failed to submit URB (%d)", ret);
            ret = UAC_RET_IO_ERR;
            goto err_out;
        }
    }
    audev->out_streaming = 1;

    return UAC_RET_OK;

err_out:
    for(i = 0; i < ISO_OUT_URB_CNT; i++)
    {
        if(audev->urbout[i])
        {
            USBH_UnlinkUrb(audev->urbout[i]);
            USBH_FreeUrb(audev->urbout[i]);
            audev->urbout[i] = NULL;
        }
    }
    audev->au_out_func = NULL;
    return ret;
}


/**
 *  @brief  Stop UAC device audio out data stream.
 *  @param[in] audev      Audio Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int32_t UAC_StopIsoOutPipe(UAC_DEV_T *audev)
{
    UAC_INFO_T  *uac_info = (UAC_INFO_T *)audev->priv;
    int    i;

    audev->out_streaming = 0;

    /* Set interface alternative settings */
    USBH_SetInterface(audev->udev, uac_info->ifd_play->bInterfaceNumber, 0);

    for(i = 0; i < ISO_OUT_URB_CNT; i++)
    {
        if(audev->urbout[i])
        {
            USBH_UnlinkUrb(audev->urbout[i]);
            USBH_FreeUrb(audev->urbout[i]);
            audev->urbout[i] = NULL;
        }
    }
    return UAC_RET_OK;
}


/*@}*/ /* end of group USBH_AS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group USBH_AS_Driver */

/*@}*/ /* end of group USB_Host_Driver */

/*@}*/ /* end of group Component_Library */



/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

