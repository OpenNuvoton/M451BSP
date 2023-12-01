/******************************************************************************
 * @file     descriptors.c
 * @brief    NuMicro series USBD descriptor file
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "M451Series.h"
#include "usbd_audio.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0xEF,           /* bDeviceClass */
    0x02,           /* bDeviceSubClass */
    0x01,           /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF,
    (USBD_PID & 0xFF00) >> 8,
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01            /* bNumConfigurations */
};
#if(HID_FUNCTION == HID_KEYBOARD)

const uint8_t gu8KeyboardReportDesc[] =
{
    0x05, 0x01,
    0x09, 0x06,
    0xA1, 0x01,
    0x05, 0x07,
    0x19, 0xE0,
    0x29, 0xE7,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,
    0x95, 0x01,
    0x75, 0x08,
    0x81, 0x01,
    0x95, 0x05,
    0x75, 0x01,
    0x05, 0x08,
    0x19, 0x01,
    0x29, 0x05,
    0x91, 0x02,
    0x95, 0x01,
    0x75, 0x03,
    0x91, 0x01,
    0x95, 0x06,
    0x75, 0x08,
    0x15, 0x00,
    0x25, 0x65,
    0x05, 0x07,
    0x19, 0x00,
    0x29, 0x65,
    0x81, 0x00,
    0xC0
};


#endif

#if(HID_FUNCTION == HID_CONSUMER)

const uint8_t gu8KeyboardReportDesc[] =
{
    0x05, 0x0C, // Usage Page (Consumer)
    0x09, 0x01, // Usage(Consumer Control)
    0xA1, 0x01, // Collection(Application )
    0x15, 0x00, // Logical Minimum(0x0 )
    0x25, 0x01, // Logical Maximum(0x1 )
    0x09, 0xE2, // Usage(Mute)
    0x09, 0xE9, // Usage(Volume Increment)
    0x09, 0xEA, // Usage(Volume Decrement)
    0x75, 0x01, // Report Size(0x1 )
    0x95, 0x03, // Report Count(0x3 )
    0x81, 0x02, // Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
    0x75, 0x01, // Report Size(0x1 )
    0x95, 0x05, // Report Count(0x5 )
    0x81, 0x03, // Input(Constant, Variable, Absolute)
    0x09, 0xB0, // Usage(Play)
    0x09, 0xB7, // Usage(Stop)
    0x09, 0xCD, // Usage(Play/Pause)
    0x09, 0xB5, // Usage(Scan Next Track)
    0x09, 0xB6, // Usage(Scan Previous Track)
    0x09, 0xB2, // Usage(Record)
    0x09, 0xB4, // Usage(Rewind)
    0x09, 0xB3, // Usage(Fast Forward)
    0x75, 0x01, // Report Size(0x1 )
    0x95, 0x08, // Report Count(0x8 )
    0x81, 0x02, // Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
    0x09, 0x00, // Usage(Undefined)
    0x75, 0x08, // Report Size(0x8 )
    0x95, 0x06, // Report Count(0x6 )
    0x81, 0x02, // Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
    0x09, 0x00, // Usage(Undefined)
    0x75, 0x08, // Report Size(0x8 )
    0x95, 0x08, // Report Count(0x8 )
    0x91, 0x00,
    0xC0
};



#endif
#define HID_KEYBOARD_REPORT_DESC_SIZE \
    sizeof(gu8KeyboardReportDesc) / sizeof(gu8KeyboardReportDesc[0])
const uint32_t gu32KeyboardReportDescSize = HID_KEYBOARD_REPORT_DESC_SIZE;

#define HID_REPORT_DESCRIPTOR_SIZE   HID_KEYBOARD_REPORT_DESC_SIZE


/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    0xEC, 0x00,     /* wTotalLength */
    0x04,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80,           /* bmAttributes */
    0x20,           /* Max power */

    // IAD
    0x08,   // bLength: Interface Descriptor size
    0x0B,   // bDescriptorType: IAD
    0x00,   // bFirstInterface
    0x03,   // bInterfaceCount
    0x01,   // bFunctionClass(0x01): AUDIO */
    0x03,   /* bFunctionSubClass */
    0x20,   /* bFunctionProtocol */
    0x00,   // iFunction

    /* Standard AC inteface */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x00,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x01,           /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Class-spec AC interface descriptor */
    0x0A,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x01,           /* bDescriptorSubType:HEADER */
    0x00, 0x01,     /* bcdADC:1.0 */
    0x46, 0x00,     /* wTotalLength */
    0x02,           /* bInCollection */
    0x01,           /* baInterfaceNr(1) */
    0x02,           /* baInterfaceNr(n) */

    /* TID 1: Input for usb streaming */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,               /* bTerminalID */
    0x01, 0x01,         /* wTerminalType: 0x0101 usb streaming */
    0x00,               /* bAssocTerminal */
    PLAY_CHANNELS,      /* bNrChannels */
    PLAY_CH_CFG, 0x00,  /* wChannelConfig */
    0x00,               /* iChannelNames */
    0x00,               /* iTerminal */

    /* UNIT ID 5: Feature Unit */
    0x0A,               /* bLength */
    0x24,               /* bDescriptorType */
    0x06,               /* bDescriptorSubType */
    REC_FEATURE_UNITID, /* bUnitID */
    0x04,               /* bSourceID */
    0x01,               /* bControlSize */
    0x01,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x00,               /* iFeature */

    /* TID 2: Output Terminal for usb streaming */
    0x09,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x02,               /* bTerminalID */
    0x01, 0x01,         /* wTerminalType: 0x0101 usb streaming */
    0x00,               /* bAssocTerminal */
    REC_FEATURE_UNITID, /* bSourceID */
    0x00,               /* iTerminal */

    /* UNIT ID 6: Feature Unit */
    0x0A,               /* bLength */
    0x24,               /* bDescriptorType */
    0x06,               /* bDescriptorSubType */
    PLAY_FEATURE_UNITID, /* bUnitID */
    0x01,               /* bSourceID */
    0x01,               /* bControlSize */
    0x01,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x00,               /* iFeature */

    /* TID 3: Output for speaker */
    0x09,           /* bLength*/
    0x24,           /* bDescriptorType:CS_INTERFACE*/
    0x03,           /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x03,           /* bTerminalID*/
    0x01, 0x03,     /* wTerminalType: 0x0301 speaker*/
    0x00,           /* bAssocTerminal*/
    0x06, /* bSourceID*/
    0x00,           /* iTerminal*/

    /* TID 4: Input Terminal for microphone */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,               /* bTerminalID*/
    0x01, 0x02,         /* wTerminalType: 0x0201 microphone*/
    0x00,               /* bAssocTerminal*/
    REC_CHANNELS,       /* bNrChannels*/
    REC_CH_CFG, 0x00,   /* wChannelConfig*/
    0x00,               /* iChannelNames*/
    0x00,               /* iTerminal*/

    /* Standard AS interface 1, alternate 0 */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x00,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Standard AS interface 1, alternate 1 */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x01,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Class-spec AS interface, this interface's endpoint connect to TID 0x02 */
    0x07,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x01,           /* bDescriptorSubType:AS_GENERAL */
    0x02,           /* bTernimalLink */
    0x01,           /* bDelay */
    0x01, 0x00,     /* wFormatTag:0x0001 PCM */

    /* Type I format type Descriptor */
    0x0B,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x02,           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,           /* bFormatType:FORMAT_TYPE_I */
    REC_CHANNELS,   /* bNrChannels */
    0x02,           /* bSubFrameSize */
    0x10,           /* bBitResolution */
    0x01,           /* bSamFreqType : 0 continuous; 1 discrete */
    REC_RATE_LO,
    REC_RATE_MD,
    REC_RATE_HI,    /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint */
    0x09,                       /* bLength */
    0x05,                       /* bDescriptorType */
    ISO_IN_EP_NUM | EP_INPUT,   /* bEndpointAddress */
    0x0d,                       /* bmAttributes */
    (EP2_MAX_PKT_SIZE & 0xFF), ((EP2_MAX_PKT_SIZE >> 8) & 0xFF), /* wMaxPacketSize*/
    0x01,                       /* bInterval*/
    0x00,                       /* bRefresh*/
    0x00,                       /* bSynchAddress*/

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,           /* bLength */
    0x25,           /* bDescriptorType:CS_ENDPOINT */
    0x01,           /* bDescriptorSubType:EP_GENERAL */
    0x00,           /* bmAttributes */
    0x00,           /* bLockDelayUnits */
    0x00, 0x00,     /* wLockDelay */

    /* Standard AS interface 2, alternate 0 */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x02,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x00,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Standard AS interface 2, alternate 1 */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x02,           /* bInterfaceNumber */
    0x01,           /* bAlternateSetting */
    0x02,    // 2 EP    /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Class-spec AS inf this interface's endpoint connect to TID 0x01 */
    0x07,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x01,           /* bDescriptorSubType:AS_GENERAL */
    0x01,           /* bTernimalLink */
    0x01,           /* bDelay */
    0x01, 0x00,     /* wFormatTag:0x0001 PCM */

    /* Type I format type Descriptor */
    0x0B,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x02,           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,           /* bFormatType:FORMAT_TYPE_I */
    PLAY_CHANNELS,  /* bNrChannels */
    0x02,           /* bSubFrameSize */
    0x10,           /* bBitResolution */
    0x01,           /* bSamFreqType : 0 continuous; 1 discrete */
    PLAY_RATE_LO,
    PLAY_RATE_MD,
    PLAY_RATE_HI,   /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint, output, addtess 2, Max 0x40 */
    0x09,                       /* bLength */
    0x05,                       /* bDescriptorType */
    ISO_OUT_EP_NUM | EP_OUTPUT, /* bEndpointAddress */
    0x05,  //Async Sink                     /* bmAttributes */
    EP3_MAX_PKT_SIZE, 0x00,     /* wMaxPacketSize */
    0x01,                       /* bInterval */
    0x00,                       /* bRefresh */
    ISO_IN_FB_EP_NUM | EP_INPUT, //0x00,   /* bSynchAddress */

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,           /* bLength */
    0x25,           /* bDescriptorType:CS_ENDPOINT */
    0x01,           /* bDescriptorSubType:EP_GENERAL */
    0x80,           /* bmAttributes */
    0x00,           /* bLockDelayUnits */
    0x00, 0x00,     /* wLockDelay */
    // feedback endpoint for Ray added
    0x09,                       /* bLength */
    0x05,                       /* bDescriptorType */
    ISO_IN_FB_EP_NUM | EP_INPUT, /* bEndpointAddress */
    0x01,                       /* bmAttributes */
    EP5_MAX_PKT_SIZE, 0x00,     /* wMaxPacketSize */
    0x01,                       /* bInterval */
    REFRESH,                           /* bRefresh rate = 2^n (0x07 ==> 128ms)  */
    0x00,                       /* bSynchAddress */

    //------------------------------------------------------------------------
    /* I/F descr: HID */
    LEN_INTERFACE,  // bLength
    DESC_INTERFACE, // bDescriptorType
    0x03,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x01,           // bNumEndpoints
    0x03,           // bInterfaceClass
    0x01,           // bInterfaceSubClass
    0x01,           // bInterfaceProtocol
    0x00,           // iInterface

    // HID Descriptor
    LEN_HID,        // Size of this descriptor in UINT8s.
    DESC_HID,       // HID descriptor type.
    0x10, 0x01,     // HID Class Spec. release number.
    0x00,           // H/W target country.
    0x01,           // Number of HID class descriptors to follow.
    DESC_HID_RPT,   // Dscriptor type.

    /* Total length of report descriptor */
    HID_REPORT_DESCRIPTOR_SIZE & 0x00FF,
    (HID_REPORT_DESCRIPTOR_SIZE & 0xFF00) >> 8,

    /* EP Descriptor: interrupt in */
    LEN_ENDPOINT,   // bLength
    DESC_ENDPOINT,  // bDescriptorType
    (HID_IN_EP_NUM | EP_INPUT),         // bEndpointAddress
    EP_INT,         // bmAttributes
    // wMaxPacketSize
    EP4_MAX_PKT_SIZE & 0x00FF,
    (EP4_MAX_PKT_SIZE & 0xFF00) >> 8,
    10              // bInterval
};

/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] =
{
    36,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'A', 0, 'u', 0, 'd', 0, 'i', 0, 'o', 0,
    ' ', 0, 'a', 0, 'n', 0, 'd', 0, ' ', 0,
    'H', 0, 'I', 0, 'D', 0
};

const uint8_t gu8StringSerial[26] =
{
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '4', 0, '1', 0, '0', 0, '0', 0, '2', 0, '0', 0, '1', 0
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gpu8UsbHidReport[4] =
{
    NULL,
    NULL,
    NULL,
    gu8KeyboardReportDesc
};

const uint32_t gu32UsbHidReportLen[4] =
{
    0,
    0,
    0,
    sizeof(gu8KeyboardReportDesc)
};

const uint32_t gu32ConfigHidDescIdx[4] =
{
    0,
    0,
    0,
    (sizeof(gu8ConfigDescriptor) - LEN_ENDPOINT - LEN_HID)
};

const S_USBD_INFO_T gsInfo =
{
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gpu8UsbHidReport,
    gu32UsbHidReportLen,
    gu32ConfigHidDescIdx
};

