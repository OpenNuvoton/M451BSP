/**************************************************************************//**
 * @file     usbh_config.h
 * @version  V1.00
 * $Revision 2 $
 * $Date: 17/07/04 3:24p $
 * @brief    USB Host core configuration file
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  _USB_CONFIG_H_
#define  _USB_CONFIG_H_


#ifdef __ICCARM__
#define __inline    inline
#endif


/*
 *  Debug messages...
 */
//#define USB_DEBUG                       /*!< Enable debug message                       */
//#define USB_VERBOSE_DEBUG
//#define DUMP_DEV_DESCRIPTORS


/*
 *  Static Memory Settings...
 */
#define DEV_MAX_NUM             8       /*!< Maximum number of connected devices        */
#define URB_MAX_NUM             12      /*!< Maximum number of URBs in memory pool      */
#define ED_MAX_NUM              12      /*!< Maximum number of OHCI EDs in memory pool  */
#define TD_MAX_NUM              32      /*!< Maximum number of OHCI TDs in memory pool  */

#define MAX_ENDPOINTS           16      /*!< Maximum number of endpoints per device     */
#define MAX_DRIVER_PER_DEV      3       /*!< Maximum number of drivers for a device     */
#define MAX_TD_PER_OHCI_URB     8       /*!< Maximum number of OHCI TDs per URB         */
#define MAX_HUB_DEVICE          2       /*!< Maximum number of connected Hub devices    */


#define ISO_FRAME_COUNT         1       /*!< Transfer frames per Isohronous TD \hideinitializer     */
#define OHCI_ISO_DELAY          8       /*!< Delay isochronous transfer frame time      */

/*
 * Class driver support...
 */
#define SUPPORT_HUB_CLASS               /*!< Support Hub driver                         */


/// @cond HIDDEN_SYMBOLS

/*
 *  Debug/Warning/Information to be printed on console or not
 */
#define USB_error               printf
#ifdef USB_DEBUG
#define USB_debug               printf
#else
#define USB_debug(...)
#endif

#ifdef USB_VERBOSE_DEBUG
#define USB_warning             printf
#define USB_info                printf
#else
#define USB_warning(...)
#define USB_info(...)
#endif

#define DISABLE_USB_INT()       NVIC_DisableIRQ(USBH_IRQn);
#define ENABLE_USB_INT()        NVIC_EnableIRQ(USBH_IRQn);


/*
 * I/O
 */
#define SUPPORT_HUB_CLASS
#define OHCI_BASE_ADDR          0x40009000

#define readl(addr)             (*(volatile uint32_t *)(addr))
#define writel(x,addr)          ((*(volatile uint32_t *)(addr)) = (volatile uint32_t)x)

//#define inpw(addr)            (*(volatile uint32_t *)(addr))

#define USB_JIFFY               (OHCI->FMNUM & 0xffff)


/*---  CPU clock speed ---*/
#define HZ                      (72)

#define USB_SWAP16(x)           (((x>>8)&0xff)|((x&0xff)<<8))
#define USB_SWAP32(x)           (((x>>24)&0xff)|((x>>8)&0xff00)|((x&0xff00)<<8)|((x&0xff)<<24))

/// @endcond HIDDEN_SYMBOLS

#endif  /* _USB_CONFIG_H_ */


