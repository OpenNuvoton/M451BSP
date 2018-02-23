#ifndef _USB_ERR_CODE_H_
#define _USB_ERR_CODE_H_

#define USB_OK                  0
#define USB_ERR_NOENT           0xf0001002      /* No such file or directory */
#define USB_ERR_IO              0xf0001005      /* I/O error */
#define USB_ERR_AGAIN           0xf0001011      /* Try again */
#define USB_ERR_NOMEM           0xf0001012      /* Out of memory */
#define USB_ERR_BUSY            0xf0001016      /* Device or resource busy */
#define USB_ERR_XDEV            0xf0001018      /* Cross-device link */
#define USB_ERR_NODEV           0xf0001019      /* No such device */
#define USB_ERR_INVAL           0xf0001022      /* Invalid argument */
#define USB_ERR_PIPE            0xf0001032      /* Broken pipe */
#define EWOULDBLOCK             USB_ERR_AGAIN   /* Operation would block */
#define USB_ERR_NOLINK          0xf0001067      /* Link has been severed */
#define USB_ERR_CONNRESET       0xf0001104      /* Connection reset by peer */
#define USB_ERR_SHUTDOWN        0xf0001108      /* Cannot send after transport endpoint shutdown */
#define USB_ERR_TIMEOUT         0xf0001110      /* Connection timed out */
#define USB_ERR_INPROGRESS      0xf0001115      /* Operation now in progress */

#define USB_ERR_URB_PENDING     USB_ERR_INPROGRESS

#define CC_ERR_CRC              0xf0003001      /* OHCI CC - CRC Error */
#define CC_ERR_BITSTUFF         0xf0003002      /* OHCI CC - Bit Stuff */
#define CC_ERR_DATA_TOGGLE      0xf0003003      /* OHCI CC - Data Togg */
#define CC_ERR_STALL            0xf0003004      /* OHCI CC - Stall */
#define CC_ERR_NORESPONSE       0xf0003005      /* OHCI CC - Device Not Responding */
#define CC_ERR_PID_CHECK        0xf0003006      /* OHCI CC - PID Check Failure */
#define CC_ERR_INVALID_PID      0xf0003007      /* OHCI CC - Invalid PID */
#define CC_ERR_DATAOVERRUN      0xf0003008      /* OHCI CC - Data Overrun */
#define CC_ERR_DATAUNDERRUN     0xf0003009      /* OHCI CC - Data Underrun */
#define CC_ERR_NOT_DEFINED      0xf0003010
#define CC_ERR_BUFFEROVERRUN    0xf0003012      /* OHCI CC - Buffer Overrun */
#define CC_ERR_BUFFERUNDERRUN   0xf0003013      /* OHCI CC - Buffer Underrun */
#define CC_ERR_NOT_ACCESS       0xf0003014      /* OHCI CC - Not Access */

#endif  /* _USB_ERR_CODE_H_ */
