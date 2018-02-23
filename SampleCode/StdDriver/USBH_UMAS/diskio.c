/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various existing      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "M451Series.h"
#include "diskio.h"     /* FatFs lower layer API */
#include "usbh_core.h"
#include "usbh_umas.h"

#define SUPPORT_USBH
//#define SUPPORT_SDIO


/* Definitions of physical drive number for each media */
#define DRV_USBH    0
#define DRV_SD      1
#define DRV_FMC     2


/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(
    BYTE pdrv               /* Physical drive number (0..) */
)
{
    switch(pdrv)
    {

#ifdef SUPPORT_USBH
        case DRV_USBH :
            USBH_ProcessHubEvents();
            // translate the result code here
            return RES_OK;
#endif

        case DRV_SD :
            return RES_PARERR;
    }

    return RES_PARERR;
}


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
    BYTE pdrv       /* Physical drive number (0..) */
)
{
    switch(pdrv)
    {

#ifdef SUPPORT_USBH
        case DRV_USBH :
            return usbh_umas_disk_status();
#endif

        case DRV_SD :
            break;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Sector address (LBA) */
    BYTE count      /* Number of sectors to read (1..128) */
)
{
    switch(pdrv)
    {

#ifdef SUPPORT_USBH
        case DRV_USBH :
            return usbh_umas_read(buff, sector, count);
#endif

        case DRV_SD :
            break;

    }
    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write(
    BYTE pdrv,          /* Physical drive number (0..) */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Sector address (LBA) */
    BYTE count          /* Number of sectors to write (1..128) */
)
{
    switch(pdrv)
    {

#ifdef SUPPORT_USBH
        case DRV_USBH :
            return usbh_umas_write((uint8_t *)buff, sector, count);
#endif

        case DRV_SD :
            break;
    }
    return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    switch(pdrv)
    {

#ifdef SUPPORT_USBH
        case DRV_USBH :
            return usbh_umas_ioctl(cmd, buff);
#endif

        case DRV_SD :
            break;

    }
    return RES_PARERR;
}
#endif
