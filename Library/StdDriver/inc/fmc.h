/**************************************************************************//**
 * @file     FMC.h
 * @version  V2.1
 * $Revision: 19 $
 * $Date: 15/08/11 10:26a $
 * @brief    M451 Series Flash Memory Controller Driver Header File
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __FMC_H__
#define __FMC_H__

#include "M451Series.h"

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/

/** @addtogroup FMC_EXPORTED_CONSTANTS FMC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define ISBEN   0

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00000000UL    /*!< APROM  Base Address         */
#define FMC_LDROM_BASE          0x00100000UL    /*!< LDROM  Base Address         */
#define FMC_SPROM_BASE          0x00200000UL    /*!< SPROM  Base Address         */
#define FMC_CONFIG_BASE         0x00300000UL    /*!< CONFIG Base Address         */

#define FMC_CONFIG0_ADDR        (FMC_CONFIG_BASE)       /*!< CONFIG 0 Address */
#define FMC_CONFIG1_ADDR        (FMC_CONFIG_BASE + 4)   /*!< CONFIG 1 Address */


#define FMC_FLASH_PAGE_SIZE     0x800           /*!< Flash Page Size (2048 Bytes) */
#define FMC_LDROM_SIZE          0x1000          /*!< LDROM Size (4 kBytes)       */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCTL_BS_LDROM     0x2     /*!< ISPCTL setting to select to boot from LDROM */
#define FMC_ISPCTL_BS_APROM     0x0     /*!< ISPCTL setting to select to boot from APROM */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00     /*!< ISP Command: Read Flash               */
#define FMC_ISPCMD_PROGRAM      0x21     /*!< ISP Command: 32-bit Program Flash     */
#define FMC_ISPCMD_WRITE_8      0x61     /*!< ISP Command: 64-bit program Flash     */
#define FMC_ISPCMD_PAGE_ERASE   0x22     /*!< ISP Command: Page Erase Flash         */
#define FMC_ISPCMD_READ_CID     0x0B     /*!< ISP Command: Read Company ID          */
#define FMC_ISPCMD_READ_UID     0x04     /*!< ISP Command: Read Unique ID           */
#define FMC_ISPCMD_READ_DID     0x0C     /*!< ISP Command: Read Device ID           */
#define FMC_ISPCMD_VECMAP       0x2E     /*!< ISP Command: Set vector mapping       */
#define FMC_ISPCMD_CHECKSUM     0x0D     /*!< ISP Command: Read Checksum            */
#define FMC_ISPCMD_CAL_CHECKSUM 0x2D     /*!< ISP Command: Run Check Calculation    */
#define FMC_ISPCMD_MULTI_PROG   0x27     /*!< ISP Command: Flash Multi-Word Program */

/*---------------------------------------------------------------------------------------------------------*/
/*  FTCTL constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_FTCTL_OPTIMIZE_DISABLE      0x00       /*!< Frequency Optimize Mode disable */
#define FMC_FTCTL_OPTIMIZE_12MHZ        0x01       /*!< Frequency Optimize Mode <= 12Mhz */
#define FMC_FTCTL_OPTIMIZE_36MHZ        0x02       /*!< Frequency Optimize Mode <= 36Mhz */
#define FMC_FTCTL_OPTIMIZE_60MHZ        0x04       /*!< Frequency Optimize Mode <= 60Mhz */
#define FMC_FTCTL_OPTIMIZE_72MHZ        0x05       /*!< Frequency Optimize Mode <= 72Mhz */

/*---------------------------------------------------------------------------------------------------------*/
/* FMC Time-out Handler Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_TIMEOUT_READ            (SystemCoreClock>>3) /*!< Read command time-out 125 ms       */
#define FMC_TIMEOUT_WRITE           (SystemCoreClock>>3) /*!< Write command time-out 125 ms      */
#define FMC_TIMEOUT_ERASE           (SystemCoreClock>>2) /*!< Erase command time-out 250 ms      */
#define FMC_TIMEOUT_CHKSUM          (SystemCoreClock<<1) /*!< Get checksum command time-out 2 s  */
#define FMC_TIMEOUT_CHKALLONE       (SystemCoreClock<<1) /*!< Check-all-one command time-out 2 s */

/*@}*/ /* end of group FMC_EXPORTED_CONSTANTS */

extern int32_t g_FMC_i32ErrCode;

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief      Enable ISP Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set ISPEN bit of ISPCTL control register to enable ISP function.
 *
 */
#define FMC_ENABLE_ISP()          (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)  /*!< Enable ISP Function  */

/**
 * @brief      Disable ISP Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will clear ISPEN bit of ISPCTL control register to disable ISP function.
 *
 */
#define FMC_DISABLE_ISP()         (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)  /*!< Disable ISP Function */

/**
 * @brief      Enable LDROM Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set LDUEN bit of ISPCTL control register to enable LDROM update function.
 *             User needs to set LDUEN bit before they can update LDROM.
 *
 */
#define FMC_ENABLE_LD_UPDATE()    (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)  /*!< Enable LDROM Update Function   */

/**
 * @brief      Disable LDROM Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will clear LDUEN bit of ISPCTL control register to disable LDROM update function.
 *
 */
#define FMC_DISABLE_LD_UPDATE()   (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)  /*!< Disable LDROM Update Function  */

/**
 * @brief      Enable User Configuration Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set CFGUEN bit of ISPCTL control register to enable User Configuration update function.
 *             User needs to set CFGUEN bit before they can update User Configuration area.
 *
 */
#define FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */

/**
 * @brief      Disable User Configuration Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will clear CFGUEN bit of ISPCTL control register to disable User Configuration update function.
 *
 */
#define FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


/**
 * @brief      Enable APROM Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set APUEN bit of ISPCTL control register to enable APROM update function.
 *             User needs to set APUEN bit before they can update APROM in APROM boot mode.
 *
 */
#define FMC_ENABLE_AP_UPDATE()    (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)  /*!< Enable APROM Update Function   */

/**
 * @brief      Disable APROM Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will clear APUEN bit of ISPCTL control register to disable APROM update function.
 *
 */
#define FMC_DISABLE_AP_UPDATE()   (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)  /*!< Disable APROM Update Function  */

/**
 * @brief      Next Booting Selection function
 *
 * @param[in]  x   Booting from APROM(0)/LDROM(1)
 *
 * @return     None
 *
 * @details    This function will set MCU next booting from LDROM/APROM.
 *
 * @note       When use this macro, the Boot Loader booting selection MBS(CONFIG0[5]) must be set.
 *
 */
#define FMC_SELECT_NEXT_BOOT(x)   (FMC->ISPCTL = (FMC->ISPCTL & ~FMC_ISPCTL_BS_Msk) | ((x) << FMC_ISPCTL_BS_Pos)) /*!< Select Next Booting, x = 0 or 1 */

/**
 * @brief      Get MCU Booting Status
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will get status of chip next booting from LDROM/APROM.
 *
 */
#define FMC_GET_BOOT_STATUS()     ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk)?1:0) /*!< Get MCU Booting Status */

/*---------------------------------------------------------------------------------------------------------*/
/* inline functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data  32-bit Data to program
 *
 * @retval      0  Success
 * @retval     -1  Failed
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Program time-out
 *
 */
static __INLINE int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif

    u32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return -1;
        }
    }

    return 0;
}

/**
 * @brief      Program 64-bit data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data0 32-bit Data to program
 * @param[in]  u32Data1 32-bit Data to program
 *
 * @retval      0  Success
 * @retval     -1  Failed
 *
 * @details    To program two words data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Program time-out
 *
 */
static __INLINE int32_t FMC_Write8(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_WRITE_8;
    FMC->ISPADDR = u32Addr;
    FMC->MPDAT0 = u32Data0;
    FMC->MPDAT1 = u32Data1;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    u32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return -1;
        }
    }

    return 0;
}


/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Read time-out
 *
 */
static __INLINE uint32_t FMC_Read(uint32_t u32Addr)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    u32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32Addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 2048 bytes.
 *
 * @retval      0 Success
 * @retval     -1 Erase failed
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Erase failed or erase time-out
 *
 */
static __INLINE int32_t FMC_Erase(uint32_t u32Addr)
{
    int32_t i32ret = 0;
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    u32TimeOutCnt = FMC_TIMEOUT_ERASE;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            i32ret = -1;
        }
    }

    /* Check ISPFF flag to know whether erase OK or fail. */
    if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32ret = -1;
    }

    return i32ret;
}

/**
 * @brief       Read Unique ID
 *
 * @param[in]   u8Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 *
 * @return      The 32-bit unique ID data of specified UID index.
 *
 * @details     To read out 96-bit Unique ID.
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Read time-out
 *
 */
static __INLINE uint32_t FMC_ReadUID(uint8_t u8Index)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = (u8Index << 2);
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    u32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
  * @brief    Read company ID
  *
  * @param    None
  *
  * @return   The company ID (32-bit)
  *
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  *
  */
static __INLINE uint32_t FMC_ReadCID(void)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0;                          /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();                                     /* To make sure ISP/CPU be Synchronized */
#endif
    u32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)    /* Waiting for ISP Done */
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
  * @brief    Read product ID
  *
  * @param    None
  *
  * @return   The product ID (32-bit)
  *
  * @details  This function is used to read product ID.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  *
  */
static __INLINE uint32_t FMC_ReadPID(void)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04;                        /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */
#endif
    u32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)   /* Waiting for ISP Done */
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
  * @brief      To read UCID
  *
  * @param[in]  u32Index    Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  *
  * @return     The UCID of specified index
  *
  * @details    This function is used to read unique chip ID (UCID).
  *
  * @note       Global error code g_FMC_i32ErrCode
  *             -1  Read time-out
  *
  */
static __INLINE uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;          /* Set ISP Command Code */
    FMC->ISPADDR = (0x04 * u32Index) + 0x10;    /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();                                    /* To make sure ISP/CPU be Synchronized */
#endif
    u32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)   /* Waiting for ISP Done */
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
 * @brief       Set vector mapping address
 *
 * @param[in]   u32PageAddr  The page address to remap to address 0x0. The address must be page alignment.
 *
 * @retval      0   Success
 * @retval      -1  Failed
 *
 * @details     This function is used to set VECMAP to map specified page to vector page (0x0).
 *
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Command time-out
 *
 */
static __INLINE int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_VECMAP;   /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;        /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG = 0x1;                 /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();                           /* To make sure ISP/CPU be Synchronized */
#endif
    u32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPTRG)                 /* Waiting for ISP Done */
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return -1;
        }
    }

    return 0;
}

/**
 * @brief       Get current vector mapping address.
 *
 * @param       None
 *
 * @return      The current vector mapping address.
 *
 * @details     To get VECMAP value which is the page address for remapping to vector page (0x0).
 *
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 *
 */
static __INLINE uint32_t FMC_GetVECMAP(void)
{
    return (FMC->ISPSTS & FMC_ISPSTS_VECMAP_Msk);
}

/**
 * @brief       Get Flash Checksum
 *
 * @param[in]   u32Addr    Specific flash start address
 * @param[in]   i32Size    Specific a size of Flash area
 *
 * @return      A checksum value of a flash block.
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              -1  Command time-out
 *
 */
static __INLINE uint32_t FMC_GetCheckSum(uint32_t u32Addr, int32_t i32Size)
{
    uint32_t u32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_CAL_CHECKSUM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = i32Size;
    FMC->ISPTRG = 0x1;
#if ISBEN
    __ISB();
#endif
    u32TimeOutCnt = FMC_TIMEOUT_CHKSUM;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    FMC->ISPCMD = FMC_ISPCMD_CHECKSUM;
    FMC->ISPTRG = 0x1;
    u32TimeOutCnt = FMC_TIMEOUT_CHKSUM;
    while(FMC->ISPTRG)
    {
        if(--u32TimeOutCnt == 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
 * @brief      Program Multi-Word data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  pu32Buf  A data pointer is point to a data buffer start address;
 *
 * @retval     0   Success
 * @retval     -1  Program Failed
 *
 * @details    To program multi-words data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of Technical Reference Manual.
 *
 * @note       Global error code g_FMC_i32ErrCode
 *             -1  Program failed or time-out
 *
 */
static __INLINE int32_t FMC_Write256(uint32_t u32Addr, uint32_t *pu32Buf)
{
    int32_t i, idx;
    volatile uint32_t *pu32IspData;
    uint32_t u32TimeOutCnt;
    //int32_t i32Err;

    //i32Err = 0;
    g_FMC_i32ErrCode = 0;
    idx = 0;
    FMC->ISPCMD = FMC_ISPCMD_MULTI_PROG;
    FMC->ISPADDR = u32Addr;

retrigger:

    //if(i32Err)
    //    printf("idx=%d  ISPADDR = 0x%08x\n",idx, FMC->ISPADDR);

    FMC->MPDAT0 = pu32Buf[idx + 0];
    FMC->MPDAT1 = pu32Buf[idx + 1];
    FMC->MPDAT2 = pu32Buf[idx + 2];
    FMC->MPDAT3 = pu32Buf[idx + 3];



    FMC->ISPTRG = 0x1;

    pu32IspData = &FMC->MPDAT0;
    idx += 4;

    for(i = idx; i < 256 / 4; i += 4) // Max data length is 256 bytes (256/4 words)
    {

        __set_PRIMASK(1); // Mask interrupt to avoid status check coherence error
        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        do
        {
            if((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0)
            {
                __set_PRIMASK(0);
                //printf("%d %x\n", i, FMC->MPADDR);
                FMC->ISPADDR = FMC->MPADDR & (~0xful);
                idx = (FMC->ISPADDR - u32Addr) / 4;
                //i32Err = -1;
                goto retrigger;
            }

            if(--u32TimeOutCnt== 0)
            {
                __set_PRIMASK(0);
                g_FMC_i32ErrCode = -1;
                return -1;
            }
        }
        while(FMC->MPSTS & (3 << FMC_MPSTS_D0_Pos));

        // Update new data for D0
        pu32IspData[0] = pu32Buf[i  ];
        pu32IspData[1] = pu32Buf[i + 1];

        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        do
        {
            if((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0)
            {
                __set_PRIMASK(0);
                //printf("%d %x\n", i, FMC->MPADDR);
                FMC->ISPADDR = FMC->MPADDR & (~0xful);
                idx = (FMC->ISPADDR - u32Addr) / 4;
                //i32Err = -1;
                goto retrigger;
            }

            if(--u32TimeOutCnt== 0)
            {
                __set_PRIMASK(0);
                g_FMC_i32ErrCode = -1;
                return -1;
            }
        }
        while(FMC->MPSTS & (3 << FMC_MPSTS_D2_Pos));

        // Update new data for D2
        pu32IspData[2] = pu32Buf[i + 2];
        pu32IspData[3] = pu32Buf[i + 3];
        __set_PRIMASK(0);
    }

    u32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if(--u32TimeOutCnt== 0)
        {
            g_FMC_i32ErrCode = -1;
            return -1;
        }
    }

    return 0;
}

void FMC_Open(void);
void FMC_Close(void);
void FMC_EnableAPUpdate(void);
void FMC_DisableAPUpdate(void);
void FMC_EnableConfigUpdate(void);
void FMC_DisableConfigUpdate(void);
void FMC_EnableLDUpdate(void);
void FMC_DisableLDUpdate(void);
int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
void FMC_SetBootSource(int32_t i32BootSrc);
int32_t FMC_GetBootSource(void);
uint32_t FMC_ReadDataFlashBaseAddr(void);
void FMC_EnableFreqOptimizeMode(uint32_t u32Mode);
void FMC_DisableFreqOptimizeMode(void);
/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group FMC_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif


#endif

