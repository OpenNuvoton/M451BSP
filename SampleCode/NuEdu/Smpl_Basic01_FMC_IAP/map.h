/**************************************************************************//**
 * @file     map.h
 * @version  V1.00
 * $Revision 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    FMC VECMAP sample program header file
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __MAP_H__
#define __MAP_H__
#define FMC_APROM_END           0x0001ffffUL    /*!< APROM End Address           */
#define USER_AP0_ENTRY           FMC_APROM_BASE
#define USER_AP1_ENTRY          (60*1024)
#define USER_AP1_MAX_SIZE       (FMC_APROM_END-USER_AP1_ENTRY)
#define LD_BOOT_CODE_ENTRY      FMC_LDROM_BASE

typedef void (FUNC_PTR)(void);
extern uint32_t ApImage1Base, ApImage1Limit;
extern uint32_t loaderImage1Base, loaderImage1Limit;
extern uint32_t loaderImage2Base, loaderImage2Limit;

#endif  // __MAP_H__

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/

