/***************************************************************************//**
 * @file     fmc_user.h
 * @brief    M451 series FMC driver header file
 * @version  2.0.0
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FMC_USER_H
#define FMC_USER_H

#include "targetdev.h"

#define Config0         FMC_CONFIG0_ADDR
#define Config1         FMC_CONFIG1_ADDR

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _FMC_ENABLE_CFG_UPDATE()   (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _FMC_DISABLE_CFG_UPDATE()  (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */


extern int EraseAP(unsigned int addr_start, unsigned int addr_end);
extern void UpdateConfig(unsigned int *data, unsigned int *res);
extern void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);
extern void WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);
extern void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
int FMC_Write_User(unsigned int u32Addr, unsigned int u32Data);
int FMC_Read_User(unsigned int u32Addr, unsigned int *data);
int FMC_Erase_User(unsigned int u32Addr);

#endif

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
