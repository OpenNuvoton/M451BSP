/***************************************************************************//**
 * @file     fmc_user.h
 * @brief    M451 series FMC driver header file
 * @version  2.0.0
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FMC_USER_H
#define FMC_USER_H

#include "M451Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ISPGO           0x01


int FMC_Read_User(uint32_t u32Addr, uint32_t *data);
void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);

#endif  /* FMC_USER_H */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
