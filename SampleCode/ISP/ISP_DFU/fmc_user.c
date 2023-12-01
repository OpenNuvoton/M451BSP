/***************************************************************************//**
 * @file     fmc_user.c
 * @brief    M451 series FMC driver source file
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "fmc_user.h"


int FMC_Proc(unsigned int u32Cmd, unsigned int addr_start, unsigned int addr_end, unsigned int *data)
{
    unsigned int u32Addr, Reg;
    uint32_t u32TimeOutCnt;

    for(u32Addr = addr_start; u32Addr < addr_end; data++, u32Addr += 4)
    {
        FMC->ISPADDR = u32Addr;

        if((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0 && u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPTRG = 0x1;

            u32TimeOutCnt = FMC_TIMEOUT_ERASE;
            while(FMC->ISPTRG & 0x1)
            {
                if(--u32TimeOutCnt == 0)
                    return -1;
            }
        }

        FMC->ISPCMD = u32Cmd;

        if(u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *data;
        }

        FMC->ISPTRG = 0x1;
        //        __ISB();

        /* Wait for ISP command done. */
        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while(FMC->ISPTRG & 0x1)
        {
            if(--u32TimeOutCnt == 0)
                return -1;
        }

        Reg = FMC->ISPCTL;

        if(Reg & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL = Reg;
            return -1;
        }

        if(u32Cmd == FMC_ISPCMD_READ)
        {
            *data = FMC->ISPDAT;
        }

    }

    return 0;
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function. User can check the status of
 *              Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Read_User(uint32_t u32Addr, uint32_t *data)
{
    return FMC_Proc(FMC_ISPCMD_READ, u32Addr, u32Addr + 4, data);
}

void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)    // Read data from flash
{
    FMC_Proc(FMC_ISPCMD_READ, addr_start, addr_end, data);
    return;
}

void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)  // Write data into flash
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, addr_start, addr_end, data);
    return;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
