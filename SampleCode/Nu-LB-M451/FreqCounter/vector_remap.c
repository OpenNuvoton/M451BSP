/**************************************************************************//**
 * @file     vector_remap.c
 * @version  V1.0
 * $Revision: 2 $
 * $Date: 15/02/03 10:41a $
 * @brief    Support to remap vector map to speed up interrupt handler speed.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "M451Series.h"

__align(256) uint32_t g_u32VectorEntry[80] = {0};

void VectorRemap(void)
{
    int32_t i;
    uint32_t *pu32;

    /* Copy interrupt vectors to SRAM to speed up interrupt processing.
       It is used only when application is booting from SPI Flash and
       executed in SRAM */
    pu32 = 0;
    for(i = 0; i < 80; i++)
        g_u32VectorEntry[i] = pu32[i];

    /* Set CPU vector base address to the vector entry in SRAM */
    SCB->VTOR = (uint32_t)&g_u32VectorEntry[0];

}


/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
