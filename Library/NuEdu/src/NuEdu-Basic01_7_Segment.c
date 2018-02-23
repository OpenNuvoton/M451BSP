/**************************************************************************//**
 * @file     NuEdu-Basic01_7_Segment.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:02a $
 * @brief    M451 series NuEdu Basic01 7-Segment driver source file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_7_Segment.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup M451_Device_Driver M451 Device Driver
  @{
*/

/** @addtogroup NuEdu-Basic01_7_Segment Driver
  @{
*/


/** @addtogroup NuEdu-Basic01_7_Segment Exported Functions
  @{
*/


/**
  * @brief  Configure the 7-Segment module
  *
  * @param[in]  None
  *
  * @return     None
  *
  * @details    Configure GPIOs of 7-Segment as output mode and turn off all 7-Segement..
  */
void Open_Seven_Segment(void)
{
    GPIO_SetMode(PB, BIT11, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT13, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT11, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT8, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT8, GPIO_MODE_OUTPUT);

    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
}


/**
  * @brief  Show number at 7-Segment module
  *
  * @param[in]  no The number that will be showed at the specified 7-Segment module.
  * @param[in]  number The group number of the specified 7-Segment module.
  *
  * @return     None
  *
  * @details    Configure GPIOs of 7-Segment to show number at 7-Segment module 
  */
void Show_Seven_Segment(unsigned char no, unsigned char number)
{
    SEG_A_OFF;
    SEG_B_OFF;
    SEG_C_OFF;
    SEG_D_OFF;
    SEG_E_OFF;
    SEG_F_OFF;
    SEG_G_OFF;
    SEG_H_OFF;
    SEG_CONTROL1_OFF;
    SEG_CONTROL2_OFF;
    switch(no)
    {
        //show 0
        case 0:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_D_ON;
            SEG_E_ON;
            SEG_F_ON;

            break;

        //show 1
        case 1:
            SEG_B_ON;
            SEG_C_ON;
            break;

        //show 2
        case 2:
            SEG_A_ON;
            SEG_B_ON;
            SEG_G_ON;
            SEG_E_ON;
            SEG_D_ON;
            break;

        //show 3
        case 3:
            SEG_A_ON;
            SEG_B_ON;
            SEG_G_ON;
            SEG_C_ON;
            SEG_D_ON;
            break;

        //show 4
        case 4:
            SEG_F_ON;
            SEG_B_ON;
            SEG_G_ON;
            SEG_C_ON;
            break;

        //show 5
        case 5:
            SEG_A_ON;
            SEG_F_ON;
            SEG_G_ON;
            SEG_C_ON;
            SEG_D_ON;
            break;

        //show 6
        case 6:
            SEG_A_ON;
            SEG_F_ON;
            SEG_E_ON;
            SEG_G_ON;
            SEG_C_ON;
            SEG_D_ON;
            break;

        //show 7
        case 7:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_F_ON;
            break;

        //show 8
        case 8:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_D_ON;
            SEG_E_ON;
            SEG_F_ON;
            SEG_G_ON;
            break;

        //show 9
        case 9:
            SEG_A_ON;
            SEG_B_ON;
            SEG_C_ON;
            SEG_F_ON;
            SEG_G_ON;
            break;
    }

    switch(number)
    {
        case 1:
            SEG_CONTROL1_ON;
            break;

        //show 1
        case 2:
            SEG_CONTROL2_ON;
            break;
    }
}


/*@}*/ /* end of group NuEdu-Basic01_7_Segment_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NuEdu-Basic01_7_Segment_Driver */

/*@}*/ /* end of group M451_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/


