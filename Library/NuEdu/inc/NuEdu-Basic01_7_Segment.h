/**************************************************************************//**
 * @file     NuEdu-Basic01_7_Segment.h
 * @version  V0.10
 * $Revision: 4 $
 * $Date: 15/09/02 10:02a $
 * @brief    M451 series NuEdu Basic01 7-Segment driver Header File
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __NuEdu_Basic01_7_Segment_H__
#define __NuEdu_Basic01_7_Segment_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Include related headers                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#include "M451Series.h"

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


/** @addtogroup NuEdu-Basic01_7_Segment Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* 7-Segment GPIO constant definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define SEG_A_ON			PB11=0	/*!< 7_Segment setting for turning on 7-Segment A. */
#define SEG_B_ON			PB12=0	/*!< 7_Segment setting for turning on 7-Segment B. */
#define SEG_C_ON			PB13=0	/*!< 7_Segment setting for turning on 7-Segment C. */
#define SEG_D_ON			PB14=0	/*!< 7_Segment setting for turning on 7-Segment D. */
#define SEG_E_ON			PB15=0	/*!< 7_Segment setting for turning on 7-Segment E. */
#define SEG_F_ON			PB5=0	/*!< 7_Segment setting for turning on 7-Segment F. */
#define SEG_G_ON			PD11=0	/*!< 7_Segment setting for turning on 7-Segment G. */
#define SEG_H_ON			PF2=0	/*!< 7_Segment setting for turning on 7-Segment H. */
#define SEG_CONTROL1_ON		PD8=1	/*!< 7_Segment setting for turning on 7-Segment C1. */
#define SEG_CONTROL2_ON		PC8=1	/*!< 7_Segment setting for turning on 7-Segment C2. */


#define SEG_A_OFF			PB11=1	/*!< 7_Segment setting for turning off 7-Segment A. */
#define SEG_B_OFF			PB12=1	/*!< 7_Segment setting for turning off 7-Segment B. */
#define SEG_C_OFF			PB13=1	/*!< 7_Segment setting for turning off 7-Segment C. */
#define SEG_D_OFF			PB14=1	/*!< 7_Segment setting for turning off 7-Segment D. */
#define SEG_E_OFF			PB15=1	/*!< 7_Segment setting for turning off 7-Segment E. */
#define SEG_F_OFF			PB5=1	/*!< 7_Segment setting for turning off 7-Segment F. */
#define SEG_G_OFF			PD11=1	/*!< 7_Segment setting for turning off 7-Segment G. */
#define SEG_H_OFF			PF2=1	/*!< 7_Segment setting for turning off 7-Segment H. */
#define SEG_CONTROL1_OFF    PD8=0	/*!< 7_Segment setting for turning off 7-Segment C1. */
#define SEG_CONTROL2_OFF    PC8=0	/*!< 7_Segment setting for turning off 7-Segment C2. */

/*@}*/ /* end of group NuEdu-Basic01_7_Segment_EXPORTED_CONSTANTS */

/** @addtogroup NuEdu-Basic01_7_Segment Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Define Macros and functions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/* Function prototype declaration */
extern void Show_Seven_Segment(unsigned char no, unsigned char number);
extern void Open_Seven_Segment(void);


/*@}*/ /* end of group NuEdu-Basic01_7_Segment_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NuEdu-Basic01_7_Segment_Driver */

/*@}*/ /* end of group M451_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif//__NuEdu_Basic01_7_Segment_H__


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

