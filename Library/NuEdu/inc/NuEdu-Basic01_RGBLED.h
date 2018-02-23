/**************************************************************************//**
 * @file     NuEdu-Basic01_RGBLED.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:02a $
 * @brief    NuEdu-Basic01 RGB LED driver header file for NuEdu-SDK-M451 
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __NuEdu_Basic01_RGBLED_H__
#define __NuEdu_Basic01_RGBLED_H__
/** @addtogroup M451_Library M451 Library
  @{
*/

/** @addtogroup NuEdu-SDK-M451_Basic01 M451_Basic01 Library
  @{
*/

/** @addtogroup M451_Basic01_FUNCTIONS RGB LED Functions
	@{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/  
extern void Initial_PWM_LED(void);
extern void PWM_LED(unsigned char ch, unsigned int ch0_fre, unsigned int ch0_dut, unsigned int ch1_fre, unsigned int ch1_dut, unsigned int ch2_fre, unsigned int ch2_dut);

#endif
/*@}*/ /* end of group M451_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-M451_Basic01 */

/*@}*/ /* end of group M451_Library */

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
