/**************************************************************************//**
 * @file     NuEdu-NuEdu-Basic01_RGBLED.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:02a $
 * @brief    NuEdu-Basic01_RGBLED driver source file for NuEdu-SDK-M451 
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_RGBLED.h"

/** @addtogroup M451_Library M451 Library
  @{
*/

/** @addtogroup NuEdu-SDK-M451_Basic01 M451_Basic01 Library
  @{
*/

/** @addtogroup M451_Basic01_FUNCTIONS RGB LED Functions
  @{
*/

/**
 * @brief       Set multi-function pins for PWM1 channel 0,1,2
 * @return      None
 */
void Initial_PWM_LED(void)
{
    /* Set PC9~PC11 multi-function pins for PWM1 Channel0~2  */
    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC9MFP_Msk | SYS_GPC_MFPH_PC10MFP_Msk | SYS_GPC_MFPH_PC11MFP_Msk);
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC9MFP_PWM1_CH0 | SYS_GPC_MFPH_PC10MFP_PWM1_CH1 | SYS_GPC_MFPH_PC11MFP_PWM1_CH2;

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Reset PWM1 channel 0~5 */
    SYS_ResetModule(PWM1_RST);
}

/**
 * @brief       Set PWM clock enable and HCLK as PWM clock source,
  *
 * @param[in]   ch		Channel numbers that will be enabled.
 *									
 * @param[in]   ch0_fre		Channel 0 frequency.
 *									
 * @param[in]   ch0_dut		Channel 0 duty.
 *
 * @param[in]   ch1_fre		Channel 1 frequency.
 *									
 * @param[in]   ch1_dut		Channel 1 duty.
 *
 * @param[in]   ch2_fre		Channel 2 frequency.
 *									
 * @param[in]   ch2_dut		Channel 2 duty.
 *
 * @return      None
 */
void PWM_LED(unsigned char ch, unsigned int ch0_fre, unsigned int ch0_dut, unsigned int ch1_fre, unsigned int ch1_dut, unsigned int ch2_fre, unsigned int ch2_dut)
{
    /* set PWMA channel 1 output configuration */
    PWM_ConfigOutputChannel(PWM1, 0, ch0_fre, ch0_dut);
    PWM_ConfigOutputChannel(PWM1, 1, ch1_fre, ch1_dut);
    PWM_ConfigOutputChannel(PWM1, 2, ch2_fre, ch2_dut);

    /* Enable PWM Output path for PWMA channel 0 */
    PWM_EnableOutput(PWM1, ch);

    // Start
    PWM_Start(PWM1, ch);
}

/*@}*/ /* end of group M451_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-M451_Basic01 */

/*@}*/ /* end of group M451_Library */

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. **/

