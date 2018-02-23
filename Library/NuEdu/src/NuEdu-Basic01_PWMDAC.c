#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_PWMDAC.h"


void Write_PWMDAC(unsigned char Enable, unsigned char ch0_dut)
{
    /* set PWMB channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM1, 4, 1000, ch0_dut);

    // Start PWM COUNT
    PWM_Start(PWM1, 1 << 4);

    if(Enable == 0)
        /* Enable PWM Output path for PWMB channel 0 */
        PWM_DisableOutput(PWM1, 1 << 4);
    else
        /* Diable PWM Output path for PWMB channel 0 */
        PWM_EnableOutput(PWM1, 1 << 4);
}


void Initial_PWM_DAC(void)
{
    GPIO_SetMode(PC, BIT13, GPIO_MODE_INPUT); //avoid to pwm dac out
    SYS->GPC_MFPH &= ~SYS_GPC_MFPH_PC13MFP_Msk ;
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC13MFP_PWM1_CH4;

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Reset PWM1 channel 0~5 */
    SYS_ResetModule(PWM1_RST);
}

