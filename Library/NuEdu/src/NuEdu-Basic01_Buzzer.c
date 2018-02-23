#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_Buzzer.h"

void Open_Buzzer(void)
{
    /* Set PC14 multi-function pins for PWM1 Channel0~2  */
    SYS->GPC_MFPH &= ~SYS_GPC_MFPH_PC14MFP_Msk;
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC14MFP_PWM1_CH5;

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Reset PWM1 channel 0~5 */
    SYS_ResetModule(PWM1_RST);
}

void Write_Buzzer(unsigned Enable, unsigned  int Frequence, unsigned int Duty)
{
    /* set PWM1 channel 5 output configuration */
    PWM_ConfigOutputChannel(PWM1, 5, Frequence, Duty);

    // Start
    PWM_Start(PWM1, 1 << 5);
    if(Enable == 1)
        /* Enable PWM Output path for PWM1 channel 5 */
        PWM_EnableOutput(PWM1, 1 << 5);
    else
        PWM_DisableOutput(PWM1, 1 << 5);
}



