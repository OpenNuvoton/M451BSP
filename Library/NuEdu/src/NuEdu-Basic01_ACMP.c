#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_ACMP.h"

void Open_ACMP(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD.9 multi-function pin for ACMP1 positive input pin */
    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD9MFP_Msk;
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD9MFP_ACMP1_P3;

    /* Set PD.0 multi-function pin for ACMP1 negative input pin */
    SYS->GPD_MFPL &= ~SYS_GPD_MFPL_PD0MFP_Msk;
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_ACMP1_N;

    /* Disable digital input path of analog pin ACMP1_P3 and ACMP1_N to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PD, (1ul << 9));
    GPIO_DISABLE_DIGITAL_PATH(PD, (1ul << 0));

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Configure ACMP1. Enable ACMP1 and select ACMP1_N as negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_PIN, ACMP_CTL_HYSTERESIS_DISABLE);
    /* select ACMP1_p3 as positive input pin */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P3);
}

uint32_t Get_ACMP(void)
{
    uint32_t ACMP1_Output_Level;

    /* Get ACMP1 output value */
    ACMP1_Output_Level = (ACMP_GET_OUTPUT(ACMP01, 1));      //((acmp)->STATUS & (ACMP_STATUS_ACMPO0_Msk<<((u32ChNum)%2)))?1:0

    /* Clear ACMP1 Flags */
    ACMP_CLR_INT_FLAG(ACMP01, 1);               //ACMP->STATUS = (ACMP_STATUS_ACMPIF0_Msk<<(1%2))

    return ACMP1_Output_Level;
}
