#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_ADC_Knob.h"

volatile uint32_t g_u32AdcIntFlag;
void Open_ADC_Knob(void)
{
    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is HCLK(72MHz), set divider to 8, ADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Configure the GPB9 for ADC analog input pins.  */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB9MFP_EADC_CH6 | SYS_GPB_MFPH_PB10MFP_EADC_CH7;

    /* Disable the GPB9 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT9);
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT10);

    /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);
    EADC_SetInternalSampleTime(EADC, 6);

    /* Configure the sample module 0 for analog input channel 6 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 6);
    EADC_ConfigSampleModule(EADC, 1, EADC_SOFTWARE_TRIGGER, 7);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);//Enable sample module 0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, 0x2);//Enable sample module 0 interrupt.

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x3);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC, 0x3);//Enable sample module A/D ADINT0 interrupt.
}



uint32_t Get_ADC_Knob(void)
{
    uint32_t ADC_Raw_Data;
    /* Clear the A/D ADI NT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    //Trigger sample module 0 to start A/D conversion
    EADC_START_CONV(EADC, 0x1);

    /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    while(EADC_GET_INT_FLAG(EADC, 0x1) == 0);
    ADC_Raw_Data = EADC_GET_CONV_DATA(EADC, 0);

    return ADC_Raw_Data;
}

uint32_t Get_ADC_PWMDAC(void)
{
    uint32_t ADC_Raw_Data;
    /* Clear the A/D ADI NT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, 0x2);

    //Trigger sample module 0 to start A/D conversion
    EADC_START_CONV(EADC, 0x2);

    /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    while(EADC_GET_INT_FLAG(EADC, 0x2) == 0);
    ADC_Raw_Data = EADC_GET_CONV_DATA(EADC, 1);

    return ADC_Raw_Data;
}

