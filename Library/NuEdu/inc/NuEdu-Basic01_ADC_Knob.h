#ifndef __NuEdu_Basic01_ADC_Knob_H__
#define __NuEdu_Basic01_ADC_Knob_H__

#define _ADC_Clock  300000          //ADC_F_Max = 16M or 8M (AVDD = 5V or 3V)

extern uint32_t Open_ADC_Knob_Fail;

extern void Open_ADC_Knob(void);
extern void Close_ADC_Knob(void);
extern uint32_t Get_ADC_Knob(void);
extern uint32_t Get_ADC_PWMDAC(void);

#endif
