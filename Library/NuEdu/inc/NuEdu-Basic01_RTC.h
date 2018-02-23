#ifndef __NuEdu_Basic01_RTC_H__
#define __NuEdu_Basic01_RTC_H__

extern void RTC_Init(void);
extern void Time_and_Tick_function(void);
extern void Alarm_Wakeup_function(void);

extern void get_key(void);
extern void Initial_Key_Input(void);
extern unsigned char Get_Key_Input(void);
extern void SetAlarmWakeup_function(void);
// extern SYS_Init(void);
// extern UART0_Init(void);

#endif
