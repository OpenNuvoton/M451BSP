#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_Button.h"

#define LB_KEY1 PA8
#define LB_KEY2 PD11
#define LB_KEY3 PF2
#define LB_KEY4 PD2
#define LB_KEY5 PD3
#define LB_KEY6 PC0

void Initial_LB_Key_Input(void)
{
    GPIO_SetMode(PE, BIT2, GPIO_MODE_INPUT);
    GPIO_SetMode(PA, BIT8,  GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
}

unsigned char Get_LB_Key_Input(void)
{
    unsigned char temp = 0;
    if(LB_KEY1 == 0)
        temp |= 0x1;

    if(LB_KEY2 == 0)
        temp |= 0x2;

    if(LB_KEY3 == 0)
        temp |= 0x4;

    if(LB_KEY4 == 0)
        temp |= 0x8;
	
    if(LB_KEY5 == 0)
        temp |= 0xC;

    if(LB_KEY6 == 0)
        temp |= 0xF;

    return   temp;
}
