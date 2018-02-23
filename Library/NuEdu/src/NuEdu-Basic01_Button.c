#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_Button.h"
#define KEY1 PE2
#define KEY2 PA8
#define KEY3 PB6
#define KEY4 PB7
void Initial_Key_Input(void)
{
    GPIO_SetMode(PE, BIT2, GPIO_MODE_INPUT);
    GPIO_SetMode(PA, BIT8,  GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT7, GPIO_MODE_INPUT);
}

unsigned char Get_Key_Input(void)
{
    unsigned char temp = 0;
    if(KEY1 == 0)
        temp |= 0x1;


    if(KEY2 == 0)
        temp |= 0x2;


    if(KEY3 == 0)
        temp |= 0x4;


    if(KEY4 == 0)
        temp |= 0x8;

    return   temp;
}
