#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_LED.h"

#define _LED_Bar_Count      7

#define _LED1               PB2
#define _LED2               PB3
#define _LED3               PC3
#define _LED4               PC2
#define _LED5               PA9
#define _LED6               PB1
#define _LED7               PC7



void Write_LED_Bar(uint32_t Number)
{
    uint32_t i;
    volatile uint32_t *ptrLED[_LED_Bar_Count] = {&_LED1, &_LED2, &_LED3, &_LED4, &_LED5, &_LED6, &_LED7};

    for(i = 0; i < _LED_Bar_Count; i++)
    {
        if((Number > i) & 0x01)
            *ptrLED[i] = 0; //LED ON
        else
            *ptrLED[i] = 1; //LED OFF
    }
}

void Initial_LED(void)
{
    GPIO_SetMode(PB, BIT2, GPIO_MODE_OUTPUT); //LED1
    GPIO_SetMode(PB, BIT3, GPIO_MODE_OUTPUT); //LED2
    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT); //LED3
    GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT); //LED4
    GPIO_SetMode(PA, BIT9, GPIO_MODE_OUTPUT); //LED5
    GPIO_SetMode(PB, BIT1, GPIO_MODE_OUTPUT); //LED6
    GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT); //LED7

}

void LED_On(unsigned int temp)
{
    if((temp & 1) != 1)
        _LED1 = 1;
    else
        _LED1 = 0;

    temp = temp >> 1;

    if((temp & 1) != 1)
        _LED2 = 1;
    else
        _LED2 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED3 = 1;
    else
        _LED3 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED4 = 1;
    else
        _LED4 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED5 = 1;
    else
        _LED5 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED6 = 1;
    else
        _LED6 = 0;

    temp = temp >> 1;
    if((temp & 1) != 1)
        _LED7 = 1;
    else
        _LED7 = 0;

}
