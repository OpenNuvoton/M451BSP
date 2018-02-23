/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:03a $
 * @brief    NuEdu Basic01 Timer Sample Code
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  TMR0 IRQ handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t volatile TimerCounter = 0;
void TMR0_IRQHandler(void)
{
    TimerCounter == 99 ? (TimerCounter = 0) : (TimerCounter++);
    // clear Timer0 interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    //Initial System
    SYS_Init();

    //Enable Timer0 clock and select Timer0 clock source
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    //Initial Timer0 to periodic mode with 2Hz
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 2);
    //Enable Timer0 interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    //Initial 7-Segment
    Open_Seven_Segment();

    //Start Timer0
    TIMER_Start(TIMER0);

    while(1)
    {
        Show_Seven_Segment(TimerCounter / 10, 1);
        CLK_SysTickDelay(200);
        Show_Seven_Segment(TimerCounter % 10, 2);
        CLK_SysTickDelay(200);
    }
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
