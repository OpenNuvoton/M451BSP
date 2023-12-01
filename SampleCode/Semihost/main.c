/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/11/02 9:50a $
 * @brief    Show how to debug with semi-host message print.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
# if defined (__GNUC__)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main()
{
    int8_t item;
#if (defined (__GNUC__) && (!(defined(__ARMCC_VERSION))))
    initialise_monitor_handles();
#endif

    /*
        To enable semihost, user must define "DEBUG_ENABLE_SEMIHOST" constant when build code with M451Series BSP.
        This sample code is used to show how to print message/getchar on IDE debug environment.
        It will echo all input character back on UART #1 of KEIL IDE.

        In KEIL MDK, user need to open "View->Serial Window->UART #1" windows in debug mode.
        In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

        NOTE1: HardFault_Handler handler is implemented in retarget.c.
        NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
        NOTE3: It does not print any message if Nuvoton NuLink ICE Dongle is not connected.
    */

    printf("\n Start SEMIHOST test: \n");

    while(1)
    {
        /* Get input character */
        item = getchar();

        /* Print input character back */
        printf("%c\n", item);
    }
}

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
