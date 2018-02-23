/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to simulate a USB CD-ROM device.
 * @note
 *
 *           This sample simulate a USB CD-ROM with a test.txt file.
 *           User can change file within CD-ROM, follow steps below:
 *
 *           (1) Generate an ISO file via ISO Workshop.
 *               http://www.glorylogic.com/iso-workshop/
 *               -> Option : Select ISO9660 Level1
 *
 *           (2) Convert the .iso file into C code file via SRecord tool
 *               http://srecord.sourceforge.net
 *
 *               -> srec_cat InputFile.iso -binary -o OutputFile.c -C-Array
 *          
 *           (3) The System Area, the first 32768 data bytes is unused by ISO 9660.
 *               Therefore, user need to delete the first 32768 bytes on the DiskImg.c 
 *               manually to save space. Finally, replace DiskImg.c in this project.  
 *
 *               -> EPROM_LENGTH in DiskImg.c is the size of your .iso image. 
 *                  Define MSC_ImageSize value in massstorage.h in this project.
 *                  Modify MSC_ImageSize value to hold the file size.
 *
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "massstorage.h"

/*--------------------------------------------------------------------------*/

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV0_USB(3));

    /* Enable USB LDO33 */
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPD multi-function pins for UART0 RXD and TXD, and Clock Output */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD6MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD | SYS_GPD_MFPL_PD6MFP_CLKO);

    /* Enable CLKO (PD.6) for monitor HCLK. CLKO = HCLK/8 Hz */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 2, 0);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB MassStorage Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    printf("NuMicro USB MassStorage Start!\n");

    USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

    /* Endpoint configuration */
    MSC_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);

    while(1)
    {
        MSC_ProcessCmd();
    }
}



/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

