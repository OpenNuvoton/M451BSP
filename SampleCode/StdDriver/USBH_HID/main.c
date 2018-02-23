/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 18 $
 * $Date: 15/09/02 10:05a $
 * @brief
 *           Show how to implement a USB Host and recognize a HID device when device plug-in.
 *
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"
#include "usbh_core.h"
#include "usbh_hid.h"

uint8_t  desc_buff[1024];


void Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBH_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBH_MODULE, 0, CLK_CLKDIV0_USB(3));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USB Host clock                                                                                     */
    /*---------------------------------------------------------------------------------------------------------*/

    // Configure OTG function as Host-Only
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk | SYS_USBPHY_USBROLE_STD_USBH;

#ifdef AUTO_POWER_CONTROL
    /* Below settings is use power switch IC to enable/disable USB Host power.
       Set PC.4 is VBUS_EN function pin and PC.3 VBUS_ST function pin             */
    //SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
    //SYS->GPC_MFPL |=  (SYS_GPC_MFPL_PC3MFP_USB_VBUS_ST | SYS_GPC_MFPL_PC4MFP_USB_VBUS_EN);

    /* Below settings is use power switch IC to enable/disable USB Host power.
       Set PA.2 is VBUS_EN function pin and PA.3 VBUS_ST function pin             */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_USB_VBUS_ST | SYS_GPA_MFPL_PA2MFP_USB_VBUS_EN);

    CLK->APBCLK0 |= CLK_APBCLK0_OTGCKEN_Msk;   //Enable OTG_EN clock
#else
    /* Below settings is use GPIO to enable/disable USB Host power.
       Set PC.4 output high to enable USB power                               */

    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC4MFP_Msk;
    PC->MODE = (PC->MODE & ~GPIO_MODE_MODE4_Msk) | (0x1 << GPIO_MODE_MODE4_Pos);
    PC->DOUT |= 0x10;
#endif
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void  int_read_callback(HID_DEV_T *hdev, uint8_t *rdata, int data_len)
{
    int  i;
    printf("INT-in pipe data %d bytes received =>\n", data_len);
    for(i = 0; i < data_len; i++)
        printf("0x%02x ", rdata[i]);
    printf("\n");
}

static uint8_t  _write_data_buff[4];

void  int_write_callback(HID_DEV_T *hdev, uint8_t **wbuff, int *buff_size)
{
    printf("INT-out pipe request to write data.\n");

    *wbuff = &_write_data_buff[0];
    *buff_size = 4;
}

uint32_t CLK_GetUSBFreq(void)
{  
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/  
    /* USB peripheral clock = PLL_CLOCK/USBDIV+1) */    
    return CLK_GetPLLClockFreq()/(((CLK->CLKDIV0 & CLK_CLKDIV0_USBDIV_Msk)>>CLK_CLKDIV0_USBDIV_Pos)+1);
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    int          i, ret;
    HID_DEV_T    *hdev;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf(" System clock:   %d Hz.\n", SystemCoreClock);
    printf(" USB Host clock: %d Hz.\n", CLK_GetUSBFreq());
    printf("+--------------------------------------+\n");
    printf("|                                      |\n");
    printf("|  M451 USB Host HID sample program    |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");

    USBH_Open();

    USBH_HidInit();

    printf("Wait until any HID devices connected...\n");
    while(1)
    {
        USBH_ProcessHubEvents();             /* USB Host port detect polling and management */

        hdev = USBH_HidGetDeviceList();
        if(hdev != NULL)
            break;
    }

    ret = HID_HidGetReportDescriptor(hdev, desc_buff, 1024);
    if(ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        for(i = 0; i < ret; i++)
        {
            if((i % 16) == 0)
                printf("\n");
            printf("%02x ", desc_buff[i]);
        }
        printf("\n\n");
    }

    /*
     *  Example: GET_PROTOCOL request.
     */
    ret = HID_HidGetProtocol(hdev, desc_buff);
    printf("[GET_PROTOCOL] ret = %d, protocol = %d\n", ret, desc_buff[0]);

    /*
     *  Example: SET_PROTOCOL request.
     */
    ret = HID_HidSetProtocol(hdev, desc_buff[0]);
    printf("[SET_PROTOCOL] ret = %d, protocol = %d\n", ret, desc_buff[0]);

    /*
     *  Example: GET_REPORT request on report ID 0x1, report type FEATURE.
     */
    ret = HID_HidGetReport(hdev, RT_FEATURE, 0x1, desc_buff, 64);
    if(ret > 0)
    {
        printf("[GET_REPORT] Data => ");
        for(i = 0; i < ret; i++)
            printf("%02x ", desc_buff[i]);
        printf("\n");
    }

    printf("\nUSBH_HidStartIntReadPipe...\n");
    if(USBH_HidStartIntReadPipe(hdev, int_read_callback) == HID_RET_OK)
    {
        printf("Interrupt in transfer started...\n");
    }

    //if (USBH_HidStartIntWritePipe(hdev, int_write_callback) == HID_RET_OK)
    //{
    //  printf("Interrupt out transfer started...\n");
    //}

    printf("Done.\n");

    while(1);
}


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
