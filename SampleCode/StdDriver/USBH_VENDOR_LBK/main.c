/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample shows how to do transfers on a known device with
 *           a vendor driver. This sample requires a M451 USB device running
 *           sample USBD_VENDOR_LBK be connected.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "M451Series.h"

#include "usb.h"
#include "hub.h"
#include "lbk_driver.h"

#ifdef DEBUG_ENABLE_SEMIHOST
#error This sample cannot execute with semihost enabled
#endif
extern int kbhit(void);    /* in retarget.c */

volatile int       has_error;
volatile int       int_in_cnt, int_out_cnt;
volatile int       iso_in_cnt, iso_out_cnt;
volatile uint32_t  g_tick_cnt;

void SysTick_Handler(void)
{
    g_tick_cnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    g_tick_cnt = 0;
    if(SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
    }
}

uint32_t get_ticks()
{
    return g_tick_cnt;
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while(!TIMER0->INTSTS);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

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

    /* Enable OTG clock */
    CLK->APBCLK0 |= CLK_APBCLK0_OTGCKEN_Msk;

    /* Configure OTG function as Host-Only */
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk | SYS_USBPHY_USBROLE_STD_USBH;

    /* Set GPD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PA.2 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA2MFP_Msk) | SYS_GPA_MFPL_PA2MFP_USB_VBUS_EN;

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PA.3 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_USB_VBUS_ST;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*
 *  USB device connect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  HSUSBH port, USBH port, and USB hub device ports. Once a new device connected, it
 *  will be detected and enumerated in the call to usbh_pooling_hubs(). This callback
 *  will be invoked from USB core once a newly connected device was successfully enumerated.
 */
void connect_func(UDEV_T *udev, int param)
{
    struct hub_dev_t *parent;
    int    i;

    parent = udev->parent;

    printf("Device [0x%x,0x%x] was connected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
    printf("    Speed:    %s-speed\n", (udev->speed == SPEED_HIGH) ? "high" : ((udev->speed == SPEED_FULL) ? "full" : "low"));
    printf("    Location: ");

    if(parent == NULL)
    {
        if(udev->port_num == 1)
            printf("USB 2.0 port\n");
        else
            printf("USB 1.1 port\n");
    }
    else
    {
        if(parent->pos_id[0] == '1')
            printf("USB 2.0 port");
        else
            printf("USB 1.1 port");

        for(i = 1; parent->pos_id[i] != 0; i++)
        {
            printf(" => Hub port %c", parent->pos_id[i]);
        }

        printf(" => Hub port %d\n", udev->port_num);

        printf("\n");
    }
    printf("\n");
}


/*
 *  USB device disconnect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  HSUSBH port, USBH port, and USB hub device ports. Once a device was disconnected, it
 *  will be detected and removed in the call to usbh_pooling_hubs(). This callback
 *  will be invoked from USB core prior to remove that device.
 */
void disconnect_func(UDEV_T *udev, int param)
{
    printf("Device [0x%x,0x%x] was disconnected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
}


void demo_ctrl_xfer(void)
{
    uint32_t   loop, msg_tick;
    uint8_t    buff_out[64], buff_in[64];

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
                return;
        }

        memset(buff_out, (loop & 0xff), sizeof(buff_out));

        if(!lbk_device_is_connected())
            return;

        if(lbk_vendor_set_data(buff_out) != 0)
        {
            printf("Control-out transfer failed. Stop control transfer loop.\n");
            return;
        }

        if(!lbk_device_is_connected())
            return;

        if(lbk_vendor_get_data(buff_in) != 0)
        {
            printf("Control-in transfer failed. Stop control transfer loop.\n");
            return;
        }

        if(memcmp(buff_out, buff_in, 64) != 0)
        {
            printf("CTRL data compare error!\n");
        }

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Control transfer loop count %d.     \r", loop);
            msg_tick = get_ticks();
        }
    }
}

void demo_bulk_xfer(void)
{
    uint32_t   loop, msg_tick, xfer_len;
    uint8_t    buff_out[512], buff_in[512];

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    if(lbk_device_is_high_speed())
        xfer_len = 512;
    else
        xfer_len = 64;

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
                return;
        }

        memset(buff_out, (loop & 0xff), xfer_len);

        if(!lbk_device_is_connected())
            return;

        if(lbk_bulk_write(buff_out, xfer_len, 100) != 0)
        {
            printf("Bulk-out transfer failed. Stop bulk transfer loop.\n");
            return;
        }

        if(!lbk_device_is_connected())
            return;

        if(lbk_bulk_read(buff_in, xfer_len, 100) != 0)
        {
            printf("Bulk-in transfer failed. Stop bulk transfer loop.\n");
            return;
        }

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Bulk transfer loop count %d.    \r", loop);
            msg_tick = get_ticks();
        }
    }
}

int int_in_callback(int status, uint8_t *rdata, int data_len)
{
    if(status < 0)
    {
        printf("Interrupt-in trnasfer error %d!\n", status);
        has_error = 1;
        return 0;
    }
    int_in_cnt++;
    return 0;
}

int int_out_callback(int status, uint8_t *rdata, int data_len)
{
    if(status < 0)
    {
        printf("interrupt out transfer error.\n");
        has_error = 1;
        return 0;
    }

    /* add code here to send data to device */
    /* ... */
    memset(rdata, (int_out_cnt & 0xff), data_len);
    int_out_cnt++;
    return data_len;
}

void demo_interrupt_xfer(void)
{
    uint32_t   loop, msg_tick;

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    int_in_cnt = 0;
    int_out_cnt = 0;
    has_error = 0;

    lbk_interrupt_in_start(int_in_callback);
    lbk_interrupt_out_start(int_out_callback);

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
            {
                lbk_interrupt_in_stop();
                lbk_interrupt_out_stop();
                return;
            }
        }

        if(!lbk_device_is_connected() || has_error)
            return;

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Interrupt transfer loop RX: %d, TX: %d    \r", int_in_cnt, int_out_cnt);
            msg_tick = get_ticks();
        }
    }
}

/*
 *  rdata    - isochronous in data buffer
 *  data_len - length of data received
 *  return value - not used
 */
int iso_in_callback(uint8_t *rdata, int data_len)
{
    /* add code here to collect data recevied from device */
    /* ... */
    iso_in_cnt++;
    return 0;
}

/*
 *  rdata    - isochronous out data buffer
 *  data_len - length of data received
 *  return value - the length of data to be send, must be <= data_len
 */
int iso_out_callback(uint8_t *rdata, int data_len)
{
    /* add code here to send data to device */
    /* ... */
    memset(rdata, (iso_out_cnt & 0xff), data_len);
    iso_out_cnt++;
    return data_len;
}

void demo_isochronous_xfer(void)
{
    uint32_t   loop, msg_tick;

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    iso_in_cnt = 0;
    iso_out_cnt = 0;
    has_error = 0;

    lbk_isochronous_in_start(iso_in_callback);
    lbk_isochronous_out_start(iso_out_callback);

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
            {
                lbk_isochronous_in_stop();
                lbk_isochronous_out_stop();
                return;
            }
        }

        if(!lbk_device_is_connected())
            return;

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Isochronous transfer loop RX: %d, TX: %d    \r", iso_in_cnt, iso_out_cnt);
            msg_tick = get_ticks();
        }
    }
}

void vendor_lbk_demo(void)
{
    int   item;

    usbh_pooling_hubs();
    if(!lbk_device_is_connected())
        return;

    while(1)
    {
        printf("\n");
        printf("+------------------------------------------+\n");
        printf("|  USB Host transfer demo                  |\n");
        printf("+------------------------------------------+\n");
        printf("| [1] Control transfer demo                |\n");
        printf("| [2] Bulk transfer demo                   |\n");
        printf("| [3] Interrupt transfer demo              |\n");
        printf("| [4] Isochronous transfer demo            |\n");
        printf("+------------------------------------------+\n");

        usbh_memory_used();
        printf("\nSelect [1~9;A~S]: \n");

        item = getchar();

        usbh_pooling_hubs();
        if(!lbk_device_is_connected())
            return;

        switch(item)
        {
            case '1':
                demo_ctrl_xfer();
                break;

            case '2':
                demo_bulk_xfer();
                break;

            case '3':
                demo_interrupt_xfer();
                break;

            case '4':
                demo_isochronous_xfer();
                break;
        }

        usbh_pooling_hubs();
        if(!lbk_device_is_connected())
            return;
    }
}

int32_t main(void)
{
    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|                                       |\n");
    printf("|     USB Host Vendor Loopback Demo     |\n");
    printf("|                                       |\n");
    printf("+---------------------------------------+\n");

    usbh_core_init();
    usbh_lbk_init();
    usbh_install_conn_callback(connect_func, disconnect_func);
    usbh_pooling_hubs();

    if(!lbk_device_is_connected())
        printf("Waitng for M451 Vendor Loopback device be connected...\n");

    while(1)
    {
        usbh_pooling_hubs();

        if(!lbk_device_is_connected())
            continue;

        /* do not return unless LBK device disconnected */
        vendor_lbk_demo();

        printf("\n\nWaitng for M451 Vendor Loopback device be connected...\n");
        usbh_memory_used();
    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
