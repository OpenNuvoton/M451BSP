/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 18 $
 * $Date: 18/01/30 10:05a $
 * @brief
 *           USB CDC class virtual COM sample code.
 *
 * @note
 * Copyright (C) 2018~2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"
#include "usbh_core.h"
#include "usbh_cdc.h"

char Line[64];             /* Console input buffer */

static volatile int  g_rx_ready = 0;

extern int kbhit(void);

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


void  vcom_status_callback(CDC_DEV_T *cdev, uint8_t *rdata, int data_len)
{
    int  i;
    printf("[VCOM STS] ");
    for(i = 0; i < data_len; i++)
        printf("0x%02x ", rdata[i]);
    printf("\n");
}

void  vcom_rx_callback(CDC_DEV_T *cdev, uint8_t *rdata, int data_len)
{
    int  i;
    
    //printf("[VCOM DATA %d] ", data_len);
    for (i = 0; i < data_len; i++)
    {
        //printf("0x%02x ", rdata[i]);
        printf("%c", rdata[i]);
    }
    //printf("\n");
    
    g_rx_ready = 1;   
}


void show_line_coding(LINE_CODING_T *lc)
{
	printf("[CDC device line coding]\n");
	printf("====================================\n");
	printf("Baud rate:  %d bps\n", lc->baud);
	printf("Parity:     ");
	switch (lc->parity)
	{
		case 0: printf("None\n");  break;
		case 1: printf("Odd\n");   break;
		case 2: printf("Even\n");  break;
		case 3: printf("Mark\n");  break;
		case 4: printf("Space\n"); break;
		default:  printf("Invalid!\n"); break;
	}
	printf("Data Bits:  ");
	switch (lc->data_bits)
	{
		case 5 :
		case 6 :
		case 7 :
		case 8 :
		case 16:
			printf("%d\n", lc->data_bits);
			break;
		default:
		    printf("Invalid!\n");
		    break;
	}
	printf("Stop Bits:  %s\n\n", (lc->stop_bits == 0) ? "1" : ((lc->stop_bits == 1) ? "1.5" : "2"));
}

int  init_cdc_device(CDC_DEV_T *cdev)
{
    int     ret;
    LINE_CODING_T  line_code;

    printf("\n\n==================================\n");
    printf("  Init CDC device : 0x%x\n", (int)cdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", cdev->udev->descriptor.idVendor, cdev->udev->descriptor.idProduct);

	ret = USBH_CDC_GetLineCoding(cdev, &line_code);
	if (ret < 0)
	{
		printf("Get Line Coding command failed: %d\n", ret);
	}
	else
	    show_line_coding(&line_code);
	    
	line_code.baud = 115200;
	line_code.parity = 0;
	line_code.data_bits = 8;
	line_code.stop_bits = 0;

	ret = USBH_CDC_SetLineCoding(cdev, &line_code);
	if (ret < 0)
	{
		printf("Set Line Coding command failed: %d\n", ret);
	}

	ret = USBH_CDC_GetLineCoding(cdev, &line_code);
	if (ret < 0)
	{
		printf("Get Line Coding command failed: %d\n", ret);
	}
	else
	{
		printf("New line coding =>\n");
	    show_line_coding(&line_code);
	}
	
	USBH_CDC_SetControlLineState(cdev, 1, 1);

    printf("USBH_CDC_StartStatusPipe...\n");
    USBH_CDC_StartStatusPipe(cdev, vcom_status_callback);

    printf("USBH_CDC_StartRxPipe...\n");
    USBH_CDC_StartRxPipe(cdev, vcom_rx_callback);

    return 0;
}


/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/
void get_line(CDC_DEV_T *cdev, char *buff, int len)
{
    char c;
    int idx = 0;
    for(;;)
    {
    	while (kbhit())   /* wait until any key input */
    	{
        	if (cdev->rx_busy == 0)
     			USBH_CDC_StartRxPipe(cdev, vcom_rx_callback);
    	}
        c = getchar();
        putchar(c);
        if(c == '\r') break;
        if((c == '\b') && idx) idx--;
        if((c >= ' ') && (idx < len - 1)) buff[idx++] = c;
    }
    buff[idx] = 0;
    putchar('\n');
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
    CDC_DEV_T    *cdev;
    int          ret;

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
    printf("+---------------------------------------------------------+\n");
    printf("|           M451 USB Host VCOM sample program             |\n");
    printf("+---------------------------------------------------------+\n");
    printf("|  (NOTE: This sample supports only one CDC device, but   |\n"); 
    printf("|         driver supports multiple CDC devices. If you    |\n");
    printf("|         want to support multiple CDC devices, you       |\n");
    printf("|         have to modify this sample.                     |\n");
    printf("+---------------------------------------------------------+\n");

    USBH_Open();

    USBH_CDCInit();

    printf("Wait until any CDC devices connected...\n");

    while(1)
    {
        if (USBH_ProcessHubEvents())              /* USB Host port detect polling and management */
        {
            cdev = USBH_CDCGetDeviceList();
            if (cdev == NULL)
                continue;

            while (cdev != NULL)
            {
                init_cdc_device(cdev);

                if (cdev != NULL)
                    cdev = cdev->next;
            }
        }
        
        cdev = USBH_CDCGetDeviceList();
        if (cdev == NULL)
            continue;
        
        if (g_rx_ready)
        {
        	g_rx_ready = 0;

            if (cdev->rx_busy == 0)
      			USBH_CDC_StartRxPipe(cdev, vcom_rx_callback);
    	}
	
		/* 
		 *  Check user input and send to CDC device immediately 
		 *  (You can also modify it send multiple characters at one time.)
		 */
    	if (kbhit() == 0)
    	{
            Line[0] = getchar();
        	ret = USBH_CDC_SendData(cdev, (uint8_t *)Line, 1);                
        	if (ret != 0)
            	printf("\n!! Send data failed, 0x%x!\n", ret);
    	}
    }
}


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
