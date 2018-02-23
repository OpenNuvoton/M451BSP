/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/04/11 11:20a $
 * @brief     Android Open Accessory device sample.
 *
 * @note
 * Copyright (C) 2016~2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"
#include "usbh_core.h"
#include "AOA_driver.h"

#define PLL_CLOCK    72000000

static volatile int  g_tick_cnt = 29020;
static volatile int  g_tick_cnt2 = 1;

void SysTick_Handler(void) 
{  	
	g_tick_cnt++;
}


void enable_sys_ticks(int ticks_per_second)
{
	uint32_t  systick_load;
	/*
	 *  Configure system tick to be 0.1 second count
	 */
	systick_load = ((PLL_CLOCK/(2 * ticks_per_second)) & SysTick_LOAD_RELOAD_Msk) - 1;
	/* system tick from HCLK/2 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | (3 << CLK_CLKSEL0_STCLKSEL_Pos	);	
	SysTick->LOAD  = systick_load; 
  	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
    SysTick->VAL = 0;
   	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

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

uint32_t CLK_GetUSBFreq(void)
{  
    /*---------------------------------------------------------------------------------------------------------*/
    /* Get USB Peripheral Clock                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/  
    /* USB peripheral clock = PLL_CLOCK/USBDIV+1) */    
    return CLK_GetPLLClockFreq()/(((CLK->CLKDIV0 & CLK_CLKDIV0_USBDIV_Msk)>>CLK_CLKDIV0_USBDIV_Pos)+1);
}

/*
 *  AOA bulk data in callback function.
 */
void aoa_read_func(uint8_t *data_buff, int data_len)
{
	int  i;
	printf("AOA RX: ");
	for (i = 0; i < data_len; i++)
		printf("%02d ", data_buff[i]);
	printf("\n");
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    int   t0 = 0, connected;
    char  buff[64];

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
    printf("|  M451 USB Host AOA sample program    |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");

    enable_sys_ticks(100);
    t0 = g_tick_cnt;

    connected = 0;

    USBH_Open();

    AOA_Init(aoa_read_func);
    
    while (1)
    {
        USBH_ProcessHubEvents();

        if (AOA_IsConnected())
        {
            if (!connected) 
            {
                connected = 1;
                printf("\nAOA device found!\n");
            }
        } 
        else 
        {
            connected = 0;
            continue;
        }

		if (connected)
		{      
			if (g_tick_cnt - t0 >= 2)
			{  
        		memset(buff, 0, 64);
        		sprintf(buff, "%08d", g_tick_cnt2++);
        		
        		printf("AOA write: %s\n", buff);
        
        		AOA_WriteData((uint8_t *)buff, 64);
        		
        		t0 = g_tick_cnt;
        	}
        }
    }
}

/*** (C) COPYRIGHT 2016~2018 Nuvoton Technology Corp. ***/
