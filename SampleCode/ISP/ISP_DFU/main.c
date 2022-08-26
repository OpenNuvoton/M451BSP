/***************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to upgrade firmware between USB device and PC through USB DFU (Device Firmware Upgrade) class.
 *           A Windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "fmc_user.h"
#include "dfu_transfer.h"

#define PLLCTL_SETTING      CLK_PLLCTL_144MHz_HXT
#define PLL_CLOCK           144000000

#define V6M_AIRCR_VECTKEY_DATA    0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ     0x00000004UL

#define DetectPin   PD0

uint32_t g_apromSize;

uint32_t GetApromSize()
{
    //the smallest of APROM size is 2K
    uint32_t size = 0x800, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if(result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while(1);
}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        if( --u32TimeOutCnt == 0) return -1;

    /* Enable external XTAL 12 MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for external XTAL clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk))
        if( --u32TimeOutCnt == 0) return -1;

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(2);

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;                // PLL
    SystemCoreClock = PLL_CLOCK / 2;            // HCLK
    CyclesPerUs     = PLL_CLOCK / 2 / 1000000;  // For CLK_SysTickDelay()

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_USBDIV_Msk)) | CLK_CLKDIV0_USB(3);

    /* Enable USB LDO33 */
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk;

    return 0;
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-funcition I/O */
    if( SYS_Init() < 0 ) goto _APROM;

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_ISPFF_Msk;
    g_apromSize = GetApromSize();

    /* Open USB controller */
    USBD_Open(&gsInfo, DFU_ClassRequest, NULL);

    /*Init Endpoint configuration for DFU */
    DFU_Init();

    /* Start USB device */
    USBD_Start();

    /* polling USBD interrupt flag */
    while(DetectPin == 0)
    {
        USBD_IRQHandler();
    }

_APROM:

    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);//clear bit
    FMC->ISPCTL &=  ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
