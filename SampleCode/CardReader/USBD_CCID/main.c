/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 6 $
 * $Date: 15/09/02 10:03a $
 * @brief    CCID smart card reader sample code.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "ccid.h"
#include "ccid_if.h"
#include "sclib.h"

#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define INT_BUFFER_SIZE     64    /* Interrupt message buffer size */
#define BULK_BUFFER_SIZE    512   /* bulk message buffer size */

uint8_t UsbIntMessageBuffer[INT_BUFFER_SIZE];
uint8_t UsbMessageBuffer[BULK_BUFFER_SIZE];

uint8_t volatile gu8IsDeviceReady;
uint8_t volatile gu8AbortRequestFlag;
uint8_t volatile gu8IsBulkOutReady;
uint8_t volatile gu8IsBulkInReady;

uint8_t *pu8IntInBuf;
uint8_t *pUsbMessageBuffer;
uint32_t volatile u32BulkSize;

int32_t volatile gi32UsbdMessageLength;

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    // Please don't remove any of the function calls below
    if(SCLIB_CheckCDEvent(0))
    {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        return; // Card insert/remove event occurred, no need to check other event...
    }

    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable USB LDO33 */
    SYS->USBPHY |= SYS_USBPHY_LDO33EN_Msk;
    SYS->USBPHY &= ~SYS_USBPHY_USBROLE_Msk; //Standard USB device

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HXT, CLK_CLKDIV1_SC0(3));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV0_USB(3));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set PA.0 ~ PA.3 and PB.2 for SC0 interface */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk |
                       SYS_GPA_MFPL_PA1MFP_Msk |
                       SYS_GPA_MFPL_PA2MFP_Msk |
                       SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SC0_CLK |
                      SYS_GPA_MFPL_PA1MFP_SC0_DAT |
                      SYS_GPA_MFPL_PA2MFP_SC0_RST |
                      SYS_GPA_MFPL_PA3MFP_SC0_PWR);
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB2MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB2MFP_SC0_CD;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
//     SCLIB_CARD_INFO_T s_info;
//     int retval, i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("NuMicro USB CCID SmartCard Reader\n");

    // Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    SC0->INTEN = SC_INTEN_CDIEN_Msk;

    USBD_Open(&gsInfo, CCID_ClassRequest, NULL);

    /* Endpoint configuration */
    CCID_Init();
    //Set priority is a must under current architecture. Otherwise smartcard interrupt will be blocked by USBD interrupt
    NVIC_SetPriority(USBD_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();




//     while(SC_IsCardInserted(SC0) == FALSE);
//     // Activate slot 0
//     retval = SCLIB_Activate(0, FALSE);
//     if(retval == SCLIB_SUCCESS) {
//        SCLIB_GetCardInfo(0, &s_info);
//         printf("ATR: ");
//         for(i = 0; i < s_info.ATR_Len; i++)
//             printf("%x ", s_info.ATR_Buf[i]);
//         printf("\n");
//     } else
//         printf("Smartcard activate failed\n");
    while(1);

}
