/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "uart_transfer.h"

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#else
__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#endif

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel interrupt event                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    /* Determine interrupt source */
    uint32_t u32IntSrc = UART1->INTSTS;

    /* RDA FIFO interrupt and RDA timeout interrupt */
    if (u32IntSrc & (UART_INTSTS_RXTOIF_Msk|UART_INTSTS_RDAIF_Msk)) { 

        /* Read data until RX FIFO is empty or data is over maximum packet size */
        while (((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) {	
            uart_rcvbuf[bufhead++] = UART1->DAT;
        }
    }

    /* Reset data buffer index */
    if (bufhead == MAX_PKT_SIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if (u32IntSrc & UART_INTSTS_RXTOIF_Msk) {
        bufhead = 0;
    }
}

extern __align(4) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    /* UART send response to master */    
    for (i = 0; i < MAX_PKT_SIZE; i++) {
        
        /* Wait for TX not full */
        while ((UART1->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        /* UART send data */
        UART1->DAT = response_buff[i];
    }
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function mode */
    UART1->FUNCSEL = UART_FUNCSEL_UART;
    /* Set UART line configuration */
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Set UART Rx and RTS trigger level */
    UART1->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    /* Set UART baud rate */
    UART1->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparator */
    UART1->TOUT = (UART1->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);
    NVIC_SetPriority(UART1_IRQn, 2);
    NVIC_EnableIRQ(UART1_IRQn);
    /* Enable tim-out counter, Rx tim-out interrupt and Rx ready interrupt */
    UART1->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
