/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 15/09/02 10:03a $
 * @brief
 *           NuEdu-SDK-M451 SPI Flash with PDMA sample code.
						 Use PDMA Channel 11 to transfer Data from Memmory to SPI Flash
						 and PDMA Channel 10 to receive Data from SPI Flash to Memmory. 
 *           
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01.h"

#define PLL_CLOCK           72000000
#define TEST_NUMBER 				10 /* page numbers */
#define	TEST_LENGTH					255	/* length */
#define SPI_TX_DMA_CH 			1
#define SPI_RX_DMA_CH 			2
unsigned char	SrcArray[256];
unsigned char DestArray[256];

void UART0_Init(void)
{
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

      /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD(PD.6) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    unsigned int u32ByteCount;
    unsigned int u32PageNumber;
    unsigned int u32ProgramFlashAddress = 0;
    unsigned int u32VerifyFlashAddress = 0;
    unsigned int MidDid;

    /* Initial system */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("\nHello World.\n");
    printf("PLL Clock = %d Hz\n", CLK_GetPLLClockFreq());
    printf("Core Clock = %d Hz\n\n", CLK_GetHCLKFreq());
    printf("+-------------------------------------------------------+\n");
    printf("|        M451 Series SPI_Flash_w_PDMA Sample Code       |\n");
    printf("+-------------------------------------------------------+\n");

    /* Open 7-Seg */
    Open_Seven_Segment();

    /* Open SPI for Serial Flash */
    Open_SPI_Flash();

    /* Enable IP clock */
    CLK_EnableModuleClock(PDMA_MODULE);
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Read MID & DID */
    MidDid = SpiFlash_w_PDMA_ReadMidDid();
    printf("\nMID and DID = %x", MidDid);

    /* Erase SPI Flash */
    SpiFlash_w_PDMA_ChipErase();
    printf("\nFlash Erasing... ");	
	
    /* Wait ready */
    SpiFlash_w_PDMA_WaitReady();
    printf("Done!");

    /* Fill the Source Data and clear Destination Data Buffer */
    for(u32ByteCount=0; u32ByteCount<256; u32ByteCount++){
        SrcArray[u32ByteCount] = u32ByteCount;
        DestArray[u32ByteCount] = 0;
    }
        
    u32ProgramFlashAddress = 0;
    u32VerifyFlashAddress = 0;
    for(u32PageNumber=0; u32PageNumber<TEST_NUMBER; u32PageNumber++){
        printf("\n\nTest Page Number = %d", u32PageNumber);
        Show_Seven_Segment(u32PageNumber,1);
        CLK_SysTickDelay(200000);

        /*=== Program SPI Flash ===*/	
        printf("\nFlash Programming... ");

        /* Page Program */
        SpiFlash_w_PDMA_PageProgram((uint32_t)SrcArray, u32ProgramFlashAddress);
        SpiFlash_w_PDMA_WaitReady();
        u32ProgramFlashAddress += 0x100;
        printf("Done!");

        /*=== Read Back and Compare Data ===*/
        printf("\nFlash Verifying... ");

        /* Page Read */
        SpiFlash_w_PDMA_ReadData((uint32_t)DestArray, u32VerifyFlashAddress);
        u32VerifyFlashAddress += 0x100;

        for(u32ByteCount=0; u32ByteCount<256; u32ByteCount++){
            if(DestArray[u32ByteCount]!=u32ByteCount){
                /* Error */
                printf("SPI Flash R/W Fail!");
                while(1);
            }
        }
			
        /* Clear Destination Data Buffer */
        for(u32ByteCount=0; u32ByteCount<256; u32ByteCount++)
            DestArray[u32ByteCount] = 0;
        printf("Done!");
    }

    printf("\n\nSPI Flash Test Ok!");
    printf("\n\n");

    while(1);
	
}
