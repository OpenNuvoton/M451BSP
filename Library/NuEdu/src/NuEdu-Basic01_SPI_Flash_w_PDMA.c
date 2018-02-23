#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_SPI_Flash_w_PDMA.h"

#define SPI_FLASH_PORT  SPI2

#define SPI_TX_DMA_CH 11
#define SPI_RX_DMA_CH 10
#define TEST_LENGTH                 256
#define SPI_CLR_TXFIFO_MASK 9
#define SPI_CLR_RXFIFO_MASK 8

void Init_PDMA_CH1_for_SPI2_TX(uint32_t u32SrcAddr)
{
    /* Enable PDMA channels */
    PDMA_Open((1 << SPI_TX_DMA_CH));

    /* SPI PDMA TX channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(SPI_TX_DMA_CH, PDMA_WIDTH_8, TEST_LENGTH);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(SPI_TX_DMA_CH, (uint32_t)u32SrcAddr, PDMA_SAR_INC, (uint32_t)&SPI2->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(SPI_TX_DMA_CH, PDMA_SPI2_TX, FALSE, 0);
    /* Single request type */
    PDMA_SetBurstType(SPI_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

void Init_PDMA_CH2_for_SPI2_RX(uint32_t u32DstAddr)
{
    /* Enable PDMA channels */
    PDMA_Open((1 << SPI_RX_DMA_CH));

    /* SPI PDMA RX channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(SPI_RX_DMA_CH, PDMA_WIDTH_8, TEST_LENGTH);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(SPI_RX_DMA_CH, (uint32_t)&SPI2->RX, PDMA_SAR_FIX, (uint32_t)u32DstAddr, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(SPI_RX_DMA_CH, PDMA_SPI2_RX, FALSE, 0);
    /* Single request type */
    PDMA_SetBurstType(SPI_RX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[SPI_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

}

void Initial_SPI2_GPIO(void)
{
    /* Set PD12, PD13, PD14 and PD15 for SPI2 */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_SPI2_SS | SYS_GPD_MFPH_PD13MFP_SPI2_MOSI | SYS_GPD_MFPH_PD14MFP_SPI2_MISO | SYS_GPD_MFPH_PD15MFP_SPI2_CLK;
}

// **************************************
void Open_SPI_Flash(void)
{
    /* Init GPIO for SPI2 */
    Initial_SPI2_GPIO();

    /* Enable SPI2 IP clock */
    CLK_EnableModuleClock(SPI2_MODULE);

    /* Select PCLK0 as the clock source of SPI2 */
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL2_SPI2SEL_PCLK0, MODULE_NoMsk);

    /* Configure SPI2 as a master, SPI clock rate 2 MHz,
         clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Disable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_DisableAutoSS(SPI2);
    SPI_SET_SS_HIGH(SPI2);

}

// **************************************
// For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
// For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
unsigned int SpiFlash_w_PDMA_ReadMidDid(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // send Command: 0x90, Read Manufacturer/Device ID
    au32SourceData = 0x90;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // send 24-bit '0', dummy
    au32SourceData = 0x0;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI2, 16);
    // receive
    SPI_ClearRxFIFO(SPI2);
    au32SourceData = 0x0;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);
    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI2);

    return (au32DestinationData & 0xffff);

}

// **************************************
void SpiFlash_w_PDMA_ChipErase(void)
{
    unsigned int au32SourceData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    // send Command: 0xC7, Chip Erase
    au32SourceData = 0xc7;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

}

// **************************************
unsigned int SpiFlash_w_PDMA_ReadStatusReg1(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    SPI_ClearRxFIFO(SPI2);
    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI2, 16);
    // send Command: 0x05, Read status register 1
    au32SourceData = 0x0500;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);
    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI2);

    return (au32DestinationData & 0xFF);

}


// **************************************
unsigned int SpiFlash_w_PDMA_ReadStatusReg2(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    SPI_ClearRxFIFO(SPI2);
    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI2, 16);
    // send Command: 0x035, Read status register 2
    au32SourceData = 0x3500;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);
    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI2);

    return (au32DestinationData & 0xFF);

}


// **************************************
void SpiFlash_w_PDMA_WaitReady(void)
{
    unsigned int ReturnValue;

    do
    {
        ReturnValue = SpiFlash_w_PDMA_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue != 0); // check the BUSY bit

}

// **************************************
void SpiFlash_w_PDMA_PageProgram(unsigned int u32SrcAddr, unsigned int StartAddress)
{
    unsigned int au32SourceData;
    unsigned int u32RegValue, u32Abort;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);
    SPI_ClearTxFIFO(SPI2);

    // enable SPI PDMA
    /* Initial PDMA Channel */
    Init_PDMA_CH1_for_SPI2_TX((uint32_t)u32SrcAddr);
    SPI_TRIGGER_TX_PDMA(SPI2);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS();
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS() & (1 << SPI_TX_DMA_CH)) == (1 << SPI_TX_DMA_CH))
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(1 << SPI_TX_DMA_CH);
                /* Disable SPI PDMA TX function */
                SPI_DISABLE_TX_PDMA(SPI2);
                break;
            }

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS();
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(u32Abort);
                break;
            }
        }
    }

    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

}


// **************************************
void SpiFlash_w_PDMA_ReadData(unsigned int u32DestAddr, unsigned int StartAddress)
{
    unsigned int au32SourceData;
    unsigned int u32RegValue, u32Abort;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);
    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);
    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_WRITE_TX(SPI2, au32SourceData);
    // wait
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);
    SPI_ClearRxFIFO(SPI2);

    // enable SPI PDMA
    Init_PDMA_CH2_for_SPI2_RX((uint32_t)u32DestAddr);
    SPI_TRIGGER_RX_PDMA(SPI2);

    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS();
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS() & (1 << SPI_RX_DMA_CH)) == (1 << SPI_RX_DMA_CH))
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(1 << SPI_RX_DMA_CH);
                /* Disable SPI PDMA RX function */
                SPI_DISABLE_RX_PDMA(SPI2);
                break;
            }

            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS();
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(u32Abort);
                break;
            }
        }
    }

    // wait
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);
}
