/**************************************************************************//**
 * @file     NuEdu-Basic01_SPI_Flash.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/09/02 10:02a $
 * @brief    NuEdu-Basic01_SPI_Flash driver source file for NuEdu-SDK-M451 
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_SPI_Flash.h"

/** @addtogroup M451_Library M451 Library
  @{
*/

/** @addtogroup NuEdu-SDK-M451_Basic01 M451_Basic01 Library
  @{
*/

/** @addtogroup M451_Basic01_FUNCTIONS SPI Flash Functions
  @{
*/


/**
 * @brief       Open GPIO port for SPI interface and configure this SPI controller as Master,  
 *              MSB first, clock idle low, TX at falling-edge, RX at rising-edge, 32-bit length  
 *		transaction, disable the automatic hardware slave select function and SPI serial 
 *		clock rate = 2 MHz.
 *
 * @return      None
 */
void Open_SPI_Flash(void)
{
    /* Init GPIO for for SPI Flash Port, Set PD12, PD13, PD14 and PD15 for SPI2 */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_SPI2_SS | SYS_GPD_MFPH_PD13MFP_SPI2_MOSI | SYS_GPD_MFPH_PD14MFP_SPI2_MISO | SYS_GPD_MFPH_PD15MFP_SPI2_CLK;

    /* Enable SPI2 peripheral clock */
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

/**
 * @brief       Read back the Manufacturer ID and Device ID from SPI Flash device.
 *		
 * @return      Manufacturer ID and Device ID of SPI Flash device.
 *  		For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
 *		For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
*/
unsigned int SpiFlash_ReadMidDid(void)
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
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);

    // send 24-bit '0', dummy
    au32SourceData = 0x0;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI2, 16);

    I2S_CLR_RX_FIFO(SPI2);

    // receive
    au32SourceData = 0x0;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI2);

    return (au32DestinationData & 0xffff);

}

/**
 * @brief       This function do the chip erasing to SPI Flash device. 
 *		 
 * @return      None
 */
void SpiFlash_ChipErase(void)
{
    unsigned int au32SourceData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    // send Command: 0xC7, Chip Erase
    au32SourceData = 0xc7;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    //CS: de-active
    SPI_SET_SS_HIGH(SPI2);

}

/**
 * @brief       Read back the Status Register 1 from SPI Flash device.
 *
 * @return      Status Register 1 value of SPI Flash device.
 */
unsigned int SpiFlash_ReadStatusReg1(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    I2S_CLR_RX_FIFO(SPI2);

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI2, 16);

    // send Command: 0x05, Read status register 1
    au32SourceData = 0x0500;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI2);

    return (au32DestinationData & 0xFF);

}


/**
 * @brief       Read back the Status Register 2 from SPI Flash device.
 *
 * @return      Status Register 2 value of SPI Flash device.
 */
unsigned int SpiFlash_ReadStatusReg2(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    I2S_CLR_RX_FIFO(SPI2);

    // configure transaction length as 16 bits
    SPI_SET_DATA_WIDTH(SPI2, 16);

    // send Command: 0x35, Read status register 2
    au32SourceData = 0x3500;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // dump Rx register
    au32DestinationData = SPI_READ_RX(SPI2);

    return (au32DestinationData & 0xFF);
}


/**
 * @brief       Waiting for the BUSY bit of SPI Flash that be cleared to 0.
 *
 * @return      None
 */
void SpiFlash_WaitReady(void)
{
    unsigned int ReturnValue;

    do
    {
        ReturnValue = SpiFlash_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue != 0); // check the BUSY bit

}

/**
 * @brief       This function do the page programming to SPI Flash device.
 *
 * @param[in]   *DataBuffer	A Point that point to source data buffer. 
 *									
 * @param[in]   StartAddress	A start address of SPI Flash that will be programmed.
 *									
 * @param[in]   ByteCount	Byte count number that will be programmed.
 *
 * @return      None
 */
void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int Counter;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // send data to program
        au32SourceData = DataBuffer[Counter];
        SPI_WRITE_TX(SPI2, au32SourceData);
        while(SPI_IS_BUSY(SPI2));
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

}


/**
 * @brief       This function do the data reading from SPI Flash device. 
 *
 * @param[in]   *DataBuffer	A Point that point to destination data buffer. 
 *									
 * @param[in]   StartAddress	A start address of SPI Flash that will be read.
 *									
 * @param[in]   ByteCount	Byte count number that will be read.
 *
 * @return      None
 */
void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;
    unsigned int Counter;

    // /CS: active
    SPI_SET_SS_LOW(SPI2);

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 24 bits
    SPI_SET_DATA_WIDTH(SPI2, 24);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_WRITE_TX(SPI2, au32SourceData);
    while(SPI_IS_BUSY(SPI2));

    // configure transaction length as 8 bits
    SPI_SET_DATA_WIDTH(SPI2, 8);

    I2S_CLR_RX_FIFO(SPI2);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // receive
        au32SourceData = 0x0;
        SPI_WRITE_TX(SPI2, au32SourceData);
        while(SPI_IS_BUSY(SPI2));

        // dump Rx register
        au32DestinationData = SPI_READ_RX(SPI2);
        DataBuffer[Counter] = (unsigned char) au32DestinationData;
    }

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI2);

}

/*@}*/ /* end of group M451_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-M451_Basic01 */

/*@}*/ /* end of group M451_Library */

/*** (C) COPYRIGHT 2013~2015 Nuvoton Technology Corp. ***/
