/**************************************************************************//**
 * @file     NuEdu-Basic01_EEPROM.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/09/02 10:02a $
 * @brief    NUC200 Series EEPROM Library
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_EEPROM.h"
#define I2C_EEPROM I2C1
#define I2C_EEPROM_IRQn I2C1_IRQn

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr = 0x50;
uint8_t g_au8TxData[3];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2CHandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C_EEPROM);

    if(I2C_GET_TIMEOUT_FLAG(I2C_EEPROM))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C_EEPROM);
    }
    else
    {
        if(s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Initial Function                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
__INLINE void I2C_PIN_Init(void)
{
    /* Set GPA multi-function pins for I2C1 SDA and SCL */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC4MFP_Msk;
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC4MFP_I2C1_SCL;

    SYS->GPE_MFPL &= ~SYS_GPE_MFPL_PE0MFP_Msk;
    SYS->GPE_MFPL |= SYS_GPE_MFPL_PE0MFP_I2C1_SDA;


    /* Enable I2C1 module clock */
    CLK_EnableModuleClock(I2C1_MODULE);

}

void I2C_EEPROM_Init(void)
{
    I2C_PIN_Init();
    /* Open I2C module and set bus clock */
    I2C_Open(I2C_EEPROM, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_EEPROM));

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C_EEPROM);
    NVIC_EnableIRQ(I2C_EEPROM_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C_EEPROM, (g_u8DeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C_EEPROM, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C_EEPROM);
        I2C_START(I2C_EEPROM);       
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen != 2)
        {
            I2C_SET_DATA(I2C_EEPROM, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C_EEPROM, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = (unsigned char) I2C_GET_DATA(I2C_EEPROM);
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_STO_SI);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C_EEPROM, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C_EEPROM, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C_EEPROM);
        I2C_START(I2C_EEPROM); 
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen != 3)
        {
            I2C_SET_DATA(I2C_EEPROM, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_STO_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write I2C EEPROM                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_EEPROM_Write(uint16_t u16Address, uint8_t u8Data)
{
    g_au8TxData[0] = u16Address >> 8;
    g_au8TxData[1] = u16Address & 0xFF;
    g_au8TxData[2] = u8Data;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_STA);

    /* Wait I2C Tx Finish */
    while(g_u8EndFlag == 0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Read I2C EEPROM                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_EEPROM_Read(uint16_t u16Address)
{
    g_au8TxData[0] = u16Address >> 8;
    g_au8TxData[1] = u16Address & 0xFF;

    g_u8DataLen = 0;
    g_u8EndFlag = 0;

    /* I2C function to write data to slave */
    s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;

    /* I2C as master sends START signal */
    I2C_SET_CONTROL_REG(I2C_EEPROM, I2C_CTL_STA);

    /* Wait I2C Tx Finish */
    while(g_u8EndFlag == 0);

    return g_u8RxData;
}




