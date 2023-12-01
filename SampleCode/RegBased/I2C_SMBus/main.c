/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 14 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Show how to control SMBus interface and use SMBus protocol between Host and Slave.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

#define SMBUS_ALERT_RESPONSE_ADDRESS 0x0C
#define SMBUS_DEFAULT_ADDRESS        0x61
#define ARP_COMMAND 0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
const uint8_t g_au8MasterAddr[4] = {0x15, 0x35, 0x55, 0x75};
const uint8_t g_au8SlaveAddr[4] = {0x18, 0x38, 0x58, 0x78};

volatile uint32_t slave_buff_addr;
volatile uint8_t g_u8SlvData[256];
volatile uint8_t g_au8RxData[4];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8ARA_Addr;
volatile uint8_t g_au8TxData[4];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen0;
volatile uint8_t g_u8DataLen1;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8SendPEC = 0;
volatile uint8_t g_u8AlertInt0 = 0;
volatile uint8_t g_u8AlertInt1 = 0;
volatile uint8_t g_u8AlertAddrAck0 = 0;
volatile uint8_t g_u8AlertAddrAck1 = 0;
volatile uint8_t g_u8PECErr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;
static volatile I2C_FUNC s_I2C1HandlerFn = NULL;


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C0->STATUS;
    //printf("I2C0 STS: 0x%X\n", u32Status);

    /* Check Transmit byte done interrupt flag */
    if((I2C0->BUSSTS & I2C_BUSSTS_BCDONE_Msk) == I2C_BUSSTS_BCDONE_Msk)
    {
        I2C0->BUSSTS |= I2C_BUSSTS_BCDONE_Msk;
        //printf("I2C0 Byte Transmit Byte Done Interrupt !\n");
        return;
    }

    /* Occur receive PEC packet error */
    if((I2C0->BUSSTS & I2C_BUSSTS_PECERR_Msk) == I2C_BUSSTS_PECERR_Msk)
    {
        I2C0->BUSSTS |= I2C_BUSSTS_PECERR_Msk;
        //printf("I2C0 PEC Error Interrupt !\n");
        return;
    }

    /* Check Alert Interrupt when I2C0 is Host */
    if(((I2C0->BUSSTS & I2C_BUSSTS_ALERT_Msk) == I2C_BUSSTS_ALERT_Msk) &
            ((I2C0->BUSCTL & I2C_BUSCTL_BMHEN_Msk) == I2C_BUSCTL_BMHEN_Msk))
    {
        I2C0->BUSSTS |= I2C_BUSSTS_ALERT_Msk;
        //printf("I2C0 Alert Interrupt !\n");
        g_u8AlertInt0 = 1;
        return ;
    }

    if(I2C0->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C0 Time-out flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C1->STATUS;
    //printf("I2C1 STS: 0x%X\n", u32Status);

    /* Check Transmit byte done interrupt flag */
    if((I2C1->BUSSTS & I2C_BUSSTS_BCDONE_Msk) == I2C_BUSSTS_BCDONE_Msk)
    {
        I2C1->BUSSTS |= I2C_BUSSTS_BCDONE_Msk;
        //printf("I2C1 Byte Receive Byte Done Interrupt !\n");
        return;
    }

    /* Occur receive PEC packet error */
    if((I2C1->BUSSTS & I2C_BUSSTS_PECERR_Msk) == I2C_BUSSTS_PECERR_Msk)
    {
        I2C1->BUSSTS |= I2C_BUSSTS_PECERR_Msk;
        //printf("I2C1 PEC Error Interrupt !\n");
        return;
    }

    /* Check Alert Interrupt when I2C1 is Host */
    if(((I2C1->BUSSTS & I2C_BUSSTS_ALERT_Msk) == I2C_BUSSTS_ALERT_Msk) &
            ((I2C1->BUSCTL & I2C_BUSCTL_BMHEN_Msk) == I2C_BUSCTL_BMHEN_Msk))
    {
        I2C1->BUSSTS |= I2C_BUSSTS_ALERT_Msk;
        //printf("I2C1 Alert Interrupt !\n");
        g_u8AlertInt1 = 1;
        return ;
    }

    if(I2C1->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C1 Timeout Flag */
        I2C1->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Tx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C0->DAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8TxData[g_u8DataLen0++];
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen0 != 3)
        {
            I2C0->DAT = g_au8TxData[g_u8DataLen0++];
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            if(g_u8SendPEC == 0)
            {
                g_u8SendPEC = 1;
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8EndFlag = 1;
            }
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Alert Callback Function                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterAlert(uint32_t u32Status)
{
    if(u32Status == 0x08)                                  /* START has been transmitted */
    {
        I2C0->DAT = (g_u8DeviceAddr << 1) + 1;             /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                             /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8TxData[g_u8DataLen0++];
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                             /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x40)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x50)
    {
        if(g_u8DataLen0 == 0)
        {
            g_au8RxData[g_u8DataLen0] = (unsigned char)(I2C0->DAT);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            g_u8DataLen0++;
        }
        else
        {
            g_au8RxData[g_u8DataLen0] = (unsigned char)(I2C0->DAT);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI_AA);
            g_u8AlertAddrAck0 = 1;
        }
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen0 != 3)
        {
            I2C0->DAT = g_au8TxData[g_u8DataLen0++];
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            if(g_u8SendPEC == 0)
            {
                g_u8SendPEC = 1;
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8EndFlag = 1;
            }
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Default Address and Acknowledge by Manual Callback Function                                 */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterDefaultAddrACKM(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C0->DAT = g_u8DeviceAddr << 1;        /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8TxData[g_u8DataLen0++];
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8SendPEC == 0)
        {
            g_u8SendPEC = 1;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
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
/*  I2C Slave TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen1] = (unsigned char)(I2C1->DAT);
        g_u8DataLen1++;

        if(g_u8DataLen1 == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }
        if(g_u8DataLen1 == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];
        }
        if(g_u8DataLen1 == 4)
        {
            if(g_au8RxData[3] != I2C1->PKTCRC)
            {
                g_u8PECErr = 1;
            }
            g_u8DataLen1 = 0;
        }
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {

        I2C1->DAT = g_u8SlvData[slave_buff_addr];
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Alert Callback Function                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveAlert(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen1] = (unsigned char)(I2C1->DAT);
        g_u8DataLen1++;

        if(g_u8DataLen1 == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }
        if(g_u8DataLen1 == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];

        }
        if(g_u8DataLen1 == 4)
        {
            if(g_au8RxData[3] != I2C1->PKTCRC)
            {
                g_u8PECErr = 1;
            }
            g_u8DataLen1 = 0;
        }
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C1->DAT = g_au8SlaveAddr[0];
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Default Address and Acknowledge by Manual Callback Function                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveDefaultAddrACKM(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0xF0)
    {
        /* Enable ACKMEN & SLV in Receive Mode */
        g_au8RxData[g_u8DataLen1] = I2C1->DAT;
        g_u8DataLen1++;

        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);           //Acknowledge by Manual
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen1] = (unsigned char)(I2C1->DAT);
        g_u8DataLen1++;

        if(g_u8DataLen1 == 2)
        {
            if(g_au8RxData[1] != I2C1->PKTCRC)
            {
                g_u8PECErr = 1;
            }
            g_u8DataLen1 = 0;
        }
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {

        I2C1->DAT = g_u8SlvData[slave_buff_addr];
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    //PllClock        = PLL_CLOCK;            // PLL
    //SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    //CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock and I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk | CLK_APBCLK0_I2C1CKEN_Msk);

    /* Select UART module clock source as HXT */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set I2C0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C0_SDA | SYS_GPA_MFPL_PA3MFP_I2C0_SCL);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA14MFP_I2C0_SMBAL | SYS_GPA_MFPH_PA15MFP_I2C0_SMBSUS);

    /* Set I2C1 multi-function pins */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE5MFP_Msk | SYS_GPE_MFPL_PE4MFP_Msk);
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE5MFP_I2C1_SDA | SYS_GPE_MFPL_PE4MFP_I2C1_SCL);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC6MFP_I2C1_SMBAL | SYS_GPC_MFPL_PC7MFP_I2C1_SMBSUS);

    /* I2C pins enable schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk);
    PE->SMTEN |= (GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk);
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


#define GET_PCLK0_DIV()     ((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk)? 1 : 0)
#define GET_PCLK1_DIV()     ((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK1SEL_Msk)? 1 : 0)

void I2C0_Init(uint32_t u32BusClock)
{
    uint32_t u32Div, u32Pclk;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;

    /* Get I2C0 PCLK0 Clock */
    u32Pclk = SystemCoreClock / (1<<(GET_PCLK0_DIV()));

    /* Calculate */
    u32Div = (uint32_t)(((u32Pclk * 10u) / (u32BusClock * 4u) + 5u) / 10u - 1u); /* Compute proper divider for I2C clock */
    I2C0->CLKDIV = u32Div;

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", (u32Pclk / ((u32Div + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(uint32_t u32BusClock)
{
    uint32_t u32Div, u32Pclk;

    /* Reset I2C1 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C1RST_Msk;

    /* Enable I2C1 Controller */
    I2C1->CTL |= I2C_CTL_I2CEN_Msk;

    /* Get I2C1 PCLK1 Clock */
    u32Pclk = SystemCoreClock / (1<<(GET_PCLK1_DIV()));

    /* Calculate */
    u32Div = (uint32_t)(((u32Pclk * 10u) / (u32BusClock * 4u) + 5u) / 10u - 1u); /* Compute proper divider for I2C clock */
    I2C1->CLKDIV = u32Div;

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", (u32Pclk / ((u32Div + 1) << 2)));

    /* Set I2C1 4 Slave addresses */
    /* Slave address : 0x16 */
    I2C1->ADDR0 = (I2C1->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (g_au8SlaveAddr[0] << I2C_ADDR0_ADDR_Pos);
    /* Slave address : 0x36 */
    I2C1->ADDR1 = (I2C1->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (g_au8SlaveAddr[1] << I2C_ADDR1_ADDR_Pos);
    /* Slave address : 0x56 */
    I2C1->ADDR2 = (I2C1->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (g_au8SlaveAddr[2] << I2C_ADDR2_ADDR_Pos);
    /* Slave address : 0x76 */
    I2C1->ADDR3 = (I2C1->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (g_au8SlaveAddr[3] << I2C_ADDR3_ADDR_Pos);

    /* Set I2C1 4 Slave addresses mask bits*/
    /* Slave address mask Bits: 0x04 */
    I2C1->ADDRMSK0 = (I2C1->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave address mask Bits: 0x02 */
    I2C1->ADDRMSK1 = (I2C1->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x02 << I2C_ADDRMSK1_ADDRMSK_Pos);
    /* Slave address mask Bits: 0x04 */
    I2C1->ADDRMSK2 = (I2C1->ADDRMSK2 & ~I2C_ADDRMSK2_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK2_ADDRMSK_Pos);
    /* Slave address mask Bits: 0x02 */
    I2C1->ADDRMSK3 = (I2C1->ADDRMSK3 & ~I2C_ADDRMSK3_ADDRMSK_Msk) | (0x02 << I2C_ADDRMSK3_ADDRMSK_Pos);

    /* Enable I2C1 interrupt and set corresponding NVIC bit */
    I2C1->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;

}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C1->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C1->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C1CKEN_Msk;
}

int32_t SMBusSendByteTest(uint8_t slvaddr)
{
    uint32_t i, u32TimeOutCnt;

    g_u8DeviceAddr = slvaddr;

    for(i = 0; i < 0x100; i++)
    {
        /* Init transmission bytes */
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen0 = 0;
        g_u8EndFlag = 0;
        g_u8SendPEC = 0;

        /* I2C0 function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C0 as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C0 transmit finish */
        u32TimeOutCnt = I2C_TIMEOUT;
        while(g_u8EndFlag == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for I2C transmit finish time-out!\n");
                return -1;
            }
        }
        g_u8EndFlag = 0;

        if(g_u8PECErr)
        {
            printf("PEC Check Error !\n");
            return -1;
        }
    }
    return 0;
}

int32_t SMBusAlertTest(uint8_t slvaddr)
{
    uint32_t u32TimeOutCnt;

    g_u8DeviceAddr = slvaddr;

    /* I2C function to Send Alert Response Address to bus */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterAlert;

    /* I2C0 Send Start condition */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Init receive data index */
    g_u8DataLen0 = 0;

    /* Waiting for Get Alert Address */
    u32TimeOutCnt = I2C_TIMEOUT;
    while(g_u8AlertAddrAck0 == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C get alert address time-out!\n");
            return -1;
        }
    }
    g_u8AlertAddrAck0 = 0;

    if(g_u8PECErr)
    {
        printf("PEC Check Error !\n");
        return -1;
    }

    return 0;
}

int32_t SMBusDefaultAddressTest(uint8_t slvaddr)
{
    uint32_t u32TimeOutCnt;

    g_u8DeviceAddr = slvaddr;

    /* Set Transmission ARP command */
    g_au8TxData[0] = ARP_COMMAND;

    g_u8DataLen0 = 0;
    g_u8DataLen1 = 0;
    g_u8EndFlag = 0;
    g_u8SendPEC = 0;

    /* I2C0 function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterDefaultAddrACKM;

    /* I2C0 as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C0 transmit finish */
    u32TimeOutCnt = I2C_TIMEOUT;
    while(g_u8EndFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C transmit finish time-out!\n");
            return -1;
        }
    }
    g_u8EndFlag = 0;

    if(g_u8PECErr)
    {
        printf("PEC Check Error !\n");
        return -1;
    }

    printf("\n");
    printf("Master Sends ARP Command(0x01) to Slave (0x%X) Test OK\n", slvaddr);

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i, ch = NULL, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    while(ch != 0x30)
    {
        /*
            This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
            and Byte Read operations, and check if the read data is equal to the programmed data.
        */
        printf("\n");
        printf("+-------------------------------------------------------+\n");
        printf("|  M451 I2C SMBUS Driver Sample Code                    |\n");
        printf("|                                                       |\n");
        printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
        printf("+-------------------------------------------------------+\n");

        printf("Configure I2C0 as master and I2C1 as a slave.\n");
        printf("\n");
        printf("I2C0_SDA(PA.2), I2C0_SCL(PA.3), I2C_ALERT(PA.14), I2C_SUSPEND(PA.15)\n");
        printf("I2C1_SDA(PE.5), I2C1_SCL(PE.4), I2C_ALERT(PC. 6), I2C_SUSPEND(PC. 7)\n\n");

        /* Init I2C0 */
        I2C0_Init(100000);

        /* Init I2C1 */
        I2C1_Init(100000);

        /* I2C1 enter no address SLV mode */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);


        printf("\n");
        printf("I2C1 Slave Mode is Running.\n");
        for(i = 0; i < 0x100; i++)
        {
            g_u8SlvData[i] = 0;
        }

        printf("\n");
        printf("[1] SMBus Send Bytes Protocol with PEC Test\n");
        printf("[2] SMBus Alert Function Test\n");
        printf("[3] Simple ARP and ACK Control by Manual Test\n");
        printf("[0] Exit\n");

        ch = getchar();
        if(ch == '1')
        {
            /* I2C1 Bus Management init */
            I2C1->BUSCTL |= (I2C_BUSCTL_BUSEN_Msk | I2C_BUSCTL_BMDEN_Msk);

            /* I2C1 Bus PEC check enable */
            I2C1->BUSCTL |= I2C_BUSCTL_PECEN_Msk ;

            /* I2C0 Bus Management enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_BUSEN_Msk | I2C_BUSCTL_BMHEN_Msk);

            /* I2C0 Bus PEC Check and transmit enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_PECEN_Msk | I2C_BUSCTL_PECTXEN_Msk);

            /* Set I2C0 Payload Bytes */
            I2C0->PKTSIZE = 4;                                   // I2C0: 1 address byte + 3 data bytes

            /* Set I2C1 Payload Bytes */
            I2C1->PKTSIZE = 4;                   // I2C1: 1 address byte + 3 data bytes

            g_u8PECErr = 0;

            /* I2C function to Slave receive/transmit data */
            s_I2C1HandlerFn = I2C_SlaveTRx;

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Send Bytes Protocol test ==\n");

            /* SMBus send byte protocol test*/
            if( SMBusSendByteTest(g_au8SlaveAddr[0]) < 0 ) goto lexit;

            printf("\n");
            printf("SMBus transmit data done.\n");
        }
        else if(ch == '2')
        {
            /* I2C0 Bus Management Enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_BUSEN_Msk | I2C_BUSCTL_BMHEN_Msk);

            /* I2C0 Bus PEC Check and transmit Enable */
            I2C0->BUSCTL |= I2C_BUSCTL_PECEN_Msk;

            /* I2C1 Bus Management init */
            I2C1->BUSCTL |= (I2C_BUSCTL_BUSEN_Msk | I2C_BUSCTL_BMDEN_Msk);

            /* I2C1 Bus PEC Check enable */
            I2C1->BUSCTL |= (I2C_BUSCTL_PECEN_Msk | I2C_BUSCTL_PECTXEN_Msk);

            /* Set I2C0 Payload bytes */
            I2C0->PKTSIZE = 2;                                   // I2C0: 1 address byte + 1 data byte

            /* Set I2C1 Payload bytes */
            I2C1->PKTSIZE = 2;                                   // I2C1: 1 address byte + 1 data byte

            /* Alert pin support if BMHEN(I2C0->BUSCTL[4]) = 0 */
            I2C0->BUSCTL |= I2C_BUSCTL_ALERTEN_Msk;

            /* Enable SUSCON pin function and output Hi */
            I2C0->BUSCTL |= I2C_BUSCTL_SCTLOEN_Msk;
            I2C0->BUSCTL |= I2C_BUSCTL_SCTLOSTS_Msk;

            /* Release Alert pin to Hi */
            I2C1->BUSCTL &= ~I2C_BUSCTL_ALERTEN_Msk;

            /* Set SUSCON pin is Input */
            I2C1->BUSCTL &= ~I2C_BUSCTL_SCTLOEN_Msk;

            g_u8PECErr = 0;

            /* I2C function to Slave receive/transmit data */
            s_I2C1HandlerFn = I2C_SlaveAlert;

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Alert Function Test ==\n");


            /* I2C1 has a Alert request */
            I2C1->BUSCTL |= I2C_BUSCTL_ALERTEN_Msk;
            printf("\n");
            printf("I2C1 has Alert Request and Alert Pin Pull Lo. \n");

            /* Wait I2C0 get Alert interrupt */
            u32TimeOutCnt = I2C_TIMEOUT;
            while(g_u8AlertInt0 == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for I2C alert interrupt time-out!\n");
                    goto lexit;
                }
            }

            /* I2C0 Get Alert Request */
            g_u8AlertInt0 = 0;
            printf("I2C0 Get Alert Interrupt Request\n");

            /* I2C0 Send Alert Response Address(ARA) to I2C bus */
            if( SMBusAlertTest(SMBUS_ALERT_RESPONSE_ADDRESS) < 0 ) goto lexit;

            /* Printf the Alert Slave address */
            printf("\n");
            printf("Get Alert Address 0x%X test OK.\n", g_au8RxData[0]);

            /* Show I2C1 SUSCON pin state before I2C0 pull Lo */
            printf("\n");
            printf("I2C1 SUSCON Pin state is %d\n", (int)((I2C1->BUSSTS & I2C_BUSSTS_SCTLDIN_Msk) >> 4));

            /* Output I2C0 SUSCON pin Lo */
            I2C0->BUSCTL &= ~I2C_BUSCTL_SCTLOSTS_Msk;
            printf("I2C0 SUSCON Pin output Lo\n");

            /* Show I2C1 SUSCON pin state after I2C0 pull Lo */
            printf("\n");
            printf("I2C1 SUSCON Pin state change to %d\n", (int)((I2C1->BUSSTS & I2C_BUSSTS_SCTLDIN_Msk) >> 4));

            printf("\n");
            printf("SMBus Alert Test Done\n");
        }
        else if(ch == '3')
        {
            /* I2C0 Bus management enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_BUSEN_Msk | I2C_BUSCTL_BMHEN_Msk);

            /* I2C0 Bus PEC check and transmit enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_PECEN_Msk | I2C_BUSCTL_PECTXEN_Msk);

            /* I2C1 Bus Management init */
            I2C1->BUSCTL |= (I2C_BUSCTL_BUSEN_Msk | I2C_BUSCTL_BMDEN_Msk);

            /* I2C1 Bus PEC check enable */
            I2C1->BUSCTL |= I2C_BUSCTL_PECEN_Msk ;

            /* I2C1 Acknowledge by Manual enable */
            I2C1->BUSCTL |= I2C_BUSCTL_ACKMEN_Msk;

            /* Set I2C0 Payload bytes */
            I2C0->PKTSIZE = 2;                                   // I2C0: 1 address byte  + 1 data byte

            /* Set I2C1 Payload bytes */
            I2C1->PKTSIZE = 2;                                   // I2C1: 1 address byte  + 1 data byte

            g_u8PECErr = 0;

            /* I2C function to Slave receive/transmit data */
            s_I2C1HandlerFn = I2C_SlaveDefaultAddrACKM;

            printf("\n");
            printf("== Simple ARP and Acknowledge by Manual Test ==\n");

            /* I2C0 sends Default Address and ARP Command (0x01) to Slave */
            if( SMBusDefaultAddressTest(SMBUS_DEFAULT_ADDRESS) < 0 ) goto lexit;

            /* Show I2C1 get ARP command from  I2C0 */
            printf("\n");
            printf("Slave Get ARP Command is 0x%X\n", g_au8RxData[0]);

            /* Check Slave get command */
            printf("\n");
            if(g_au8RxData[0] != ARP_COMMAND)
            {
                printf("Get Wrong ARP Command, Please check again !\n");
            }
            else
            {
                printf("Default Address and Acknowledge by Manual test OK.\n");
            }
        }
    }

lexit:

    s_I2C0HandlerFn = NULL;
    s_I2C1HandlerFn = NULL;

    printf("SMBus Test Exit\n");

    /* Close I2C0 */
    I2C0_Close();

    /* Close I2C1 */
    I2C1_Close();

    while(1);
}



