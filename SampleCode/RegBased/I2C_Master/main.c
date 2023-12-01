/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 13 $
 * $Date: 15/09/02 10:03a $
 * @brief
 *           Show a Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8DeviceAddr;
static volatile uint8_t g_au8MstTxData[3];
static volatile uint8_t g_u8MstRxData;
static volatile uint8_t g_u8MstDataLen;
static volatile uint8_t g_u8MstEndFlag = 0;
static volatile uint8_t g_u8MstTxAbortFlag = 0;
static volatile uint8_t g_u8MstRxAbortFlag = 0;
static volatile uint8_t g_u8MstReStartFlag = 0;
static volatile uint8_t g_u8TimeoutFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C0->STATUS;

    if(I2C0->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C0->DAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 2)
        {
            I2C0->DAT = g_au8MstTxData[g_u8MstDataLen++];
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C0->DAT = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData = I2C0->DAT;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = I2C_TIMEOUT;
        while(I2C0->CTL & I2C_CTL_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C0->DAT = g_u8DeviceAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 3)
        {
            I2C0->DAT = g_au8MstTxData[g_u8MstDataLen++];
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            I2C_SET_DATA(I2C0, (uint32_t)((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = I2C_TIMEOUT;
        while(I2C0->CTL & I2C_CTL_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Enable external HXT 12MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for external HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    //PllClock        = PLL_CLOCK;            // PLL
    //SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    //CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock and I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk);

    /* Select UART module clock source as HXT */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C0_SDA | SYS_GPA_MFPL_PA3MFP_I2C0_SCL);

    /* I2C pins enable schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk);
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

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
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

int32_t Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    do
    {
        /* Enable I2C timeout */
        I2C0->TOCTL |= I2C_TOCTL_TOCEN_Msk;
        g_u8MstReStartFlag = 0;
        g_u8DeviceAddr = slvaddr;
        g_u8TimeoutFlag = 0;

        for(i = 0; i < 0x100; i++)
        {
            g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;

            /* I2C function to write data to slave */
            s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort */
            do
            {
                if(g_u8TimeoutFlag)
                {
                    printf(" MasterTx time out, any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init(100000);
                    /* Set MasterTx abort flag */
                    g_u8MstTxAbortFlag = 1;
                }
            } while(g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag */
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag */
                g_u8MstReStartFlag = 1;
                break;
            }

            /* I2C function to read data from slave */
            s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

            g_u8MstDataLen = 0;
            g_u8DeviceAddr = slvaddr;

            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort */
            do {
                if(g_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP */
                    printf(" MasterRx time out, any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init(100000);
                    /* Set MasterRx abort flag */
                    g_u8MstRxAbortFlag = 1;
                }
            } while(g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstRxAbortFlag )
            {
                /* Clear MasterRx abort flag */
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag */
                g_u8MstReStartFlag = 1;
                break;
            }
        }
    } while(g_u8MstReStartFlag); /* If unexpected abort happens, re-start the transmition */

    /* Compare data */
    if(g_u8MstRxData != g_au8MstTxData[2])
    {
        printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
        return -1;
    }

    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("| M451 I2C Driver Sample Code(Master) for access Slave  |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a master.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PA.2), I2C0_SCL(PA.3)\n");

    /* Init I2C0 */
    I2C0_Init(100000);

    printf("\n");
    printf("Check I2C Slave(I2C0) is running first!\n");
    printf("Press any key to continue.\n");
    getchar();

    /* Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    Read_Write_SLAVE(0x15);
    Read_Write_SLAVE(0x35);
    Read_Write_SLAVE(0x55);
    Read_Write_SLAVE(0x75);
    printf("SLAVE Address test OK.\n");

    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    Read_Write_SLAVE(0x15 & ~0x01);
    Read_Write_SLAVE(0x35 & ~0x04);
    Read_Write_SLAVE(0x55 & ~0x01);
    Read_Write_SLAVE(0x75 & ~0x04);
    printf("SLAVE Address Mask test OK.\n");

    s_I2C0HandlerFn = NULL;

    /* Close I2C0 */
    I2C0_Close();

    while(1);
}



