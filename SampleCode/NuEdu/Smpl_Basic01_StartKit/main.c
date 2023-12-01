/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 15/09/02 10:03a $
 * @brief    Demonstrate how to set GPIO pin mode and use pin data input/output control.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "M451Series.h"
#include "NuEdu-Basic01.h"

#define PLL_CLOCK       72000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.6) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD6MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD6MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}
uint8_t a8IrDAtemp[4] = {0x13,0x25,0x37,~0x37};
void IrDA_Code_Exe(uint8_t* IR_CODE1)
{
    if(IR_CODE1[0]!=a8IrDAtemp[0])
        Write_Buzzer(1,20000,50);
    else if(IR_CODE1[1]!=a8IrDAtemp[1])
        Write_Buzzer(1,20000,50); 
    else if(IR_CODE1[2]!=a8IrDAtemp[2])
        Write_Buzzer(1,20000,50); 
    else if(IR_CODE1[3]!=a8IrDAtemp[3])
        Write_Buzzer(1,20000,50);        
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t temp,MidDid;
    int32_t i,j,k = 1;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nNuEdu-SDK-M451 Start Kit\n");

    Open_Seven_Segment();
    Initial_Key_Input();
    Initial_PWM_LED();
    Initial_LED();
    I2C_EEPROM_Init();
    Open_ADC_Knob();
    Open_Buzzer();
    Open_SPI_Flash();
    IrDA_NEC_TxRx_Init();

    /* Read MID & DID */
	MidDid = SpiFlash_ReadMidDid();
	printf("MID and DID = 0x%x\n", MidDid);
	k=1;
	while(MidDid!=0xef14)

    /* Read/Write EEPROM */
    I2C_EEPROM_Write(0x0010,0x55);
    temp=I2C_EEPROM_Read(0x0010);
    printf("EEPROM = 0x%x\n", temp);
	while(temp!=0x55)
    
    
    while(1)
    {
        if(PD3)
        {
            for(i=0;i<10;i++)
            {
                //led test	                
                Write_LED_Bar(i);
                Show_Seven_Segment(i, k);

                //adc test
                temp = Get_ADC_Knob();					//Volume Range: 0 ~ 4095		
                temp=temp * (100) / 4096;
                PWM_LED(0x07,3000,temp,3000,temp,3000,temp);

                for(j=0;j<100;j++)
                    CLK_SysTickDelay(1000);          
            }
            k++;
            if(k>2) k = 1;
            //buzzer test
            if(Get_Key_Input()==0x01)
                Write_Buzzer(0,0,0);
            if(Get_Key_Input()==0x02)
                Write_Buzzer(1,1000,50);
            if(Get_Key_Input()==0x04)
                Write_Buzzer(1,10000,50);
            if(Get_Key_Input()==0x08)
                Write_Buzzer(1,20000,50);

            a8IrDAtemp[0]++;
            a8IrDAtemp[1]++;
            a8IrDAtemp[2]++;
            a8IrDAtemp[3] = ~a8IrDAtemp[2];
            SendNEC(a8IrDAtemp);
        }
    }
}
