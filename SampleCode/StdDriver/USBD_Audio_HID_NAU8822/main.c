/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to implement a USB audio class device with HID key.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 * @note
 * Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "usbd_audio.h"

uint32_t g_u32M45xG_E; /* 0: M45xD/M45xC;  1: M45xG/M45xE. */

extern uint8_t volatile g_u32EP4Ready;

void HID_UpdateKbData(void);

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and set HCLK divider to 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Set core clock */
    CLK_SetCoreClock(72000000);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(USBD_MODULE, 0, CLK_CLKDIV0_USB(3));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PLL, 0);

    /* Enable USB LDO33 */
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD.0, PD.1, PD.6, PD.4 and PD.5 as SPI1_I2SMCLK, UART0 TXD, UART0 RXD, I2C0_SDA and I2C0_SCL function pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD6MFP_Msk |
                       SYS_GPD_MFPL_PD4MFP_Msk | SYS_GPD_MFPL_PD5MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_SPI1_I2SMCLK | SYS_GPD_MFPL_PD1MFP_UART0_TXD | SYS_GPD_MFPL_PD6MFP_UART0_RXD |
                      SYS_GPD_MFPL_PD4MFP_I2C0_SDA | SYS_GPD_MFPL_PD5MFP_I2C0_SCL);

    /* Set I2S1 interface: I2S1_LRCLK (PA.4), I2S1_DO (PA.5), I2S1_DI (PA.6), I2S1_BCLK (PA.7) */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_SPI1_SS | SYS_GPA_MFPL_PA5MFP_SPI1_MOSI | SYS_GPA_MFPL_PA6MFP_SPI1_MISO | SYS_GPA_MFPL_PA7MFP_SPI1_CLK);

    /* Set PC.1 as CLKO pin */
	SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC1MFP_Msk;
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC1MFP_CLKO;

    /* Enable CLKO (PC.1) for monitor HCLK. CLKO = HCLK/8 Hz */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 2, 0);
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}

void KEY_Init(void)
{
    /* Init key I/O */
    PA->MODE |= (0x3ul << 8 * 2);
    PC->MODE |= (0x3ul << 8 * 2);
    PD->MODE |= (0x3ul << 11 * 2) | (0x3ul << 8 * 2);
    PF->MODE |= (0x3ul << 2 * 2);
    PA8 = 1;   // Play/pause
    PD11 = 1;  // Next (Not support on M45xD/M45xC)
    PF2 = 1;   // Previous
    PD8 = 1;   // Volumn Up
    PC8 = 1;   // Volumn Down

    /* Enable Debounce and set debounce time */
    PA->DBEN = (1 << 8);
    PC->DBEN = (1 << 8);
    PD->DBEN = (1 << 11) | (1 << 8);
    PF->DBEN = (1 << 2);
    GPIO->DBCTL = 0x17; // ~12.8 ms

}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    /*
        This sample code is used to demo USB Audio Class + NAU8822 (NAU8822) with HID key.
        User can define PLAY_RATE in usbd_audio.h to support 48000Hz, 32000Hz, 16000Hz and 8000Hz.

        The audio is input from NAU8822 AUXIN.
        The audio is output by NAU8822 Headphone output.

        NAU8822 is connect with I2S1 (PA.4~PA.7) and controlled by I2C0 (PD.4, PD.5).
        NAU8822 clock source is also come from I2S1 (MCLK, PD.0).
        Headphone MUTE control by PD.7.

        PC.1 is used to output clock (HCLK/8) to check HCLK frequency.

        HID key could be configured as HID keyboard or HID consumer (Media key). Default is HID consumer.
        (Defined in usbd_audio.h)

        keyboard:
        PA8 ==> 'a'
        PC8 ==> 'b'

        consumber:
        PA8  ==> Play/Pause
        PD11 ==> Next (Not support on M45xD/M45xC)
        PF2  ==> Previous
        PD8  ==> Vol+
        PC8  ==> Vol-
    */


    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    /* Check if M45xG or M45xE */
    PD->DOUT = 0xFFFF;
    if(PD->DOUT != 0xFFFF)
        g_u32M45xG_E = 0;
    else
        g_u32M45xG_E = 1;

    /* Init HID key */
    KEY_Init();

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C0 to access NAU8822 */
    I2C0_Init();

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Headphone MUTE off */
    PD->MODE |= (GPIO_MODE_QUASI << GPIO_MODE_MODE7_Pos);
    PD7 = 0;

    /* User can change audio codec settings through UART and I2C interfaces. Set PD.6 (UART0_RXD) to Quasi-bidirectional mode to avoid floating. */
    PD->MODE |= (GPIO_MODE_QUASI << GPIO_MODE_MODE6_Pos);
    
    I2S_Open(SPI1, I2S_MODE_SLAVE, PLAY_RATE, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(SPI1, 12000000);

    /* Fill dummy data to I2S TX for start I2S iteration */
    for(i = 0; i < 4; i++)
        I2S_WRITE_TX_FIFO(SPI1, 0);

    /* Start I2S play iteration */
    I2S_EnableInt(SPI1, I2S_FIFO_TXTH_INT_MASK | I2S_FIFO_RXTH_INT_MASK);

    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);
    /* Endpoint configuration */
    UAC_Init();
    USBD_Start();
    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);

    /* SPI (I2S) interrupt has higher frequency then USBD interrupt.
       Therefore, we need to set SPI (I2S) with higher priority to avoid
       SPI (I2S) interrupt pending too long time when USBD interrupt happen. */
    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_SetPriority(SPI1_IRQn, 2);


    /* start to IN data */
    g_u32EP4Ready = 1;


    while(SYS->PDID)
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);

        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if(!kbhit())
        {
            printf("\nEnter codec setting:\n");
            // Get Register number
            ch = getchar();
            u32Reg = ch - '0';
            ch = getchar();
            u32Reg = u32Reg * 10 + (ch - '0');
            printf("%d\n", u32Reg);

            // Get data
            ch = getchar();
            u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            printf("%03x\n", u32Data);
            I2C_WriteNAU8822(u32Reg,  u32Data);
        }

        /* HID Keyboard */
        HID_UpdateKbData();

    }
}

void HID_UpdateKbData(void)
{
    int32_t i;
    uint8_t *buf;
    uint32_t key = 0xF;
    static uint32_t preKey;
    int32_t n;

    n = 8;
    if(g_u32EP4Ready)
    {
        buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));

        // PA8, play/pause
        // PD11, Next (Not support on M45xD/M45xC)
        // PF2, Previous
        // PD8, Vol+
        // PC8, Vol-
        if(g_u32M45xG_E == 1)
            key = (!PA8) | (!PD11 << 1) | (!PF2 << 1) | (!PD8 << 1) | (!PC8 << 1);
        else
            key = (!PA8) | (!PF2 << 1) | (!PD8 << 1) | (!PC8 << 1);

        if(key == 0)
        {
            for(i = 0; i < n; i++)
            {
                buf[i] = 0;
            }

            if(key != preKey)
            {
                preKey = key;
                g_u32EP4Ready = 0;
                /* Trigger to note key release */
                USBD_SET_PAYLOAD_LEN(EP4, n);
            }
        }
        else
        {

#if(HID_FUNCTION == HID_KEYBOARD)
            preKey = key;

            if(!PA8)
                buf[2] = 0x04; /* Key 'a' */
            else if(!PC8)
                buf[2] = 0x05;

            g_u32EP4Ready = 0;
            USBD_SET_PAYLOAD_LEN(EP4, n);

#elif(HID_FUNCTION == HID_CONSUMER)
            // Don't repeat key when it is media key
            if(preKey != key)
            {
                preKey = key;
                buf[0] = 0;
                buf[1] = 0;
                if(!PA8)
                    buf[1] |= HID_CTRL_PAUSE;
                else if((!PD11) && (g_u32M45xG_E == 1))
                    buf[1] |= HID_CTRL_NEXT;
                else if(!PF2)
                    buf[1] |= HID_CTRL_PREVIOUS;
                else if(!PD8)
                    buf[0] |= HID_CTRL_VOLUME_INC;
                else if(!PC8)
                    buf[0] |= HID_CTRL_VOLUME_DEC;

                g_u32EP4Ready = 0;
                USBD_SET_PAYLOAD_LEN(EP4, n);
            }
#endif

        }
    }
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

