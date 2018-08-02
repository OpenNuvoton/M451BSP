#include <stdio.h>
#include "M451Series.h"
#include "NuEdu-Basic01_IrDA_NEC.h"

extern void ILI9341_LCD_PutString(uint16_t x, uint16_t y, uint8_t *s, uint32_t fColor, uint32_t bColor);
void IrDA_Code_Exe(uint8_t* IR_CODE1);

#define     Percent             0.04        // 4 % 
#define     MaxValue            0xFFFF
#define Yellow          0xFFE0
#define Red             0xF800
// Leader code range
#define     IR_LDC_MAX          (13460 * (1 + Percent))
#define     IR_LDC_MIN          (13460 * (1 - Percent))
// Repeater code range
#define     IR_RPC_MAX          (11280 * (1 + Percent))
#define     IR_RPC_MIN          (11280 * (1 - Percent))
// Bit = 1 range
#define     IR_BIT_1_MAX        (2236 * (1 + Percent))
#define     IR_BIT_1_MIN        (2236 * (1 - Percent))
// Bit = 0 range
#define     IR_BIT_0_MAX        (1120 * (1 + Percent))
#define     IR_BIT_0_MIN        (1120 * (1 - Percent))

volatile    uint8_t             IR_State = 0;       // IR State
volatile    uint8_t             IR_LDC_Ready = 0;   // LeaDer Code is Ready
volatile    uint8_t             IR_CTC_Ready = 0;   // CusTomer Code is Ready
volatile    uint8_t             IR_CTC0 = 0;        // Received CusTomer Code 0
volatile    uint8_t             IR_CTC1 = 0;        // Received CusTomer Code 1
volatile    uint8_t             IR_DAC = 0;         // Received DAta Code
volatile    uint8_t             IR_DAB = 0;         // Received DAta Bar code
volatile    uint8_t             IR_cnt = 0;
uint8_t     IR_CODE[4]  =   {0x00, 0x00, 0x00, 0x00};

void IrDa_NEC_Rx(uint32_t u32Time)
{

    //uint8_t IR_cnt, IR_CTC0, IR_CTC1, IR_DAC, IR_DAB;
    uint8_t Message[20];
    char* Message_Print;

    Message_Print = (char*)Message;


    if(IR_State == 0)
    {
        IR_LDC_Ready = 0;                                                   // Clear LeaDer Code Ready
        IR_CTC_Ready = 0;                                                   // Clear CusTomer Code Ready
        IR_State++;
    }
    // Leader or Repeater code
    else if(IR_State == 1)
    {
        // Leader code
        if((u32Time >= IR_LDC_MIN) && (u32Time <= IR_LDC_MAX))
        {
            IR_LDC_Ready = 1;                                               // Set LeaDer Code Ready
            IR_State++;
        }
        else
        {
            IR_State = 1;
            IR_LDC_Ready = 0;                                               // Clear LeaDer Code Ready
            IR_CTC_Ready = 0;                                               // Clear CusTomer Code Ready
        }
    }
    // Customer code 0
    else if((IR_State >= 2 && IR_State < 10) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_CTC0 = IR_CTC0 >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_CTC0 &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_CTC0 |= 0x80;
        else
            IR_State = 0;
    }
    // Customer code 1
    else if((IR_State >= 10 && IR_State < 18) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_CTC1 = IR_CTC1 >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_CTC1 &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_CTC1 |= 0x80;
        else
            IR_State = 0;
    }
    // Data
    else if((IR_State >= 18 && IR_State < 26) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_DAC = IR_DAC >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_DAC &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_DAC |= 0x80;
        else
            IR_State = 0;

    }
    // Data bar
    else if((IR_State >= 26 && IR_State < 34) && (IR_LDC_Ready == 1))
    {
        IR_State++;
        IR_DAB = IR_DAB >> 1;
        if((u32Time >= IR_BIT_0_MIN) && (u32Time <= IR_BIT_0_MAX))          // 1.12ms = 0
            IR_DAB &= 0x7f;
        else if((u32Time >= IR_BIT_1_MIN) && (u32Time <= IR_BIT_1_MAX))     // 2.25ms = 1
            IR_DAB |= 0x80;
        else
            IR_State = 0;

        if(IR_State == 34)
        {
            if((IR_DAC ^ IR_DAB) == 0xff)
            {
                IR_LDC_Ready = 0;                                           // Clear LeaDer Code Ready
                IR_CODE[0] = IR_CTC0;
                IR_CODE[1] = IR_CTC1;
                IR_CODE[2] = IR_DAC;
                IR_CODE[3] = IR_DAB;
                IR_cnt++;

                //print decode value to LCD
                sprintf(Message_Print, "IR_cnt= %02x,", IR_cnt);
                ILI9341_LCD_PutString(120, 0, Message, Red, Yellow);
                sprintf(Message_Print, "CTC0= %02x,", IR_CTC0);
                ILI9341_LCD_PutString(135, 0, Message, Red, Yellow);
                sprintf(Message_Print, "CTC1= %02x,", IR_CTC1);
                ILI9341_LCD_PutString(135, 72, Message, Red, Yellow);
                sprintf(Message_Print, "DAC= %02x,", IR_DAC);
                ILI9341_LCD_PutString(150, 0, Message, Red, Yellow);
                sprintf(Message_Print, "DAB= %02x,", IR_DAB);
                ILI9341_LCD_PutString(150, 64, Message, Red, Yellow);
            }
            IR_State = 0;
        }
    }
}

void PWM1P1_IRQHandler(void)
{
    uint32_t TDR1_tmp;
    TDR1_tmp = MaxValue - PWM_GET_CAPTURE_FALLING_DATA(PWM1, 3);
    PWM_ClearCaptureIntFlag(PWM1, 3, PWM_CAPTURE_INT_FALLING_LATCH);
    IrDa_NEC_Rx(TDR1_tmp);
}

#define     NEC_LDC_MARK        16      // 16 x 560us = 8960us =   9ms
#define     NEC_LDC_SPACE       8       //  8 x 560us = 4480us = 4.5ms
#define     NEC_BIT_MARK        1       // 560us
#define     NEC_ONE_SPACE       3       //  3 x 560us = 1680us = 1690us
#define     NEC_ZERO_SPACE      1       // 560us
#define     NEC_BYTES           4


//*******************************
// Mark
//
// Pulse = 1/3 duty @38KHz frequency
//*******************************
void Mark(uint8_t N)
{
    /* Switch to PWM output waveform */
    PWM_EnableOutput(PWM1, PWM_CH_4_MASK);
    CLK_SysTickDelay(560 * N);
    PWM_DisableOutput(PWM1, PWM_CH_4_MASK);

}
void SPACE(uint8_t N)
{
    CLK_SysTickDelay(560 * N);
}
//*******************************
// Send NEC IR Format
//
// PC13 (PWM1_CH4)
//*******************************
void SendNEC(uint8_t* data)
{
    uint8_t nbyte;
    uint8_t nbit;

    /* Send out Leader code */
    Mark(NEC_LDC_MARK);
    SPACE(NEC_LDC_SPACE);

    /* Send out Customer code and Data code */
    for(nbyte = 0; nbyte < NEC_BYTES; nbyte++)
    {
        for(nbit = 0; nbit < 8; nbit++)
        {
            Mark(NEC_BIT_MARK);
            if(data[nbyte] & (1 << nbit))       // LSB first
                SPACE(NEC_ONE_SPACE);
            else
                SPACE(NEC_ZERO_SPACE);
        }
    }
    /* Send out Stop bit */
    Mark(NEC_BIT_MARK);
}


void IrDA_NEC_TxRx_Init(void)
{
    /* Set PC multi-function pins for PWM1 Channel 3 and 4 */
    SYS->GPC_MFPH = SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk);
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC12MFP_PWM1_CH3;
    SYS->GPC_MFPH = SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC13MFP_Msk);
    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC13MFP_PWM1_CH4;

    /*-------------PWM output------------------*/
    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Reset PWM1 channel 0~5 */
    SYS_ResetModule(PWM1_RST);

    /* set PWM1 channel 4 output configuration */
    PWM_ConfigOutputChannel(PWM1, 4, 38000, 30);

    /* Enable Timer for PWM1 channel 4 */
    PWM_Start(PWM1, PWM_CH_4_MASK);

    /*-------------PWM capture------------------*/
    /* set PWM1 channel 3 capture configuration */
    PWM_ConfigCaptureChannel(PWM1, 3, 1000, 0);

    /* Enable capture falling edge interrupt for PWM1 channel 3 */
    PWM_EnableCaptureInt(PWM1, 3, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Enable PWM1 NVIC interrupt */
    NVIC_EnableIRQ(PWM1P1_IRQn);

    /* Enable Timer for PWM1 channel 3 */
    PWM_Start(PWM1, PWM_CH_3_MASK);

    /*Enable Input Schmitt Trigger*/
    PC->SMTEN |= GPIO_SMTEN_SMTEN12_Msk;

    /* Enable Capture Function for PWM1 channel 3 */
    PWM_EnableCapture(PWM1, PWM_CH_3_MASK);

    /* Enable falling capture reload */
    PWM1->CAPCTL |= PWM_CAPCTL_FCRLDEN3_Msk;

}



