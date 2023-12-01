/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of Cortex-M4 MPU.
 *
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M451Series.h"

#define PLL_CLOCK           72000000

#define Region_Size_1K     0x9
#define Region_Size_16K    0xD
#define Region_Size_32K    0xE
#define Region_Size_64K    0xF
#define Region_Size_128K   0x10
#define Region_Size_512K   0x12

/* MPU Attribute Register: Access Permission Definition */
#define AP_No_Access       0x0
#define AP_Pri_RW_User_NO  0x1
#define AP_Pri_RW_User_RO  0x2
#define AP_Pri_RW_User_RW  0x3
#define AP_Pri_RO_User_NO  0x5
#define AP_Pri_RO_User_RO  0x6

/* MPU Attribute Register: Region Enable Bit */
#define MPU_ATTR_EN        1

uint32_t ReadMemCore(uint32_t address)
{
    __IO uint32_t val = 0;
    uint32_t *a = (uint32_t*) address;
    val = *a;

    return val;
}

void MemManage_Handler(void)
{
    /* NOTE1: Disable MPU to allow simple return from mem_manage handler
              MemManage fault typically indicates code failure, and would
              be resolved by reset or terminating faulty thread in OS.
       NOTE2: The code set MPU->CTRL below will allow the code touch
              illegal address to be executed after return from
              MemManage_Handler(). If this line is comment out, this code
              will keep enter MemManage_Handler() */
    MPU->CTRL = 0x0;

    /* Clear Fault status register */
    SCB->CFSR = 0x000000BB;

    printf("\n Memory Fault !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_PLL, CLK_CLKDIV0_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD, TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);
}

void UART0_Init()
{
    UART_Open(UART0, 115200);
}

void MPU_Test(void)
{
    int32_t i32TestItem = 0;

    /*------------------------------
      Configure MPU memory regions
      ------------------------------*/

    /*
      Region 1 (Flash Memory Space)
      Start address = 0x0
      Size = 128KB
      Permission = Full access
    */
    /* Base address = Base address :OR: Region number :OR: VALID bit */
    MPU->RBAR = ((0x00000000 & MPU_RBAR_ADDR_Msk) | (0x1 & MPU_RBAR_REGION_Msk) | MPU_RBAR_VALID_Msk);
    /* Attribute = Full access :OR: SRD = 0 :OR: Size = 128KB :OR: ENABLE */
    MPU->RASR = ((AP_Pri_RW_User_RW << MPU_RASR_AP_Pos)| ( Region_Size_128K << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk);

    /*
      Region 2 (SRAM Memory Space)
      Start address = 0x20000000
      Size = 16KB
      Permission = Full access
    */

    /* Base address = Base address :OR: Region number :OR: VALID bit */
    MPU->RBAR = ((0x20000000 & MPU_RBAR_ADDR_Msk) | (0x2 & MPU_RBAR_REGION_Msk) | MPU_RBAR_VALID_Msk);
    /* Attribute = Full access :OR: SRD = 0 :OR: Size = 16KB :OR: ENABLE */
    MPU->RASR = ((AP_Pri_RW_User_RW << MPU_RASR_AP_Pos)| ( Region_Size_16K << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk);

    /*
      Region 3 (Test Memory Space)
      Start address = 0x20004000
      Size = 1KB
      Permission = No Access
    */
    /* Base address = Base address :OR: Region number :OR: VALID bit */
    MPU->RBAR = ((0x20004000 & MPU_RBAR_ADDR_Msk) | (0x3 & MPU_RBAR_REGION_Msk) | MPU_RBAR_VALID_Msk);
    /* Attribute = No Access :OR: SRD = 0 :OR: Size = 1KB :OR: ENABLE */
    MPU->RASR = ((AP_No_Access << MPU_RASR_AP_Pos)| ( Region_Size_1K << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk);

    /* Enable MemFault enable bit */
    SCB->SHCSR = SCB_SHCSR_MEMFAULTENA_Msk;
    /* Enable MPU */
    MPU->CTRL |= MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;

    printf("\n\n ==============================================\n");
    printf(" Memory Region 1 (Flash Memory) configuration:\n");
    printf(" ==============================================\n");
    printf(" Start address : 0x00000000\n");
    printf(" End address   : 0x0001FFFF\n");
    printf(" Size          : 128 KB\n");
    printf(" Permission    : Full access\n");
    printf(" ----------------------------------------------\n");
    printf(" Please Press '1' to read memory successfully from region 1 (Flash Memory).\n");

    while(i32TestItem != '1') i32TestItem = getchar();

    printf("\n Read value from 0x00000000 is 0x%08X.\n", ReadMemCore(0x00000000));

    printf("\n\n ==============================================\n");
    printf(" Memory Region 2 (SRAM Memory) configuration:\n");
    printf(" ==============================================\n");
    printf(" Start address : 0x20000000\n");
    printf(" End address   : 0x20003FFF\n");
    printf(" Size          : 16 KB\n");
    printf(" Permission    : Full access\n");
    printf(" ----------------------------------------------\n");
    printf(" Please Press '2' to read memory successfully from region 2 (SRAM Memory).\n");

    while(i32TestItem != '2') i32TestItem = getchar();

    printf("\n Read value from 0x20000000 is 0x%08X.\n", ReadMemCore(0x20000000));

    printf("\n\n ==============================================\n");
    printf(" Memory Region 3 (Test Memory) configuration:\n");
    printf(" ==============================================\n");
    printf(" Start address : 0x20004000\n");
    printf(" End address   : 0x200043FF\n");
    printf(" Size          : 1 KB\n");
    printf(" Permission    : No access\n");
    printf(" ----------------------------------------------\n");
    printf(" Please Press '3' to read memory from region 3 (Test Memory).\n");
    printf(" (It should trigger a memory fault exception!)\n");

    while(i32TestItem != '3') i32TestItem = getchar();

    /* Read memory from address 0x20004000 */
    ReadMemCore(0x20004000);
}

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n Start MPU test: \n");

    MPU_Test();

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
