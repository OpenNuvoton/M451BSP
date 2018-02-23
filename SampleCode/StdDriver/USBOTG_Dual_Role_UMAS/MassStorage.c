/******************************************************************************
 * @file     MassStorage.c
 * @brief    M451 series USBD mass storage sample file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "M451Series.h"
#include "massstorage.h"

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */
int32_t g_TotalSectors = 0;

uint8_t volatile g_u8EP2Ready = 0;
uint8_t volatile g_u8EP3Ready = 0;

/* USB flow control variables */
uint8_t g_u8BulkState;
uint8_t g_u8Prevent = 0;
uint8_t g_u8Size;
uint8_t g_au8SenseKey[4];

uint32_t g_u32DataFlashStartAddr;
uint32_t g_u32Address;
uint32_t g_u32Length;
uint32_t g_u32LbaAddress;
uint32_t g_u32BytesInStorageBuf;

uint32_t g_u32BulkBuf0, g_u32BulkBuf1;

/* CBW/CSW variables */
struct CBW g_sCBW;
struct CSW g_sCSW;

uint32_t MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t Storage_Block[STORAGE_BUFFER_SIZE / 4];

/*--------------------------------------------------------------------------*/
uint8_t g_au8InquiryID[36] =
{
    0x00,                   /* Peripheral Device Type */
    0x80,                   /* RMB */
    0x00,                   /* ISO/ECMA, ANSI Version */
    0x00,                   /* Response Data Format */
    0x1F, 0x00, 0x00, 0x00, /* Additional Length */

    /* Vendor Identification */
    'N', 'u', 'v', 'o', 't', 'o', 'n', ' ',

    /* Product Identification */
    'U', 'S', 'B', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e',

    /* Product Revision */
    '1', '.', '0', '0'
};

// code = 5Ah, Mode Sense
static uint8_t g_au8ModePage_01[12] =
{
    0x01, 0x0A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00
};

static uint8_t g_au8ModePage_05[32] =
{
    0x05, 0x1E, 0x13, 0x88, 0x08, 0x20, 0x02, 0x00,
    0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x05, 0x1E, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x68, 0x00, 0x00
};

static uint8_t g_au8ModePage_1B[12] =
{
    0x1B, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

static uint8_t g_au8ModePage_1C[8] =
{
    0x1C, 0x06, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00
};


void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if(USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if(u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
            /* USB bus suspend */
        }
        if(u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if(u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if(u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if(u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if(u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk OUT
            EP3_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if(u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if(u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if(u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
}


void EP2_Handler(void)
{
    g_u8EP2Ready = 1;
    MSC_AckCmd();
}


void EP3_Handler(void)
{
    /* Bulk OUT */
    g_u8EP3Ready = 1;
}


void MSC_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 3 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /*****************************************************/
    g_u32BulkBuf0 = EP3_BUF_BASE;
    g_u32BulkBuf1 = EP2_BUF_BASE;

    g_sCSW.dCSWSignature = CSW_SIGNATURE;
    g_TotalSectors = DATA_FLASH_STORAGE_SIZE / UDC_SECTOR_SIZE;
}

void MSC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80)    /* request data transfer direction */
    {
        // Device to host
        switch(buf[1])
        {
            case GET_MAX_LUN:
            {
                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 0;
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 1);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch(buf[1])
        {
            case BULK_ONLY_MASS_STORAGE_RESET:
            {
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);

                /* TODO: reset channel ? or trigger CBW? */

                break;
            }
            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}


void MSC_RequestSense(void)
{
    uint8_t tmp[20];

    memset(tmp, 0, 18);
    if(g_u8Prevent)
    {
        g_u8Prevent = 0;
        tmp[0] = 0x70;
    }
    else
        tmp[1] = 0xf0;

    tmp[2] = g_au8SenseKey[0];
    tmp[7] = 0x0a;
    tmp[12] = g_au8SenseKey[1];
    tmp[13] = g_au8SenseKey[2];
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), tmp, 20);

    g_au8SenseKey[0] = 0;
    g_au8SenseKey[1] = 0;
    g_au8SenseKey[2] = 0;
}

void MSC_ReadFormatCapacity(void)
{
    memset((uint8_t *)MassCMD_BUF, 0, 36);

    *((uint8_t *)(MassCMD_BUF + 3)) = 0x10;
    *((uint8_t *)(MassCMD_BUF + 4)) = *((uint8_t *)&g_TotalSectors + 3);
    *((uint8_t *)(MassCMD_BUF + 5)) = *((uint8_t *)&g_TotalSectors + 2);
    *((uint8_t *)(MassCMD_BUF + 6)) = *((uint8_t *)&g_TotalSectors + 1);
    *((uint8_t *)(MassCMD_BUF + 7)) = *((uint8_t *)&g_TotalSectors + 0);
    *((uint8_t *)(MassCMD_BUF + 8)) = 0x02;
    *((uint8_t *)(MassCMD_BUF + 10)) = 0x02;
    *((uint8_t *)(MassCMD_BUF + 12)) = *((uint8_t *)&g_TotalSectors + 3);
    *((uint8_t *)(MassCMD_BUF + 13)) = *((uint8_t *)&g_TotalSectors + 2);
    *((uint8_t *)(MassCMD_BUF + 14)) = *((uint8_t *)&g_TotalSectors + 1);
    *((uint8_t *)(MassCMD_BUF + 15)) = *((uint8_t *)&g_TotalSectors + 0);
    *((uint8_t *)(MassCMD_BUF + 18)) = 0x02;
}

void MSC_Read(void)
{
    uint32_t u32Len;

    if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf0);
    else
        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf1);

    /* Trigger to send out the data packet */
    USBD_SET_PAYLOAD_LEN(EP2, g_u8Size);

    g_u32Length -= g_u8Size;
    g_u32BytesInStorageBuf -= g_u8Size;

    if(g_u32Length)
    {
        if(g_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            g_u8Size = EP2_MAX_PKT_SIZE;
            if(g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            g_u32Address += g_u8Size;
        }
        else
        {
            u32Len = g_u32Length;
            if(u32Len > STORAGE_BUFFER_SIZE)
                u32Len = STORAGE_BUFFER_SIZE;

            MSC_ReadMedia(g_u32LbaAddress, u32Len, (uint8_t *)STORAGE_DATA_BUF);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            g_u8Size = EP2_MAX_PKT_SIZE;
            if(g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            g_u32Address += g_u8Size;
        }
    }
}

void MSC_ReadTrig(void)
{
    uint32_t u32Len;

    if(g_u32Length)
    {
        if(g_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            g_u8Size = EP2_MAX_PKT_SIZE;
            if(g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            g_u32Address += g_u8Size;
        }
        else
        {
            u32Len = g_u32Length;
            if(u32Len > STORAGE_BUFFER_SIZE)
                u32Len = STORAGE_BUFFER_SIZE;

            MSC_ReadMedia(g_u32LbaAddress, u32Len, (uint8_t *)STORAGE_DATA_BUF);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            g_u8Size = EP2_MAX_PKT_SIZE;
            if(g_u8Size > g_u32Length)
                g_u8Size = g_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), (uint8_t *)g_u32Address, g_u8Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
            g_u32Address += g_u8Size;
        }

        /* DATA0/DATA1 Toggle */
        if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
            USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf0);
        else
            USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf1);

        /* Trigger to send out the data packet */
        USBD_SET_PAYLOAD_LEN(EP2, g_u8Size);

        g_u32Length -= g_u8Size;
        g_u32BytesInStorageBuf -= g_u8Size;

    }
    else
        USBD_SET_PAYLOAD_LEN(EP2, 0);
}


void MSC_ReadCapacity(void)
{
    uint32_t tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    tmp = g_TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = *((uint8_t *)&tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 1)) = *((uint8_t *)&tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 2)) = *((uint8_t *)&tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 3)) = *((uint8_t *)&tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 6)) = 0x02;
}

void MSC_ModeSense10(void)
{
    uint8_t i, j;
    uint8_t NumHead, NumSector;
    uint16_t NumCyl = 0;

    /* Clear the command buffer */
    *((uint32_t *)MassCMD_BUF) = 0;
    *((uint32_t *)MassCMD_BUF + 1) = 0;

    switch(g_sCBW.au8Data[0])
    {
        case 0x01:
            *((uint8_t *)MassCMD_BUF) = 19;
            i = 8;
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_01[j];
            break;

        case 0x05:
            *((uint8_t *)MassCMD_BUF) = 39;
            i = 8;
            for(j = 0; j < 32; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_05[j];

            NumHead = 2;
            NumSector = 64;
            NumCyl = g_TotalSectors / 128;

            *((uint8_t *)(MassCMD_BUF + 12)) = NumHead;
            *((uint8_t *)(MassCMD_BUF + 13)) = NumSector;
            *((uint8_t *)(MassCMD_BUF + 16)) = (uint8_t)(NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 17)) = (uint8_t)(NumCyl & 0x00ff);
            break;

        case 0x1B:
            *((uint8_t *)MassCMD_BUF) = 19;
            i = 8;
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1B[j];
            break;

        case 0x1C:
            *((uint8_t *)MassCMD_BUF) = 15;
            i = 8;
            for(j = 0; j < 8; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1C[j];
            break;

        case 0x3F:
            *((uint8_t *)MassCMD_BUF) = 0x47;
            i = 8;
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_01[j];
            for(j = 0; j < 32; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_05[j];
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1B[j];
            for(j = 0; j < 8; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = g_au8ModePage_1C[j];

            NumHead = 2;
            NumSector = 64;
            NumCyl = g_TotalSectors / 128;

            *((uint8_t *)(MassCMD_BUF + 24)) = NumHead;
            *((uint8_t *)(MassCMD_BUF + 25)) = NumSector;
            *((uint8_t *)(MassCMD_BUF + 28)) = (uint8_t)(NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 29)) = (uint8_t)(NumCyl & 0x00ff);
            break;

        default:
            g_au8SenseKey[0] = 0x05;
            g_au8SenseKey[1] = 0x24;
            g_au8SenseKey[2] = 0x00;
    }
}

void MSC_Write(void)
{
    uint32_t lba, len;

    if(g_u32Length > EP3_MAX_PKT_SIZE)
    {
        if(USBD_GET_EP_BUF_ADDR(EP3) == g_u32BulkBuf0)
        {
            USBD_SET_EP_BUF_ADDR(EP3, g_u32BulkBuf1);
            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), EP3_MAX_PKT_SIZE);
        }
        else
        {
            USBD_SET_EP_BUF_ADDR(EP3, g_u32BulkBuf0);
            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), EP3_MAX_PKT_SIZE);
        }

        g_u32Address += EP3_MAX_PKT_SIZE;
        g_u32Length -= EP3_MAX_PKT_SIZE;

        /* Buffer full. Writer it to storage first. */
        if(g_u32Address >= (STORAGE_DATA_BUF + STORAGE_BUFFER_SIZE))
        {
            DataFlashWrite(g_u32DataFlashStartAddr, STORAGE_BUFFER_SIZE, (uint32_t)STORAGE_DATA_BUF);

            g_u32Address = STORAGE_DATA_BUF;
            g_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE;
        }
    }
    else
    {
        if(USBD_GET_EP_BUF_ADDR(EP3) == g_u32BulkBuf0)
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0), g_u32Length);
        else
            USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), g_u32Length);
        g_u32Address += g_u32Length;
        g_u32Length = 0;


        if(g_sCBW.u8OPCode == UFI_WRITE_10)
        {
            lba = get_be32(&g_sCBW.au8Data[0]);
            len = g_sCBW.dCBWDataTransferLength;

            len = lba * UDC_SECTOR_SIZE + g_sCBW.dCBWDataTransferLength - g_u32DataFlashStartAddr;
            if(len)
                DataFlashWrite(g_u32DataFlashStartAddr, len, (uint32_t)STORAGE_DATA_BUF);
        }

        g_u8BulkState = BULK_IN;
        MSC_AckCmd();
    }
}

void MSC_ProcessCmd(void)
{
    uint8_t u8Len;
    int32_t i;

    if(g_u8EP3Ready)
    {
        g_u8EP3Ready = 0;
        if(g_u8BulkState == BULK_CBW)
        {
            u8Len = USBD_GET_PAYLOAD_LEN(EP3);

            /* Check Signature & length of CBW */
            /* Bulk Out buffer */
            if((*(uint32_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0) != CBW_SIGNATURE) || (u8Len != 31))
            {
                return;
            }

            /* Get the CBW */
            for(i = 0; i < u8Len; i++)
                *((uint8_t *)(&g_sCBW.dCBWSignature) + i) = *(uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf0 + i);

            /* Prepare to echo the tag from CBW to CSW */
            g_sCSW.dCSWTag = g_sCBW.dCBWTag;

            /* Parse Op-Code of CBW */
            switch(g_sCBW.u8OPCode)
            {
                case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
                {
                    if(g_sCBW.au8Data[2] & 0x01)
                    {
                        g_au8SenseKey[0] = 0x05;  //INVALID COMMAND
                        g_au8SenseKey[1] = 0x24;
                        g_au8SenseKey[2] = 0;
                        g_u8Prevent = 1;
                    }
                    else
                        g_u8Prevent = 0;
                    g_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_VERIFY_10:
                case UFI_START_STOP:
                case UFI_TEST_UNIT_READY:
                {
                    g_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_REQUEST_SENSE:
                {
                    MSC_RequestSense();
                    g_u8BulkState = BULK_IN;
                    USBD_SET_PAYLOAD_LEN(EP2, 18);
                    return;
                }
                case UFI_READ_FORMAT_CAPACITY:
                {
                    if(g_u32Length == 0)
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = MassCMD_BUF;
                    }
                    MSC_ReadFormatCapacity();
                    g_u8BulkState = BULK_IN;
                    if(g_u32Length > 0)
                    {
                        if(g_u32Length > EP2_MAX_PKT_SIZE)
                            g_u8Size = EP2_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

                        g_u32Address += g_u8Size;
                        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf0);
                        MSC_Read();
                    }
//                     else
//                         MSC_AckCmd();
                    return;
                }
                case UFI_READ_CAPACITY:
                {
                    if(g_u32Length == 0)
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = MassCMD_BUF;
                    }

                    MSC_ReadCapacity();
                    g_u8BulkState = BULK_IN;
                    if(g_u32Length > 0)
                    {
                        if(g_u32Length > EP2_MAX_PKT_SIZE)
                            g_u8Size = EP2_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

                        g_u32Address += g_u8Size;
                        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf0);
                        MSC_Read();
                    }
//                     else
//                         MSC_AckCmd();
                    return;
                }
                case UFI_MODE_SELECT_10:
                {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32Address = MassCMD_BUF;

                    if(g_u32Length > 0)
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        g_u8BulkState = BULK_OUT;
                    }
                    return;
                }
                case UFI_MODE_SENSE_10:
                {
                    if(g_u32Length == 0)
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = MassCMD_BUF;
                    }

                    MSC_ModeSense10();
                    g_u8BulkState = BULK_IN;
                    if(g_u32Length > 0)
                    {
                        if(g_u32Length > EP2_MAX_PKT_SIZE)
                            g_u8Size = EP2_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32Length;
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);

                        g_u32Address += g_u8Size;

                        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf0);
                        MSC_Read();
                    }
//                     else
//                         MSC_AckCmd();
                    return;
                }
                case UFI_INQUIRY:
                {
                    /* Bulk IN buffer */
                    USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_au8InquiryID, 36);
                    USBD_SET_PAYLOAD_LEN(EP2, 36);
                    g_u8BulkState = BULK_IN;

                    return;
                }
                case UFI_READ_10:
                {
                    /* Check if it is a new transfer */
                    if(g_u32Length == 0)
                    {
                        /* Prepare the data for Bulk IN transfer */

                        /* Get LBA address */
                        g_u32Address = get_be32(&g_sCBW.au8Data[0]);
                        g_u32LbaAddress = g_u32Address * UDC_SECTOR_SIZE;
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32BytesInStorageBuf = g_u32Length;

                        i = g_u32Length;
                        if(i > STORAGE_BUFFER_SIZE)
                            i = STORAGE_BUFFER_SIZE;

                        MSC_ReadMedia(g_u32Address * UDC_SECTOR_SIZE, i, (uint8_t *)STORAGE_DATA_BUF);
                        g_u32BytesInStorageBuf = i;
                        g_u32LbaAddress += i;
                    }
                    g_u32Address = STORAGE_DATA_BUF;

                    /* Indicate the next packet should be Bulk IN Data packet */
                    g_u8BulkState = BULK_IN;

                    if(g_u32BytesInStorageBuf > 0)
                    {
                        /* Set the packet size */
                        if(g_u32BytesInStorageBuf > EP2_MAX_PKT_SIZE)
                            g_u8Size = EP2_MAX_PKT_SIZE;
                        else
                            g_u8Size = g_u32BytesInStorageBuf;

                        /* Prepare the first data packet (DATA1) */
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                        g_u32Address += g_u8Size;

                        /* kick - start */
                        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf1);
                        /* Trigger to send out the data packet */
                        USBD_SET_PAYLOAD_LEN(EP2, g_u8Size);
                        g_u32Length -= g_u8Size;
                        g_u32BytesInStorageBuf -= g_u8Size;
                    }
//                     else
//                         MSC_AckCmd();

                    return;
                }
                case UFI_WRITE_10:
                {
                    if(g_u32Length == 0)
                    {
                        g_u32Length = g_sCBW.dCBWDataTransferLength;
                        g_u32Address = STORAGE_DATA_BUF;
                        g_u32DataFlashStartAddr = get_be32(&g_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
                    }

                    if((g_u32Length > 0))
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        g_u8BulkState = BULK_OUT;
                    }
                    return;
                }
                default:
                {
                    /* Unsupported command */
                    g_au8SenseKey[0] = 0x05;
                    g_au8SenseKey[1] = 0x20;
                    g_au8SenseKey[2] = 0x00;

                    /* If CBW request for data phase, just return zero packet to end data phase */
                    if(g_sCBW.dCBWDataTransferLength > 0)
                    {
                        /* Data Phase, zero/short packet */
                        if((g_sCBW.bmCBWFlags & 0x80) != 0)
                        {
                            /* Data-In */
                            g_u8BulkState = BULK_IN;
                            USBD_SET_PAYLOAD_LEN(EP2, 0);
                        }
                    }
                    else
                    {
                        /* Status Phase */
                        g_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;
                }
            }
        }
        else if(g_u8BulkState == BULK_OUT)
        {
            switch(g_sCBW.u8OPCode)
            {
                case UFI_WRITE_10:
                case UFI_MODE_SELECT_10:
                {
                    MSC_Write();
                    return;
                }
            }
        }
    }
}

void MSC_AckCmd(void)
{
    /* Bulk IN */
    int32_t volatile idx;

    if(g_u8BulkState == BULK_CSW)
    {
        /* Prepare to receive the CBW */
        g_u8BulkState = BULK_CBW;

        USBD_SET_EP_BUF_ADDR(EP3, g_u32BulkBuf0);
        USBD_SET_PAYLOAD_LEN(EP3, 31);

    }
    else if(g_u8BulkState == BULK_IN)
    {
        switch(g_sCBW.u8OPCode)
        {
            case UFI_READ_FORMAT_CAPACITY:
            case UFI_READ_CAPACITY:
            case UFI_MODE_SENSE_10:
            {
                if(g_u32Length > 0)
                {
                    MSC_Read();
                    return;
                }
            }
            case UFI_READ_10:
            {
                if(g_u32Length > 0)
                {
                    MSC_ReadTrig();
                    return;
                }
            }

            case UFI_REQUEST_SENSE:
            case UFI_INQUIRY:
            {
                g_sCSW.dCSWDataResidue = 0;
                g_sCSW.bCSWStatus = 0;
                break;
            }

            case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
            case UFI_VERIFY_10:
            case UFI_START_STOP:
            case UFI_WRITE_10:
            {
                int32_t tmp;

                tmp = g_sCBW.dCBWDataTransferLength - STORAGE_BUFFER_SIZE;
                if(tmp < 0)
                    tmp = 0;

                g_sCSW.dCSWDataResidue = tmp;
                g_sCSW.bCSWStatus = 0;
                break;
            }
            case UFI_TEST_UNIT_READY:
            {
                g_sCSW.dCSWDataResidue = 0;
                g_sCSW.bCSWStatus = 0;
                break;
            }
            default:
            {
                /* Unsupported command. Return command fail status */
                g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength;
                g_sCSW.bCSWStatus = 0x01;
                break;
            }
        }

        /* Return the CSW */
        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf1);

        /* Bulk IN buffer */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)&g_sCSW.dCSWSignature, 16);

        g_u8BulkState = BULK_CSW;
        USBD_SET_PAYLOAD_LEN(EP2, 13);
    }
}

void MSC_ReadMedia(uint32_t addr, uint32_t size, uint8_t *buffer)
{
    DataFlashRead(addr, size, (uint32_t)buffer);
}

void MSC_WriteMedia(uint32_t addr, uint32_t size, uint8_t *buffer)
{
}



