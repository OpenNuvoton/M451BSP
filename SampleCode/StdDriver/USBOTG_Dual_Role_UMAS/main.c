/**************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how USB works as a dual role device.
 *           If it works as USB Host, it can access a mass storage device.
 *           If it works as USB Device, it acts as a mass storage device.
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M451Series.h"
#include "diskio.h"
#include "ff.h"
#include "usbh_core.h"
#include "usbh_umas.h"
#include "massstorage.h"

#define PLL_CLOCK           96000000

static UINT blen = 16 * 1024;
volatile UINT Timer = 0;        /* Performance timer (1kHz increment) */
DWORD acc_size;             /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;
FATFS FatFs[_VOLUMES];      /* File system object for logical drive */
char Line[256];             /* Console input buffer */
#if _USE_LFN
char Lfname[512];
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;       /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */
#else
BYTE Buff[4096] __attribute__ ((__align(32) ));
#endif

void Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}

uint8_t bIsBdevice = 0, bIsAdevice = 0;

/* OTG interrupt handler */
void USBOTG_IRQHandler(void)
{
    __IO uint32_t u32INTSTS, u32INTEN;

    u32INTEN = OTG->INTEN;
    u32INTSTS = OTG->INTSTS;

    if(u32INTSTS & u32INTEN & OTG_INTSTS_IDCHGIF_Msk)
    {
        printf("[ID changed]\n");
        /* Check ID status */
        if(OTG_GET_STATUS(OTG_STATUS_IDSTS_Msk))
            bIsAdevice = 0;

        /* Clear ID status changed interrupt flag */
        OTG_CLR_INT_FLAG(OTG_INTSTS_IDCHGIF_Msk);
    }

    if(u32INTSTS & u32INTEN & OTG_INTSTS_BVLDCHGIF_Msk)
    {
        printf("[B session valid (OTG_STATUS: 0x%x)]\n", OTG->STATUS);
        /* Check ID status */
        if(OTG_GET_STATUS(OTG_STATUS_IDSTS_Msk) == 0)
            bIsBdevice = 0;

        /* Clear B-device session valid state change interrupt flag */
        OTG_CLR_INT_FLAG(OTG_INTSTS_BVLDCHGIF_Msk);
    }

    /* Clear all interrupt flags */
    OTG->INTSTS = u32INTSTS;
}




/*----------------------------------------------*/
/* Get a value of the string                    */
/*----------------------------------------------*/
/*  "123 -5   0x3ff 0b1111 0377  w "
        ^                           1st call returns 123 and next ptr
           ^                        2nd call returns -5 and next ptr
                   ^                3rd call returns 1023 and next ptr
                          ^         4th call returns 15 and next ptr
                               ^    5th call returns 255 and next ptr
                                  ^ 6th call fails and returns 0
*/

int xatoi(          /* 0:Failed, 1:Successful */
    TCHAR **str,    /* Pointer to pointer to the string */
    long *res       /* Pointer to a variable to store the value */
)
{
    unsigned long val;
    unsigned char r, s = 0;
    TCHAR c;


    *res = 0;
    while((c = **str) == ' ')(*str)++;      /* Skip leading spaces */

    if(c == '-')        /* negative? */
    {
        s = 1;
        c = *(++(*str));
    }

    if(c == '0')
    {
        c = *(++(*str));
        switch(c)
        {
            case 'x':       /* hexadecimal */
                r = 16;
                c = *(++(*str));
                break;
            case 'b':       /* binary */
                r = 2;
                c = *(++(*str));
                break;
            default:
                if(c <= ' ') return 1;  /* single zero */
                if(c < '0' || c > '9') return 0;    /* invalid char */
                r = 8;      /* octal */
        }
    }
    else
    {
        if(c < '0' || c > '9') return 0;    /* EOL or invalid char */
        r = 10;         /* decimal */
    }

    val = 0;
    while(c > ' ')
    {
        if(c >= 'a') c -= 0x20;
        c -= '0';
        if(c >= 17)
        {
            c -= 7;
            if(c <= 9) return 0;    /* invalid char */
        }
        if(c >= r) return 0;        /* invalid char for current radix */
        val = val * r + c;
        c = *(++(*str));
    }
    if(s) val = 0 - val;            /* apply sign if needed */

    *res = val;
    return 1;
}


/*----------------------------------------------*/
/* Dump a block of byte array                   */

void put_dump(
    const unsigned char* buff,  /* Pointer to the byte array to be dumped */
    unsigned long addr,         /* Heading address value */
    int cnt                     /* Number of bytes to be dumped */
)
{
    int i;


    printf(_T("%08lX "), addr);

    for(i = 0; i < cnt; i++)
        printf(_T(" %02X"), buff[i]);

    putchar(' ');
    for(i = 0; i < cnt; i++)
        putchar((TCHAR)((buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.'));

    putchar('\n');
}


/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */
/*--------------------------------------------------------------------------*/

static
FRESULT scan_files(
    char* path      /* Pointer to the path name working buffer */
)
{
    DIR dirs;
    FRESULT res;
    BYTE i;
    char *fn;


    if((res = f_opendir(&dirs, path)) == FR_OK)
    {
        i = strlen(path);
        while(((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0])
        {
            if(_FS_RPATH && Finfo.fname[0] == '.') continue;
#if _USE_LFN
            fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
            fn = Finfo.fname;
#endif
            if(Finfo.fattrib & AM_DIR)
            {
                acc_dirs++;
                *(path + i) = '/';
                strcpy(path + i + 1, fn);
                res = scan_files(path);
                *(path + i) = '\0';
                if(res != FR_OK) break;
            }
            else
            {
                /*              printf("%s/%s\n", path, fn); */
                acc_files++;
                acc_size += Finfo.fsize;
            }
        }
    }

    return res;
}



void put_rc(FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    //FRESULT i;
    uint32_t i;
    for(i = 0; (i != (UINT)rc) && *p; i++)
    {
        while(*p++) ;
    }
    printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

void get_line(char *buff, int len)
{
    TCHAR c;
    int idx = 0;
//  DWORD dw;


    for(;;)
    {
        c = getchar();
        putchar(c);
        if(c == '\r') break;
        if((c == '\b') && idx) idx--;
        if((c >= ' ') && (idx < len - 1)) buff[idx++] = c;
    }
    buff[idx] = 0;

    putchar('\n');

}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
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

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Configure PLL */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, PLL_CLOCK);

    /* Switch HCLK clock source to PLL. HCLK clock rate is F_PLL/2. */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    /* OTG module clock divider as 2. USB clock rate is F_PLL/2. */
    CLK_SetModuleClock(OTG_MODULE, MODULE_NoMsk, CLK_CLKDIV0_USB(2));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(OTG_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(USBH_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;                  // PLL
    SystemCoreClock = PLL_CLOCK / 2;              // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PD multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

    /* Configure PA.3 and PA.2 as USB_VBUS_ST and USB_VBUS_EN */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_USB_VBUS_ST | SYS_GPA_MFPL_PA2MFP_USB_VBUS_EN);

}


static FIL file1, file2;        /* File objects */

int USBH_Process()
{
    char *ptr, *ptr2;
    long p1, p2, p3;
    BYTE *buf;
    FATFS *fs;              /* Pointer to file system object */
    FRESULT res;

    DIR dir;                /* Directory object */
    UINT s1, s2, cnt;
    static const BYTE ft[] = {0, 12, 16, 32};
    DWORD ofs = 0, sect = 0;

    USBH_MassInit();

    Delay(0x500000);
    USBH_ProcessHubEvents();

    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    f_mount(0, &FatFs[0]);

    for(;;)
    {

        if(!bIsAdevice)
        {
            printf("id change\n");
            return 0;
        }

        USBH_ProcessHubEvents();
        printf(_T(">"));
        ptr = Line;
        get_line(ptr, sizeof(Line));
        switch(*ptr++)
        {

            case 'q' :  /* Exit program */
                return 0;

            case '1' :  /* USB re-init */
                printf("USBH_Close() ----------\n");
                USBH_Close();
                printf("USBH_Open()  ----------\n");
                USBH_Open();
                USBH_MassInit();
                Delay(0x500000);
                USBH_ProcessHubEvents();
                printf("\n\nUSBH port status: 0x%x, 0x%x\n", USBH->HcRhPortStatus[0], USBH->HcRhPortStatus[1]);
                break;

            case '2' :  /* USB suspend */
                USBH_Suspend();
                printf("\n\nUSBH port status: 0x%x, 0x%x\n", USBH->HcRhPortStatus[0], USBH->HcRhPortStatus[1]);
                break;

            case '3' :  /* USB resume */
                USBH_Resume();
                printf("\n\nUSBH port status: 0x%x, 0x%x\n", USBH->HcRhPortStatus[0], USBH->HcRhPortStatus[1]);
                break;

            case 'd' :
                switch(*ptr++)
                {
                    case 'd' :  /* dd [<lba>] - Dump sector */
                        if(!xatoi(&ptr, &p2)) p2 = sect;
                        res = (FRESULT)disk_read(0, Buff, p2, 1);
                        if(res)
                        {
                            printf("rc=%d\n", (WORD)res);
                            break;
                        }
                        sect = p2 + 1;
                        printf("Sector:%lu\n", p2);
                        for(buf = (unsigned char*)Buff, ofs = 0; ofs < 0x200; buf += 16, ofs += 16)
                            put_dump(buf, ofs, 16);
                        break;

                    case 'i' :  /* di - Initialize disk */
                        printf("rc=%d\n", (WORD)disk_initialize(0));
                        break;
                }
                break;

            case 'b' :
                switch(*ptr++)
                {
                    case 'd' :  /* bd <addr> - Dump R/W buffer */
                        if(!xatoi(&ptr, &p1)) break;
                        for(ptr = (char*)&Buff[p1], ofs = p1, cnt = 32; cnt; cnt--, ptr += 16, ofs += 16)
                            put_dump((BYTE*)ptr, ofs, 16);
                        break;

                    case 'e' :  /* be <addr> [<data>] ... - Edit R/W buffer */
                        if(!xatoi(&ptr, &p1)) break;
                        if(xatoi(&ptr, &p2))
                        {
                            do
                            {
                                Buff[p1++] = (BYTE)p2;
                            }
                            while(xatoi(&ptr, &p2));
                            break;
                        }
                        for(;;)
                        {
                            printf("%04X %02X-", (WORD)p1, Buff[p1]);
                            get_line(Line, sizeof(Line));
                            ptr = Line;
                            if(*ptr == '.') break;
                            if(*ptr < ' ')
                            {
                                p1++;
                                continue;
                            }
                            if(xatoi(&ptr, &p2))
                                Buff[p1++] = (BYTE)p2;
                            else
                                printf("???\n");
                        }
                        break;

                    case 'r' :  /* br <sector> [<n>] - Read disk into R/W buffer */
                        if(!xatoi(&ptr, &p2)) break;
                        if(!xatoi(&ptr, &p3)) p3 = 1;
                        printf("rc=%u\n", disk_read(0, Buff, p2, p3));
                        break;

                    case 'w' :  /* bw <sector> [<n>] - Write R/W buffer into disk */
                        if(!xatoi(&ptr, &p2)) break;
                        if(!xatoi(&ptr, &p3)) p3 = 1;
                        printf("rc=%u\n", disk_write(0, Buff, p2, p3));
                        break;

                    case 'f' :  /* bf <n> - Fill working buffer */
                        if(!xatoi(&ptr, &p1)) break;
                        memset(Buff, (int)p1, sizeof(Buff));
                        break;

                }
                break;

            case 'f' :
                switch(*ptr++)
                {
                    case 'i' :  /* fi - Force initialized the logical drive */
                        put_rc(f_mount(0, &FatFs[0]));
                        break;

                    case 's' :  /* fs - Show logical drive status */
                        res = f_getfree("", (DWORD*)&p2, &fs);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("FAT type = FAT%u\nBytes/Cluster = %lu\nNumber of FATs = %u\n"
                               "Root DIR entries = %u\nSectors/FAT = %lu\nNumber of clusters = %lu\n"
                               "FAT start (lba) = %lu\nDIR start (lba,clustor) = %lu\nData start (lba) = %lu\n\n...",
                               ft[fs->fs_type & 3], fs->csize * 512UL, fs->n_fats,
                               fs->n_rootdir, fs->fsize, fs->n_fatent - 2,
                               fs->fatbase, fs->dirbase, fs->database
                              );
                        acc_size = acc_files = acc_dirs = 0;
#if _USE_LFN
                        Finfo.lfname = Lfname;
                        Finfo.lfsize = sizeof(Lfname);
#endif
                        res = scan_files(ptr);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("\r%u files, %lu bytes.\n%u folders.\n"
                               "%lu KB total disk space.\n%lu KB available.\n",
                               acc_files, acc_size, acc_dirs,
                               (fs->n_fatent - 2) * (fs->csize / 2), p2 * (fs->csize / 2)
                              );
                        break;
                    case 'l' :  /* fl [<path>] - Directory listing */
                        while(*ptr == ' ') ptr++;
                        res = f_opendir(&dir, ptr);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        p1 = s1 = s2 = 0;
                        for(;;)
                        {
                            res = f_readdir(&dir, &Finfo);
                            if((res != FR_OK) || !Finfo.fname[0]) break;
                            if(Finfo.fattrib & AM_DIR)
                            {
                                s2++;
                            }
                            else
                            {
                                s1++;
                                p1 += Finfo.fsize;
                            }
                            printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
                                   (Finfo.fattrib & AM_DIR) ? 'D' : '-',
                                   (Finfo.fattrib & AM_RDO) ? 'R' : '-',
                                   (Finfo.fattrib & AM_HID) ? 'H' : '-',
                                   (Finfo.fattrib & AM_SYS) ? 'S' : '-',
                                   (Finfo.fattrib & AM_ARC) ? 'A' : '-',
                                   (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
                                   (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, Finfo.fsize, Finfo.fname);
#if _USE_LFN
                            for(p2 = strlen(Finfo.fname); p2 < 14; p2++)
                                putchar(' ');
                            printf("%s\n", Lfname);
#else
                            putchar('\n');
#endif
                        }
                        printf("%4u File(s),%10lu bytes total\n%4u Dir(s)", s1, p1, s2);
                        if(f_getfree(ptr, (DWORD*)&p1, &fs) == FR_OK)
                            printf(", %10lu bytes free\n", p1 * fs->csize * 512);
                        break;

                    case 'o' :  /* fo <mode> <file> - Open a file */
                        if(!xatoi(&ptr, &p1)) break;
                        while(*ptr == ' ') ptr++;
                        put_rc(f_open(&file1, ptr, (BYTE)p1));
                        break;

                    case 'c' :  /* fc - Close a file */
                        put_rc(f_close(&file1));
                        break;

                    case 'e' :  /* fe - Seek file pointer */
                        if(!xatoi(&ptr, &p1)) break;
                        res = f_lseek(&file1, p1);
                        put_rc(res);
                        if(res == FR_OK)
                            printf("fptr=%lu(0x%lX)\n", file1.fptr, file1.fptr);
                        break;

                    case 'd' :  /* fd <len> - read and dump file from current fp */
                        if(!xatoi(&ptr, &p1)) break;
                        ofs = file1.fptr;
                        while(p1)
                        {
                            if((UINT)p1 >= 16)
                            {
                                cnt = 16;
                                p1 -= 16;
                            }
                            else
                            {
                                cnt = p1;
                                p1 = 0;
                            }
                            res = f_read(&file1, Buff, cnt, &cnt);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            if(!cnt) break;
                            put_dump(Buff, ofs, cnt);
                            ofs += 16;
                        }
                        break;

                    case 'r' :  /* fr <len> - read file */
                        if(!xatoi(&ptr, &p1)) break;
                        p2 = 0;
                        Timer = 0;
                        while(p1)
                        {
                            if((UINT)p1 >= blen)
                            {
                                cnt = blen;
                                p1 -= blen;
                            }
                            else
                            {
                                cnt = p1;
                                p1 = 0;
                            }
                            res = f_read(&file1, Buff, cnt, &s2);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            p2 += s2;
                            if(cnt != s2) break;
                        }
                        printf("%lu bytes read with %lu kB/sec.\n", p2, p2 / Timer);
                        break;

                    case 'w' :  /* fw <len> <val> - write file */
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
                        memset(Buff, (BYTE)p2, blen);
                        p2 = 0;
                        Timer = 0;
                        while(p1)
                        {
                            if((UINT)p1 >= blen)
                            {
                                cnt = blen;
                                p1 -= blen;
                            }
                            else
                            {
                                cnt = p1;
                                p1 = 0;
                            }
                            res = f_write(&file1, Buff, cnt, &s2);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            p2 += s2;
                            if(cnt != s2) break;
                        }
                        printf("%lu bytes written with %lu kB/sec.\n", p2, p2 / Timer);
                        break;

                    case 'n' :  /* fn <old_name> <new_name> - Change file/dir name */
                        while(*ptr == ' ') ptr++;
                        ptr2 = strchr(ptr, ' ');
                        if(!ptr2) break;
                        *ptr2++ = 0;
                        while(*ptr2 == ' ') ptr2++;
                        put_rc(f_rename(ptr, ptr2));
                        break;

                    case 'u' :  /* fu <name> - Unlink a file or dir */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_unlink(ptr));
                        break;

                    case 'v' :  /* fv - Truncate file */
                        put_rc(f_truncate(&file1));
                        break;

                    case 'k' :  /* fk <name> - Create a directory */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_mkdir(ptr));
                        break;

                    case 'a' :  /* fa <atrr> <mask> <name> - Change file/dir attribute */
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
                        while(*ptr == ' ') ptr++;
                        put_rc(f_chmod(ptr, p1, p2));
                        break;

                    case 't' :  /* ft <year> <month> <day> <hour> <min> <sec> <name> - Change timestamp */
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                        Finfo.fdate = (WORD)(((p1 - 1980) << 9) | ((p2 & 15) << 5) | (p3 & 31));
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                        Finfo.ftime = (WORD)(((p1 & 31) << 11) | ((p1 & 63) << 5) | ((p1 >> 1) & 31));
                        put_rc(f_utime(ptr, &Finfo));
                        break;

                    case 'x' : /* fx <src_name> <dst_name> - Copy file */
                        while(*ptr == ' ') ptr++;
                        ptr2 = strchr(ptr, ' ');
                        if(!ptr2) break;
                        *ptr2++ = 0;
                        while(*ptr2 == ' ') ptr2++;
                        printf("Opening \"%s\"", ptr);
                        res = f_open(&file1, ptr, FA_OPEN_EXISTING | FA_READ);
                        putchar('\n');
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("Creating \"%s\"", ptr2);
                        res = f_open(&file2, ptr2, FA_CREATE_ALWAYS | FA_WRITE);
                        putchar('\n');
                        if(res)
                        {
                            put_rc(res);
                            f_close(&file1);
                            break;
                        }
                        printf("Copying...");
                        p1 = 0;
                        for(;;)
                        {
                            res = f_read(&file1, Buff, sizeof(Buff), &s1);
                            if(res || s1 == 0) break;    /* error or eof */
                            res = f_write(&file2, Buff, s1, &s2);
                            p1 += s2;
                            if(res || s2 < s1) break;    /* error or disk full */
                        }
                        printf("\n%lu bytes copied.\n", p1);
                        f_close(&file1);
                        f_close(&file2);
                        break;
#if _FS_RPATH
                    case 'g' :  /* fg <path> - Change current directory */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_chdir(ptr));
                        break;

                    case 'j' :  /* fj <drive#> - Change current drive */
                        if(xatoi(&ptr, &p1))
                        {
                            put_rc(f_chdrive((BYTE)p1));
                        }
                        break;
#endif
#if _USE_MKFS
                    case 'm' :  /* fm <partition rule> <sect/clust> - Create file system */
                        if(!xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                        printf("The memory card will be formatted. Are you sure? (Y/n)=");
                        get_line(ptr, sizeof(Line));
                        if(*ptr == 'Y')
                            put_rc(f_mkfs(0, (BYTE)p2, (WORD)p3));
                        break;
#endif
                    case 'z' :  /* fz [<rw size>] - Change R/W length for fr/fw/fx command */
                        if(xatoi(&ptr, &p1) && p1 >= 1 && (size_t)p1 <= sizeof(Buff))
                            blen = p1;
                        printf("blen=%u\n", blen);
                        break;
                }
                break;
            case '?':       /* Show usage */
                printf(
                    _T("dd [<lba>] - Dump sector\n")
                    _T("di - Initialize disk\n")
                    //_T("ds <pd#> - Show disk status\n")
                    _T("\n")
                    _T("bd <ofs> - Dump working buffer\n")
                    _T("be <ofs> [<data>] ... - Edit working buffer\n")
                    _T("br <pd#> <sect> [<num>] - Read disk into working buffer\n")
                    _T("bw <pd#> <sect> [<num>] - Write working buffer into disk\n")
                    _T("bf <val> - Fill working buffer\n")
                    _T("\n")
                    _T("fi - Force initialized the logical drive\n")
                    _T("fs - Show volume status\n")
                    _T("fl [<path>] - Show a directory\n")
                    _T("fo <mode> <file> - Open a file\n")
                    _T("fc - Close the file\n")
                    _T("fe <ofs> - Move fp in normal seek\n")
                    //_T("fE <ofs> - Move fp in fast seek or Create link table\n")
                    _T("fd <len> - Read and dump the file\n")
                    _T("fr <len> - Read the file\n")
                    _T("fw <len> <val> - Write to the file\n")
                    _T("fn <object name> <new name> - Rename an object\n")
                    _T("fu <object name> - Unlink an object\n")
                    _T("fv - Truncate the file at current fp\n")
                    _T("fk <dir name> - Create a directory\n")
                    _T("fa <atrr> <mask> <object name> - Change object attribute\n")
                    _T("ft <year> <month> <day> <hour> <min> <sec> <object name> - Change timestamp of an object\n")
                    _T("fx <src file> <dst file> - Copy a file\n")
                    _T("fg <path> - Change current directory\n")
                    _T("fj <ld#> - Change current drive\n")
                    _T("fm <ld#> <rule> <cluster size> - Create file system\n")
                    _T("\n")
                );
                break;
        }
    }
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Register write-protection disabled */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|                                                     |\n");
    printf("|  M451 OTG ID dependent Mass Storage sample program  |\n");
    printf("|                                                     |\n");
    printf("+-----------------------------------------------------+\n");

    printf("The USB role is determined by the ID pin state of USB cable.\n");
    printf("If acts as a USB host, it can access a mass storage device with a simple file system. ");
    printf("Type '?' on command line to show all supported commands.\n");
    printf("If attempts to switch the USB role as a USB device, press 'Enter' key after unplugging the USB cable to exit the file system. ");
    printf("Then plug a proper USB connector to switch the USB role as a mass storage device. The internal data flash is used as the storage.\n");
    printf("Note: If acts as a USB host, need a power switch (NCT3520U, enable input is active high) to control USB VBUS.\n");
    printf("      PA.3 and PA.2 are configured as USB_VBUS_ST function pin and USB_VBUS_EN function pin respectively.\n");
    printf("      If acts as a USB device, remember to enable data flash and set the base address as 0x10000.\n\n");

    USBH_Open();
    SYS_UnlockReg();
    /* Enable USB LDO. USB interface is used as a ID dependent device. */
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk | (2 << SYS_USBPHY_USBROLE_Pos);

    /* Clear interrupt flags */
    OTG->INTSTS = 0xffff;
    NVIC_EnableIRQ(USBOTG_IRQn);
    /* Enable ID detection function */
    OTG_ENABLE_ID_DETECT();
    Delay(1000);

    while(1)
    {
        if(OTG_GET_STATUS(OTG_STATUS_IDSTS_Msk))   /* B-device */
        {
            /* Enable USB PHY */
            USBD_ENABLE_PHY();
            Delay(1000);
            if(OTG_GET_STATUS(OTG_STATUS_VBUSVLD_Msk))   /* plug-in */
            {
                bIsBdevice = 1;
                printf("B-device (OTG_STATUS: 0x%x)\n", OTG->STATUS);
                USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

                /* Endpoint configuration */
                MSC_Init();

                SYS_UnlockReg();
                /* Enable USBD interrupt */
                NVIC_EnableIRQ(USBD_IRQn);

                /* Enable FMC ISP function */
                FMC_Open();

                /* Start transaction */
                while(1)
                {
                    if(USBD_IS_ATTACHED())
                    {
                        USBD_Start();
                        break;
                    }
                    if(OTG_GET_STATUS(OTG_STATUS_BVLD_Msk) == 0)
                        break;
                }

                /* Clear B-device session valid state change interrupt flag */
                OTG_CLR_INT_FLAG(OTG_INTSTS_BVLDCHGIF_Msk);
                /* Enable B-device session valid state change interrupt */
                OTG_ENABLE_INT(OTG_INTEN_BVLDCHGIEN_Msk);

                while(1)
                {
                    if(OTG_GET_STATUS(OTG_STATUS_BVLD_Msk) == 0)
                        break;
                    MSC_ProcessCmd();
                }
                /* Disable B-device session valid state change interrupt */
                OTG->INTEN &= ~OTG_INTEN_BVLDCHGIEN_Msk;
                /* Clear B-device session valid state change interrupt flag */
                OTG_CLR_INT_FLAG(OTG_INTSTS_BVLDCHGIF_Msk);
                printf("break-B (OTG_STATUS: 0x%x)\n", OTG->STATUS);
                /* Disable FMC ISP function */
                FMC_Close();
                /* Lock protected registers */
                SYS_LockReg();
            }
        }
        else    /* A-device */
        {
            bIsAdevice = 1;
            printf("A-device (OTG_STATUS: 0x%x)\n", OTG->STATUS);
            /* Clear ID status changed interrupt flag */
            OTG_CLR_INT_FLAG(OTG_INTSTS_IDCHGIF_Msk);
            /* Enable ID status changed interrupt */
            OTG_ENABLE_INT(OTG_INTEN_IDCHGIEN_Msk);
            USBH_Process();
            /* Disable ID status changed interrupt */
            OTG_DISABLE_INT(OTG_INTEN_IDCHGIEN_Msk);
        }
    }
}


/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
