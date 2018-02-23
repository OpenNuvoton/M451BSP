#ifndef __NuEdu_Basic01_SPI_FLASH_W_PDMA_H__
#define __NuEdu_Basic01_SPI_FLASH_W_PDMA_H__

extern void Open_SPI_Flash(void);
extern void Init_PDMA_CH1_for_SPI2_TX(uint32_t u32SrcAddr);
extern void Init_PDMA_CH2_for_SPI2_RX(uint32_t u32DstAddr);
extern unsigned int SpiFlash_w_PDMA_ReadMidDid(void);
extern void SpiFlash_w_PDMA_ChipErase(void);
extern unsigned int SpiFlash_w_PDMA_ReadStatusReg1(void);
extern unsigned int SpiFlash_w_PDMA_ReadStatusReg2(void);
extern void SpiFlash_w_PDMA_WaitReady(void);
extern void SpiFlash_w_PDMA_PageProgram(unsigned int u32SrcAddr, unsigned int StartAddress);
extern void SpiFlash_w_PDMA_ReadData(unsigned int u32DestAddr, unsigned int StartAddress);
#endif
