/*
 * spiflash.h
 *
 *  Created on: 11 июн. 2020 г.
 *      Author: YOBA-i516G
 */

// MX25L6406E
#ifndef INC_SPIFLASH_H_
#define INC_SPIFLASH_H_
#include "cmsis_os.h"
#include "main.h"
#include "spi.h"
#include "utils_bkte.h"

#define SPIFLASH_CS_SEL   HAL_GPIO_WritePin(SPI2_CS_MEM_GPIO_Port, SPI2_CS_MEM_Pin, GPIO_PIN_RESET)
#define SPIFLASH_CS_UNSEL HAL_GPIO_WritePin(SPI2_CS_MEM_GPIO_Port, SPI2_CS_MEM_Pin, GPIO_PIN_SET)

#define SPIFLASH_WRDI         0x04
#define SPIFLASH_WREN         0x06
#define SPIFLASH_SE           0x20
#define SPIFLASH_BE           0xD8  // 0x52
#define SPIFLASH_CE           0xC7  // 0x60
#define SPIFLASH_CMD_JEDEC_ID 0x9F
#define SPIFLASH_RDSR         0x05
#define SPIFLASH_PP           0x02
#define SPIFLASH_FAST_READ    0x0B

#define SPIFLASH_NUM_PG_IN_SEC 16

#define SPIFLASH_NUM_PG_GNSS     512
#define BKTE_SAVE_NUM_PAGE       2047
#define SPIFLASH_NUM_SEC_GNSS    32
#define BKTE_BAD_PG_MAP_NUM_PAGE 2000

typedef struct
{
    u8  uniqID[8];
    u16 pgSz;
    u32 pgCnt;
    u32 secSz;
    u16 secCnt;
    u32 blSz;
    u8  blCnt;
    u32 capacityKb;
    u8  statusRegister1;
    u32 headNumPg;
    u32 tailNumPg;
} SPIFlash;

extern SPIFlash spiFlash64;

void spiFlashInit(u8* buf);

void spiFlashEC();
void spiFlashES(u32 numSec);
void spiFlashEB(u32 numBl);
u8   spiFlashWaitReady();

u32  spiFlashReadID();
void spiFlashWrEn();

u8   spiFlashTxRxCmd(u8* data, u16 sz);
u8   spiFlashTxData(u8* data, u16 sz);
u8   spiFlashRxData(u8* data, u16);
u8   spiFlashWriteNextPg(u8* pBuf, u32 sz, u32 offset);
u8   spiFlashReadLastPg(u8* pBuf, u32 sz, u32 offset);
u16  getDelayPages();
void spiFlashSaveInfo();

#endif /* INC_SPIFLASH_H_ */
