/*
 * utils_flash.h
 *
 *  Created on: 5 мая 2020 г.
 *      Author: annsi
 */

#ifndef INC_UTILS_FLASH_H_
#define INC_UTILS_FLASH_H_
#include "main.h"
#include "utils_bkte.h"

#define FLASH_ADDR_ID_BOOT          0x0800C000
#define FLASH_ADDR_IS_NEW_FIRMWARE  0x0800C004
#define FLASH_ADDR_SZ_NEW_FIRMWARE  0x0800C008
#define FLASH_ADDR_CRC_NEW_FIRMWARE 0x0800C00C
#define FLASH_ADDR_ERR_NEW_FIRMWARE 0x0800C010
#define FLASH_ADDR_START            0x08000000
#define FLASH_ADDR_FIRMWARE         0x08010000

#define FLASH_EMPTY_SECTOR 0xFFFFFFFF

#define FLASH_ADDR_BUF_NEW_FIRMWARE 0x080E0000

u32  getFlashData(u32 ADDR);
void flashClearPage(u32 page);
void flashWrite(u8* pData, u32 szHex, u32* distAddr);

#endif /* INC_UTILS_FLASH_H_ */
