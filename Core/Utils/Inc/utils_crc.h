#ifndef __UTILS_CRC_H
#define __UTILS_CRC_H
#include "main.h"

u16      calcCrc16(unsigned char *pcBlock, unsigned short len);
uint32_t crc32_byte(uint8_t *p, uint32_t bytelength);
void     crc32_chank(uint32_t *crc, uint8_t *p, uint32_t bytelength);
u16      crc16WirelesSens(uint8_t *pcBlock, uint16_t len);
#endif