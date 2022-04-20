#ifndef __TASK_WIRELESS_SENS_H
#define __TASK_WIRELESS_SENS_H

#include "circularBuffer.h"
#include "cmsis_os.h"
#include "main.h"
#include "utils_bkte.h"
#include "utils_crc.h"

#define SZ_HEADER_CRC16 6
#define SZ_BLOCK_SENS   6

typedef enum {
    BLOCK_WIRELESS_TEMP = 1,
    BLOCK_WIRELESS_RSSI
} TYPE_BLOCK_WIRELESS;

void parseWirelessSens(u8* data, u16 len);

#endif