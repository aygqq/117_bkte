#ifndef __TASK_GET_NEW_BIN_H
#define __TASK_GET_NEW_BIN_H

#include "cmsis_os.h"
#include "crc.h"
#include "main.h"
#include "utils_bkte.h"
#include "utils_flash.h"

#define SZ_PART_FIRMW 7000
#define SZ_TCP_PCKG   1400

void updBootInfo();
void lockAllTasks();
u32  getSzFirmware();

ErrorStatus getPartFirmware(u8* reqData, u8* answBuf, u16 szAnsw, u8 szReq);
#endif