#ifndef __TASK_KEEP_ALIVE_H
#define __TASK_KEEP_ALIVE_H

#include "adc.h"
#include "circularBuffer.h"
#include "cmsis_os.h"
#include "main.h"
#include "utils_bkte.h"
#include "utils_sd.h"

u16  getAdcVoltBat();
void pwrOffBkte();
void updRTC();
void generateMsgKeepAlive();
void generateMsgBat();
void generateMsgDevOff();

ErrorStatus sendMsgTaskStat();
ErrorStatus sendMsgFWUpdated();
ErrorStatus sendMsgFWUpdateBegin();
ErrorStatus sendMsgDevOn();
ErrorStatus sendMsgDevOff();
ErrorStatus sendMsgDevOffValue(u32 val);
ErrorStatus sendInitTelemetry();
ErrorStatus sendMsgStatistics();

#endif