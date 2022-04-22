#ifndef __TASK_CURR_MEASURE_H
#define __TASK_CURR_MEASURE_H

#include "circularBuffer.h"
#include "cmsis_os.h"
#include "main.h"
#include "utils_bkte.h"
#include "utils_sd.h"

typedef struct {
    u16* p_buf;
    u16  size;

    u32 sum;
    u32 avg;
    u16 min;
    u16 max;

    u16 arr_max[100];
    u16 arr_min[100];
    u16 ptr_max;
    u16 ptr_min;
} adc_measure_t;

void unLockTasks();
void generateInitTelemetry();

#endif