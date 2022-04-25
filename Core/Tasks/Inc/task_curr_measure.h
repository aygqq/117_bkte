#ifndef __TASK_CURR_MEASURE_H
#define __TASK_CURR_MEASURE_H

#include "circularBuffer.h"
#include "cmsis_os.h"
#include "main.h"
#include "utils_bkte.h"
#include "utils_sd.h"

#define ADC_CHAN_CNT      4
#define ADC_CHAN_BUF_SIZE 1000
#define ADC_BUF_SIZE      (ADC_CHAN_BUF_SIZE * ADC_CHAN_CNT * 2)
#define ADC_MAX_VAL       0x0FFFU
#define ADC_MIN_VAL       0x0U
#define ADC_MIN_MAX_DELTA 100
#define WINDOW_SIZE       15
#define WINDOW_STEP       8

#define CRANE_STOP 0
#define CRANE_MOVE 1

typedef struct {
    u32 sum;
    u32 avg;
    u16 min;
    u16 max;

    float coef;  // calibration current coefficient

    u16 arr_max[100];
    u16 arr_min[100];
    u16 ptr_max;
    u16 ptr_min;
} adc_chan_t;

typedef struct {
    u16* p_buf;
    u16  size;
    u32  stopTime;

    adc_chan_t chan[ADC_CHAN_CNT - 1];
} adc_measure_t;

void unLockTasks();
void generateInitTelemetry();

#endif