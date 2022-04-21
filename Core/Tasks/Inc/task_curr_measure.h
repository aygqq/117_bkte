#ifndef __TASK_CURR_MEASURE_H
#define __TASK_CURR_MEASURE_H

#include "circularBuffer.h"
#include "cmsis_os.h"
#include "main.h"
#include "utils_bkte.h"
#include "utils_sd.h"

void unLockTasks();
void generateInitTelemetry();

#endif