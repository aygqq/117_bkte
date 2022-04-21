#ifndef __TASK_IWDG_H
#define __TASK_IWDG_H

#include "cmsis_os.h"
#include "main.h"

extern u16 iwdgTaskReg;

#define IWDG_TASK_REG_WEB_PCKG 0x0001
#define IWDG_TASK_CURR_MEASURE 0x0002
#define IWDG_TASK_REG_ALIVE    0x0010
#define IWDG_TASK_REG_WEB_EXCH 0x0020
#define IWDG_TASK_REG_NEW_BIN  0x0040

#define IWDG_TASK_REG_ALL 0x0033

#endif