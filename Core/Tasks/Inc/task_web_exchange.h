#ifndef __TASK_WEB_EXCHANGE_H
#define __TASK_WEB_EXCHANGE_H

#include "circularBuffer.h"
#include "cmsis_os.h"
#include "main.h"
#include "simcom.h"
#include "spiflash.h"
#include "utils_bkte.h"
#include "utils_pckgs_manager.h"

#define SESSION_OPENING     1
#define SESSION_AUTHORIZE   2
#define SESSION_SENDING     3
#define SESSION_CLOSING     4
#define SESSION_TCP_CLOSING 5

#endif