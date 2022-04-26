#ifndef __DEBUG_H
#define __DEBUG_H

#include "main.h"

#define DEBUG 1

#define DEBUG_LEVEL_COMMON LEVEL_INFO
#define DEBUG_LEVEL_USART  LEVEL_ERROR
#define DEBUG_LEVEL_SIM    LEVEL_INFO
#define DEBUG_LEVEL_PARAM  LEVEL_INFO
#define DEBUG_LEVEL_CBUF   LEVEL_ERROR
#define DEBUG_LEVEL_FLASH  LEVEL_MAIN
#define DEBUG_LEVEL_WEB    LEVEL_INFO
#define DEBUG_LEVEL_SD     LEVEL_INFO
#define DEBUG_LEVEL_GPS    LEVEL_INFO
#define DEBUG_LEVEL_PWR    LEVEL_INFO

#define LEVEL_ERROR 1
#define LEVEL_MAIN  2
#define LEVEL_INFO  3
#define LEVEL_DEBUG 4

#ifdef DEBUG
#define PRINT_TIME(millis)                                  \
    do {                                                    \
        printf("%06d.%03d:", millis / 1000, millis % 1000); \
    } while (0)

#define PRINT_LEVEL(level)                 \
    do {                                   \
        if (level == LEVEL_ERROR) {        \
            printf("[ERR] :");             \
        } else if (level == LEVEL_MAIN) {  \
            printf("[main]:");             \
        } else if (level == LEVEL_INFO) {  \
            printf("[info]:");             \
        } else if (level == LEVEL_DEBUG) { \
            printf("[dbg] :");             \
        }                                  \
    } while (0)

#define LOGG(str, const_level, level, ...) \
    do {                                   \
        if (const_level >= level) {        \
            PRINT_TIME(HAL_GetTick());     \
            PRINT_LEVEL(level);            \
            printf(str);                   \
            printf(__VA_ARGS__);           \
        }                                  \
    } while (0)

#define LOG(level, ...)       LOGG("     :", DEBUG_LEVEL_COMMON, level, __VA_ARGS__)
#define LOG_USART(level, ...) LOGG("UART :", DEBUG_LEVEL_USART, level, __VA_ARGS__)
#define LOG_SIM(level, ...)   LOGG("SIM  :", DEBUG_LEVEL_SIM, level, __VA_ARGS__)
#define LOG_PARAM(level, ...) LOGG("PARAM:", DEBUG_LEVEL_PARAM, level, __VA_ARGS__)
#define LOG_CBUF(level, ...)  LOGG("CBUF :", DEBUG_LEVEL_CBUF, level, __VA_ARGS__)
#define LOG_FLASH(level, ...) LOGG("FLASH:", DEBUG_LEVEL_FLASH, level, __VA_ARGS__)
#define LOG_WEB(level, ...)   LOGG("WEB  :", DEBUG_LEVEL_WEB, level, __VA_ARGS__)
#define LOG_SD(level, ...)    LOGG("SD   :", DEBUG_LEVEL_SD, level, __VA_ARGS__)
#define LOG_PWR(level, ...)   LOGG("PWR  :", DEBUG_LEVEL_PWR, level, __VA_ARGS__)

#else

#define LOG(level, ...)
#define LOG_USART(level, ...)
#define LOG_SIM(level, ...)
#define LOG_PARAM(level, ...)
#define LOG_CBUF(level, ...)
#define LOG_FLASH(level, ...)
#define LOG_WEB(level, ...)
#define LOG_SD(level, ...)
#define LOG_PWR(level, ...)

#endif

#endif
