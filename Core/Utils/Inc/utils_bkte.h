/*
 * utils_bkte.h
 *
 *  Created on: Mar 16, 2020
 *      Author: bls
 */

#ifndef INC_UTILS_BKTE_H_
#define INC_UTILS_BKTE_H_

#include "FreeRTOS.h"
#include "circularBuffer.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "main.h"
#include "simcom.h"
#include "spiflash.h"
#include "stdlib.h"
#include "task_curr_measure.h"
#include "time.h"
#include "usart.h"
#include "version.h"

//!-------------CONFIGURE PARAMS---------------
#define BKTE_ID_TRAINCAR     1
#define BKTE_ID_TRAINCAR_MAX 2
#define BKTE_IS_LORA_MASTER  1

#define BKTE_ID_FIRMWARE MINOR_VERSION
#define BKTE_ID_BOOT     2
#define BKTE_ID_PCB      3
#define BKTE_ID_TRAIN    1706
//!------------CONFIGURE PARAMS----------------
#define BKTE_THRESHOLD_CNT_PAGES 2
#define BKTE_AMOUNTS             (BKTE_ID_TRAINCAR_MAX + 1)
#define BKTE_ADDR_ID_MCU         0x1FFF7A10
#define BKTE_MAX_CNT_1WIRE       4
#define LEN_TIMESTAMP            13

#define BKTE_ID_DEV_BKTE 0x10
#define BKTE_ID_DEV_BKT  0x11
#define BKTE_ID_DEV_BSG  0x12

#define SZ_PART_NEW_SOFTWARE 1300

#define BKTE_VER_BETA_FIRMWARE   (char)'B'
#define BKTE_VER_STABLE_FIRMWARE (char)'S'

#define BKTE_PERCENT_DEVIATION_ENERGY_DATA (float)0.03
#define BKTE_MEASURE_FULL_LOOP             (u8)60

#define SZ_MAX_TX_DATA 4096

#define BKTE_BAD_TIMESTAMP 2997993600

#define BKTE_BIG_DIF_RTC_SERVTIME 600

typedef struct {
    u64 header;
    u8  idBoot;
    u8  idFirmware;
    u8  idNewFirmware;
    u8  szNewFirmware;
} fw_info_t;

typedef struct {
    fw_info_t fw;
    u8        server;
    u32       idMCU[3];
    u64       imei;
    u8        niacIdent[40];
} bkte_info_t;

typedef struct {
    u8 isSentData;
    u8 isTCPOpen;
    u8 csq;
    u8 isSpiFlashReady;
} bkte_state_t;

typedef union {
    struct {
        u16 simAT      : 1;
        u16 simSAPBR   : 1;
        u16 simHINIT   : 1;
        u16 simHPARA   : 1;
        u16 simHDATA   : 1;
        u16 simHDATAU  : 1;
        u16 simHACT    : 1;
        u16 simHREAD   : 1;
        u16 simHCODE   : 1;
        u16 simCSQINF  : 1;
        u16 flashNOIRQ : 1;
    };
    u16 errReg;
} ErrorFlags;

typedef struct {
    u32   posFile;
    u32   lenLog;
    char* fNameLog;
    char* fNameAddr;
} FInfo;

typedef union {
    struct {
        u16 isChrgBat  : 1;
        u16 isPwrState : 1;
        u16 adcVoltBat : 9;
    };
    u16 pwrReg;
} PWRInfo;

typedef union {
    struct {
        u8 isFatMount : 1;
        u8 isDS2482   : 1;
        u8 isSPIFlash : 1;
    };
    u8 regHardWareStat;
} HardWareStatus;

typedef struct {
    u32 cr_web;
    u32 new_bin;
    u32 alive;
    u32 web_exchng;

    u32 pageWrCount;
    u32 pageRdCount;
    u32 pageBadCount;

    u32 simSendCnt;
    u32 simErrCnt;
    u32 simResetCnt;
    u32 simOpenCnt;
    u32 simBadCsqCnt;
    u32 simLowCsqCnt;
    u32 simGoodCsqCnt;
    u32 simHighCsqCnt;

    u32 queueErrCount;
} statistics_t;

typedef struct {
    u32 tcp_open_time;
    u32 tcp_close_time;
    u32 tcp_send_time;
    u32 tcp_all_time;
} time_stat_t;

typedef struct {
    u8             isOwActive[BKTE_MAX_CNT_1WIRE];
    HardWareStatus hwStat;
    PWRInfo        pwrInfo;
    ErrorFlags     erFlags;

    bkte_state_t  state;
    bkte_info_t   info;
    time_stat_t   timers;
    statistics_t  stat;
    adc_measure_t measure;
} BKTE;

typedef enum {
    CMD_DATA_VOLTAMPER = 1,
    CMD_DATA_ENERGY,
    CMD_DATA_TEMP,
    CMD_DATA_GRMC,
    CMD_DATA_TELEMETRY = 5,
    CMD_DATA_VOLTAMPER_127,
    CMD_DATA_ENERGY_127,
    CMD_DATA_PERCRSSI_127,
    CMD_DATA_DOORS,
    CMD_DATA_GEO_PLUS,
    CMD_DATA_ADC_CURR = 0x30
} CMD_DATA_TYPE;

typedef enum {
    TEL_GR_GENINF = 1,
    TEL_GR_HARDWARE_STATUS,
    TEL_GR_PROJECT,
    TEL_GR_PROJECT_STAT,
    TEL_GR_PROJECT_MEM,
    TEL_GR_SIMCOM,
    TEL_GR_PROJECT_RADIO_STAT,
    TEL_GR_BKTE_STAT,
    TEL_GR_CRANE = 0x10
} TELEMETRY_GROUP;

typedef enum {
    TEL_CD_GENINF_NUM_FIRMWARE = 1,
    TEL_CD_GENINF_NUM_BOOT,
    TEL_CD_GENINF_PHONE_NUM1,
    TEL_CD_GENINF_PHONE_NUM2,
    TEL_CD_GENINF_SIMCARD_MONEY,
    TEL_CD_GENINF_NUM_PCB
} TELEMETRY_CODE_GEN_INF;

typedef enum {
    TEL_CD_HW_BKTE = 1,
    TEL_CD_HW_SD,
    TEL_CD_HW_DS2482,
    TEL_CD_HW_SPI_FLASH,
    TEL_CD_HW_LORA,
    TEL_CD_HW_BATTERY,
    TEL_CD_HW_BKTE_ALIVE,
    TEL_CD_HW_WIRELESS_SENS_RSSI,
    TEL_CD_HW_UPDATED,
    TEL_CD_HW_UPDATE_LEN,
    TEL_CD_HW_UPDATE_ERR
} TELEMETRY_CODE_STATES;

typedef enum {
    TEL_CD_TASK_CR_WEB = 1,
    TEL_CD_TASK_ENERGY,
    TEL_CD_TASK_NEW_BIN,
    TEL_CD_TASK_TEMP,
    TEL_CD_TASK_ALIVE,
    TEL_CD_TASK_WEB_EXCHANGE,
    TEL_CD_TASK_WIRELESS,
    TEL_CD_TASK_CR_WEB_ERR
} TELEMETRY_CODE_TASK_STAT;

typedef enum {
    TEL_CD_IU_PAGE_WR = 1,
    TEL_CD_IU_PAGE_RD,
    TEL_CD_IU_PAGE_BAD,
    TEL_CD_BKTE_PAGE_WR = 4,
    TEL_CD_BKTE_PAGE_RD,
    TEL_CD_BKTE_PAGE_BAD,
    TEL_CD_BKTE_PAGE_FROM_IU,
    TEL_CD_BKTE_PAGE_HEAD,
    TEL_CD_BKTE_PAGE_TAIL
} TELEMETRY_CODE_MEM;

typedef enum {
    TEL_CD_SIM_SEND = 1,
    TEL_CD_SIM_ERR,
    TEL_CD_SIM_RESET,
    TEL_CD_SIM_OPEN,
    TEL_CD_SIM_BAD_CSQ,
    TEL_CD_SIM_LOW_CSQ,
    TEL_CD_SIM_GOOD_CSQ,
    TEL_CD_SIM_HIGH_CSQ,
    TEL_CD_SIM_TIME_OPEN,
    TEL_CD_SIM_TIME_CLOSE,
    TEL_CD_SIM_TIME_SEND,
    TEL_CD_SIM_TIME_ALL,
    TEL_CD_SIM_GPS_INV_CNT,
    TEL_CD_SIM_GPS_PARSE_ER_CNT,
    TEL_CD_SIM_GPS_PARSE_ERROR,
    TEL_CD_SIM_GPS_TIMESYNC,
    TEL_CD_SIM_GPS_RESET
} TELEMETRY_CODE_SIMCOM;

typedef enum {
    TEL_CD_CRANE_MOVE = 1,
    TEL_CD_CRANE_TIME_ERR,
    TEL_CD_CRANE_PERIOD_ERR
} TELEMETRY_CODE_CRANE;

typedef enum {
    CMD_REQUEST_SERVER_TIME = 0x11,
    CMD_REQUEST_NUM_FIRMWARE,
    CMD_REQUEST_SZ_FIRMWARE,
    CMD_REQUEST_PART_FIRMWARE
} CMD_REQUEST;

typedef __packed struct {
    u32 unixTimeStamp;
    u8  group;
    u8  code;
    u32 data;
} PckgTelemetry;

typedef __packed struct {
    u32 unixTimeStamp;
    s16 min[3];
    s16 max[3];
    s16 avg[3];
    u16 cnt_max[3];
    u16 first_max[3];
} PckgAdcCurr;

typedef __packed struct {
    u32 fromByte;
    u32 toByte;
} PckgUpdFirmware;

void bkteInit();

ErrorStatus getServerTime();
u32         getUnixTimeStamp();
void        setUnixTimeStamp(u32 timeStamp);

void offAllLeds();
void offAllRedLeds();
void toggleGreenLeds();
void toggleRedLeds();

void saveData(u8* data, u8 sz, u8 cmdData, CircularBuffer* cbuf);
void updSpiFlash(CircularBuffer* cbuf);
void copyTelemetry(u8* buf, PckgTelemetry* pckgTel);
void saveTelemetry(PckgTelemetry* pckg, CircularBuffer* cbuf);

u8 waitGoodCsq(u32 timeout);

u8 isCrcOk(char* pData, int len);
u8 isDataFromFlashOk(char* pData, u8 len);

void getNumFirmware();

extern BKTE bkte;

#endif /* INC_UTILS_BKTE_H_ */
