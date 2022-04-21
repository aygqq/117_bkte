/*
 * utils_bkte.c
 *
 *  Created on: Mar 16, 2020
 *      Author: bls
 */

#include "utils_bkte.h"

#include "utils_crc.h"
#include "utils_flash.h"
#include "utils_pckgs_manager.h"
#include "utils_sd.h"

extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef  hrtc;
extern osMutexId          mutexRTCHandle;
extern osMutexId          mutexBigBufHandle;
extern osMutexId          mutexWebHandle;
extern osMutexId          mutexSpiFlashHandle;
extern osThreadId         getNewBinHandle;
extern osSemaphoreId      semCreateWebPckgHandle;
static RTC_TimeTypeDef    tmpTime;
static RTC_DateTypeDef    tmpDate;

void bkteInit() {
    HAL_GPIO_WritePin(SD_PWR_EN_GPIO_Port, SD_PWR_EN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(BAT_PWR_EN_GPIO_Port, BAT_PWR_EN_Pin, GPIO_PIN_SET);
    bkte.pwrInfo.isPwrState = HAL_GPIO_ReadPin(PWR_STATE_GPIO_Port, PWR_STATE_Pin);
    if (bkte.pwrInfo.isPwrState) {
        HAL_GPIO_WritePin(BAT_PWR_EN_GPIO_Port, BAT_PWR_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
        HAL_Delay(5000);
        NVIC_SystemReset();
    }

    for (u8 i = 0; i < 3; i++)
        bkte.info.idMCU[i] = getFlashData(BKTE_ADDR_ID_MCU + (i * 4));
    LOG(LEVEL_MAIN, "%08x%08x%08x\r\n", (uint)bkte.info.idMCU[0], (uint)bkte.info.idMCU[1], (uint)bkte.info.idMCU[2]);

    bkte.hwStat.regHardWareStat = 0;
    bkte.erFlags.errReg = 0;
    bkte.info.fw.idFirmware = BKTE_ID_FIRMWARE;
    bkte.info.fw.idBoot = BKTE_ID_BOOT;
    bkte.info.server = SERVER_NIAC;
    u8 id[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
    memcpy(bkte.info.niacIdent, id, 40);
    memset(&bkte.stat, 0, sizeof(statistics_t));
    memset(&bkte.timers, 0, sizeof(time_stat_t));
}

u32 getUnixTimeStamp() {
    time_t           t;
    static struct tm curTime;

    osMutexWait(mutexRTCHandle, osWaitForever);
    HAL_RTC_GetTime(&hrtc, &tmpTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &tmpDate, RTC_FORMAT_BIN);
    curTime.tm_year = tmpDate.Year + 100;
    curTime.tm_mday = tmpDate.Date;
    curTime.tm_mon = tmpDate.Month - 1;

    curTime.tm_hour = tmpTime.Hours;
    curTime.tm_min = tmpTime.Minutes;
    curTime.tm_sec = tmpTime.Seconds;
    osMutexRelease(mutexRTCHandle);
    curTime.tm_isdst = 0;

    t = mktime(&curTime);
    return (u32)t;
}

void setUnixTimeStamp(u32 timeStamp) {
    if (timeStamp < 946684800) {
        LOG(LEVEL_ERROR, "BAD TIMESTAMP\r\n");
        return;
    }

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    osMutexWait(mutexRTCHandle, osWaitForever);

    struct tm time_tm;
    time_t    now = (time_t)timeStamp;
    time_tm = *(localtime(&now));

    sTime.Hours = (uint8_t)time_tm.tm_hour;
    sTime.Minutes = (uint8_t)time_tm.tm_min;
    sTime.Seconds = (uint8_t)time_tm.tm_sec;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        LOG(LEVEL_ERROR, "Failed to set time\r\n");
    }

    if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; }  // the chip goes mon tue wed thu fri sat sun
    sDate.WeekDay = (uint8_t)time_tm.tm_wday;
    sDate.Month = (uint8_t)time_tm.tm_mon + 1;  // momth 1- This is why date math is frustrating.
    sDate.Date = (uint8_t)time_tm.tm_mday;
    sDate.Year = (uint16_t)(time_tm.tm_year + 1900 - 2000);  // time.h is years since 1900, chip is years since 2000

    /*
     * update the RTC
     */
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        LOG(LEVEL_ERROR, "Failed to set date\r\n");
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);  // lock it in with the backup registers
    osMutexRelease(mutexRTCHandle);
}

ErrorStatus getServerTime() {
    u8 bufTime[4];
    if (generateWebPckgReq(CMD_REQUEST_SERVER_TIME, NULL, 0, SZ_REQUEST_GET_SERVER_TIME, bufTime, 4, (u8*)&bkte.info.idMCU) == ERROR) {
        LOG_WEB(LEVEL_ERROR, "ERROR: bad server time\r\n");
        return ERROR;
    } else {
        time_t t =
            bufTime[0] << 24 | bufTime[1] << 16 | bufTime[2] << 8 | bufTime[3];
        struct tm* pTm;
        pTm = gmtime(&t);
        if (pTm != NULL) {
            tmpTime.Hours = pTm->tm_hour;
            tmpTime.Minutes = pTm->tm_min;
            tmpTime.Seconds = pTm->tm_sec;

            tmpDate.Date = pTm->tm_mday;
            tmpDate.Month = pTm->tm_mon + 1;
            tmpDate.Year = pTm->tm_year - 100;

            osMutexWait(mutexRTCHandle, osWaitForever);
            if (tmpDate.Year < 30 &&
                tmpDate.Year > 19) {  // sometimes timestamp is wrong and has
                                      // value like 2066 year
                HAL_RTC_SetTime(&hrtc, &tmpTime, RTC_FORMAT_BIN);
                HAL_RTC_SetDate(&hrtc, &tmpDate, RTC_FORMAT_BIN);
            }
            osMutexRelease(mutexRTCHandle);
        }
    }
    return SUCCESS;
}

void getNumFirmware() {
    u8 bufFirmware[4];
    if (generateWebPckgReq(CMD_REQUEST_NUM_FIRMWARE, NULL, 0, SZ_REQUEST_GET_NUM_FIRMWARE, bufFirmware, 4, (u8*)&bkte.info.idMCU) == ERROR) {
        LOG_WEB(LEVEL_ERROR, "ERROR: getBKTENumFw()\r\n");
    } else {
        u32 numFirmware = bufFirmware[0] << 24 | bufFirmware[1] << 16 | bufFirmware[2] << 8 | bufFirmware[3];
        LOG_WEB(LEVEL_INFO, "BKTE FIRMWARE v.:%d\r\n", (int)numFirmware);
        if (numFirmware != BKTE_ID_FIRMWARE && numFirmware > 0) {
            LOG_WEB(LEVEL_MAIN, "New FIRMWARE v.:%d\r\n", (int)numFirmware);
            bkte.info.fw.idNewFirmware = (u8)numFirmware;
            vTaskResume(getNewBinHandle);
        }
    }
}

u8 isCrcOk(char* pData, int len) {
    u32 crcCalc = crc32_byte(pData, len);
    u32 crcRecv = pData[len] << 24 | pData[len + 1] << 16 | pData[len + 2] << 8 | pData[len + 3];
    if (crcCalc != crcRecv) {
        LOG(LEVEL_ERROR, "isCrcOk bad crc \r\n");
    }
    for (u8 i = 0; i < sizeof(u32); i++) {
        pData[len + i] = 0xFF;  //! clear crc32
    }

    return crcCalc == crcRecv;
}

void offAllLeds() {
    HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED1R_GPIO_Port, LED1R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3G_GPIO_Port, LED3G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4G_GPIO_Port, LED4G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4R_GPIO_Port, LED4R_Pin, GPIO_PIN_SET);
}

/*void offAllRedLeds(){
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED1R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED4R_GPIO_Port, LED4R_Pin, GPIO_PIN_SET);
}

void toggleGreenLeds(){
        HAL_GPIO_TogglePin(LED1G_GPIO_Port, LED1G_Pin);
        HAL_GPIO_TogglePin(LED2G_GPIO_Port, LED2G_Pin);
        HAL_GPIO_TogglePin(LED4G_GPIO_Port, LED4G_Pin);
}

void toggleRedLeds(){
          HAL_GPIO_TogglePin(LED1R_GPIO_Port, LED1R_Pin);
          HAL_GPIO_TogglePin(LED2R_GPIO_Port, LED2R_Pin);
          HAL_GPIO_TogglePin(LED4R_GPIO_Port, LED4R_Pin);
}*/

void updSpiFlash(CircularBuffer* cbuf) {
    u16 bufEnd[2] = {0, BKTE_PREAMBLE};

    osMutexWait(mutexBigBufHandle, osWaitForever);

    bufEnd[0] = calcCrc16(cbuf->buf, cbuf->readAvailable);
    cBufWriteToBuf(cbuf, (u8*)bufEnd, 4);
    spiFlashWriteNextPg(cbuf->buf, cbuf->readAvailable, 0);
    cBufReset(cbuf);

    osMutexRelease(mutexBigBufHandle);

    LOG_FLASH(LEVEL_INFO, "updSpiFlash()\r\n");
}

u8 waitGoodCsq(u32 timeout) {
    u8  csq = 0;
    u16 cntNOCsq = 0;
    u16 cntNOCsqMax = timeout / 3;

    while ((csq = simCheckCSQ()) < 5 || csq > 99) {
        osDelay(3000);
        cntNOCsq++;
        if (cntNOCsq == cntNOCsqMax) {
            cntNOCsq = 0;
            return 0;
        }
        LOG_SIM(LEVEL_DEBUG, "ER: CSQ %d\r\n", csq);
    }
    LOG_SIM(LEVEL_DEBUG, "OK: CSQ %d\r\n", csq);
    return 1;
}

void saveData(u8* data, u8 sz, u8 cmdData, CircularBuffer* cbuf) {
    u16 bufEnd[2] = {0, BKTE_PREAMBLE};

    osMutexWait(mutexBigBufHandle, osWaitForever);

    if (cbuf->writeAvailable < sz + 2 + 4) {
        bufEnd[0] = calcCrc16(cbuf->buf, cbuf->readAvailable);
        cBufWriteToBuf(cbuf, (u8*)bufEnd, 4);
        spiFlashWriteNextPg(cbuf->buf, cbuf->readAvailable, 0);
        cBufReset(cbuf);
    }

    cBufWriteToBuf(cbuf, &cmdData, 1);
    cBufWriteToBuf(cbuf, data, sz);

    osMutexRelease(mutexBigBufHandle);
}

u8 isDataFromFlashOk(char* pData, u8 len) {
    u16 crc;
    for (u8 i = len - 1; i; --i) {
        if (pData[i] == BKTE_PREAMBLE_LSB && pData[i - 1] == BKTE_PREAMBLE_MSB) {
            crc = pData[i - 3] | pData[i - 2] << 8;
            if (calcCrc16(pData, i - 3) == crc) {
                len = (i + 1) - 4;
                return len;
            }
        }
    }
    return 0;
}

void copyTelemetry(u8* buf, PckgTelemetry* pckgTel) {
    memcpy(buf, &pckgTel->unixTimeStamp, 4);
    memcpy(buf + 4, &pckgTel->group, 1);
    memcpy(buf + 5, &pckgTel->code, 1);
    memcpy(buf + 6, &pckgTel->data, 4);
}

void saveTelemetry(PckgTelemetry* pckg, CircularBuffer* cbuf) {
    u8 buf[10];
    pckg->unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(buf, pckg);
    saveData(buf, SZ_CMD_TELEMETRY, CMD_DATA_TELEMETRY, cbuf);
}