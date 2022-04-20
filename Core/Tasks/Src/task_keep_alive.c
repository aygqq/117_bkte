#include "../Tasks/Inc/task_keep_alive.h"

#include "../Tasks/Inc/task_iwdg.h"
#include "utils_flash.h"
#include "utils_pckgs_manager.h"

extern u16 iwdgTaskReg;

extern osThreadId    keepAliveHandle;
extern osThreadId    getTempHandle;
extern osThreadId    getEnergyHandle;
extern osThreadId    wirelessSensHandle;
extern osTimerId     timerPowerOffHandle;
extern osMutexId     mutexWriteToEnergyBufHandle;
extern osMutexId     mutexWebHandle;
extern osMutexId     mutexRTCHandle;
extern osMutexId     mutexSDHandle;
extern osMutexId     mutexSpiFlashHandle;
extern osSemaphoreId semCreateWebPckgHandle;

extern u8             isRxNewFirmware;
extern CircularBuffer circBufAllPckgs;

u8 bufTxData[256];

void taskKeepAlive(void const* argument) {
    u32 timeout = 1;
    vTaskSuspend(keepAliveHandle);

    for (;;) {
        HAL_GPIO_TogglePin(LED1G_GPIO_Port, LED1G_Pin);
        if (!(timeout % 6000) && !isRxNewFirmware) {
            LOG(LEVEL_MAIN, "sendMsgStatistics\r\n");
            osTimerStart(timerPowerOffHandle, 1100000);
            sendMsgStatistics();
            osTimerStop(timerPowerOffHandle);
        }
        if (!(timeout % 600) && !isRxNewFirmware) {
            LOG(LEVEL_MAIN, "getBKTENumFw\r\n\r\n");
            osTimerStart(timerPowerOffHandle, 1100000);
            getNumFirmware();
            osTimerStop(timerPowerOffHandle);
        }
        if (!(timeout % 6000) && !isRxNewFirmware) {
            LOG(LEVEL_MAIN, "\r\ngenerateMsgKeepAlive\r\n\r\n");
            osTimerStart(timerPowerOffHandle, 1100000);
            generateMsgKeepAlive();
            osTimerStop(timerPowerOffHandle);
        }
        if (!(timeout % 36000) && !isRxNewFirmware) {
            LOG(LEVEL_MAIN, "updRTC\r\n\r\n");
            osTimerStart(timerPowerOffHandle, 1100000);
            updRTC();
            osTimerStop(timerPowerOffHandle);
        }
        if (bkte.pwrInfo.isPwrState) {
            HAL_GPIO_WritePin(LED1R_GPIO_Port, LED1R_Pin, GPIO_PIN_RESET);
            osTimerStart(timerPowerOffHandle, 1100000);
            pwrOffBkte();
            osTimerStop(timerPowerOffHandle);
        }

        bkte.pwrInfo.isPwrState = HAL_GPIO_ReadPin(PWR_STATE_GPIO_Port, PWR_STATE_Pin);
        timeout++;
        osDelay(200);
        iwdgTaskReg |= IWDG_TASK_REG_ALIVE;
        bkte.stat.alive++;
    }
}

void timerPowerOff_callback(void const* argument) {
    LOG_PWR(LEVEL_INFO, "\r\nTIMER POWER OFF\r\n\r\n");
    HAL_GPIO_WritePin(BAT_PWR_EN_GPIO_Port, BAT_PWR_EN_Pin, GPIO_PIN_RESET);  // OFF
    osDelay(1000);
    NVIC_SystemReset();
}

u16 getAdcVoltBat() {
    u16 adc;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 250);
    adc = HAL_ADC_GetValue(&hadc1);
    return (u16)(adc * 3.3 * 2 / 4096 * 100);
}

void pwrOffBkte() {
    char strVolts[4];
    u16  delayPages = BKTE_THRESHOLD_CNT_PAGES + 1;
    u32  curTime = 0;
    u8   cnt;

    osMutexWait(mutexWriteToEnergyBufHandle, osWaitForever);
    osMutexWait(mutexRTCHandle, osWaitForever);
    osMutexWait(mutexSpiFlashHandle, osWaitForever);
    osMutexWait(mutexSDHandle, osWaitForever);
    osMutexWait(mutexWebHandle, osWaitForever);

    vTaskSuspend(getEnergyHandle);
    vTaskSuspend(getTempHandle);
    vTaskSuspend(wirelessSensHandle);

    osMutexRelease(mutexWriteToEnergyBufHandle);
    osMutexRelease(mutexRTCHandle);
    osMutexRelease(mutexSpiFlashHandle);
    osMutexRelease(mutexSDHandle);
    osMutexRelease(mutexWebHandle);

    osDelay(2000);
    LOG_PWR(LEVEL_INFO, "PWR OFF START\r\n");
    // cBufReset(&circBufPckgEnergy);

    bkte.pwrInfo.adcVoltBat = getAdcVoltBat();
    generateMsgBat();
    LOG_PWR(LEVEL_INFO, "PWR OFF WAIT: %d\r\n", getUnixTimeStamp());

    curTime = 0;
    delayPages = getDelayPages();
    while (delayPages > BKTE_THRESHOLD_CNT_PAGES && (bkte.pwrInfo.adcVoltBat = getAdcVoltBat()) > 390) {
        osDelay(5000);
        curTime += 5000;
        delayPages = getDelayPages();
        printf("wait dalaypages %d\r\n", delayPages);

        if (curTime > 300000) {
            bkte.state.isTCPOpen = 0;
            if (sendMsgDevOffValue(11) != SUCCESS) {
                LOG_PWR(LEVEL_ERROR, "Send dev off val\r\n");
            }
            break;
        }
    }

    curTime = 0;
    while ((cnt = getCntFreePckg()) != CNT_WEBPCKGS && (bkte.pwrInfo.adcVoltBat = getAdcVoltBat()) > 385) {
        osDelay(5000);
        curTime += 5000;
        printf("wait free pckg %d\r\n", cnt);

        if (curTime > 300000) {
            bkte.state.isTCPOpen = 0;
            if (sendMsgDevOffValue(12) != SUCCESS) {
                LOG_PWR(LEVEL_ERROR, "Send dev off val\r\n");
            }
            break;
        }
    }

    bkte.pwrInfo.adcVoltBat = getAdcVoltBat();
    generateMsgBat();
    generateMsgDevOff();
    LOG_PWR(LEVEL_INFO, "OFF  VOLT: %d\r\n", bkte.pwrInfo.adcVoltBat);

    updSpiFlash(&circBufAllPckgs);
    osSemaphoreRelease(semCreateWebPckgHandle);
    osDelay(3000);

    curTime = 0;
    bkte.state.isSentData = 0;
    while (!bkte.state.isSentData && (bkte.pwrInfo.adcVoltBat = getAdcVoltBat()) > 380) {
        osDelay(5000);
        curTime += 5000;
        printf("wait data send\r\n");

        if (curTime > 300000) {
            bkte.state.isTCPOpen = 0;
            if (sendMsgDevOffValue(13) != SUCCESS) {
                LOG_PWR(LEVEL_ERROR, "Send dev off val\r\n");
            }
            break;
        }
    }
    if (!bkte.state.isSentData) {
        sdWriteLog(SD_MSG_NOT_SENT, SD_LEN_NOT_SENT, NULL, 0, &sdSectorLogs);
        sdUpdLog(&sdSectorLogs);
    }

    LOG_PWR(LEVEL_INFO, "PWR OFF SENT TELEMETRY: %d\r\n", getUnixTimeStamp());

    sprintf(strVolts, "%03d", bkte.pwrInfo.adcVoltBat);
    sdWriteLog(SD_MSG_OFF_BKTE, SD_LEN_OFF_BKTE, strVolts, 3, &sdSectorLogs);
    sdUpdLog(&sdSectorLogs);

    spiFlashSaveInfo();

    osDelay(10000);
    HAL_GPIO_WritePin(BAT_PWR_EN_GPIO_Port, BAT_PWR_EN_Pin, GPIO_PIN_RESET);  // OFF
    osDelay(5000);
    NVIC_SystemReset();
}

void updRTC() {
    getServerTime();
    // fillTelemetry(&curPckgEnergy, TEL_CHANGE_TIME, time);
}

void generateMsgKeepAlive() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE_ALIVE;
    pckgTel.data = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    sdWriteLog(SD_MSG_KEEP_ALIVE, SD_LEN_KEEP_ALIVE, NULL, 0, &sdSectorLogs);
}

void generateMsgBat() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BATTERY;
    pckgTel.data = bkte.pwrInfo.adcVoltBat;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

void generateMsgFWUpdated() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bkte.info.fw.idNewFirmware;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

void generateMsgDevOff() {
    PckgTelemetry pckgTel;
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);
}

ErrorStatus sendMsgDevOn() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    u8            ptr = 0;
    u32           tmp;

    LOG(LEVEL_MAIN, "sendMsgDevOn\r\n");

    memset(bufTxData, 0, 128);
    pckgTel.group = TEL_GR_GENINF;
    pckgTel.code = TEL_CD_GENINF_NUM_FIRMWARE;
    pckgTel.data = bkte.info.fw.idFirmware;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_GENINF_NUM_BOOT;
    pckgTel.data = bkte.info.fw.idBoot;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = 1;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_UPDATE_ERR;
    tmp = getFlashData(FLASH_ADDR_ERR_NEW_FIRMWARE);
    if (tmp > 10) tmp = 0;
    pckgTel.data = tmp;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * ptr, ptr, (u8*)&bkte.info.idMCU);

    return ret;
}

ErrorStatus sendMsgFWUpdated() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;

    LOG(LEVEL_MAIN, "sendMsgFWUpdated\r\n");

    memset(bufTxData, 0, 20);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bkte.info.fw.idNewFirmware;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(bufTxData, &pckgTel);

    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = 0;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * 2, 2, (u8*)&bkte.info.idMCU);

    return ret;
}

ErrorStatus sendMsgTaskStat() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    u8            ptr = 0;

    LOG_PWR(LEVEL_INFO, "sendMsgTaskStat\r\n");

    memset(bufTxData, 0, 256);
    pckgTel.group = TEL_GR_BKTE_STAT;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    pckgTel.code = TEL_CD_TASK_CR_WEB;
    pckgTel.data = bkte.stat.cr_web;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_TASK_ENERGY;
    pckgTel.data = bkte.stat.enrg;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_TASK_NEW_BIN;
    pckgTel.data = bkte.stat.new_bin;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_TASK_TEMP;
    pckgTel.data = bkte.stat.temp;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_TASK_ALIVE;
    pckgTel.data = bkte.stat.alive;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_TASK_WEB_EXCHANGE;
    pckgTel.data = bkte.stat.web_exchng;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_TASK_WIRELESS;
    pckgTel.data = bkte.stat.wireless;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    // pckgTel.group = TEL_GR_HARDWARE_STATUS;
    // pckgTel.code = (u8)14;
    // pckgTel.data = spiFlash64.headNumPg;
    // copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    // pckgTel.code = (u8)15;
    // pckgTel.data = spiFlash64.tailNumPg;
    // copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * ptr, ptr, (u8*)&bkte.info.idMCU);

    return ret;
}

ErrorStatus sendMsgFWUpdateBegin() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    u8            ptr = 0;

    LOG(LEVEL_MAIN, "sendMsgFWUpdated\r\n");

    memset(bufTxData, 0, 64);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_UPDATED;
    pckgTel.data = bkte.info.fw.idNewFirmware;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    pckgTel.code = TEL_CD_HW_UPDATE_LEN;
    pckgTel.data = bkte.info.fw.szNewFirmware;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * ptr, ptr, (u8*)&bkte.info.idMCU);

    return ret;
}

ErrorStatus sendMsgDevOff() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;

    memset(bufTxData, 0, 20);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = 0;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(bufTxData, &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY, 1, (u8*)&bkte.info.idMCU);

    return ret;
}

ErrorStatus sendMsgDevOffValue(u32 val) {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;

    memset(bufTxData, 0, 20);
    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = val;
    pckgTel.unixTimeStamp = getUnixTimeStamp();
    copyTelemetry(bufTxData, &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY, 1, (u8*)&bkte.info.idMCU);

    return ret;
}

ErrorStatus sendMsgStatistics() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;

    memset(bufTxData, 0, 256);

    pckgTel.group = TEL_GR_PROJECT_MEM;
    pckgTel.code = TEL_CD_BKTE_PAGE_WR;
    pckgTel.data = bkte.stat.pageWrCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_BKTE_PAGE_RD;
    pckgTel.data = bkte.stat.pageRdCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_BKTE_PAGE_BAD;
    pckgTel.data = bkte.stat.pageBadCount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.group = TEL_GR_SIMCOM;
    pckgTel.code = TEL_CD_SIM_SEND;
    pckgTel.data = bkte.stat.simSendCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_ERR;
    pckgTel.data = bkte.stat.simErrCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_RESET;
    pckgTel.data = bkte.stat.simResetCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_OPEN;
    pckgTel.data = bkte.stat.simOpenCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_BAD_CSQ;
    pckgTel.data = bkte.stat.simBadCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_LOW_CSQ;
    pckgTel.data = bkte.stat.simLowCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_GOOD_CSQ;
    pckgTel.data = bkte.stat.simGoodCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_HIGH_CSQ;
    pckgTel.data = bkte.stat.simHighCsqCnt;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_OPEN;
    pckgTel.data = bkte.timers.tcp_open_time / 1000;
    bkte.timers.tcp_open_time = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_CLOSE;
    pckgTel.data = bkte.timers.tcp_close_time / 1000;
    bkte.timers.tcp_close_time = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_SEND;
    pckgTel.data = bkte.timers.tcp_send_time / 1000;
    bkte.timers.tcp_send_time = 0;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_SIM_TIME_ALL;
    bkte.timers.tcp_all_time = HAL_GetTick() - bkte.timers.tcp_all_time;
    pckgTel.data = bkte.timers.tcp_all_time / 1000;
    bkte.timers.tcp_all_time = HAL_GetTick();
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    return ret;
}

ErrorStatus sendInitTelemetry() {
    ErrorStatus   ret = SUCCESS;
    PckgTelemetry pckgTel;
    long long     phoneNum;
    u32           tmp;
    u8            ptr = 0;

    memset(bufTxData, 0, 256);

    pckgTel.unixTimeStamp = getUnixTimeStamp();
    pckgTel.group = TEL_GR_GENINF;
    pckgTel.code = TEL_CD_GENINF_NUM_FIRMWARE;
    pckgTel.data = BKTE_ID_FIRMWARE;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_GENINF_NUM_BOOT;
    pckgTel.data = BKTE_ID_BOOT;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_GENINF_NUM_PCB;
    pckgTel.data = BKTE_ID_PCB;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    phoneNum = simGetPhoneNum();
    if (phoneNum > 0) {
        tmp = phoneNum % 1000000000;
        pckgTel.code = TEL_CD_GENINF_PHONE_NUM1;
        pckgTel.data = tmp;
        copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);
    }

    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = 1;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_SD;
    pckgTel.data = bkte.hwStat.isFatMount;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_DS2482;
    pckgTel.data = bkte.hwStat.isDS2482;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_SPI_FLASH;
    pckgTel.data = bkte.hwStat.isSPIFlash;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_LORA;
    pckgTel.data = bkte.hwStat.isLoraOk;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = TEL_CD_HW_UPDATE_ERR;
    tmp = getFlashData(FLASH_ADDR_ERR_NEW_FIRMWARE);
    if (tmp > 2) tmp = 0;
    pckgTel.data = tmp;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = (u8)12;
    pckgTel.data = bkte.lastData.enAct;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = (u8)13;
    pckgTel.data = bkte.lastData.enReact;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = (u8)14;
    pckgTel.data = spiFlash64.headNumPg;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    pckgTel.code = (u8)15;
    pckgTel.data = spiFlash64.tailNumPg;
    copyTelemetry(&bufTxData[SZ_CMD_TELEMETRY * ptr++], &pckgTel);

    ret = sendWebPckgData(CMD_DATA_TELEMETRY, bufTxData, SZ_CMD_TELEMETRY * ptr, ptr, (u8*)&bkte.info.idMCU);

    return ret;
}