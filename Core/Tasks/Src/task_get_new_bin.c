#include "task_get_new_bin.h"

#include "task_iwdg.h"
#include "task_keep_alive.h"
#include "utils_crc.h"
#include "utils_pckgs_manager.h"

extern u16 iwdgTaskReg;

extern osThreadId getEnergyHandle;
extern osThreadId webExchangeHandle;
extern osThreadId getTempHandle;
extern osThreadId getNewBinHandle;
extern osThreadId keepAliveHandle;
extern osThreadId createWebPckgHandle;
extern osThreadId wirelessSensHandle;
extern osTimerId  timerPowerOffHandle;
extern osMutexId  mutexWriteToEnergyBufHandle;
extern osMutexId  mutexWebHandle;
extern osMutexId  mutexRTCHandle;
extern osMutexId  mutexSDHandle;
extern osMutexId  mutexSpiFlashHandle;

extern CircularBuffer circBufAllPckgs;

u8                     isRxNewFirmware = 0;
static u8              bufNumBytesFirmware[8];
static PckgUpdFirmware pckgInfoFirmware;
static u8              partFirmware[SZ_PART_FIRMW + 1];
static u32             flashAddrFirmware = FLASH_ADDR_BUF_NEW_FIRMWARE;
static u32             szSoft = 0;
static u32             crcNewFW;

void big_update_func();

void taskGetNewBin(void const* argument) {
    u8 updateFailCnt = 0;

    FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3);

    vTaskSuspend(getNewBinHandle);
    LOG(LEVEL_MAIN, "taskGetNewBin\r\n");

    lockAllTasks();
    isRxNewFirmware = 1;
    osTimerStop(timerPowerOffHandle);

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_NEW_BIN;
        big_update_func();
        osDelay(5000);
        updateFailCnt++;
        if (updateFailCnt == 5) {
            NVIC_SystemReset();
        }
    }
    /* USER CODE END taskGetNewBin */
}

void big_update_func() {
    u32 curSzSoft = 0;
    u32 szPartSoft;
    u8  cntFailTCPReq = 0;

    osMutexWait(mutexWebHandle, osWaitForever);
    closeTcp();
    osMutexRelease(mutexWebHandle);
    while (!(szSoft = getSzFirmware())) {};
    bkte.info.fw.szNewFirmware = szSoft;
    if (sendMsgFWUpdateBegin() != SUCCESS) {
        LOG(LEVEL_ERROR, "Send FW UPDATED\r\n");
    }

    flashClearPage(FLASH_SECTOR_11);
    clearAllWebPckgs();
    HAL_GPIO_WritePin(LED4G_GPIO_Port, LED4G_Pin, GPIO_PIN_SET);
    crcNewFW = 0xffffffff;

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_NEW_BIN;
        bkte.stat.new_bin++;
        if (szSoft != curSzSoft) {
            if (szSoft - curSzSoft > SZ_PART_FIRMW) {
                szPartSoft = SZ_PART_FIRMW;
            } else {
                szPartSoft = szSoft - curSzSoft;
            }
            pckgInfoFirmware.fromByte = curSzSoft;
            pckgInfoFirmware.toByte = szPartSoft + curSzSoft;
            memcpy(bufNumBytesFirmware, &pckgInfoFirmware.fromByte, 4);
            memcpy(bufNumBytesFirmware + 4, &pckgInfoFirmware.toByte, 4);
            memset(partFirmware, 0xFF, SZ_PART_FIRMW + 1);

            osMutexWait(mutexWebHandle, osWaitForever);
            if (!bkte.state.isTCPOpen) {
                while (openTcp(SERVER_MOTZ) != TCP_OK) {}
                cntFailTCPReq = 0;
            }
            osMutexRelease(mutexWebHandle);

            osDelay(100);
            if (getPartFirmware(bufNumBytesFirmware, partFirmware, szPartSoft + 4, 8) == SUCCESS &&
                isCrcOk(partFirmware, szPartSoft)) {
                crc32_chank(&crcNewFW, partFirmware, szPartSoft);
                HAL_GPIO_TogglePin(LED4G_GPIO_Port, LED4G_Pin);
                cntFailTCPReq = 0;

                flashWrite(partFirmware, szPartSoft, &flashAddrFirmware);

                curSzSoft += szPartSoft;
                LOG(LEVEL_MAIN, "DOWNLOAD %d BYTES addr 0x%08x\r\n", (int)curSzSoft, flashAddrFirmware);
            } else {
                LOG(LEVEL_ERROR, "httpPost() DOWNLOAD\r\n");
                cntFailTCPReq++;
                if (cntFailTCPReq > 10) {
                    cntFailTCPReq = 0;
                    // simReset();
                    return;
                }
            }
        } else {
            osMutexWait(mutexWebHandle, osWaitForever);
            if (!bkte.state.isTCPOpen) {
                while (openTcp(SERVER_MOTZ) != TCP_OK) {}
            }
            osMutexRelease(mutexWebHandle);
            if (sendMsgFWUpdated() != SUCCESS) {
                LOG(LEVEL_ERROR, "Send FW UPDATED\r\n");
            }
            LOG(LEVEL_MAIN, "DOWNLOAD COMPLETE\r\n");
            osMutexWait(mutexWebHandle, osWaitForever);
            closeTcp();
            osMutexRelease(mutexWebHandle);

            crcNewFW ^= 0xffffffff;
            LOG(LEVEL_MAIN, "DOWNLOAD COMPLETECRC 0x%08x\r\n", crcNewFW);

            updBootInfo();
            osTimerStart(timerPowerOffHandle, 300000);

            osDelay(100);
            spiFlashSaveInfo();

            osDelay(1000);
            NVIC_SystemReset();
        }
    }
}

void updBootInfo() {
    while (HAL_FLASH_Unlock() != HAL_OK) LOG_FLASH(LEVEL_ERROR, "HAL_FLASH_Unlock()\r\n");
    FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3);
    LOG_FLASH(LEVEL_MAIN, "FLASH_Erase_Sector\r\n");
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_ID_BOOT, BKTE_ID_BOOT))
        LOG_FLASH(LEVEL_ERROR, "HAL_FLASH_Program(BOOT_ADDR_ID_LOADER)\r\n");
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_IS_NEW_FIRMWARE, (u32)1))
        LOG_FLASH(LEVEL_ERROR, "HAL_FLASH_Program(BOOT_ADDR_IS_NEW_FIRWARE)\r\n");
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_SZ_NEW_FIRMWARE, (u32)(szSoft)))
        LOG_FLASH(LEVEL_ERROR, "HAL_FLASH_Program(FLASH_ADDR_SZ_NEW_FIRMWARE)\r\n");
    while (HAL_FLASH_Program(TYPEPROGRAM_WORD, FLASH_ADDR_CRC_NEW_FIRMWARE, (u32)(crcNewFW)))
        LOG_FLASH(LEVEL_ERROR, "HAL_FLASH_Program(FLASH_ADDR_CRC_NEW_FIRMWARE)\r\n");

    while (HAL_FLASH_Lock() != HAL_OK) LOG_FLASH(LEVEL_ERROR, "HAL_FLASH_Lock()\r\n");
    LOG_FLASH(LEVEL_MAIN, "BOOT_ID: %d\r\n", (int)getFlashData(FLASH_ADDR_ID_BOOT));
    LOG_FLASH(LEVEL_MAIN, "IS_NEW_FIRMARE: %d\r\n", (int)getFlashData(FLASH_ADDR_IS_NEW_FIRMWARE));
    LOG_FLASH(LEVEL_MAIN, "SZ_NEW_FIRMWARE: %d\r\n", (int)getFlashData(FLASH_ADDR_SZ_NEW_FIRMWARE));
    LOG_FLASH(LEVEL_MAIN, "CRC_NEW_FIRMARE: 0x%08x\r\n", (int)getFlashData(FLASH_ADDR_CRC_NEW_FIRMWARE));
}

void lockAllTasks() {
    osMutexWait(mutexWriteToEnergyBufHandle, osWaitForever);
    osMutexWait(mutexRTCHandle, osWaitForever);
    osMutexWait(mutexSpiFlashHandle, osWaitForever);
    osMutexWait(mutexSDHandle, osWaitForever);
    osMutexWait(mutexWebHandle, osWaitForever);

    vTaskSuspend(webExchangeHandle);
    vTaskSuspend(getEnergyHandle);
    vTaskSuspend(getTempHandle);
    vTaskSuspend(keepAliveHandle);
    // vTaskSuspend(loraHandle);
    vTaskSuspend(createWebPckgHandle);
    vTaskSuspend(wirelessSensHandle);

    osMutexRelease(mutexWriteToEnergyBufHandle);
    osMutexRelease(mutexRTCHandle);
    osMutexRelease(mutexSpiFlashHandle);
    osMutexRelease(mutexSDHandle);
    osMutexRelease(mutexWebHandle);
}

u32 getSzFirmware() {
    u8  bufSzFirmware[4];
    u8* idMCU;
    idMCU = (u8*)&bkte.info.idMCU;

    if (generateWebPckgReq(CMD_REQUEST_SZ_FIRMWARE, NULL, 0, SZ_REQUEST_GET_SZ_FIRMWARE, bufSzFirmware, 4, idMCU) == ERROR) {
        LOG(LEVEL_ERROR, "sz firmware\r\n");
        return 0;
    } else {
        u32 numFirmware = bufSzFirmware[0] << 24 | bufSzFirmware[1] << 16 | bufSzFirmware[2] << 8 | bufSzFirmware[3];
        LOG(LEVEL_INFO, "OK: sz firmware %d\r\n", numFirmware);
        return numFirmware;
    }
}

ErrorStatus getPartFirmware(u8* reqData, u8* answBuf, u16 szAnsw, u8 szReq) {
    WebPckg*    curPckg;
    ErrorStatus ret = SUCCESS;

    u8* idMCU;
    idMCU = (u8*)&bkte.info.idMCU;

    curPckg = createWebPckgReq(CMD_REQUEST_PART_FIRMWARE, reqData, szReq, SZ_REQUEST_GET_PART_FIRMWARE, idMCU);
    osMutexWait(mutexWebHandle, osWaitForever);
    if (sendTcp(SERVER_MOTZ, curPckg->buf, curPckg->shift) != TCP_OK) {
        LOG(LEVEL_ERROR, "part Firmware\r\n");
        ret = ERROR;
    } else {
        waitIdleCnt("wait IDLE part firmware", &(uInfoSim.irqFlags), szAnsw / SZ_TCP_PCKG, 100, 10000);
        osDelay(100);
        memcpy(answBuf, &uInfoSim.pRxBuf[11], szAnsw);
    }
    osMutexRelease(mutexWebHandle);
    clearWebPckg(curPckg);
    return ret;
}
