#include "../Tasks/Inc/task_get_energy.h"

#include "../Tasks/Inc/task_iwdg.h"
#include "../Tasks/Inc/task_keep_alive.h"

extern u16 iwdgTaskReg;

extern osThreadId    getEnergyHandle;
extern osThreadId    webExchangeHandle;
extern osThreadId    getTempHandle;
extern osThreadId    keepAliveHandle;
extern osThreadId    createWebPckgHandle;
extern osThreadId    getNewBinHandle;
extern osThreadId    wirelessSensHandle;
extern osSemaphoreId semCreateWebPckgHandle;
extern osMutexId     mutexWriteToEnergyBufHandle;

extern CircularBuffer rxUart1CircBuf;
extern CircularBuffer circBufAllPckgs;

static u16 testBufUart1[SZ_RX_UART1];

static PckgVoltAmper pckgVoltAmper;
static PckgEnergy    pckgEnergy;

u8 isVoltAmperFresh(PckgVoltAmper *pckg);
u8 isEnergyFresh(PckgEnergy *pckg);

// u8 test = 0;

void taskGetEnergy(void const *argument) {
    // vTaskSuspend(getEnergyHandle);
    u8  numIteration = 0;
    u16 retLen;

    spiFlashInit(circBufAllPckgs.buf);
    cBufReset(&circBufAllPckgs);
    sdInit();
    simInit();
    while (getServerTime() != SUCCESS)
        ;

    sendInitTelemetry();
    unLockTasks();
    rxUart1_IT();

    for (;;) {
        memset(testBufUart1, '\0', sizeof(testBufUart1));
        retLen = cBufRead(&rxUart1CircBuf, (u8 *)testBufUart1, 0);
        if (retLen == BKTE_SZ_UART_MSG) {
            numIteration = (numIteration + 1) % BKTE_MEASURE_FULL_LOOP;
            fillPckgVoltAmper(&pckgVoltAmper, testBufUart1);
            // LOG_PARAM(LEVEL_INFO, "Energy: volt %d, amper %d (prev: %d %d)\r\n", pckgVoltAmper.volt, pckgVoltAmper.amper, bkte.lastData.volt, bkte.lastData.current));
            if (isVoltAmperFresh(&pckgVoltAmper) || !numIteration) {
                saveData((u8 *)&pckgVoltAmper, SZ_CMD_VOLTAMPER, CMD_DATA_VOLTAMPER, &circBufAllPckgs);
                // LOG_PARAM(LEVEL_INFO, "Energy: volt %d, amper %d\r\n", pckgVoltAmper.volt, pckgVoltAmper.amper));
            }

            fillPckgEnergy(&pckgEnergy, testBufUart1);
            // LOG_PARAM(LEVEL_INFO, "Energy: act %d, react %d (prev: %d %d)\r\n", pckgEnergy.enAct, pckgEnergy.enReact, bkte.lastData.enAct, bkte.lastData.enReact));
            if (isEnergyFresh(&pckgEnergy) || !numIteration) {
                saveData((u8 *)&pckgEnergy, SZ_CMD_ENERGY, CMD_DATA_ENERGY, &circBufAllPckgs);
                // LOG_PARAM(LEVEL_INFO, "Energy: act %d, react %d (prev: %d %d)\r\n", pckgEnergy.enAct, pckgEnergy.enReact, bkte.lastData.enAct, bkte.lastData.enReact));
            }
            // LOG_PARAM(LEVEL_INFO, "OK: enAct: %08x, enReact: %08x", pckgEnergy.enAct, pckgEnergy.enReact));
        }
        osDelay(400);
        iwdgTaskReg |= IWDG_TASK_REG_ENERGY;
        bkte.stat.enrg++;
    }
}

u8 isVoltAmperFresh(PckgVoltAmper *pckg) {
    if (pckg->amper != bkte.lastData.current || pckg->volt != bkte.lastData.volt) {
        bkte.lastData.current = pckg->amper;
        bkte.lastData.volt = pckg->volt;
        return 1;
    }
    return 0;
}

u8 isEnergyFresh(PckgEnergy *pckg) {
    u8 ret = 0;

    if (pckg->enAct < bkte.lastData.enAct) {
        if (pckg->enAct + 20 < bkte.lastData.enAct) {
            bkte.lastData.enAct = pckg->enAct;
        } else {
            pckg->enAct = 0;
        }
        ret |= 1;
    } else if (pckg->enAct > bkte.lastData.enAct) {
        bkte.lastData.enAct = pckg->enAct;
        ret |= 1;
    }

    if (pckg->enReact < bkte.lastData.enReact) {
        if (pckg->enReact + 20 < bkte.lastData.enReact) {
            bkte.lastData.enReact = pckg->enReact;
        } else {
            pckg->enReact = 0;
        }
        ret |= 1;
    } else if (pckg->enReact > bkte.lastData.enReact) {
        bkte.lastData.enReact = pckg->enReact;
        ret |= 1;
    }

    return ret;
}

void unLockTasks() {
    LOG(LEVEL_MAIN, "unLockTasks\r\n");
    // vTaskResume(getNewBinHandle); //! debug download firmware
    vTaskResume(webExchangeHandle);
    vTaskResume(getTempHandle);
    vTaskResume(createWebPckgHandle);
    vTaskResume(wirelessSensHandle);
    vTaskResume(keepAliveHandle);
}

void generateInitTelemetry() {
    PckgTelemetry pckgTel;
    long long     phoneNum;
    u32           tmp;
    pckgTel.group = TEL_GR_GENINF;
    pckgTel.code = TEL_CD_GENINF_NUM_FIRMWARE;
    pckgTel.data = BKTE_ID_FIRMWARE;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_GENINF_NUM_BOOT;
    pckgTel.data = BKTE_ID_BOOT;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_GENINF_NUM_PCB;
    pckgTel.data = BKTE_ID_PCB;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    phoneNum = simGetPhoneNum();
    if (phoneNum > 0) {
        tmp = phoneNum % 1000000000;
        pckgTel.code = TEL_CD_GENINF_PHONE_NUM1;
        pckgTel.data = tmp;
        saveTelemetry(&pckgTel, &circBufAllPckgs);
    }

    pckgTel.group = TEL_GR_HARDWARE_STATUS;
    pckgTel.code = TEL_CD_HW_BKTE;
    pckgTel.data = 1;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_HW_SD;
    pckgTel.data = bkte.hwStat.isFatMount;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_HW_DS2482;
    pckgTel.data = bkte.hwStat.isDS2482;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_HW_SPI_FLASH;
    pckgTel.data = bkte.hwStat.isSPIFlash;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = TEL_CD_HW_LORA;
    pckgTel.data = bkte.hwStat.isLoraOk;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = (u8)12;
    pckgTel.data = bkte.lastData.enAct;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    pckgTel.code = (u8)13;
    pckgTel.data = bkte.lastData.enReact;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    updSpiFlash(&circBufAllPckgs);
    osSemaphoreRelease(semCreateWebPckgHandle);
}
