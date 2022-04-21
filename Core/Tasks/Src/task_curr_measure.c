#include "task_curr_measure.h"

#include "task_iwdg.h"
#include "task_keep_alive.h"
#include "tim.h"

extern u16 iwdgTaskReg;

extern osThreadId    getCurrentHandle;
extern osThreadId    webExchangeHandle;
extern osThreadId    keepAliveHandle;
extern osThreadId    createWebPckgHandle;
extern osThreadId    getNewBinHandle;
extern osSemaphoreId semCreateWebPckgHandle;
extern osMutexId     mutexBigBufHandle;

extern CircularBuffer circBufAllPckgs;

volatile uint16_t adc[8000] = {0};
int               cnt = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    u32 sum = 0;
    u16 max = 0;
    u16 min = 4096;
    if (hadc->Instance == ADC1)  // check if the interrupt comes from ACD1
    {
        for (int i = 1000; i < 2000; i++) {
            sum += adc[4 * i + 1];
            if (adc[4 * i + 1] > max) max = adc[4 * i + 1];
            if (adc[4 * i + 1] < min) min = adc[4 * i + 1];
            // printf("%d\t%d\n", (cnt * 1000 + i), adc[4 * i + 1]);
        }
        LOG(LEVEL_INFO, "2 %d\t%d\t%d\r\n", sum / 1000, min, max);
        bkte.pwrInfo.adcVoltBat = (u16)(adc[0] * 3.3 * 2 / 4096 * 100);
        cnt++;
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    u32 sum = 0;
    u16 max = 0;
    u16 min = 4096;
    if (hadc->Instance == ADC1)  // check if the interrupt comes from ACD1
    {
        for (int i = 0; i < 1000; i++) {
            sum += adc[4 * i + 1];
            if (adc[4 * i + 1] > max) max = adc[4 * i + 1];
            if (adc[4 * i + 1] < min) min = adc[4 * i + 1];
            // printf("%d\t%d\n", (cnt * 1000 + i), adc[4 * i + 1]);
        }
        LOG(LEVEL_INFO, "1 %d\t%d\t%d\r\n", sum / 1000, min, max);
        bkte.pwrInfo.adcVoltBat = (u16)(adc[0] * 3.3 * 2 / 4096 * 100);
        cnt++;
    }
}

// u8 test = 0;

void taskCurMeasure(void const *argument) {
    // vTaskSuspend(getCurrentHandle);
    u8  numIteration = 0;
    u16 retLen;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc, 8000);
    HAL_TIM_Base_Start(&htim2);

    // spiFlashInit(circBufAllPckgs.buf);
    // cBufReset(&circBufAllPckgs);
    // sdInit();
    // simInit();
    // while (getServerTime() != SUCCESS) {};

    // sendInitTelemetry();
    // unLockTasks();

    for (;;) {
        osDelay(400);
        iwdgTaskReg |= IWDG_TASK_CURR_MEASURE;
    }
}

void unLockTasks() {
    LOG(LEVEL_MAIN, "unLockTasks\r\n");
    // vTaskResume(getNewBinHandle); //! debug download firmware
    vTaskResume(webExchangeHandle);
    vTaskResume(createWebPckgHandle);
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

    pckgTel.code = TEL_CD_HW_SPI_FLASH;
    pckgTel.data = bkte.hwStat.isSPIFlash;
    saveTelemetry(&pckgTel, &circBufAllPckgs);

    updSpiFlash(&circBufAllPckgs);
    osSemaphoreRelease(semCreateWebPckgHandle);
}
