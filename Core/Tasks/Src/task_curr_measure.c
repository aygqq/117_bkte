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
extern osSemaphoreId semCurrMeasureHandle;
extern osMutexId     mutexBigBufHandle;

extern CircularBuffer circBufAllPckgs;

volatile u16 buf_adc[8000] = {0};
volatile u8  flag_adc = 0;

void searchExtremums(adc_measure_t *meas);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1)  // check if the interrupt comes from ACD1
    {
        flag_adc = 2;
        osSemaphoreRelease(semCurrMeasureHandle);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1)  // check if the interrupt comes from ACD1
    {
        flag_adc = 1;
        osSemaphoreRelease(semCurrMeasureHandle);
    }
}

// u8 test = 0;

void taskCurMeasure(void const *argument) {
    u32           cnt = 0;
    adc_measure_t meas;
    // vTaskSuspend(getCurrentHandle);

    // spiFlashInit(circBufAllPckgs.buf);
    // cBufReset(&circBufAllPckgs);
    // sdInit();
    // simInit();
    // while (getServerTime() != SUCCESS) {};

    // sendInitTelemetry();
    // unLockTasks();

    osSemaphoreWait(semCurrMeasureHandle, 0);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf_adc, 8000);
    HAL_TIM_Base_Start(&htim2);

    meas.size = 1000;

    for (;;) {
        if (osSemaphoreWait(semCurrMeasureHandle, 2000) != osOK) {
            LOG(LEVEL_ERROR, "No ADC semaphore\r\n");
            continue;
        }
        meas.sum = 0;
        meas.min = 4096;
        meas.max = 0;
        if (flag_adc == 1) {
            meas.p_buf = (u16 *)&buf_adc[0];
        } else if (flag_adc == 2) {
            meas.p_buf = (u16 *)&buf_adc[meas.size];
        }
        for (u16 i = 0; i < meas.size; i++) {
            meas.sum += meas.p_buf[4 * i + 1];
            if (meas.p_buf[4 * i + 1] > meas.max) meas.max = meas.p_buf[4 * i + 1];
            if (meas.p_buf[4 * i + 1] < meas.min) meas.min = meas.p_buf[4 * i + 1];
            // printf("%d\t%d\n", (cnt * meas.size + i), meas.p_buf[4 * i + 1]);
        }
        meas.avg = meas.sum / meas.size;
        // LOG(LEVEL_INFO, "1 %d\t%d\t%d\r\n", meas.sum / meas.size, meas.min, meas.max);
        bkte.pwrInfo.adcVoltBat = (u16)(meas.p_buf[0] * 3.3 * 2 / 4096 * 100);
        cnt++;
        // HAL_ADC_Stop_DMA(&hadc1);
        searchExtremums(&meas);

        iwdgTaskReg |= IWDG_TASK_CURR_MEASURE;
    }
}

void searchExtremums(adc_measure_t *meas) {
    u16 max, min;
    u16 idx_max, idx_min;

    meas->ptr_max = 0;
    meas->ptr_min = 0;
    for (u16 i = 0; i < meas->size - 10; i = i + 5) {
        min = 4096;
        max = 0;
        for (u16 k = 4 * i; k < 4 * (i + 10); k += 4) {
            if (meas->p_buf[k + 1] > max) {
                max = meas->p_buf[k + 1];
                idx_max = k + 1;
            }
            if (meas->p_buf[k + 1] < min) {
                min = meas->p_buf[k + 1];
                idx_min = k + 1;
            }
        }
        if (max > (meas->avg + 100) && idx_max > 4 * (i + 2) && idx_max < 4 * (i + 8) && meas->arr_max[meas->ptr_max] != idx_max) {
            meas->ptr_max++;
            meas->arr_max[meas->ptr_max] = idx_max;
        }
        if (min < (meas->avg - 100) && idx_min > 4 * (i + 2) && idx_min < 4 * (i + 8) && meas->arr_min[meas->ptr_min] != idx_min) {
            meas->ptr_min++;
            meas->arr_min[meas->ptr_min] = idx_min;
        }
    }
    // printf("Max min %d\t%d\r\n", ptr_max, ptr_min);
    printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", meas->ptr_max, meas->ptr_min, meas->arr_max[1], meas->arr_max[2], meas->arr_min[1], meas->arr_min[2]);
    // for (u16 i = 0; i < ptr_max; i++) {
    //     printf("%d\r\n", arr_max[i]);
    // }
    // printf("\r\n");
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
