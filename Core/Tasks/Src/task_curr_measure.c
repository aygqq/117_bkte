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

volatile u16  buf_adc[ADC_BUF_SIZE + WINDOW_SIZE * ADC_CHAN_CNT] = {0};
volatile u8   flag_adc = 0;
adc_measure_t meas;

void calcBasicParams(adc_measure_t *meas);
void searchAllMinMax(adc_measure_t *meas);

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
    // vTaskSuspend(getCurrentHandle);

    // spiFlashInit(circBufAllPckgs.buf);
    // cBufReset(&circBufAllPckgs);
    // sdInit();
    // simInit();
    // while (getServerTime() != SUCCESS) {};

    // sendInitTelemetry();
    // unLockTasks();

    osSemaphoreWait(semCurrMeasureHandle, 0);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf_adc, ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2);
    LOG(LEVEL_MAIN, "ADC started\r\n");

    memset(&meas, 0, sizeof(adc_measure_t));
    meas.size = ADC_CHAN_BUF_SIZE;

    for (;;) {
        if (osSemaphoreWait(semCurrMeasureHandle, 2000) != osOK) {
            LOG(LEVEL_ERROR, "No ADC semaphore\r\n");
            continue;
        }

        calcBasicParams(&meas);
        searchAllMinMax(&meas);

        iwdgTaskReg |= IWDG_TASK_CURR_MEASURE;
    }
}

void calcBasicParams(adc_measure_t *meas) {
    u16         idx;
    adc_chan_t *chan;

    if (flag_adc == 1) {
        meas->p_buf = (u16 *)&buf_adc[0];
    } else if (flag_adc == 2) {
        meas->p_buf = (u16 *)&buf_adc[meas->size * ADC_CHAN_CNT];
    }
    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        chan = &meas->chan[ch];
        chan->sum = 0;
        chan->min = ADC_MAX_VAL;
        chan->max = ADC_MIN_VAL;
        for (u16 i = 0; i < meas->size; i++) {
            idx = ADC_CHAN_CNT * i + ch + 1;
            chan->sum += meas->p_buf[idx];
            if (meas->p_buf[idx] > chan->max) chan->max = meas->p_buf[idx];
            if (meas->p_buf[idx] < chan->min) chan->min = meas->p_buf[idx];
            // printf("%d\n", meas->p_buf[idx]);
        }
        chan->avg = chan->sum / meas->size;
        // LOG(LEVEL_INFO, "1 %d\t%d\t%d\r\n", chan->sum / meas->size, chan->min, chan->max);
    }
    bkte.pwrInfo.adcVoltBat = (u16)(meas->p_buf[0] * 3.3 * 2 / ADC_MAX_VAL * 100);
}

void searchAllMinMax(adc_measure_t *meas) {
    u16         idx;
    u16         max, min;
    u16         idx_max, idx_min;
    u16         from, to;
    adc_chan_t *chan;
    u16         cnt = 0;

    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        chan = &meas->chan[ch];
        chan->ptr_max = 0;
        chan->ptr_min = 0;
        for (u16 i = 0; i < meas->size - WINDOW_SIZE; i += WINDOW_STEP) {
            cnt++;
            min = ADC_MAX_VAL;
            max = ADC_MIN_VAL;
            for (u16 k = i; k < i + WINDOW_SIZE; k++) {
                idx = ADC_CHAN_CNT * k + ch + 1;
                if (meas->p_buf[idx] > max) {
                    max = meas->p_buf[idx];
                    idx_max = idx;
                }
                if (meas->p_buf[idx] < min) {
                    min = meas->p_buf[idx];
                    idx_min = idx;
                }
            }
            from = ADC_CHAN_CNT * (i + 1);
            to = ADC_CHAN_CNT * (i + WINDOW_SIZE - 1);
            if (max > (s32)(chan->avg + ADC_MIN_MAX_DELTA) && idx_max > from && idx_max < to && chan->arr_max[chan->ptr_max] != idx_max) {
                chan->ptr_max++;
                chan->arr_max[chan->ptr_max] = idx_max;
            }
            if (min < (s32)(chan->avg - ADC_MIN_MAX_DELTA) && idx_min > from && idx_min < to && chan->arr_min[chan->ptr_min] != idx_min) {
                chan->ptr_min++;
                chan->arr_min[chan->ptr_min] = idx_min;
            }
        }
        // printf("Max min %d\t%d\r\n", ptr_max, ptr_min);
        printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", chan->ptr_max, chan->ptr_min, chan->arr_max[1], chan->arr_max[2], chan->arr_min[1], chan->arr_min[2], cnt);
        // for (u16 i = 0; i < ptr_max; i++) {
        //     printf("%d\r\n", arr_max[i]);
        // }
        // printf("\r\n");
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
