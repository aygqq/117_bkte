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
extern osMutexId     mutexCurrMeasureHandle;

extern CircularBuffer circBufAllPckgs;
static PckgTelemetry  pckgTel;

volatile u16 buf_adc[ADC_BUF_SIZE + WINDOW_SIZE * ADC_CHAN_CNT] = {0};
volatile u8  flag_adc = 0;

u8   isCraneStopped(adc_measure_t *meas);
void calcBasicParams(adc_measure_t *meas);
void searchAllMinMax(adc_measure_t *meas);
void saveMeasureData(adc_measure_t *meas);

void taskCurMeasure(void const *argument) {
    u32 numIteration = 0;
    // vTaskSuspend(getCurrentHandle);

    spiFlashInit(circBufAllPckgs.buf);
    cBufReset(&circBufAllPckgs);
    sdInit();
    simInit();
    while (getServerTime() != SUCCESS) {};

    sendInitTelemetry();
    unLockTasks();

    osSemaphoreWait(semCurrMeasureHandle, 0);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&buf_adc, ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2);
    LOG(LEVEL_MAIN, "ADC started\r\n");

    for (;;) {
        if (osSemaphoreWait(semCurrMeasureHandle, 2000) != osOK) {
            LOG(LEVEL_ERROR, "No ADC semaphore\r\n");
            continue;
        }

        osMutexWait(mutexCurrMeasureHandle, 100);

        calcBasicParams(&bkte.measure);
        searchAllMinMax(&bkte.measure);
        if (isCraneStopped(&bkte.measure) == CRANE_MOVE) {
            // LOG(LEVEL_INFO, "Move save\r\n");
            saveMeasureData(&bkte.measure);
        } else {
            if (!(numIteration % 30)) {
                // LOG(LEVEL_INFO, "Stop save\r\n");
                saveMeasureData(&bkte.measure);
            }
        }
        numIteration++;
        osMutexRelease(mutexCurrMeasureHandle);

        iwdgTaskReg |= IWDG_TASK_CURR_MEASURE;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1)  // check if the interrupt comes from ACD1
    {
        flag_adc = 2;
        if (osMutexWait(mutexCurrMeasureHandle, 1) == osOK) {
            osSemaphoreRelease(semCurrMeasureHandle);
            osMutexRelease(mutexCurrMeasureHandle);
        } else {
            // pckgTel.group = TEL_GR_CRANE;
            // pckgTel.code = TEL_CD_CRANE_TIME_ERR;
            // pckgTel.data = flag_adc;
            // saveTelemetry(&pckgTel, &circBufAllPckgs);
            printf("alarm 2\r\n");
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1)  // check if the interrupt comes from ACD1
    {
        flag_adc = 1;
        if (osMutexWait(mutexCurrMeasureHandle, 1) == osOK) {
            osSemaphoreRelease(semCurrMeasureHandle);
            osMutexRelease(mutexCurrMeasureHandle);
        } else {
            // pckgTel.group = TEL_GR_CRANE;
            // pckgTel.code = TEL_CD_CRANE_TIME_ERR;
            // pckgTel.data = flag_adc;
            // saveTelemetry(&pckgTel, &circBufAllPckgs);
            printf("alarm 1\r\n");
        }
    }
}

u8 isCraneStopped(adc_measure_t *meas) {
    adc_chan_t *chan;
    u8          move = 0;

    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        chan = &meas->chan[ch];
        if (chan->max > chan->avg + ADC_MIN_MAX_DELTA) {
            move = 1;
        }
    }
    meas->stopTime = (move == 0) ? meas->stopTime++ : 0;

    if (meas->stopTime > 60) {
        return CRANE_STOP;
    }
    return CRANE_MOVE;
}

void calcBasicParams(adc_measure_t *meas) {
    u16         idx;
    adc_chan_t *chan;

    if (flag_adc == 1) {
        meas->p_buf = (u16 *)&buf_adc[0];
        HAL_GPIO_TogglePin(LED2G_GPIO_Port, LED2G_Pin);
    } else if (flag_adc == 2) {
        meas->p_buf = (u16 *)&buf_adc[meas->size * ADC_CHAN_CNT];
        HAL_GPIO_TogglePin(LED2R_GPIO_Port, LED2R_Pin);
    }
    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        chan = &meas->chan[ch];
        memset((u8 *)chan, 0, sizeof(adc_chan_t));
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

#ifdef MIN_MAX_WINDOW
void searchAllMinMax(adc_measure_t *meas) {
    u16         idx;
    u16         max, min;
    u16         idx_max, idx_min;
    u16         from, to;
    adc_chan_t *chan;

    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        chan = &meas->chan[ch];
        chan->cnt_max = 0;
        chan->cnt_min = 0;
        for (u16 i = 0; i < meas->size - WINDOW_SIZE; i += WINDOW_STEP) {
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
            if (max > (s32)(chan->avg + ADC_MIN_MAX_DELTA) && idx_max > from && idx_max < to && chan->p_max[chan->cnt_max] != idx_max) {
                chan->cnt_max++;
                chan->p_max[chan->cnt_max] = idx_max;
            }
            if (min < (s32)(chan->avg - ADC_MIN_MAX_DELTA) && idx_min > from && idx_min < to && chan->p_min[chan->cnt_min] != idx_min) {
                chan->cnt_min++;
                chan->p_min[chan->cnt_min] = idx_min;
            }
            if (chan->cnt_max == 60 || chan->cnt_min == 60) {
                break;
            }
        }
        // printf("Max min %d\t%d\r\n", cnt_max, cnt_min);
        // printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", chan->cnt_max, chan->cnt_min, chan->p_max[1], chan->p_max[2], chan->p_min[1], chan->p_min[2]);
        // for (u16 i = 0; i < cnt_max; i++) {
        //     printf("%d\r\n", p_max[i]);
        // }
        // printf("\r\n");
    }
}

void saveMeasureData(adc_measure_t *meas) {
    PckgAdcCurr pckgAdc;

    pckgAdc.unixTimeStamp = getUnixTimeStamp();
    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        pckgAdc.min[ch] = (s16)((meas->chan[ch].min - meas->calibr[ch].zero_lvl) * meas->calibr[ch].coef);
        pckgAdc.max[ch] = (s16)((meas->chan[ch].max - meas->calibr[ch].zero_lvl) * meas->calibr[ch].coef);
        pckgAdc.avg[ch] = (s16)((meas->chan[ch].avg - meas->calibr[ch].zero_lvl) * meas->calibr[ch].coef);
        pckgAdc.cnt_max[ch] = meas->chan[ch].cnt_max;
        pckgAdc.first_max[ch] = pckgAdc.cnt_max > 0 ? (meas->chan[ch].p_max[1] - ch) / 4 : 0;
        // if (ch == 0) {
        //     LOG(LEVEL_INFO, "Chan %d: %d\t%d\t%d\t%d\t%d\r\n", ch, pckgAdc.min[ch], pckgAdc.max[ch], pckgAdc.avg[ch], pckgAdc.cnt_max[ch], pckgAdc.first_max[ch]);
        // }
    }
    saveData((u8 *)&pckgAdc, SZ_CMD_ADC_CURR, CMD_DATA_ADC_CURR, &circBufAllPckgs);
}

#else

void searchAllMinMax(adc_measure_t *meas) {
    u16         idx;
    u16         max, min;
    u16         idx_max, idx_min;
    u16         comp_lvl;
    adc_chan_t *chan;

    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        chan = &meas->chan[ch];
        chan->cnt_max = 0;
        chan->cnt_min = 0;
        chan->cnt_zero = 0;
        comp_lvl = meas->calibr[ch].zero_lvl - ADC_MAX_BOUND;

        // printf("\r\nChan %d\r\n", ch);
        for (u16 i = 1; i < meas->size - 1; i++) {
            idx = ADC_CHAN_CNT * i + ch + 1;
            if (meas->p_buf[idx - ADC_CHAN_CNT] > comp_lvl &&
                meas->p_buf[idx] <= comp_lvl &&
                meas->p_buf[idx + ADC_CHAN_CNT] < comp_lvl) {
                i += 3;
                chan->p_zero[chan->cnt_zero] = idx;
                chan->cnt_zero++;
                if (chan->cnt_zero >= 100) {
                    // printf("break %d\r\n", chan->cnt_zero);
                    break;
                }
            }
        }

        for (u16 i = 0; i < chan->cnt_zero - 1; i++) {
            min = ADC_MAX_VAL;
            max = ADC_MIN_VAL;
            for (u16 idx = chan->p_zero[i]; idx < chan->p_zero[i + 1]; idx += ADC_CHAN_CNT) {
                if (meas->p_buf[idx] > max) {
                    max = meas->p_buf[idx];
                    idx_max = idx;
                }
                if (meas->p_buf[idx] < min) {
                    min = meas->p_buf[idx];
                    idx_min = idx;
                }
            }
            chan->p_max[i] = idx_max;
            chan->p_min[i] = idx_min;
        }
        // for (u16 i = 0; i < chan->cnt_zero - 1; i++) {
        //     printf("%d\t%d\t%d\t%d\r\n", i, meas->p_buf[chan->p_zero[i]], meas->p_buf[chan->p_max[i]], meas->p_buf[chan->p_min[i]]);
        // }
        // printf("\r\n");

        if (chan->cnt_zero > 1) {
            min = ADC_MAX_VAL;
            max = ADC_MIN_VAL;
            for (u16 i = 0; i < chan->cnt_zero - 1; i++) {
                if (meas->p_buf[chan->p_max[i]] > max) max = meas->p_buf[chan->p_max[i]];
                if (meas->p_buf[chan->p_max[i]] < min) min = meas->p_buf[chan->p_max[i]];
            }
            chan->max_max = max;
            chan->min_max = min;

            min = ADC_MAX_VAL;
            max = ADC_MIN_VAL;
            for (u16 i = 0; i < chan->cnt_zero - 1; i++) {
                if (meas->p_buf[chan->p_min[i]] > max) max = meas->p_buf[chan->p_min[i]];
                if (meas->p_buf[chan->p_min[i]] < min) min = meas->p_buf[chan->p_min[i]];
            }
            chan->max_min = max;
            chan->min_min = min;
        } else {
            chan->max_max = meas->calibr[ch].zero_lvl;
            chan->min_max = meas->calibr[ch].zero_lvl;
            chan->max_min = meas->calibr[ch].zero_lvl;
            chan->min_min = meas->calibr[ch].zero_lvl;
        }

        // printf("%d\t%d\t%d\t%d\t%d\r\n", chan->cnt_zero, chan->max_max, chan->min_max, chan->max_min, chan->min_min);

        // printf("Max min %d\t%d\r\n", cnt_max, cnt_min);
        // printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", chan->cnt_max, chan->cnt_min, chan->p_max[1], chan->p_max[2], chan->p_min[1], chan->p_min[2]);
        // for (u16 i = 0; i < cnt_max; i++) {
        //     printf("%d\r\n", p_max[i]);
        // }
        // printf("\r\n");
    }
}

void saveMeasureData(adc_measure_t *meas) {
    PckgAdcCurr pckgAdc;
    adc_chan_t *chan;
    u16         idx;

    pckgAdc.unixTimeStamp = getUnixTimeStamp();
    for (u8 ch = 0; ch < ADC_CHAN_CNT - 1; ch++) {
        pckgAdc.min[ch] = (s16)((meas->chan[ch].min_max - meas->calibr[ch].zero_lvl) * meas->calibr[ch].coef);
        pckgAdc.max[ch] = (s16)((meas->chan[ch].max_max - meas->calibr[ch].zero_lvl) * meas->calibr[ch].coef);
        pckgAdc.avg[ch] = (s16)((meas->chan[ch].avg - meas->calibr[ch].zero_lvl) * meas->calibr[ch].coef);
        pckgAdc.cnt_max[ch] = meas->chan[ch].cnt_zero;
        pckgAdc.first_max[ch] = pckgAdc.cnt_max[ch] > 0 ? (meas->chan[ch].p_max[0] - ch) / 4 : 0;
        // if (ch == 0) {
        //     LOG(LEVEL_INFO, "Chan %d: %d\t%d\t%d\t%d\t%d\r\n", ch, pckgAdc.min[ch], pckgAdc.max[ch], pckgAdc.avg[ch], pckgAdc.cnt_max[ch], pckgAdc.first_max[ch]);
        // }
        //     if (ch == 1 && pckgAdc.cnt_max[ch] > 50) {
        //         HAL_TIM_Base_Stop(&htim2);
        //         chan = &meas->chan[ch];
        //         pckgTel.unixTimeStamp = getUnixTimeStamp();
        //         pckgTel.group = TEL_GR_CRANE;
        //         pckgTel.code = TEL_CD_CRANE_PERIOD_ERR;
        //         for (u16 i = 0; i < chan->cnt_zero - 1; i++) {
        //             pckgTel.data = chan->p_max[i] << 16 | meas->p_buf[chan->p_max[i]];
        //             saveData((u8 *)&pckgTel, SZ_CMD_TELEMETRY, CMD_DATA_TELEMETRY, &circBufAllPckgs);
        //         }
        //         for (u16 i = 0; i < meas->size - 1; i++) {
        //             idx = ADC_CHAN_CNT * i + ch + 1;
        //             printf("%d\r\n", meas->p_buf[idx]);
        //         }
        //         printf("\r\n");
        //         HAL_TIM_Base_Start(&htim2);
        //     }
        // }
        saveData((u8 *)&pckgAdc, SZ_CMD_ADC_CURR, CMD_DATA_ADC_CURR, &circBufAllPckgs);
    }
}
#endif

void unLockTasks() {
    LOG(LEVEL_MAIN, "unLockTasks\r\n");
    // vTaskResume(getNewBinHandle); //! debug download firmware
    vTaskResume(webExchangeHandle);
    vTaskResume(createWebPckgHandle);
    vTaskResume(keepAliveHandle);
}

void generateInitTelemetry() {
    long long phoneNum;
    u32       tmp;
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
