#include "task_create_webpckg.h"

#include "task_iwdg.h"

extern u16 iwdgTaskReg;

extern osThreadId    webExchangeHandle;
extern osThreadId    keepAliveHandle;
extern osThreadId    getNewBinHandle;
extern osThreadId    createWebPckgHandle;
extern osMutexId     mutexBigBufHandle;
extern osMutexId     mutexWebHandle;
extern osMessageQId  queueWebPckgHandle;
extern osSemaphoreId semCreateWebPckgHandle;
extern osSemaphoreId semSendWebPckgHandle;

static char tmpBufPage[256];

static Page     pgAdcCurr = {.type = CMD_DATA_ADC_CURR, .szType = SZ_CMD_ADC_CURR};
static Page     pgTelemetry = {.type = CMD_DATA_TELEMETRY, .szType = SZ_CMD_TELEMETRY};
static Page    *allPages[] = {&pgAdcCurr, &pgTelemetry};
static WebPckg *curPckg;

void taskCreateWebPckg(void const *argument) {
    // vTaskSuspend(webExchangeHandle);

    u16 delayPages;
    u16 szAllPages = 0;
    u8  amntPages;
    u8  len, flush = 0;

    osSemaphoreWait(semCreateWebPckgHandle, 1);

    vTaskSuspend(createWebPckgHandle);

    LOG_WEB(LEVEL_MAIN, "taskCreateWebPckg\r\n");
    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_WEB_PCKG;
        flush = 0;
        delayPages = getDelayPages();
        if (osSemaphoreWait(semCreateWebPckgHandle, 2000) == osOK) {
            flush = 1;
            LOG_WEB(LEVEL_INFO, "FLUSH, pages: %d\r\n", delayPages);
        }
        while (flush == 1 || delayPages > 3) {
            if (delayPages == 0) {
                flush = 0;
                break;
            }
            curPckg = getFreePckg();
            if (curPckg == NULL) {
                flush = 2;
                break;
            }
            clearAllPages();
            if (bkte.state.csq < 10) {
                amntPages = delayPages > 3 ? 3 : delayPages;
            } else if (bkte.state.csq < 15) {
                amntPages = delayPages > 4 ? 4 : delayPages;
            } else if (bkte.state.csq <= 31) {
                amntPages = delayPages > 5 ? 5 : delayPages;
            } else {
                amntPages = delayPages > 3 ? 3 : delayPages;
            }
            for (u8 i = 0; i < amntPages; i++) {
                len = spiFlashReadLastPg((u8 *)tmpBufPage, 256, 0);
                if (len) {
                    parseData(tmpBufPage, len);
                }
            }
            szAllPages = getSzAllPages();
            if (szAllPages) {
                // LOG_WEB(LEVEL_INFO, "Create package\r\n");
                initWebPckg(curPckg, szAllPages, 0, (u8 *)&bkte.info.idMCU, bkte.info.server);
                addPagesToWebPckg(curPckg);
                if (osMessagePut(queueWebPckgHandle, (u32)curPckg, 180000) != osOK) {
                    bkte.stat.queueErrCount++;
                    flush = 2;
                }
                if (flush == 1) flush = 2;
            } else {
                freeWebPckg(curPckg);
                flush = 0;
            }
            delayPages = getDelayPages();
        }

        if (flush == 2) {
            flush = 0;
            // LOG_WEB(LEVEL_INFO, "FLUSH CONTINUED 1\r\n");
            osSemaphoreRelease(semSendWebPckgHandle);
        }

        if (!delayPages) {
            LOG_WEB(LEVEL_DEBUG, "no pckg in spiflash\r\n");
            bkte.state.isSentData = 1;
        }
        // osDelay(1000);
    }
}

void clearAllPages() {
    for (u8 i = 0; i < CNT_PAGES; i++) {
        clearPage(allPages[i]);
    }
}

void parseData(u8 *tmpBufPage, u8 len) {
    u8 i = 0;
    while (i < len) {
        switch (tmpBufPage[i]) {
            case CMD_DATA_TELEMETRY:
                addToPage(&pgTelemetry, &tmpBufPage[i + 1], SZ_CMD_TELEMETRY);
                i += (SZ_CMD_TELEMETRY + 1);
                break;
            case CMD_DATA_ADC_CURR:
                addToPage(&pgAdcCurr, &tmpBufPage[i + 1], SZ_CMD_ADC_CURR);
                i += (SZ_CMD_ADC_CURR + 1);
                break;
            default:
                LOG_WEB(LEVEL_ERROR, "ER: CMD_DATA_X is wrong %d\r\n", tmpBufPage[i]);
                return;
                break;
        }
    }
}

void clearPage(Page *pg) {
    if (pg->iter) {
        pg->iter = 0;
        memset(pg->buf, SZ_PAGES, '\0');
    }
}

u16 getSzAllPages() {
    u16 sz = 0;
    for (u8 i = 0; i < CNT_PAGES; i++) {
        if (allPages[i]->iter) {
            sz += (allPages[i]->iter + 1 + 1);  // sz + sizeof(type) + szeof(cnt)
        }
    }
    return sz;
}

void addToPage(Page *pg, u8 *src, u8 sz) {
    memcpy(&pg->buf[pg->iter], src, sz);
    pg->iter += sz;
}

void addPagesToWebPckg(WebPckg *pckg) {
    for (u8 i = 0; i < CNT_PAGES; i++) {
        if (allPages[i]->iter) {
            addInfoToWebPckg(pckg, allPages[i]->buf, allPages[i]->iter, allPages[i]->iter / allPages[i]->szType, allPages[i]->type);
        }
    }
    closeWebPckg(pckg, bkte.info.server);
    // showWebPckg(pckg);
}