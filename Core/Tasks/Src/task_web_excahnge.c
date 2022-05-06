#include "task_iwdg.h"
#include "task_web_exchange.h"

extern u16 iwdgTaskReg;

extern osThreadId    webExchangeHandle;
extern osThreadId    createWebPckgHandle;
extern osThreadId    createWebPckgHandle;
extern osMutexId     mutexWebHandle;
extern osMutexId     mutexSessionHandle;
extern osMessageQId  queueWebPckgHandle;
extern osSemaphoreId semSendWebPckgHandle;

static WebPckg* curPckg = NULL;

ErrorStatus sendAuthorizePckg();
ErrorStatus sendEndSessionPckg();

void taskWebExchange(void const* argument) {
    u8  statSend = TCP_OK;
    u8  sessionStep = SESSION_OPENING;
    u32 order_num = 0;

    osSemaphoreWait(semSendWebPckgHandle, 1);

    vTaskSuspend(webExchangeHandle);

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_WEB_EXCH;

        // if (bkte.state.isTCPOpen != bkte.info.server) {
        //     sessionStep = SESSION_OPENING;
        // }

        switch (sessionStep) {
            case SESSION_OPENING:
                if (bkte.state.isTCPOpen == bkte.info.server) {
                    sessionStep = SESSION_AUTHORIZE;
                    continue;
                }
                if (osSemaphoreWait(semSendWebPckgHandle, 30000) != osOK) {
                    continue;
                }
                osMutexWait(mutexSessionHandle, osWaitForever);
                LOG_WEB(LEVEL_INFO, "{--- Start sending DATA\r\n");

                osMutexWait(mutexWebHandle, osWaitForever);
                if (openTcp(bkte.info.server) != TCP_OK) {
                    osMutexRelease(mutexSessionHandle);
                }
                osMutexRelease(mutexWebHandle);
                break;
            case SESSION_AUTHORIZE:
                if (bkte.info.server == SERVER_MOTZ) {
                    sessionStep = SESSION_SENDING;
                    continue;
                }
                if (sendAuthorizePckg() == SUCCESS) {
                    sessionStep = SESSION_SENDING;
                } else {
                    sessionStep = SESSION_TCP_CLOSING;
                }
                break;
            case SESSION_SENDING:
                if (statSend == TCP_OK) {
                    if (curPckg != NULL) {
                        clearWebPckg(curPckg);
                        curPckg = NULL;
                    }
                    osEvent evt = osMessageGet(queueWebPckgHandle, 500);
                    if (evt.status == osEventMessage) {
                        curPckg = (WebPckg*)evt.value.p;
                    } else {
                        sessionStep = SESSION_CLOSING;
                        continue;
                    }
                }

                osMutexWait(mutexWebHandle, osWaitForever);
                if (bkte.info.server == SERVER_MOTZ) {
                    memcpy(&order_num, &curPckg->buf[2], 4);
                } else {
                    memcpy(&order_num, &curPckg->buf[1], 4);
                }
                u32 ttt = HAL_GetTick();
                statSend = sendTcp(bkte.info.server, curPckg->buf, curPckg->shift);
                ttt = HAL_GetTick() - ttt;
                if (statSend == TCP_OK) {
                    LOG_WEB(LEVEL_INFO, "TCP Send: num %d, sz %d, time %d\r\n", order_num, curPckg->shift, ttt);
                    clearWebPckg(curPckg);
                    curPckg = NULL;
                } else {
                    sessionStep = SESSION_TCP_CLOSING;
                    LOG_WEB(LEVEL_ERROR, "TCP Send: num %d, sz %d, time %d Failed\r\n", order_num, curPckg->shift, ttt);
                }
                osMutexRelease(mutexWebHandle);
                break;
            case SESSION_CLOSING:
                if (bkte.info.server == SERVER_MOTZ) {
                    sessionStep = SESSION_TCP_CLOSING;
                    continue;
                }
                if (sendEndSessionPckg() == SUCCESS) {
                    sessionStep = SESSION_TCP_CLOSING;
                } else {
                    sessionStep = SESSION_TCP_CLOSING;
                }
                break;
            case SESSION_TCP_CLOSING:
                if (!bkte.state.isTCPOpen) {
                    sessionStep = SESSION_OPENING;
                    osSemaphoreWait(semSendWebPckgHandle, 0);
                    LOG_WEB(LEVEL_INFO, "---} Finish sending DATA\r\n");
                } else {
                    osMutexWait(mutexWebHandle, osWaitForever);
                    closeTcp();
                    osMutexRelease(mutexWebHandle);
                }

                osMutexRelease(mutexSessionHandle);
                break;

            default:
                sessionStep = SESSION_TCP_CLOSING;
                break;
        }
    }
}

ErrorStatus sendAuthorizePckg() {
    u8          statSend = TCP_OK;
    ErrorStatus ret = SUCCESS;
    WebPckg*    pckg;

    pckg = getFreePckgReq();
    if (pckg == NULL) {
        LOG_WEB(LEVEL_ERROR, "NO FREE PCKG\r\n");
        return ERROR;
    }

    makeAuthorizePckg(pckg, bkte.info.server);

    osMutexWait(mutexWebHandle, osWaitForever);
    u32 ttt = HAL_GetTick();
    statSend = sendTcp(bkte.info.server, pckg->buf, pckg->shift);
    ttt = HAL_GetTick() - ttt;
    LOG_WEB(LEVEL_INFO, "TCP Send: authorize, time %d\r\n", ttt);
    if (statSend != TCP_OK) ret = ERROR;

    clearWebPckg(pckg);
    pckg = NULL;

    osMutexRelease(mutexWebHandle);

    return ret;
}

ErrorStatus sendEndSessionPckg() {
    u8          statSend = TCP_OK;
    ErrorStatus ret = SUCCESS;
    WebPckg*    pckg;

    pckg = getFreePckgReq();
    if (pckg == NULL) {
        LOG_WEB(LEVEL_ERROR, "NO FREE PCKG\r\n");
        return ERROR;
    }

    makeEndSessionPckg(pckg, bkte.info.server);

    osMutexWait(mutexWebHandle, osWaitForever);
    u32 ttt = HAL_GetTick();
    statSend = sendTcp(bkte.info.server, pckg->buf, pckg->shift);
    ttt = HAL_GetTick() - ttt;
    LOG_WEB(LEVEL_INFO, "TCP Send: end session, time %d\r\n", ttt);
    if (statSend != TCP_OK) ret = ERROR;

    clearWebPckg(pckg);
    pckg = NULL;

    osMutexRelease(mutexWebHandle);

    return ret;
}