#include "utils_pckgs_manager.h"

#include "utils_bkte.h"

WebPckg    webPckgs[CNT_WEBPCKGS + 1];
static u16 motzPream = BKTE_PREAMBLE;
static u8  niacPream[] = {0xa1, 0xa2, 0xa3};
static u8  endBytes[] = {0x0D, 0x0A};  // reverse order
static u8  endSessionByte = 0xE2;

extern osMessageQId queueWebPckgHandle;
extern osMutexId    mutexWebHandle;
extern osMutexId    mutexSessionHandle;

void addInfo(WebPckg* pPckg, u8* src, u16 sz) {
    memcpy(&pPckg->buf[pPckg->shift], src, sz);
    pPckg->shift += sz;
}

void clearWebPckg(WebPckg* pPckg) {
    memset(pPckg->buf, '\0', SZ_WEB_PCKG);
    pPckg->shift = 0;
    pPckg->isFull = 0;
}

void clearAllWebPckgs() {
    for (u8 i = 0; i < CNT_WEBPCKGS; i++) {
        clearWebPckg(&webPckgs[i]);
    }
}

void initWebPckg(WebPckg* pPckg, u16 len, u8 isReq, u8* idMCU, u8 server) {
    static u32 numMotz = 0;
    static u32 numNiac = 0;

    memset(pPckg->buf, '\0', SZ_WEB_PCKG);
    pPckg->shift = 0;
    if (server == SERVER_MOTZ) {
        numMotz++;
        pPckg->isRequest = isReq;
        addInfo(pPckg, (u8*)&motzPream, 2);
        addInfo(pPckg, (u8*)&numMotz, 4);
        addInfo(pPckg, (u8*)idMCU, 12);
        addInfo(pPckg, (u8*)&len, 2);
    } else if (server == SERVER_NIAC) {
        numNiac++;
        addInfo(pPckg, (u8*)&niacPream[1], 1);
        addInfo(pPckg, (u8*)&numNiac, 4);
        addInfo(pPckg, (u8*)&len, 2);
    }
}

void addInfoToWebPckg(WebPckg* pPckg, u8* src, u16 sz, u8 cnt, u8 cmdData) {
    addInfo(pPckg, &cmdData, 1);
    addInfo(pPckg, &cnt, 1);
    addInfo(pPckg, src, sz);
}

void showWebPckg(WebPckg* pPckg) {
    for (u16 i = 0; i < pPckg->shift; i++) {
        printf("%02x", pPckg->buf[i]);
    }
    printf("\r\n");
}

void closeWebPckg(WebPckg* pPckg, u8 server) {
    if (server == SERVER_MOTZ) {
        addInfo(pPckg, (u8*)endBytes, 2);
    } else if (server == SERVER_NIAC) {
        addInfo(pPckg, (u8*)&endBytes[0], 1);
    }

    pPckg->isFull = 1;
}

void freeWebPckg(WebPckg* pckg) {
    pckg->isFull = 0;
}

WebPckg* getFreePckg() {
    for (u8 i = 0; i < CNT_WEBPCKGS; i++) {
        if (!webPckgs[i].isFull) {
            webPckgs[i].isFull = 1;
            return &webPckgs[i];
        }
    }
    // LOG_WEB(LEVEL_INFO, "ER: NO FREEPCKG\r\n"));
    return NULL;
}

WebPckg* getFreePckgReq() {
    if (!webPckgs[CNT_WEBPCKGS].isFull) {
        webPckgs[CNT_WEBPCKGS].isFull = 1;
        return &webPckgs[CNT_WEBPCKGS];
    }

    // LOG_WEB(LEVEL_INFO, "ER: NO FREEPCKG\r\n"));
    return NULL;
}

u8 getCntFreePckg() {
    u8 ret = 0;
    for (u8 i = 0; i < CNT_WEBPCKGS; i++) {
        if (!webPckgs[i].isFull) ret++;
    }
    return ret;
}

void makeAuthorizePckg(WebPckg* pPckg, u8 server) {
    static u32 numSession = 0;

    memset(pPckg->buf, '\0', SZ_WEB_PCKG);
    pPckg->shift = 0;
    if (server == SERVER_NIAC) {
        numSession++;
        addInfo(pPckg, (u8*)&niacPream[0], 1);
        addInfo(pPckg, (u8*)&numSession, 2);
        addInfo(pPckg, (u8*)&bkte.imei, 8);
        addInfo(pPckg, (u8*)bkte.niacIdent, 40);

        closeWebPckg(pPckg, server);
    }
}

void makeEndSessionPckg(WebPckg* pPckg, u8 server) {
    memset(pPckg->buf, '\0', SZ_WEB_PCKG);
    pPckg->shift = 0;
    if (server == SERVER_NIAC) {
        addInfo(pPckg, (u8*)&niacPream[2], 1);
        addInfo(pPckg, (u8*)&endSessionByte, 1);

        closeWebPckg(pPckg, server);
    }
}

WebPckg* createWebPckgReq(u8 CMD_REQ, u8* data, u8 sz, u8 szReq, u8* idMCU) {
    u8       req[10];
    WebPckg* curPckg;
    req[0] = CMD_REQ;
    req[1] = 1;
    curPckg = getFreePckg();
    initWebPckg(curPckg, szReq, 1, idMCU, SERVER_MOTZ);
    if (sz) {
        memcpy(req + 2, data, sz);
    }
    addInfo(curPckg, req, szReq);
    closeWebPckg(curPckg, SERVER_MOTZ);
    // showWebPckg(curPckg);
    return curPckg;
}

ErrorStatus sendWebPckgData(u8 CMD_DATA, u8* data, u8 sz, u8 szReq, u8* idMCU) {
    WebPckg*    curPckg;
    u8          req[64];
    ErrorStatus ret = SUCCESS;

    osMutexWait(mutexSessionHandle, osWaitForever);

    req[0] = CMD_DATA;
    req[1] = szReq;
    if ((curPckg = getFreePckgReq()) != NULL) {
        initWebPckg(curPckg, sz + 2, 0, (u8*)&bkte.idMCU, SERVER_MOTZ);
        if (sz) {
            memcpy(req + 2, data, sz);
        }
        addInfo(curPckg, req, sz + 2);
        closeWebPckg(curPckg, SERVER_MOTZ);
        // showWebPckg(curPckg);

        osMutexWait(mutexWebHandle, osWaitForever);
        closeTcp();
        if (sendTcp(SERVER_MOTZ, curPckg->buf, curPckg->shift) != TCP_OK) {
            LOG_WEB(LEVEL_ERROR, "send data failed\r\n");
            ret = ERROR;
        }
        clearWebPckg(curPckg);
        closeTcp();
        osMutexRelease(mutexWebHandle);
    } else {
        ret = ERROR;
        LOG_WEB(LEVEL_ERROR, "NO FREE PCKG\r\n");
    }
    osMutexRelease(mutexSessionHandle);

    return ret;
}

ErrorStatus generateWebPckgReq(u8 CMD_REQ, u8* data, u8 sz, u8 szReq, u8* answ, u16 szAnsw, u8* idMCU) {
    ErrorStatus ret = SUCCESS;
    u8          statSend;
    u8          req[10];
    WebPckg*    curPckg;

    osMutexWait(mutexSessionHandle, osWaitForever);
    LOG_WEB(LEVEL_INFO, "{--- Start sending REQ\r\n");

    req[0] = CMD_REQ;
    req[1] = 1;
    if ((curPckg = getFreePckgReq()) != NULL) {
        initWebPckg(curPckg, szReq, 1, idMCU, SERVER_MOTZ);
        if (sz) {
            memcpy(req + 2, data, sz);
        }
        addInfo(curPckg, req, szReq);
        closeWebPckg(curPckg, SERVER_MOTZ);
        // showWebPckg(curPckg);

        osMutexWait(mutexWebHandle, osWaitForever);
        closeTcp();
        statSend = sendTcp(SERVER_MOTZ, curPckg->buf, curPckg->shift);
        if (statSend != TCP_OK)
            ret = ERROR;
        else if (uInfoSim.pRxBuf[11] == '\0' && uInfoSim.pRxBuf[12] == '\0' &&
                 uInfoSim.pRxBuf[13] == '\0' && uInfoSim.pRxBuf[14] == '\0') {
            uInfoSim.irqFlags.isIrqIdle = 0;
            if (waitIdle("wait req answ", &(uInfoSim.irqFlags), 100, 10000)) {
                memcpy(answ, &uInfoSim.pRxBuf[11], szAnsw);
            } else {
                LOG_WEB(LEVEL_ERROR, "NO ANSW REQ\r\n");
                ret = ERROR;
            }
        } else {
            osDelay(10);
            memcpy(answ, &uInfoSim.pRxBuf[11], szAnsw);
        }
        clearWebPckg(curPckg);
        closeTcp();
        osMutexRelease(mutexWebHandle);
    } else {
        ret = ERROR;
        LOG_WEB(LEVEL_ERROR, "NO FREE PCKG\r\n");
    }
    LOG_WEB(LEVEL_INFO, "---} Finish sending REQ\r\n");
    osMutexRelease(mutexSessionHandle);

    return ret;
}