/*
 * sim800.c
 *
 *  Created on: Feb 20, 2020
 *      Author: bls
 */

#include "simcom.h"

static char simBufCmd[COMMAND_BUF_SZ];
static char simBufError[COMMAND_BUF_SZ];

u8 isJson = 1;

void simInit() {
    char* retMsg;
    char* token;
    u8    fail = 0;
    u8    simBadAnsw = 0;
    u8    isInit = 0;

    bkte.state.isTCPOpen = 0;

    while (!isInit) {
        bkte.stat.simResetCnt++;
        simHardwareReset();
        simTxATCmd(SIM_CMD_ATE0, SIM_SZ_CMD_ATE0, 1000);
        retMsg = simTxATCmd(SIM_CMD_AT, SIM_SZ_CMD_AT, 1000);
        token = strtok(retMsg, SIM_SEPARATOR_TEXT);
        if (token == NULL || token[0] == '\0') token = SIM_NO_RESPONSE_TEXT;
        LOG_SIM(LEVEL_INFO, "simInit AT: %s\r\n", token);
        if ((strcmp(token, SIM_OK_TEXT)) != 0) {
            LOG_SIM(LEVEL_ERROR, "simInit() AT: %s\r\n", token);
            bkte.erFlags.simAT = 1;
            simBadAnsw = (simBadAnsw + 1) % 16;
            if (!simBadAnsw) {
                LOG_SIM(LEVEL_MAIN, "WARINTING!: T O T A L  R E S E T\r\n");
                spiFlashSaveInfo();
                osDelay(3000);
                HAL_NVIC_SystemReset();
            }
        } else {
            bkte.erFlags.simAT = 0;
            osDelay(5000);
            if (SIM_GPS_INIT() != SIM_SUCCESS) {
                fail++;
                bkte.erFlags.simSAPBR = 1;
                if (fail > 20) {
                    fail = 0;
                    spiFlashSaveInfo();
                    osDelay(3000);
                    HAL_NVIC_SystemReset();
                }
                LOG_SIM(LEVEL_ERROR, "NOT CONNECT GPS\r\n");
            } else {
                isInit = 1;
                bkte.erFlags.simSAPBR = 0;
            }
        }
    }

    LOG_SIM(LEVEL_MAIN, "OK: simInit()\r\n");
}

char* simGetStatusAnsw(u32 timeout) {
    waitIdle("", &(uInfoSim.irqFlags), 50, timeout);
    if (uInfoSim.irqFlags.isIrqIdle) {
        uInfoSim.irqFlags.isIrqIdle = 0;
        return (char*)uInfoSim.pRxBuf;
    } else {
        return SIM_NO_RESPONSE_TEXT;
    }
}

char* simTxATCmd(char* command, u16 sz, u32 timeout) {
    uInfoSim.irqFlags.isIrqIdle = 0;
    uartTx(command, sz, &uInfoSim);
    return simGetStatusAnsw(timeout);
}

u8 simCmd(char* cmdCode, char* params, u8 retriesCnt, char* SUCCESS_RET) {
    char* retMsg;
    char* token;
    memset(simBufCmd, '\0', COMMAND_BUF_SZ);
    if (params == NULL)
        sprintf((char*)simBufCmd, "AT+%s\r\n", cmdCode);
    else {
        sprintf((char*)simBufCmd, "AT+%s=%s\r\n", cmdCode, params);
    }
    for (; retriesCnt > 0; --retriesCnt) {
        retMsg = simTxATCmd(simBufCmd, strlen(simBufCmd), 20000);
        token = strtok(retMsg, SIM_SEPARATOR_TEXT);
        if (token == NULL || token[0] == '\0') token = SIM_NO_RESPONSE_TEXT;
        if (SUCCESS_RET != NULL && strcmp((const char*)token, (const char*)SUCCESS_RET)) {
            copyStr(simBufError, token, COMMAND_BUF_SZ);
            sprintf((char*)simBufError + strlen(token), "\r\n");
            if (strcmp((const char*)simBufError, (const char*)simBufCmd) == 0) {
                return SIM_RESTART;
            }
            LOG_SIM(LEVEL_ERROR, "%s ret: %s\r\n", simBufCmd, token);
        } else {
            LOG_SIM(LEVEL_DEBUG, "OK: %s ret: %s\r\n", simBufCmd, token);
            return SIM_SUCCESS;
        }
    }
    return SIM_FAIL;
}

void copyStr(char* dist, char* source, u16 distSz) {
    memset(dist, '\0', distSz);
    sprintf(dist, "%s", source);
}

char* simDownloadData(char* data, u16 sz) {
    return simTxATCmd(data, sz, 40000);
}

u8 simCheckCSQ() {
    u8    csq = 0;
    char* retMsg;
    char* token;

    retMsg = simTxATCmd("AT+CSQ\r\n", 8, 1000);  // check signal level
    token = strtok(retMsg, SIM_SEPARATOR_TEXT);
    csq = (token != NULL) && (strlen(token) > 8) ? atoi(token + 6) : 0;
    bkte.state.csq = csq;
    return csq;
}

void simOn() {
    HAL_GPIO_WritePin(SIM_PWR_EN_GPIO_Port, SIM_PWR_EN_Pin, GPIO_PIN_SET);
    osDelay(1500);
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_RESET);
    osDelay(1000);
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_SET);
    osDelay(1500);
    HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_RESET);
    osDelay(5000);
}

void simOff() {
    HAL_GPIO_WritePin(SIM_PWR_EN_GPIO_Port, SIM_PWR_EN_Pin, GPIO_PIN_RESET);
    osDelay(3000);
}

void simHardwareReset() {
    LOG_SIM(LEVEL_MAIN, "WARINIG!: R E S E T !\r\n");
    simOff();
    simOn();
}

void simReset() { simInit(); }

u8 simTCPCheckStatus(const char* stat, u16 timeout, u16 delay) {
    char* token;
    while (timeout > delay) {
        timeout -= delay;
        if (simCmd(SIM_CIPSTATUS, NULL, 3, SIM_OK_TEXT) != SIM_SUCCESS) {
            return SIM_FAIL;
        }
        osDelay(delay);
        token = strtok((char*)uInfoSim.pRxBuf + 6, SIM_SEPARATOR_TEXT);
        if (token == NULL || token[0] == '\0') token = SIM_NO_RESPONSE_TEXT;
        if (strcmp((const char*)token, stat) != 0) {
            LOG_SIM(LEVEL_ERROR, "%s instead %s\r\n", token, stat);
        } else {
            LOG_SIM(LEVEL_DEBUG, "%s\r\n", token);
            return SIM_SUCCESS;
        }
    }
    return SIM_FAIL;
}

u8 simTCPclose() {
    if (simCmd(SIM_CIPSHUT, NULL, 3, SIM_OK_CIPSHUT) != SIM_SUCCESS) {
        return SIM_FAIL;
    }
    return SIM_SUCCESS;
}

u8 simTCPinit() {
    // if (simCmd(SIM_CIPSHUT, NULL, 3, SIM_OK_CIPSHUT) != SIM_SUCCESS) {
    //     return SIM_FAIL;
    // }

    if (simCmd(SIM_CGATT, "1", 3, SIM_OK_TEXT) != SIM_SUCCESS) {
        return SIM_FAIL;
    }

    if (simTCPCheckStatus(SIM_CIPSTAT_INIT, 200, 50) != SIM_SUCCESS) {
        return SIM_FAIL;
    }

    if (simCmd(SIM_CSTT, "\"internet\",\"gdata\",\"gdata\"", 3, SIM_OK_TEXT) ==
        SIM_FAIL) {  // maybe can delete gdata. test it.
        return SIM_FAIL;
    }
    if (simTCPCheckStatus(SIM_CIPSTAT_START, 200, 50) != SIM_SUCCESS) {
        return SIM_FAIL;
    }

    if (simCmd(SIM_CIICR, NULL, 3, SIM_OK_TEXT) == SIM_FAIL) {
        return SIM_FAIL;
    }
    if (simTCPCheckStatus(SIM_CIPSTAT_GPRSACT, 200, 50) != SIM_SUCCESS) {
        return SIM_FAIL;
    }

    if (simCmd(SIM_CIFSR, NULL, 3, NULL) == SIM_FAIL) {
        return SIM_FAIL;
    }
    if (simTCPCheckStatus(SIM_CIPSTAT_STATUS, 200, 50) != SIM_SUCCESS) {
        return SIM_FAIL;
    }
    return SIM_SUCCESS;
}

u8 simTCPOpen(u8 server) {
    static char params[40];
    char*       retMsg;
    char*       token;
    memset(params, '\0', 40);
    if (server == SERVER_MOTZ) {
        sprintf(params, "\"%s\",\"%s\",%d", (char*)"TCP", urls.ourTcpAddr, urls.ourTcpPort);
    } else if (server == SERVER_NIAC) {
        sprintf(params, "\"%s\",\"%s\",%d", (char*)"TCP", urls.niacTcpAddr, urls.niacTcpPort);
    } else {
        return SIM_FAIL;
    }
    if (simCmd(SIM_CIPSTART, params, 3, SIM_OK_TEXT) == SIM_FAIL) {
        return SIM_FAIL;
    }
    retMsg = simGetStatusAnsw(7000);  // waiting for "CONNECT OK"
    // osDelay(2000);
    // LOG_SIM(LEVEL_INFO, "Open Ret: %s\r\n", (char*)retMsg + 6);
    token = strtok((char*)retMsg + 6, SIM_SEPARATOR_TEXT);
    if (token == NULL || token[0] == '\0') token = SIM_NO_RESPONSE_TEXT;
    if (strcmp((const char*)token, (const char*)"CONNECT OK") != 0) {
        LOG_SIM(LEVEL_ERROR, "TCP OPEN FAILED: %s\r\n", retMsg);
        return SIM_FAIL;
    }
    if (simTCPCheckStatus(SIM_CIPSTAT_CON_OK, 7000, 200) != SIM_SUCCESS) {
        return SIM_FAIL;
    }
    LOG_SIM(LEVEL_INFO, "TCP OPENED: %s\r\n", params);
    return SIM_SUCCESS;
}

u8 simTCPSend(u8* data, u16 sz) {
    static char params[8];
    char*       token;
    char*       retMsg;

    u32 ttt = HAL_GetTick();

    if (sz == 0) {
        LOG_SIM(LEVEL_ERROR, "Wrong size\r\n");
        return SIM_FAIL;
    }
    LOG_SIM(LEVEL_DEBUG, "simDownloadData() sz:%d\r\n", sz);
    memset(params, '\0', 8);
    sprintf(params, "%d", sz);
    if (simCmd(SIM_CIPSEND, params, 1, "> ") == SIM_FAIL) {
        return SIM_FAIL;
    }
    retMsg = simDownloadData(data, sz);
    // if (bkte.info.server == SERVER_NIAC) {
    //     waitIdle("", &(uInfoSim.irqFlags), 50, 1000);
    // }
    // LOG_SIM(LEVEL_INFO, "Send Ret: %s\r\n", retMsg);
    token = strtok(retMsg, SIM_SEPARATOR_TEXT);
    if (token == NULL || token[0] == '\0') token = SIM_NO_RESPONSE_TEXT;

    ttt = HAL_GetTick() - ttt;
    bkte.timers.tcp_send_time += ttt;

    if (strcmp((const char*)token, (const char*)"SEND OK") == 0) {
        LOG_SIM(LEVEL_INFO, "Send OK: time %d\r\n", ttt);
        return SIM_SUCCESS;
    } else if (strcmp((const char*)token, (const char*)"SEND FAIL") == 0) {
        LOG_SIM(LEVEL_ERROR, "simDownloadData() %s time %d\r\n", token, ttt);
        return SIM_FAIL;
    } else {
        LOG_SIM(LEVEL_ERROR, "simDownloadData() %s time %d\r\n", token, ttt);
        return SIM_TIMEOUT;
    }
}

long long simGetPhoneNum() {
    char* retMsg;
    retMsg = simTxATCmd(SIM_CMD_CNUM, SIM_SZ_CMD_CNUM, 2000);
    if (retMsg[0] != '\0') {
        printf("Num is %s\r\n", retMsg);
        return atoll(retMsg + 15);
    }
    return 0;
}

u64 simGetIMEI() {
    char* retMsg;
    retMsg = simTxATCmd(SIM_CMD_SIMEI, SIM_SZ_CMD_SIMEI, 2000);
    if (retMsg[2] != '\0') {
        printf("IMEI is %s\r\n", &retMsg[2]);
        return atoll(retMsg + 2);
    }
    return 0;
}

u8 procReturnStatus(u8 ret) {
    static u8 notSend = 0;
    if (ret != TCP_OK) {
        notSend++;
    } else {
        notSend = 0;
    }

    // if (ret == TCP_CONNECT_ER) {
    //     LOG_SIM(LEVEL_ERROR, "CONNECT ERROR - TOTAL RESET!\r\n");
    //     simReset();
    //     ret = TCP_SEND_ER_LOST_PCKG;
    //     notSend = 0;
    //     // NVIC_SystemReset();
    // } else
    if (ret != TCP_OK) {
        LOG_SIM(LEVEL_ERROR, "TCP ERROR %d!\r\n", ret);
        closeTcp();
        simReset();
        ret = TCP_SEND_ER_LOST_PCKG;
        notSend = 0;
    }

    return ret;
}

u8 openTcp(u8 server) {
    u8 ret = TCP_OK;
    if (bkte.state.isTCPOpen) {
        closeTcp();
    }

    u32 ttt = HAL_GetTick();
    if (!waitGoodCsq(5400)) {
        LOG_SIM(LEVEL_ERROR, "waitGoodCsq\r\n");
        ret = TCP_CSQ_ER;
        bkte.stat.simBadCsqCnt++;
    }
    osDelay(100);
    if (ret == TCP_OK && simTCPinit() != SIM_SUCCESS) {
        LOG_SIM(LEVEL_ERROR, "simTCPinit\r\n");
        ret = TCP_INIT_ER;
    }
    if (ret == TCP_OK && simTCPOpen(server) != SIM_SUCCESS) {
        LOG_SIM(LEVEL_ERROR, "simTCPOpen\r\n");
        ret = TCP_OPEN_ER;
    }
    if (ret == TCP_OK) {
        bkte.state.isTCPOpen = server;
        bkte.stat.simOpenCnt++;
        // LOG_SIM(LEVEL_INFO, "TCP CONNECTED\r\n");
    }

    ttt = HAL_GetTick() - ttt;
    bkte.timers.tcp_open_time += ttt;

    return procReturnStatus(ret);
}

u8 closeTcp() {
    u8 ret = TCP_OK;

    u32 ttt = HAL_GetTick();
    if (ret == TCP_OK && simTCPclose() != SIM_SUCCESS) {
        LOG_SIM(LEVEL_ERROR, "TCP NOT CLOSED!!!!!\r\n");
        ret = TCP_CLOSE_ER;
    }
    if (ret == TCP_OK) {
        bkte.state.isTCPOpen = 0;
        LOG_SIM(LEVEL_INFO, "TCP CLOSED\r\n");
    }

    ttt = HAL_GetTick() - ttt;
    bkte.timers.tcp_close_time += ttt;

    return ret;
}

u8 sendTcp(u8 server, u8* data, u16 sz) {
    u8 ret = TCP_OK;
    u8 cnt = 0;
    // static u32 count = 0;
    while (bkte.state.isTCPOpen != server) {
        if (cnt == 15) {
            ret = TCP_CONNECT_ER;
            break;
        }
        cnt++;
        ret = openTcp(server);
    }
    // if (ret == TCP_OK && !waitGoodCsq(90)) {
    //     LOG_SIM(LEVEL_ERROR, "waitGoodCsq\r\n");
    //     ret = TCP_CSQ_ER;
    // }
    if (ret == TCP_OK && simTCPSend(data, sz) != SIM_SUCCESS) {
        LOG_SIM(LEVEL_ERROR, "simTCPSend\r\n");
        ret = TCP_SEND_ER;
    }
    // count++;
    // if (!(count % 7)) {
    //     ret = TCP_SEND_ER;
    // }
    if (ret == TCP_OK) {
        bkte.stat.simSendCnt++;
    } else {
        bkte.stat.simErrCnt++;
    }
    if (bkte.state.csq < 15) {
        bkte.stat.simLowCsqCnt++;
    } else if (bkte.state.csq < 20) {
        bkte.stat.simLowCsqCnt++;
    } else {
        bkte.stat.simHighCsqCnt++;
    }
    return procReturnStatus(ret);
}