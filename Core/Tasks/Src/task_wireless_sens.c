#include "../Tasks/Inc/task_wireless_sens.h"

#include "../Tasks/Inc/task_iwdg.h"

extern u16 iwdgTaskReg;

extern osThreadId wirelessSensHandle;
char              bufSens[USART_SZ_BUF_RX_USART2];
CircularBuffer    circBufWirelessSens = {.buf = NULL, .max = 0};

static PckgTemp       pckgTempWireless;
static PckgTelemetry  pckgTelWireless;
extern CircularBuffer circBufAllPckgs;
static s8             tempsWireless[BKTE_MAX_CNT_1WIRE];

void taskWirelessSens(void const* argument) {
    u16 lenData, lenAllPckg;
    cBufInit(&circBufWirelessSens, uInfoWirelessSens.pRxBuf, uInfoWirelessSens.szRxBuf, CIRC_TYPE_WIRELESS);
    vTaskSuspend(wirelessSensHandle);
    USART_RE2_READ_EN();
    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_WIRELESS;
        bkte.stat.wireless++;
        uInfoWirelessSens.irqFlags.isIrqIdle = 0;
        memset(bufSens, '\0', USART_SZ_BUF_RX_USART2);
        // read data from cBuf
        while (!waitIdle("", &uInfoWirelessSens.irqFlags, 1, 30000)) {
            LOG(LEVEL_INFO, "ERROR: NO WIRELESS SENS\r\n");
            bkte.stat.wireless++;
            iwdgTaskReg |= IWDG_TASK_REG_WIRELESS;
            continue;
        }
        uInfoWirelessSens.irqFlags.isIrqIdle = 0;

        while (cBufRead(&circBufWirelessSens, (u8*)bufSens, 0)) {
            lenData = bufSens[3] << 8 | bufSens[2];
            lenAllPckg = lenData + SZ_HEADER_CRC16;
            if ((bufSens[lenAllPckg - 1] << 8 | bufSens[lenAllPckg - 2]) == crc16WirelesSens(bufSens + 4, lenData)) {
                parseWirelessSens(bufSens + 4, lenData);
                osDelay(10);
                // LOG(LEVEL_INFO, "OK: read wireless sensPckg %d\r\n", lenAllPckg);
                memset(bufSens, '\0', USART_SZ_BUF_RX_USART2);
            } else {
                LOG(LEVEL_INFO, "Error: crc16 wireless sens %d\r\n", lenAllPckg);
            }
        }
    }
}

void parseWirelessSens(u8* data, u16 len) {
    u16 cntBlocks = len / SZ_BLOCK_SENS;
    u32 value;
    u8  numSens;
    for (u16 i = 0; i < cntBlocks; i++) {
        if (data[i * SZ_BLOCK_SENS] == BLOCK_WIRELESS_TEMP) {
            value = (data[i * SZ_BLOCK_SENS + 1 + 4] << 24 | data[i * SZ_BLOCK_SENS + 1 + 3] << 16 |
                     data[i * SZ_BLOCK_SENS + 1 + 2] << 8 | data[i * SZ_BLOCK_SENS + 1 + 1]);
            numSens = data[i * SZ_BLOCK_SENS + 1];

            tempsWireless[0] = BKTE_WIRELESS_TEMP;
            tempsWireless[1] = 0;
            tempsWireless[2] = numSens;
            tempsWireless[3] = (value / 100) & 0xFF;
            // LOG(LEVEL_INFO, "SENS %d: %d\r\n", numSens, value);

            fillPckgTemp(&pckgTempWireless, tempsWireless);
            saveData((u8*)&pckgTempWireless, SZ_CMD_TEMP, CMD_DATA_TEMP, &circBufAllPckgs);
        } else if (data[i * SZ_BLOCK_SENS] == BLOCK_WIRELESS_RSSI) {
            value = (data[i * SZ_BLOCK_SENS + 1 + 4] << 24 | data[i * SZ_BLOCK_SENS + 1 + 3] << 16 |
                     data[i * SZ_BLOCK_SENS + 1 + 2] << 8 | data[i * SZ_BLOCK_SENS + 1 + 1]);
            numSens = data[i * SZ_BLOCK_SENS + 1];

            // LOG(LEVEL_INFO, "RSSI %d: %d\r\n", numSens, value);

            pckgTelWireless.group = TEL_GR_HARDWARE_STATUS;
            pckgTelWireless.code = TEL_CD_HW_WIRELESS_SENS_RSSI;
            pckgTelWireless.data = numSens << 8 | (value & 0xFF);
            saveTelemetry(&pckgTelWireless, &circBufAllPckgs);
        }
    }
}