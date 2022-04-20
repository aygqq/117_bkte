#include "../Tasks/Inc/task_get_temp.h"

#include "../Tasks/Inc/task_iwdg.h"

extern u16 iwdgTaskReg;

extern osThreadId     getTempHandle;
extern osMutexId      mutexWriteToEnergyBufHandle;
extern CircularBuffer circBufAllPckgs;
static PckgTemp       pckgTemp;
static s8             temps[BKTE_MAX_CNT_1WIRE];
u32                   testTimestamp, tmpTime;

u8 readTemp();
u8 isTemperatureFresh(PckgTemp *pckg);

void taskGetTemp(void const *argument) {
    u8 numIteration = 0;
    ds2482Init();
    vTaskSuspend(getTempHandle);
    HAL_GPIO_WritePin(LED3G_GPIO_Port, LED3G_Pin, GPIO_PIN_RESET);

    readTemp();
    osDelay(1000);

    for (;;) {
        iwdgTaskReg |= IWDG_TASK_REG_TEMP;
        bkte.stat.temp++;
        testTimestamp = HAL_GetTick();

        if (readTemp() == 0) {
            osDelay(1000);
            LOG_TEMP(LEVEL_ERROR, "temperature read\r\n");
            continue;
        }
        tmpTime = HAL_GetTick() - testTimestamp;
        if (tmpTime > 6000) {
            LOG_TEMP(LEVEL_ERROR, "too long time\r\n");
        }
        osDelay(52);
        fillPckgTemp(&pckgTemp, temps);
        if (isTemperatureFresh(&pckgTemp) || !numIteration) {
            saveData((u8 *)&pckgTemp, SZ_CMD_TEMP, CMD_DATA_TEMP, &circBufAllPckgs);
            LOG_TEMP(LEVEL_INFO, "TEMP |%d %d %d %d| %d\r\n", temps[0], temps[1], temps[2], temps[3], tmpTime);
        }
        numIteration = (numIteration + 1) % BKTE_MEASURE_FULL_LOOP;

        osDelay(800);
        // tmpTime = HAL_GetTick() - testTimestamp;
        // LOG_TEMP(LEVEL_INFO, "TEMP |%d %d %d %d| %d\r\n", temps[0], temps[1], temps[2], temps[3], tmpTime);
    }
}

u8 isTemperatureFresh(PckgTemp *pckg) {
    if (pckg->temp[3] != bkte.lastData.temp[3]) {
        bkte.lastData.temp[3] = pckg->temp[3];
        return 1;
    }
    return 0;
}

u8 readTemp() {
    u8 tempBytes[2];
    s8 tmpTemp;

    for (u8 num1Wire = 3; num1Wire < BKTE_MAX_CNT_1WIRE; num1Wire++) {
        setTempLine(num1Wire);
        // osDelay(500);
        if (ds2482OneWireReset() != DS2482_OK) {
            LOG_TEMP(LEVEL_ERROR, "ds2482OneWireReset()\r\n");
            saveErrorToTel();
            return 0;
        }
        osDelay(10);
        if (ds2482WriteByte(0xCC) != DS2482_OK) {  // Skip ROM command
            LOG_TEMP(LEVEL_ERROR, "ds2482WriteByte(0xCC)\r\n");
            saveErrorToTel();
            return 0;
        }
        osDelay(10);
        // Test reading the temperature from the DS18S20
        //	  ds2482WriteConfig(false, true, true);  // Enable strong pullup
        // for temp conversion
        if (ds2482WriteByte(0x44) != DS2482_OK) {  // Get the temp
            LOG_TEMP(LEVEL_ERROR, "ds2482WriteByte(0x44)\r\n");
            saveErrorToTel();
            return 0;
        }
        osDelay(100);  // Wait for the temp conversion
        if (ds2482OneWireReset() != DS2482_OK) {
            LOG_TEMP(LEVEL_ERROR, "ds2482OneWireReset()\r\n");
            saveErrorToTel();
            return 0;
        }
        osDelay(10);
        if (ds2482WriteByte(0xCC) != DS2482_OK) {  // Skip ROM command
            LOG_TEMP(LEVEL_ERROR, "ds2482WriteByte(0xCC)\r\n");
            saveErrorToTel();
            return 0;
        }
        osDelay(10);
        if (ds2482WriteByte(0xbe) != DS2482_OK) {  // Read scratchpad command
            LOG_TEMP(LEVEL_ERROR, "ds2482WriteByte(0xbe)\r\n");
            saveErrorToTel();
            return 0;
        }
        // osDelay(200);
        memset(tempBytes, '\0', 2);

        if (ds2482ReadByte(&tempBytes[0]) != DS2482_OK) {  // LSB temp byte
            LOG_TEMP(LEVEL_ERROR, "ds2482ReadByte(&tempBytes[0])\r\n");
            saveErrorToTel();
            return 0;
        }
        if (ds2482ReadByte(&tempBytes[1]) != DS2482_OK) {  // MSB temp byte
            LOG_TEMP(LEVEL_ERROR, "ds2482ReadByte(&tempBytes[1])\r\n");
            saveErrorToTel();
            return 0;
        }

        if ((tmpTemp = ds2482ConvTemp(tempBytes[0], tempBytes[1])) > BKTE_MAX_TEMP ||
            tmpTemp < BKTE_MIN_TEMP) {
            tmpTemp = BKTE_NO_TEMP;
        }
        temps[num1Wire] = tmpTemp;
        // LOG_TEMP(LEVEL_INFO, "TEMP %d:%d\r\n", num1Wire, tmpTemp);
        // HAL_GPIO_TogglePin(LED3G_GPIO_Port, LED3G_Pin);
        resetTempLine(num1Wire);

        return 1;
    }
    return 1;
}

void saveErrorToTel() {
    /*fillTelemetry(&curPckgTemp, TEL_NO_DS2482, 0);
    cBufWriteToBuf(&circBufPckgEnergy, (u8*)&curPckgTemp, SZ_PCKGENERGY);
    osDelay(2000);*/
}