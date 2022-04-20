/*
 * ds2482.c
 *
 *  Created on: Jun 23, 2020
 *      Author: YOBA-i516G
 */

#include "ds2482.h"

extern I2cInfo i2cInfo;

u8 ds2482Reset(void) {
    uint8_t cmd;
    cmd = DRST;
    ds2482Tx(DS2482_I2C_ADDR << 1, &cmd, 1);
    return i2cInfo.irqFlags.isIrqTx;
}

u8 ds2482SetReadPointer(Ds2482_regs regAddr) {
    u8 data[2];
    data[0] = SRP;
    data[1] = regAddr;
    return ds2482Tx(DS2482_I2C_ADDR << 1, data, 2);
}

/*bool ds2482WriteConfig(bool OneWireSpeed, bool StrongPullup, bool ActivePullup){
        u8 configByte = ((~OneWireSpeed) << 7) | ((~StrongPullup) << 6) | (1 << 5) | ((~ActivePullup) << 4) | (OneWireSpeed << 3) | (StrongPullup << 2) | (ActivePullup);

        u8 dataStream[2] = {WCFG, configByte};
        ds2482Tx(DS2482_I2C_ADDR << 1, dataStream, sizeof(dataStream));

        u8 receiveByte;
        ds2482Rx(DS2482_I2C_ADDR << 1, &receiveByte, 1);

        if(receiveByte == (0xF & configByte)){
                return true;
        }

        return false;
}*/

u8 ds2482WriteByte(uint8_t dataByte) {
    uint8_t           writeBytes[2] = {OWWB, dataByte};
    Ds2482_Status_Reg statusByte;
    ds2482Tx(DS2482_I2C_ADDR << 1, writeBytes, sizeof(writeBytes));

    for (u8 i = 0; i < 5; i++) {
        osDelay(1);
        if (ds2482Rx(DS2482_I2C_ADDR << 1, (u8*)&statusByte, 1) && statusByte.OneWB != 1) {
            return DS2482_OK;
        }
    }

    return DS2482_ERROR;
}

u8 ds2482ReadByte(u8* dataByte) {
    u8 readByteCmd = OWRB;
    // Send the one-wire read byte command
    ds2482Tx(DS2482_I2C_ADDR << 1, &readByteCmd, 1);
    // Change the read pointer to the read data register
    if (!ds2482SetReadPointer(DATA_READ_REG)) {
        return DS2482_ERROR;
    }
    // Wait for the data to be read
    return ds2482Rx(DS2482_I2C_ADDR << 1, dataByte, 1);
}

u8 ds2482OneWireReset() {
    uint8_t resetCmd = OWRS;
    return ds2482Tx(DS2482_I2C_ADDR << 1, &resetCmd, 1);
}

s8 ds2482ConvTemp(u8 LSB, u8 MSB) {
    LSB >>= 4;
    MSB <<= 4;
    return (MSB | LSB);
}

void ds2482Init() {
    u8 tmp = 0;
    i2cInfo.pHi2c1 = &hi2c1;
    for (u8 i = 0; i < BKTE_MAX_CNT_1WIRE; i++)
        resetTempLine(i);
    if (!ds2482SetReadPointer(STATUS_REG)) {
        bkte.hwStat.isDS2482 = 0;
        return;
    }
    // Wait for the data to be read
    if (ds2482Rx(DS2482_I2C_ADDR << 1, &tmp, 1)) {
        bkte.hwStat.isDS2482 = 1;
    }
    return;
}

u8 ds2482Tx(u8 addr, u8* data, u16 sz) {
    // i2cInfo.irqFlags.isIrqTx = 0;
    HAL_StatusTypeDef status;

    vPortEnterCritical();
    status = HAL_I2C_Master_Transmit(i2cInfo.pHi2c1, addr, data, sz, 1000);  // spi2
    vPortExitCritical();
    return status == HAL_OK ? 1 : 0;
}

u8 ds2482Rx(u8 addr, u8* data, u16 sz) {
    // i2cInfo.irqFlags.isIrqRx = 0;
    HAL_StatusTypeDef status;

    vPortEnterCritical();
    status = HAL_I2C_Master_Receive(i2cInfo.pHi2c1, addr, data, sz, 1000);  // spi2
    vPortExitCritical();
    return status == HAL_OK ? 1 : 0;
}
