/*
 * m_spiflash.c
 *
 *  Created on: 11 июн. 2020 г.
 *      Author: YOBA-i516G
 */

#include "spiflash.h"
SPIFlash spiFlash64;

extern osMutexId mutexSpiFlashHandle;

u16 badPgMap[2048];
u16 numPgFirstES = SPIFLASH_NUM_PG_GNSS;

void spiFlashReadMap();
void spiFlashWriteMap();
void spiFlashLoadInfo(u8 *buf);

void spiFlashInit(u8 *buf) {
    osMutexWait(mutexSpiFlashHandle, 60000);
    spiMemInfo.pHSpi = &hspi2;
    SPIFLASH_CS_UNSEL;
    osDelay(100);
    u32 id, tmp;

    id = spiFlashReadID();
    tmp = id & 0x0000FFFF;
    if (tmp) {  //! change to 2017
        bkte.hwStat.isSPIFlash = 1;
    }
    LOG_FLASH(LEVEL_MAIN, "spiFlashId, series: %d %x\r\n", (int)id, (int)tmp);

    spiFlash64.blCnt = 128;
    spiFlash64.pgSz = 256;
    spiFlash64.secSz = 0x1000;
    spiFlash64.secCnt = spiFlash64.blCnt * 16;
    spiFlash64.pgCnt = (spiFlash64.secCnt * spiFlash64.secSz) / spiFlash64.pgSz;
    spiFlash64.blSz = spiFlash64.secSz * 16;
    spiFlash64.capacityKb = (spiFlash64.secCnt * spiFlash64.secSz) / 1024;
    spiFlash64.headNumPg = 0;

    bkte.state.isSpiFlashReady = 0;

    memset((u8 *)badPgMap, 0, sizeof(badPgMap));

    osMutexRelease(mutexSpiFlashHandle);

    spiFlashLoadInfo(buf);
}

u32 spiFlashReadID(void) {
    u32 id;
    u8  tmp[4] = {SPIFLASH_CMD_JEDEC_ID, DUMMY_BYTE, DUMMY_BYTE, DUMMY_BYTE};
    SPIFLASH_CS_SEL;
    spiFlashTxRxCmd(tmp, sizeof(tmp));
    SPIFLASH_CS_UNSEL;
    id = (tmp[1] << 16) | (tmp[2] << 8) | tmp[3];
    return id;
}

u8 spiFlashTxRxCmd(u8 *data, u16 sz) {
    static u8 ret = 0;
    spiMemInfo.irqFlags.regIrq = 0;
    HAL_SPI_TransmitReceive_DMA(spiMemInfo.pHSpi, data, data, sz);  // spi2
    ret = waitRx("", &spiMemInfo.irqFlags, 50, WAIT_TIMEOUT);
    return ret;
}

u8 spiFlashTxData(u8 *data, u16 sz) {
    static u8 ret = 0;
    spiMemInfo.irqFlags.regIrq = 0;
    HAL_SPI_Transmit_DMA(spiMemInfo.pHSpi, data, sz);  // spi2
    ret = waitTx("", &spiMemInfo.irqFlags, 50, WAIT_TIMEOUT);
    return ret;
}

u8 spiFlashRxData(u8 *data, u16 sz) {
    static u8 ret = 0;
    spiMemInfo.irqFlags.regIrq = 0;
    HAL_SPI_Receive_DMA(spiMemInfo.pHSpi, data, sz);  // spi2
    ret = waitRx("", &spiMemInfo.irqFlags, 50, WAIT_TIMEOUT);
    return ret;
}

void spiFlashES(u32 numSec) {
    LOG_FLASH(LEVEL_INFO, "ErSec %d\r\n", numSec);
    u32 secAddr;

    secAddr = numSec * spiFlash64.secSz;
    u8 data[] = {SPIFLASH_SE, ((secAddr & 0xFF0000) >> 16), ((secAddr & 0xFF00) >> 8), (secAddr & 0xFF)};

    spiFlashWaitReady();
    spiFlashWrEn();

    SPIFLASH_CS_SEL;
    spiFlashTxRxCmd(data, 4);
    SPIFLASH_CS_UNSEL;

    spiFlashWaitReady();
    osDelay(10);
}

u8 spiFlashWaitReady() {
    u8 data[] = {SPIFLASH_RDSR, DUMMY_BYTE};
    osDelay(10);
    u8 i = 0, ret = 0;
    do {
        SPIFLASH_CS_SEL;
        ret = spiFlashTxRxCmd(data, sizeof(data));
        SPIFLASH_CS_UNSEL;
        osDelay(10);
        i++;
    } while (((data[1] & 0x01) == 0x01) && (i < 3));
    return ret;
}

void spiFlashWrEn() {
    u8 cmd = SPIFLASH_WREN;
    SPIFLASH_CS_SEL;
    spiFlashTxRxCmd(&cmd, 1);
    SPIFLASH_CS_UNSEL;
    osDelay(10);
}

void spiFlashWrPg(u8 *pBuf, u32 sz, u32 addr) {
    u8 data[] = {SPIFLASH_PP, ((addr & 0xFF0000) >> 16), ((addr & 0xFF00) >> 8), (addr & 0xFF)};

    spiFlashWaitReady();
    spiFlashWrEn();

    SPIFLASH_CS_SEL;
    spiFlashTxRxCmd(data, 4);
    spiFlashTxData(pBuf, sz);
    SPIFLASH_CS_UNSEL;

    spiFlashWaitReady();
    osDelay(10);
}

void spiFlashRdPg(u8 *pBuf, u32 sz, u32 addr) {
    u8 data[] = {SPIFLASH_FAST_READ, ((addr & 0xFF0000) >> 16), ((addr & 0xFF00) >> 8), (addr & 0xFF), DUMMY_BYTE};

    SPIFLASH_CS_SEL;
    spiFlashTxRxCmd(data, 5);
    spiFlashRxData(pBuf, sz);
    SPIFLASH_CS_UNSEL;

    osDelay(10);
}

u8 spiFlashIsPgBad(u32 numPage) {
    u16 curNumSec, numPgInSec;

    curNumSec = numPage / SPIFLASH_NUM_PG_IN_SEC;
    numPgInSec = numPage % SPIFLASH_NUM_PG_IN_SEC;

    return (badPgMap[curNumSec] >> numPgInSec) & 0x01;
}

void spiFlashMarkPgBad(u32 numPage) {
    u16 curNumSec, numPgInSec;
    LOG_FLASH(LEVEL_ERROR, "Mark bad page %d\r\n", numPage);

    curNumSec = numPage / SPIFLASH_NUM_PG_IN_SEC;
    numPgInSec = numPage % SPIFLASH_NUM_PG_IN_SEC;

    badPgMap[curNumSec] |= 1 << numPgInSec;
    bkte.stat.pageBadCount++;
}

u8 spiFlashWriteNextPg(u8 *pBuf, u32 sz, u32 offset) {
    u32 addr;
    u8  ret = 0;
    u8  pdBad = 1;
    u32 numPage;

    osMutexWait(mutexSpiFlashHandle, 60000);
    numPage = spiFlash64.headNumPg;

    while (pdBad) {
        LOG_FLASH(LEVEL_INFO, "WrPg %d\r\n", numPage);
        if (numPage % SPIFLASH_NUM_PG_IN_SEC == 0) {
            spiFlashES(numPage / SPIFLASH_NUM_PG_IN_SEC);
            if (numPgFirstES == SPIFLASH_NUM_PG_GNSS) numPgFirstES = numPage;
        }
        if (spiFlash64.tailNumPg > numPage &&
            (spiFlash64.tailNumPg - numPage) < SPIFLASH_NUM_PG_IN_SEC && !(numPage % SPIFLASH_NUM_PG_IN_SEC)) {
            spiFlash64.tailNumPg = (spiFlash64.tailNumPg + (SPIFLASH_NUM_PG_IN_SEC - (spiFlash64.tailNumPg % SPIFLASH_NUM_PG_IN_SEC)) + 1) % SPIFLASH_NUM_PG_GNSS;
            ret = 1;
        }

        if ((pdBad = spiFlashIsPgBad(numPage)) == 1) {
            LOG_FLASH(LEVEL_ERROR, "bad page %d\r\n", numPage);
            numPage = (numPage + 1) % SPIFLASH_NUM_PG_GNSS;
        }
    }

    if ((offset + sz) > spiFlash64.pgSz)
        sz = spiFlash64.pgSz - offset;

    addr = (numPage * spiFlash64.pgSz) + offset;
    // u32 time = HAL_GetTick();
    spiFlashWrPg(pBuf, sz, addr);
    // time = HAL_GetTick() - time;
    // printf("Wr %d\r\n", time);
    spiFlash64.headNumPg = (numPage + 1) % SPIFLASH_NUM_PG_GNSS;

    bkte.stat.pageWrCount++;
    osMutexRelease(mutexSpiFlashHandle);
    return ret;
}

u8 spiFlashReadLastPg(u8 *pBuf, u32 sz, u32 offset) {
    u32 addr;
    u8  len, pdBad = 1;
    u32 numPage;

    osMutexWait(mutexSpiFlashHandle, 60000);
    numPage = spiFlash64.tailNumPg;

    while (pdBad) {
        LOG_FLASH(LEVEL_INFO, "RdPg %d\r\n", numPage);
        if (!bkte.state.isSpiFlashReady && numPage == numPgFirstES) {
            LOG_FLASH(LEVEL_MAIN, "SPI FLASH READY\r\n\r\n");
            bkte.state.isSpiFlashReady = 1;
        }

        if ((pdBad = spiFlashIsPgBad(numPage)) == 1) {
            LOG_FLASH(LEVEL_ERROR, "bad page %d\r\n", numPage);
            numPage = (numPage + 1) % SPIFLASH_NUM_PG_GNSS;
        }
    }

    if ((offset + sz) > spiFlash64.pgSz)
        sz = spiFlash64.pgSz - offset;

    addr = (numPage * spiFlash64.pgSz) + offset;
    // u32 time = HAL_GetTick();
    spiFlashRdPg(pBuf, sz, addr);
    // time = HAL_GetTick() - time;
    // printf("Rd %d\r\n", time);

    len = isDataFromFlashOk(pBuf, 256);
    if (len == 0) {
        LOG_FLASH(LEVEL_ERROR, "bad crc isDataFromFlashOk()\r\n");
        if (bkte.state.isSpiFlashReady) {
            spiFlashMarkPgBad(numPage);
        }
    }
    spiFlash64.tailNumPg = (numPage + 1) % SPIFLASH_NUM_PG_GNSS;

    bkte.stat.pageRdCount++;
    osMutexRelease(mutexSpiFlashHandle);

    return len;
}

u16 getDelayPages() {
    u16 numPage;
    u16 delayPages, delayGoodPages;

    delayPages = spiFlash64.headNumPg >= spiFlash64.tailNumPg ? spiFlash64.headNumPg - spiFlash64.tailNumPg : spiFlash64.headNumPg + (SPIFLASH_NUM_PG_GNSS - spiFlash64.tailNumPg);
    delayGoodPages = delayPages;

    numPage = spiFlash64.tailNumPg;
    for (u16 i = 0; i < delayPages || i < SPIFLASH_NUM_PG_GNSS; ++i) {
        if (spiFlashIsPgBad(numPage)) {
            delayGoodPages--;
        }
        numPage = (numPage + 1) % SPIFLASH_NUM_PG_GNSS;
    }

    return delayGoodPages;
}

void spiFlashSaveInfo() {
    u32 addr;
    u8  buf[33];

    spiFlashWriteMap();

    osMutexWait(mutexSpiFlashHandle, 60000);

    memset(buf, 0, 33);
    memcpy(&buf[0], &spiFlash64.headNumPg, 4);
    memcpy(&buf[4], &spiFlash64.tailNumPg, 4);

    memcpy(&buf[8], &bkte.measure.calibr[0].coef, 4);
    memcpy(&buf[12], &bkte.measure.calibr[0].zero_lvl, 2);
    memcpy(&buf[14], &bkte.measure.calibr[1].coef, 4);
    memcpy(&buf[18], &bkte.measure.calibr[1].zero_lvl, 2);
    memcpy(&buf[20], &bkte.measure.calibr[2].coef, 4);
    memcpy(&buf[24], &bkte.measure.calibr[2].zero_lvl, 2);

    buf[31] = 0x01;
    buf[32] = 0x01;

    spiFlashES(BKTE_SAVE_NUM_PAGE / SPIFLASH_NUM_PG_IN_SEC);

    LOG_FLASH(LEVEL_INFO, "Save pages: head %d, tail %d\r\n", spiFlash64.headNumPg, spiFlash64.tailNumPg);

    addr = (BKTE_SAVE_NUM_PAGE * spiFlash64.pgSz);
    spiFlashWrPg(buf, 33, addr);

    osMutexRelease(mutexSpiFlashHandle);
}

void spiFlashLoadInfo(u8 *buf) {
    u8  isMapInFlash;
    u8  isCalibrInFlash;
    u32 tmp;
    u32 addr;

    addr = (BKTE_SAVE_NUM_PAGE * spiFlash64.pgSz);
    spiFlashRdPg(buf, 256, addr);

    tmp = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
    spiFlash64.headNumPg = tmp;
    LOG_FLASH(LEVEL_INFO, "Read headNumPg: %d\r\n", spiFlash64.headNumPg);

    tmp = buf[4] | buf[5] << 8 | buf[6] << 16 | buf[7] << 24;
    spiFlash64.tailNumPg = tmp;
    LOG_FLASH(LEVEL_INFO, "Read tailNumPg: %d\r\n", spiFlash64.tailNumPg);

    isMapInFlash = buf[31];
    isCalibrInFlash = buf[32];

    if (spiFlash64.headNumPg > SPIFLASH_NUM_PG_GNSS || spiFlash64.tailNumPg > SPIFLASH_NUM_PG_GNSS) {
        spiFlash64.headNumPg = 0;
        spiFlash64.tailNumPg = spiFlash64.headNumPg;
        osMutexWait(mutexSpiFlashHandle, 60000);
        spiFlashES(spiFlash64.headNumPg / SPIFLASH_NUM_PG_IN_SEC);
        osMutexRelease(mutexSpiFlashHandle);
    }

    if (isCalibrInFlash == 0x01) {
        for (u8 i = 0; i < ADC_CHAN_CNT - 1; i++) {
            memcpy(&bkte.measure.calibr[i].coef, &buf[8 + 6 * i], 4);
            memcpy(&bkte.measure.calibr[i].zero_lvl, &buf[8 + 6 * i + 4], 2);
            if (bkte.measure.calibr[i].coef > 10.0 || bkte.measure.calibr[i].zero_lvl > 4096) {
                bkte.measure.calibr[i].coef = ADC_CALIBR_DEFAULT_COEF;
                bkte.measure.calibr[i].zero_lvl = ADC_CALIBR_DEFAULT_ZERO;
            }
            LOG_FLASH(LEVEL_INFO, "Read calibr %d: %f, %d\r\n", i, bkte.measure.calibr[i].coef, bkte.measure.calibr[i].zero_lvl);
        }
    } else {
        for (u8 i = 0; i < ADC_CHAN_CNT - 1; i++) {
            bkte.measure.calibr[i].coef = ADC_CALIBR_DEFAULT_COEF;
            bkte.measure.calibr[i].zero_lvl = ADC_CALIBR_DEFAULT_ZERO;
            LOG_FLASH(LEVEL_INFO, "Default calibr %d: %f, %d\r\n", i, bkte.measure.calibr[i].coef, bkte.measure.calibr[i].zero_lvl);
        }
    }

    memset((u8 *)badPgMap, 0, sizeof(badPgMap));
    if (isMapInFlash == 0x01) {
        spiFlashReadMap();
    } else {
        osMutexWait(mutexSpiFlashHandle, 60000);
        spiFlashES(BKTE_BAD_PG_MAP_NUM_PAGE / SPIFLASH_NUM_PG_IN_SEC);
        osMutexRelease(mutexSpiFlashHandle);
    }
}

void spiFlashWriteMap() {
    u32 addr;
    u8 *pMap = (u8 *)badPgMap;
    u16 curNumPg = BKTE_BAD_PG_MAP_NUM_PAGE;

    osMutexWait(mutexSpiFlashHandle, 60000);

    spiFlashES(BKTE_BAD_PG_MAP_NUM_PAGE / SPIFLASH_NUM_PG_IN_SEC);

    for (u16 i = 0; i < SPIFLASH_NUM_PG_IN_SEC; ++i) {
        addr = (curNumPg * spiFlash64.pgSz);
        spiFlashWrPg(pMap, 256, addr);

        curNumPg += 1;
        pMap += 256;
    }

    osMutexRelease(mutexSpiFlashHandle);
}

void spiFlashReadMap() {
    u32 addr;
    u8 *pMap = (u8 *)badPgMap;
    u16 curNumPg = BKTE_BAD_PG_MAP_NUM_PAGE;

    osMutexWait(mutexSpiFlashHandle, 60000);

    for (u16 i = 0; i < SPIFLASH_NUM_PG_IN_SEC; ++i) {
        addr = (curNumPg * spiFlash64.pgSz);
        spiFlashRdPg(pMap, 256, addr);

        curNumPg += 1;
        pMap += 256;
    }

    osMutexRelease(mutexSpiFlashHandle);
}
