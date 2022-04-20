#include "utils_sd.h"
extern FATFS SDFatFS;
static FIL   fil;
static char* fileNameError = (char*)"er.txt";
static char* fileNameWarning = (char*)"wr.txt";
static u8    bufSectorLogError[SZ_SECTOR];
static u8    bufSectorLogWarning[SZ_SECTOR];
SdSector     sdSectorLogError;
SdSector     sdSectorLogs;

u8 sdInit() {
    u8 ret = FAT_OK;
    if ((ret = fatInit()) != FAT_ERROR_NOT_MOUNT) {
        LOG_SD(LEVEL_INFO, "f_mount\r\n");
        sdSectorLogError.szSector = SZ_SECTOR;
        sdSectorLogError.pBufSec = bufSectorLogError;
        sdSectorLogError.fileName = fileNameError;
        cleanSector(&sdSectorLogError);

        sdSectorLogs.szSector = SZ_SECTOR;
        sdSectorLogs.pBufSec = bufSectorLogWarning;
        sdSectorLogs.fileName = fileNameWarning;
        cleanSector(&sdSectorLogs);
        bkte.hwStat.isFatMount = 1;
    } else {
        HAL_GPIO_WritePin(LED4R_GPIO_Port, LED4R_Pin, GPIO_PIN_RESET);
        LOG_SD(LEVEL_ERROR, "f_mount\r\n");
    }
    return ret;
}

void sdWriteLog(char* strMsg, u16 szMsg, char* strParams, u16 szParams, SdSector* pSec) {
    if (bkte.hwStat.isFatMount && SDFatFS.free_clst > 10) {
        u32  time = getUnixTimeStamp();
        u32  sz = szMsg + LEN_TIME + szParams + LEN_SYMB_ENDL;
        char strTimestamp[LEN_TIME];
        sprintf(strTimestamp, "%08x ", time);
        if (pSec->freeSz < sz) {
            sdUpdLog(pSec);
        }
        strcat((char*)pSec->pBufSec, strTimestamp);
        strcat((char*)pSec->pBufSec, strMsg);
        if (szParams > 0) {
            strcat((char*)pSec->pBufSec, strParams);
        }
        strcat((char*)pSec->pBufSec, STR_ENDL);
        pSec->freeSz -= sz;
    }
}

void sdUpdLog(SdSector* pSec) {
    if (pSec->freeSz != SZ_SECTOR) {
        sdWriteSector(pSec);
        cleanSector(pSec);
    }
}

void sdWriteSector(SdSector* pSdSector) {
    u32 bWrite;
    if (bkte.hwStat.isFatMount && SDFatFS.free_clst > 10) {
        if (f_open(&fil, pSdSector->fileName, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
            if (f_write(&fil, pSdSector->pBufSec, pSdSector->szSector - pSdSector->freeSz, (UINT*)&bWrite) != FR_OK) {
                LOG_SD(LEVEL_ERROR, "f_write\r\n");
            }

            if ((f_close(&fil)) == FR_OK)
                LOG_SD(LEVEL_INFO, "f_close\r\n");
            else
                LOG_SD(LEVEL_ERROR, "f_close\r\n");
        } else {
            LOG_SD(LEVEL_ERROR, "f_open\r\n");
        }
    }
}

void cleanSector(SdSector* pSec) {
    pSec->freeSz = SZ_SECTOR;
    memset(pSec->pBufSec, pSec->szSector, '\0');
}