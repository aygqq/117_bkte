#ifndef INC_UTILS_SD_H_
#define INC_UTILS_SD_H_
#include "fatfs.h"
#include "main.h"
#include "utils_bkte.h"

#define SZ_SECTOR 4096
#define LEN_TIME  9

#define LEN_SYMB_ENDL 2
#define STR_ENDL      (char*)"\r\n"

//!-------ERROR MSG-------
#define SD_LEN_ER_MSG 3

#define SD_ER_MSG_AT (char*)"00 "

#define SD_ER_SAPBR  (char*)"01 "
#define SD_ER_CSQINF (char*)"02 "

#define SD_ER_CIPSHUT                   (char*)"10 "
#define SD_ER_CGATT                     (char*)"11 "
#define SD_ER_CIPSTAT_INIT              (char*)"12 "
#define SD_ER_CSTT                      (char*)"13 "
#define SD_ER_CIPSTAT_START             (char*)"14 "
#define SD_ER_CIICR                     (char*)"15 "
#define SD_ER_CIPSTAT_GPRSACT           (char*)"16 "
#define SD_ER_CIFSR                     (char*)"17 "
#define SD_ER_CIPSTAT_STATUS            (char*)"18 "
#define SD_ER_CIPSTART_OK               (char*)"19 "
#define SD_ER_CIPSTART_CON_OK           (char*)"1A "
#define SD_ER_CIPSTAT_CON_OK            (char*)"1B "
#define SD_ER_CIPSEND                   (char*)"1C "
#define SD_ER_DOWNLOAD_DATA_AND_SEND_OK (char*)"1D "

#define SD_ER_MSG_GET_SZ_NEW_BIN_MYFUN (char*)"20 "

#define SD_ER_BAD_SERVERTIME (char*)"21 "
#define SD_ER_NUM_FIRMWARE   (char*)"22 "
#define SD_ER_SZ_FIRMWARE    (char*)"23 "
#define SD_ER_PART_FIRMWARE  (char*)"24 "

//!-------MSG-------------
#define SD_MSG_START_BKTE (char*)"1 "
#define SD_LEN_START_BKTE 2

#define SD_MSG_OFF_BKTE (char*)"2 "
#define SD_LEN_OFF_BKTE 2

#define SD_MSG_KEEP_ALIVE (char*)"3 "
#define SD_LEN_KEEP_ALIVE 2

#define SD_MSG_SIM_RESET (char*)"4 "
#define SD_LEN_SIM_RESET 2

#define SD_MSG_MCU_RESET (char*)"5 "
#define SD_LEN_MCU_RESET 2

#define SD_MSG_NEW_BIN (char*)"6 "
#define SD_LEN_NEW_BIN 2

#define SD_MSG_NOT_SENT (char*)"7 "
#define SD_LEN_NOT_SENT 2

typedef struct {
    u16   szSector;
    u8*   pBufSec;
    u16   freeSz;
    char* fileName;
} SdSector;

extern SdSector sdSectorLogError;
extern SdSector sdSectorLogs;

u8   sdInit();
void sdWriteLog(char* strMsg, u16 szMsg, char* strParams, u16 szParams, SdSector* pSec);
void sdWriteSector(SdSector* pSdSector);
void cleanSector(SdSector* pSec);
void sdUpdLog(SdSector* pSec);

#endif /* INC_UTILS_FLASH_H_ */