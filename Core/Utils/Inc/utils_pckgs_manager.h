#ifndef INC_UTILS_PCKG_MANAGER_H_
#define INC_UTILS_PCKG_MANAGER_H_
#include "main.h"

#define CNT_WEBPCKGS 5

#define SZ_REQUEST_GET_SERVER_TIME   2
#define SZ_REQUEST_GET_NUM_FIRMWARE  2
#define SZ_REQUEST_GET_SZ_FIRMWARE   2
#define SZ_REQUEST_GET_PART_FIRMWARE 10

typedef struct {
    u16 shift;
    u8  buf[SZ_WEB_PCKG];
    u8  isFull;
    u8  isRequest;
} WebPckg;

void addInfo(WebPckg* pPckg, u8* src, u16 sz);
void clearWebPckg(WebPckg* pckg);
void initWebPckg(WebPckg* pckg, u16 len, u8 isReq, u8* idMCU, u8 server);
void addInfoToWebPckg(WebPckg* pckg, u8* src, u16 sz, u8 cnt, u8 cmdData);
void showWebPckg(WebPckg* pckg);
void closeWebPckg(WebPckg* pckg, u8 server);
void clearAllWebPckgs();

u8          isNotFullPckg();
WebPckg*    getFreePckg();
WebPckg*    getFreePckgReq();
u8          getCntFreePckg();
void        freeWebPckg(WebPckg* pckg);
void        makeAuthorizePckg(WebPckg* pPckg, u8 server);
void        makeEndSessionPckg(WebPckg* pPckg, u8 server);
WebPckg*    createWebPckgReq(u8 CMD_REQ, u8* data, u8 sz, u8 szReq, u8* idMCU);
ErrorStatus sendWebPckgData(u8 CMD_DATA, u8* data, u8 sz, u8 szReq, u8* idMCU);
ErrorStatus generateWebPckgReq(u8 CMD_REQ, u8* data, u8 sz, u8 szReq, u8* answ, u16 answSz, u8* idMCU);
#endif