/*
 * sim800.h
 *
 *  Created on: Feb 20, 2020
 *      Author: bls
 */

#ifndef INC_SIM800_H_
#define INC_SIM800_H_

/* AT Commands for HTTP Application */
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "utils_bkte.h"
#include "utils_sd.h"

// #define RESPONSE_BUF_SZ				990
#define COMMAND_BUF_SZ       100
#define COMMAND_BUF_SZ       100
#define SIM_LEN_HTTP_CODE    3
#define SIM_TIMEOUT_RESPONSE 15000

//#define SIM_SZ_MAX_TX_DATA

#define SIM_CMD_ECHO_OFF        (char*)"ATE0"
#define SIM_CMD_TYPE_ANSWER_NUM (char*)"ATV0"
#define SIM_CMD_ATE0            (char*)"ATE0\r\n"
#define SIM_SZ_CMD_ATE0         6
#define SIM_CMD_AT              (char*)"AT\r\n"
#define SIM_SZ_CMD_AT           4
#define SIM_CMD_CNUM            (char*)"AT+CNUM\r\n"
#define SIM_SZ_CMD_CNUM         9
#define SIM_CMD_SIMEI           (char*)"AT+SIMEI?\r\n"
#define SIM_SZ_CMD_SIMEI        11

#define SIM_SUCCESS               0
#define SIM_FAIL                  1
#define SIM_RESTART               2
#define SIM_HTTP_BAD_CODE_REQUEST 3
#define SIM_TIMEOUT               4

#define SIM_SZ_NO_GPSINFO 35
#define SIM_SZ_GPSINFO    100

#define SIM_SAPBR     (char*)"SAPBR"
#define SIM_CIPGSMLOC (char*)"CIPGSMLOC"
#define SIM_CCLK      (char*)"CCLK?"
#define SIM_CLTS      (char*)"CLTS"

#define SIM_CIPGSMLOC (char*)"CIPGSMLOC"
#define SIM_CCLK      (char*)"CCLK?"
#define SIM_CLTS      (char*)"CLTS"

#define SIM_CGPS     (char*)"CGPS"
#define SIM_CGPSINFO (char*)"CGPSINFO"

#define SIM_CSQ (char*)"CSQ"

#define SIM_CGNSINF (char*)"CGNSINF"

#define SIM_CIPSHUT   (char*)"CIPSHUT"
#define SIM_CIPMUX    (char*)"CIPMUX"
#define SIM_CIPMODE   (char*)"CIPMODE"
#define SIM_CGATT     (char*)"CGATT"
#define SIM_CSTT      (char*)"CSTT"
#define SIM_CIICR     (char*)"CIICR"
#define SIM_CIFSR     (char*)"CIFSR"
#define SIM_CIPSTART  (char*)"CIPSTART"
#define SIM_CIPSTATUS (char*)"CIPSTATUS"
#define SIM_CIPSEND   (char*)"CIPSEND"

#define SIM_CIPSTAT_INIT    (char*)"STATE: IP INITIAL"
#define SIM_CIPSTAT_START   (char*)"STATE: IP START"
#define SIM_CIPSTAT_GPRSACT (char*)"STATE: IP GPRSACT"
#define SIM_CIPSTAT_STATUS  (char*)"STATE: IP STATUS"
#define SIM_CIPSTAT_CON_OK  (char*)"STATE: CONNECT OK"

/* Acknowledges execution of a Command
 * */
#define SIM_OK         (char*)"0"
#define SIM_OK_CHAR    (u8)'0'
#define SIM_OK_TEXT    (char*)"OK"
#define SIM_OK_CIPSHUT (char*)"SHUT OK"

/* A connection has been established; the DCE is moving from
 * Command state to online data state
 * */
#define SIM_CONNECT      (char*)"1"
#define SIM_CONNECT_TEXT (char*)"CONNECT"

#define SIM_ERROR      (char*)"4"
#define SIM_ERROR_TEXT (char*)"ERROR"

#define SIM_NO_RESPONSE_TEXT (char*)("NO RESPONSE")
#define SIM_DOWNLOAD         (char*)"DOWNLOAD"

#define SIM_SEPARATOR_TEXT (char*)"\r\n"

#define HTTP_RESPONSE_MAXIMUM_LENGTH 64

#define TCP_OK                0
#define TCP_INIT_ER           1
#define TCP_OPEN_ER           2
#define TCP_SEND_ER           3
#define TCP_SEND_ER_LOST_PCKG 4
#define TCP_CSQ_ER            5
#define TCP_TIMEOUT_ER        6
#define TCP_CONNECT_ER        7
#define TCP_CLOSE_ER          8

void  simInit();
char* simGetStatusAnsw();
void  copyStr(char* dist, char* source, u16 distSz);

u8   simGetDateTime(char* timestamp);
u8   simCheckCSQ();
void simReset();

void simOn();

void simHardwareReset();

u8        simTCPTest();
u8        simTCPOpen(u8 server);
u8        simTCPSend(u8* data, u16 sz);
u8        simTCPCheckStatus(const char* stat, u16 timeout, u16 delay);
u8        simCmd(char* cmdCode, char* params, u8 retriesCnt, char* SUCCESS_RET);
char*     simTxATCmd(char* command, u16 sz, u32 timeout);
long long simGetPhoneNum();
u64       simGetIMEI();

u8 sendTcp(u8 server, u8* data, u16 sz);
u8 openTcp(u8 server);
u8 closeTcp();

void gnssInit();

#define SIM_GPS_INIT() simCmd(SIM_SAPBR, "1,1", 1, SIM_OK_TEXT)

#endif /* INC_SIM800_H_ */
