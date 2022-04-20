/*
 * circularBuffer.c
 *
 *  Created on: 10 ���. 2019 �.
 *      Author: annsi
 */

#include "circularBuffer.h"

void cBufInit(CircularBuffer* cbuf, u8* buf, u16 szBuf, CircTypeBuf type) {
    cbuf->buf = buf;
    cbuf->max = szBuf;
    cbuf->type = type;
    cBufReset(cbuf);
    LOG_CBUF(LEVEL_MAIN, "cBufInit()\r\n");
}

void cBufReset(CircularBuffer* cbuf) {
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->writeAvailable = cbuf->max;
    cbuf->readAvailable = 0;
    cbuf->numPckgInBuf = 0;
    cbuf->curLenMsg = 0;
    memset(cbuf->buf, '\0', cbuf->max);
}

void cBufWriteToBuf(CBufHandle cbuf, u8* data, u8 sz) {
    if (cbuf->writeAvailable > sz) {
        cbuf->remainWrite = cbuf->max - cbuf->head;
        if (sz > cbuf->remainWrite) {
            memcpy(cbuf->buf + cbuf->head, data, cbuf->remainWrite);
            memcpy(cbuf->buf, data + cbuf->remainWrite, sz - cbuf->remainWrite);
        } else {
            memcpy(cbuf->buf + cbuf->head, data, sz);
        }

        cbuf->numPckgInBuf++;
        cbuf->head = (cbuf->head + sz) % cbuf->max;
        cbuf->writeAvailable -= sz;
        cbuf->readAvailable += sz;
    } else {
        LOG_CBUF(LEVEL_INFO, "FULL CIRC BUFFER\r\n");
    }
}

void cBufSafeWrite(CBufHandle cbuf, u8* data, u8 sz, osMutexId mutex, TickType_t ticks) {
    osMutexWait(mutex, ticks);
    cBufWriteToBuf(cbuf, data, sz);
    osMutexRelease(mutex);
}

u16 cBufRead(CBufHandle cbuf, u8* dist, u8 sz) {
    u16 lenMsg;
    switch (cbuf->type) {
        case CIRC_TYPE_SIM_UART:
            if ((lenMsg = getLenMsgSimUart(cbuf))) {
                copyGetDatafromBuf(cbuf, dist, lenMsg, CIRC_TYPE_SIM_UART);
            }
            break;
        case CIRC_TYPE_ENERGY_UART:
            if ((lenMsg = getLenMsgEnergyUart(cbuf)) &&
                lenMsg == BKTE_SZ_UART_MSG) {
                copyGetDatafromBuf(cbuf, dist, lenMsg, CIRC_TYPE_ENERGY_UART);
            } else if (lenMsg) {
                cBufReset(cbuf);
                LOG_CBUF(LEVEL_ERROR, "CIRC_TYPE_ENERGY_UART lenMsg: %d\r\n", lenMsg);
            }
            break;
        case CIRC_TYPE_WIRELESS:
            if ((lenMsg = getLenMsgWirelessSens(cbuf))) {
                copyGetDatafromBuf(cbuf, dist, lenMsg, CIRC_TYPE_WIRELESS);
            } else {
                // LOG_CBUF(LEVEL_ERROR, "CIRC_TYPE_WIRELESS\r\n");
            }
            break;
            break;
        case CIRC_TYPE_PCKG_ENERGY:
        case CIRC_TYPE_PCKG_RSSI:
        case CIRC_TYPE_PCKG_TEMP:
        case CIRC_TYPE_PCKG_VOLTAMPER:
        case CIRC_TYPE_PCKG_ALL:
            lenMsg = sz;
            copyGetDatafromBuf(cbuf, dist, lenMsg, CIRC_TYPE_PCKG_ENERGY);
            break;
    }
    return lenMsg;
}

void copyGetDatafromBuf(CBufHandle cbuf, u8* dist, u16 sz, CircTypeBuf type) {
    cbuf->remainRead = cbuf->max - cbuf->tail;
    if (sz > cbuf->remainRead) {
        memcpy(dist, cbuf->buf + cbuf->tail, cbuf->remainRead);
        memcpy(dist + cbuf->remainRead, cbuf->buf, sz - cbuf->remainRead);
        memset(cbuf->buf + cbuf->tail, '\0', cbuf->remainRead);
        memset(cbuf->buf, '\0', sz - cbuf->remainRead);

    } else {
        memcpy(dist, cbuf->buf + cbuf->tail, sz);
        memset(cbuf->buf + cbuf->tail, '\0', sz);
    }

    if (type == CIRC_TYPE_SIM_UART) {
        cbuf->tail = (cbuf->tail + sz + CIRC_LEN_ENDS) % cbuf->max;
        cbuf->writeAvailable += CIRC_LEN_ENDS;
        cbuf->readAvailable -= CIRC_LEN_ENDS;
    } else if (type == CIRC_TYPE_ENERGY_UART || type == CIRC_TYPE_PCKG_ENERGY ||
               type == CIRC_TYPE_PCKG_TEMP || type == CIRC_TYPE_PCKG_ALL) {
        cbuf->tail = (cbuf->tail + sz) % cbuf->max;
    } else if (type == CIRC_TYPE_WIRELESS) {
        // cbuf->tail = cbuf->head;
        cbuf->tail = (cbuf->tail + sz) % cbuf->max;
    }
    cbuf->writeAvailable += sz;
    cbuf->readAvailable -= sz;
    --(cbuf->numPckgInBuf);
}

u8 getLenMsgSimUart(CBufHandle cbuf) {
    u16 lenMsg = 0;
    u16 tail = cbuf->tail;
    if (cbuf->numPckgInBuf > 0) {
        while (tail != cbuf->head) {
            if (cbuf->buf[tail] == CIRC_END1_MSG &&
                cbuf->buf[(tail + 1) % cbuf->max] == CIRC_END2_MSG) {
                cbuf->buf[tail] = '\0';
                cbuf->buf[(tail + 1) % cbuf->max] = '\0';
                break;
            }
            tail = (tail + 1) % cbuf->max;
            lenMsg++;
        }
    }
    return lenMsg;
}

u8 getLenMsgEnergyUart(CBufHandle cbuf) {
    u16 lenMsg = 0;
    u16 tail = cbuf->tail;
    if (cbuf->numPckgInBuf > 1) {
        while (tail != cbuf->head) {
            if (cbuf->buf[tail] == CIRC_HEADER1 &&
                cbuf->buf[(tail + 1) % cbuf->max] == CIRC_HEADER2) {
                //				cbuf->buf[tail] = '\0';
                //				cbuf->buf[(tail + 1) % cbuf->max] =
                //'\0'; 				tail = (tail + 2) % cbuf->max; 				lenMsg = 2;
                cbuf->tail = tail;
                do {
                    lenMsg++;
                    if (tail == cbuf->head) {
                        lenMsg = 0;
                        cBufReset(cbuf);
                        LOG_CBUF(LEVEL_ERROR, "NOHEADER\r\n");
                    }
                    tail = (tail + 1) % cbuf->max;

                } while (!(cbuf->buf[tail] == CIRC_HEADER1 &&
                           cbuf->buf[(tail + 1) % cbuf->max] == CIRC_HEADER2) &&
                         lenMsg);
                break;
            }
            cbuf->writeAvailable++;
            cbuf->readAvailable--;
            cbuf->buf[tail] = '\0';
            tail = (tail + 1) % cbuf->max;
        }
        // printf("WRITEAV: %d   READAVAIL: %d\r\n", cbuf->writeAvailable,
        // cbuf->readAvailable);
    }

    return lenMsg;
}

u16 getLenMsgWirelessSens(CircularBuffer* cbuf) {
    u16 tail = cbuf->tail;
    u16 lenData, lenFull;

    lenData = cbuf->buf[(tail + 3) % cbuf->max] << 8 | cbuf->buf[(tail + 2) % cbuf->max];
    lenFull = lenData + 6;

    if (tail == 195) {
        tail = cbuf->tail;
    }

    if (lenData == 0 || (cbuf->buf[(tail + lenFull - 1) % cbuf->max] == 0 &&
                         cbuf->buf[(tail + lenFull - 2) % cbuf->max] == 0)) {
        return 0;
    }
    return lenFull;
}
