#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include "cmsis_os.h"
#include "main.h"

#define SZ_CIRCULAR_BUF  1000
#define CIRC_BUF_END_MSG (u8)'\0'
#define CIRC_END1_MSG    (u8)'\r'
#define CIRC_END2_MSG    (u8)'\n'
#define CIRC_LEN_ENDS    (u8)2

#define CIRC_HEADER1 0xF2
#define CIRC_HEADER2 0x01

typedef enum {
    CIRC_TYPE_SIM_UART = 0,
    CIRC_TYPE_ENERGY_UART,
    CIRC_TYPE_PCKG_ENERGY,
    CIRC_TYPE_PCKG_VOLTAMPER,
    CIRC_TYPE_PCKG_TEMP,
    CIRC_TYPE_PCKG_ALL,
    CIRC_TYPE_PCKG_RSSI,
    CIRC_TYPE_WIRELESS
} CircTypeBuf;

typedef struct {
    u8*         buf;
    u16         head;
    u16         tail;
    u16         writeAvailable;
    u16         readAvailable;
    u16         remainRead;
    u16         remainWrite;
    u16         max;  // of the buffer
    u8          numPckgInBuf;
    u16         curLenMsg;
    CircTypeBuf type;
} CircularBuffer;

typedef CircularBuffer* CBufHandle;

void cBufInit(CBufHandle cbuf, u8* buf, u16 szBuf, CircTypeBuf type);

/// Free a circular buffer structure
/// Requires: cbuf is valid and created by circular_buf_init
/// Does not free data buffer; owner is responsible for that
// void cBufFree(CBufHandle cbuf);

/// Reset the circular buffer to empty, head == tail. Data not cleared
/// Requires: cbuf is valid and created by circular_buf_init
void cBufReset(CBufHandle cbuf);

/// Put version 1 continues to add data if the buffer is full
/// Old data is overwritten
/// Requires: cbuf is valid and created by circular_buf_init
// void cBufPut(CBufHandle cbuf, u8 data);

/// Retrieve a value from the buffer
/// Requires: cbuf is valid and created by circular_buf_init
/// Returns 0 on success, -1 if the buffer is empty
// u8 cBufGet(CBufHandle cbuf, u8* data);

/// CHecks if the buffer is empty
/// Requires: cbuf is valid and created by circular_buf_init
/// Returns true if the buffer is empty
// u8 cBufIsEmpty(CBufHandle cbuf);

/// Checks if the buffer is full
/// Requires: cbuf is valid and created by circular_buf_init
/// Returns true if the buffer is full
// u8 cBufIsFull(CBufHandle cbuf);

/// Check the capacity of the buffer
/// Requires: cbuf is valid and created by circular_buf_init
/// Returns the maximum capacity of the buffer
// u8 cBufGetCapacity(CBufHandle cbuf);

/// Check the number of elements stored in the buffer
/// Requires: cbuf is valid and created by circular_buf_init
/// Returns the current number of elements in the buffer
// u8 cBufGetQuantity(CBufHandle cbuf);

void cBufWriteToBuf(CBufHandle cbuf, u8* data, u8 sz);

void cBufSafeWrite(CBufHandle cbuf, u8* data, u8 sz, osMutexId mutex, TickType_t ticks);

u16  cBufRead(CBufHandle cbuf, u8* dist, u8 sz);
void copyGetDatafromBuf(CBufHandle cbuf, u8* dist, u16 sz, CircTypeBuf type);

u8  getLenMsgSimUart(CBufHandle cbuf);
u8  getLenMsgEnergyUart(CBufHandle cbuf);
u16 getLenMsgWirelessSens(CircularBuffer* cbuf);

// TODO: int circular_buf_get_range(circular_buf_t cbuf, uint8_t *data, size_t len);
// TODO: int circular_buf_put_range(circular_buf_t cbuf, uint8_t * data, size_t len);

#endif  // CIRCULAR_BUFFER_H_
