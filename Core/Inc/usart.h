/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#define UART_TIMEOUT		3000
#define UART_SZ_RX_RESPONSE			1500
#define SZ_RX_UART1			66

#define USART_SZ_BUF_RX_USART6    7100
#define USART_SZ_BUF_TX_USART6    1024

#define USART_SZ_BUF_TX_USART2    20
#define USART_SZ_BUF_RX_USART2    240

#define USART_TIMEOUT             15000

#define USART_RE2_WRITE_EN()  HAL_GPIO_WritePin(USART2_RE_GPIO_Port, USART2_RE_Pin, GPIO_PIN_SET)
#define USART_RE2_READ_EN()  HAL_GPIO_WritePin(USART2_RE_GPIO_Port, USART2_RE_Pin, GPIO_PIN_RESET)

typedef UART_HandleTypeDef*	PHuart;
typedef struct{
	IrqFlags irqFlags;
	u8*	pRxBuf;
  u8* pTxBuf;
  u16 szRxBuf;
  u16 szTxBuf;
  PHuart pHuart;
}UartInfo;

extern UartInfo uInfoSim;
extern UartInfo uInfoWirelessSens;

void setBaudRateUart(UART_HandleTypeDef *huart, u32 baudrate);
//void rxUartSIM_IT();
void uartTx(char* data, u16 sz, UartInfo* pUInf);
//void txUartGNSS(char* data, u16 sz);
void clearRx(UART_HandleTypeDef *huart);
//void rxUart1_IT();

void rxUart(UartInfo* pUInf);
void rxUart1_IT();
void uartRxDma(UartInfo* pUInf);
void uartInitInfo();

void uartTxLCD(char* data, u16 sz, UartInfo* pUInf);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
