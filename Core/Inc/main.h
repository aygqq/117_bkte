/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_iwdg.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_PWR_EN_Pin         GPIO_PIN_0
#define BAT_PWR_EN_GPIO_Port   GPIOC
#define IS_CHRG_BAT_Pin        GPIO_PIN_1
#define IS_CHRG_BAT_GPIO_Port  GPIOC
#define SIM_PWR_EN_Pin         GPIO_PIN_2
#define SIM_PWR_EN_GPIO_Port   GPIOC
#define ADC_BAT_Pin            GPIO_PIN_0
#define ADC_BAT_GPIO_Port      GPIOA
#define USART2_RE_Pin          GPIO_PIN_1
#define USART2_RE_GPIO_Port    GPIOA
#define ADC_CUR_3_Pin          GPIO_PIN_4
#define ADC_CUR_3_GPIO_Port    GPIOA
#define ADC_CUR_1_Pin          GPIO_PIN_6
#define ADC_CUR_1_GPIO_Port    GPIOA
#define ADC_CUR_2_Pin          GPIO_PIN_7
#define ADC_CUR_2_GPIO_Port    GPIOA
#define PWR_STATE_Pin          GPIO_PIN_1
#define PWR_STATE_GPIO_Port    GPIOB
#define PWR_STATE_EXTI_IRQn    EXTI1_IRQn
#define MEM_HOLD_Pin           GPIO_PIN_2
#define MEM_HOLD_GPIO_Port     GPIOB
#define ONEWIRE_3_EN_Pin       GPIO_PIN_8
#define ONEWIRE_3_EN_GPIO_Port GPIOE
#define ONEWIRE_2_EN_Pin       GPIO_PIN_9
#define ONEWIRE_2_EN_GPIO_Port GPIOE
#define ONEWIRE_4_EN_Pin       GPIO_PIN_10
#define ONEWIRE_4_EN_GPIO_Port GPIOE
#define ONEWIRE_1_EN_Pin       GPIO_PIN_11
#define ONEWIRE_1_EN_GPIO_Port GPIOE
#define RF_PWR_Pin             GPIO_PIN_10
#define RF_PWR_GPIO_Port       GPIOB
#define SPI2_CS_MEM_Pin        GPIO_PIN_12
#define SPI2_CS_MEM_GPIO_Port  GPIOB
#define GSM_PWRKEY_Pin         GPIO_PIN_13
#define GSM_PWRKEY_GPIO_Port   GPIOD
#define UART1_RE_Pin           GPIO_PIN_8
#define UART1_RE_GPIO_Port     GPIOA
#define SD_PWR_EN_Pin          GPIO_PIN_0
#define SD_PWR_EN_GPIO_Port    GPIOD
#define SD_DETECTED_Pin        GPIO_PIN_1
#define SD_DETECTED_GPIO_Port  GPIOD
#define LED1G_Pin              GPIO_PIN_3
#define LED1G_GPIO_Port        GPIOD
#define LED3G_Pin              GPIO_PIN_4
#define LED3G_GPIO_Port        GPIOD
#define LED4R_Pin              GPIO_PIN_5
#define LED4R_GPIO_Port        GPIOD
#define LED4G_Pin              GPIO_PIN_6
#define LED4G_GPIO_Port        GPIOD
#define LED2R_Pin              GPIO_PIN_7
#define LED2R_GPIO_Port        GPIOD
#define LED2G_Pin              GPIO_PIN_0
#define LED2G_GPIO_Port        GPIOE
#define LED1R_Pin              GPIO_PIN_1
#define LED1R_GPIO_Port        GPIOE
/* USER CODE BEGIN Private defines */
#include "cmsis_os.h"
#include "debug.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define LOG_SZ_ERROR 100
#define WAIT_TIMEOUT 15000
#define DUMMY_BYTE   0xFF

#define SERVER_MOTZ       1
#define SERVER_NIAC       2
#define URL_TCP_ADDR      (char*)"idim772.ru"
#define URL_TCP_PORT      8103
#define URL_NIAC_TCP_ADDR (char*)"gis.ru.net"
#define URL_NIAC_TCP_PORT 9878

#define SZ_CMD_ENERGY    12
#define SZ_CMD_VOLTAMPER 8
#define SZ_CMD_TEMP      8
#define SZ_CMD_TELEMETRY 10
#define SZ_CMD_ADC_CURR  34

#define SZ_PAGE          255
#define CNT_PAGES        2
#define AMOUNT_MAX_PAGES 5
#define SZ_PAGES         1275  // SZ_PAGE * AMOUNT_MAX_PAGES

#define BKTE_PREAMBLE     0xABCD
#define BKTE_PREAMBLE_LSB 0xAB
#define BKTE_PREAMBLE_MSB 0xCD

#define SZ_WEB_PCKG 1400

extern char          logError[LOG_SZ_ERROR];
typedef uint8_t      u8;
typedef uint16_t     u16;
typedef uint32_t     u32;
typedef uint64_t     u64;
typedef unsigned int uint;

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;

typedef union {
    struct {
        u8 isIrqTx   : 1;
        u8 isIrqRx   : 1;
        u8 isIrqIdle : 3;
    };
    u8 regIrq;
} IrqFlags;

typedef struct {
    char* ourTcpAddr;
    u32   ourTcpPort;
    char* niacTcpAddr;
    u32   niacTcpPort;
} Urls;

extern Urls urls;
u8          waitRx(char* waitStr, IrqFlags* pFlags, u16 pause, u32 timeout);
u8          waitTx(char* waitStr, IrqFlags* pFlags, u16 pause, u32 timeout);
u8          waitIdle(char* waitStr, IrqFlags* pFlags, u16 pause, u32 timeout);
u8          waitIdleCnt(char* waitStr, IrqFlags* pFlags, u8 cnt, u16 pause, u32 timeout);
void        urlsInit();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
