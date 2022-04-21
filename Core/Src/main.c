/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#include "circularBuffer.h"
#include "fatfs.h"
#include "simcom.h"
#include "utils_bkte.h"
#include "utils_flash.h"

CircularBuffer circBufAllPckgs = {.buf = NULL, .max = 0};

u8 bufPckgs[SZ_PAGE];

BKTE bkte;
Urls urls;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    volatile fw_info_t info = {.header = 0x1122334455667788,
                               .idFirmware = BKTE_ID_FIRMWARE,
                               .idBoot = (u8)getFlashData(FLASH_ADDR_ID_BOOT)};

    memcpy(&bkte.info.fw, (void*)&info, sizeof(fw_info_t));

    LOG(LEVEL_MAIN, "Firmware: %d.%d\r\n\r\n", info.idFirmware, info.idBoot);

    cBufInit(&circBufAllPckgs, bufPckgs, SZ_PAGE, CIRC_TYPE_PCKG_ALL);
    HAL_GPIO_WritePin(GPIOB, MEM_HOLD_Pin, GPIO_PIN_SET);
    bkteInit();
    urlsInit();
    uartInitInfo();

    //  simOn();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  }
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
u8 waitTx(char* waitStr, IrqFlags* pFlags, u16 pause, u32 timeout) {
    u32 tout = 0;
    while (!pFlags->isIrqTx && tout < timeout) {
        osDelay(pause);
        tout += pause;
    }
    if (strlen(waitStr) > 1) LOG(LEVEL_INFO, "%s timeout: %d\r\n", waitStr, tout);
    return pFlags->isIrqTx;
}

u8 waitRx(char* waitStr, IrqFlags* pFlags, u16 pause, u32 timeout) {
    u32 tout = 0;
    while (!(pFlags->isIrqRx) && tout < timeout) {
        osDelay(pause);
        tout += pause;
    }
    if (strlen(waitStr) > 1) LOG(LEVEL_INFO, "%s timeout: %d\r\n", waitStr, tout);
    return pFlags->isIrqRx;
}

u8 waitIdle(char* waitStr, IrqFlags* pFlags, u16 pause, u32 timeout) {
    u32 tout = 0;
    while (!(pFlags->isIrqIdle) && tout < timeout) {
        osDelay(pause);
        tout += pause;
    }
    if (strlen(waitStr) > 1) LOG(LEVEL_INFO, "%s timeout: %d\r\n", waitStr, tout);
    return pFlags->isIrqIdle;
}

u8 waitIdleCnt(char* waitStr, IrqFlags* pFlags, u8 cnt, u16 pause, u32 timeout) {
    u32 tout = 0;
    while ((pFlags->isIrqIdle) < cnt && tout < timeout) {
        osDelay(pause);
        tout += pause;
    }
    if (strlen(waitStr) > 1) LOG(LEVEL_INFO, "%s timeout: %d\r\n", waitStr, tout);
    return pFlags->isIrqIdle;
}

void urlsInit() {
    urls.ourTcpAddr = URL_TCP_ADDR;
    urls.ourTcpPort = URL_TCP_PORT;
    urls.niacTcpAddr = URL_NIAC_TCP_ADDR;
    urls.niacTcpPort = URL_NIAC_TCP_PORT;
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
