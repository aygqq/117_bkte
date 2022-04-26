/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

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
/* USER CODE BEGIN Variables */
#include "usart.h"
#include "utils_pckgs_manager.h"

/* USER CODE END Variables */
osThreadId    getCurrentHandle;
osThreadId    getNewBinHandle;
osThreadId    keepAliveHandle;
osThreadId    webExchangeHandle;
osThreadId    manageIWDGHandle;
osThreadId    createWebPckgHandle;
osMessageQId  queueWebPckgHandle;
osTimerId     timerPowerOffHandle;
osMutexId     mutexBigBufHandle;
osMutexId     mutexWebHandle;
osMutexId     mutexRTCHandle;
osMutexId     mutexSDHandle;
osMutexId     mutexSpiFlashHandle;
osMutexId     mutexSessionHandle;
osSemaphoreId semCreateWebPckgHandle;
osSemaphoreId semSendWebPckgHandle;
osSemaphoreId semCurrMeasureHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void taskCurMeasure(void const *argument);
void taskGetNewBin(void const *argument);
void taskKeepAlive(void const *argument);
void taskWebExchange(void const *argument);
void taskManageIWDG(void const *argument);
void taskCreateWebPckg(void const *argument);
void timerPowerOff_callback(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t  xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */
    /* Create the mutex(es) */
    /* definition and creation of mutexBigBuf */
    osMutexDef(mutexBigBuf);
    mutexBigBufHandle = osMutexCreate(osMutex(mutexBigBuf));

    /* definition and creation of mutexWeb */
    osMutexDef(mutexWeb);
    mutexWebHandle = osMutexCreate(osMutex(mutexWeb));

    /* definition and creation of mutexRTC */
    osMutexDef(mutexRTC);
    mutexRTCHandle = osMutexCreate(osMutex(mutexRTC));

    /* definition and creation of mutexSD */
    osMutexDef(mutexSD);
    mutexSDHandle = osMutexCreate(osMutex(mutexSD));

    /* definition and creation of mutexSpiFlash */
    osMutexDef(mutexSpiFlash);
    mutexSpiFlashHandle = osMutexCreate(osMutex(mutexSpiFlash));

    /* definition and creation of mutexSession */
    osMutexDef(mutexSession);
    mutexSessionHandle = osMutexCreate(osMutex(mutexSession));

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* definition and creation of semCreateWebPckg */
    osSemaphoreDef(semCreateWebPckg);
    semCreateWebPckgHandle = osSemaphoreCreate(osSemaphore(semCreateWebPckg), 1);

    /* definition and creation of semSendWebPckg */
    osSemaphoreDef(semSendWebPckg);
    semSendWebPckgHandle = osSemaphoreCreate(osSemaphore(semSendWebPckg), 1);

    /* definition and creation of semCurrMeasure */
    osSemaphoreDef(semCurrMeasure);
    semCurrMeasureHandle = osSemaphoreCreate(osSemaphore(semCurrMeasure), 1);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */
    /* definition and creation of timerPowerOff */
    osTimerDef(timerPowerOff, timerPowerOff_callback);
    timerPowerOffHandle = osTimerCreate(osTimer(timerPowerOff), osTimerOnce, NULL);

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* definition and creation of queueWebPckg */
    osMessageQDef(queueWebPckg, 8, WebPckg *);
    queueWebPckgHandle = osMessageCreate(osMessageQ(queueWebPckg), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of getCurrent */
    osThreadDef(getCurrent, taskCurMeasure, osPriorityNormal, 0, 400);
    getCurrentHandle = osThreadCreate(osThread(getCurrent), NULL);

    /* definition and creation of getNewBin */
    osThreadDef(getNewBin, taskGetNewBin, osPriorityNormal, 0, 300);
    getNewBinHandle = osThreadCreate(osThread(getNewBin), NULL);

    /* definition and creation of keepAlive */
    osThreadDef(keepAlive, taskKeepAlive, osPriorityNormal, 0, 300);
    keepAliveHandle = osThreadCreate(osThread(keepAlive), NULL);

    /* definition and creation of webExchange */
    osThreadDef(webExchange, taskWebExchange, osPriorityNormal, 0, 300);
    webExchangeHandle = osThreadCreate(osThread(webExchange), NULL);

    /* definition and creation of manageIWDG */
    osThreadDef(manageIWDG, taskManageIWDG, osPriorityNormal, 0, 128);
    manageIWDGHandle = osThreadCreate(osThread(manageIWDG), NULL);

    /* definition and creation of createWebPckg */
    osThreadDef(createWebPckg, taskCreateWebPckg, osPriorityNormal, 0, 300);
    createWebPckgHandle = osThreadCreate(osThread(createWebPckg), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_taskCurMeasure */
/**
 * @brief  Function implementing the getCurrent thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskCurMeasure */
__weak void taskCurMeasure(void const *argument) {
    /* USER CODE BEGIN taskCurMeasure */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END taskCurMeasure */
}

/* USER CODE BEGIN Header_taskGetNewBin */
/**
 * @brief Function implementing the getNewBin thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskGetNewBin */
__weak void taskGetNewBin(void const *argument) {
    /* USER CODE BEGIN taskGetNewBin */
    /* Infinite loop */
    for (;;) {
        osDelay(1000);
    }
    /* USER CODE END taskGetNewBin */
}

/* USER CODE BEGIN Header_taskKeepAlive */
/**
 * @brief Function implementing the keepAlive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskKeepAlive */
__weak void taskKeepAlive(void const *argument) {
    /* USER CODE BEGIN taskKeepAlive */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END taskKeepAlive */
}

/* USER CODE BEGIN Header_taskWebExchange */
/**
 * @brief Function implementing the webExchange thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskWebExchange */
__weak void taskWebExchange(void const *argument) {
    /* USER CODE BEGIN taskWebExchange */
    /* Infinite loop */
    for (;;) {
        osDelay(3000);
    }
    /* USER CODE END taskWebExchange */
}

/* USER CODE BEGIN Header_taskManageIWDG */
/**
 * @brief Function implementing the manageIWDG thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskManageIWDG */
__weak void taskManageIWDG(void const *argument) {
    /* USER CODE BEGIN taskManageIWDG */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END taskManageIWDG */
}

/* USER CODE BEGIN Header_taskCreateWebPckg */
/**
 * @brief Function implementing the createWebPckg thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskCreateWebPckg */
__weak void taskCreateWebPckg(void const *argument) {
    /* USER CODE BEGIN taskCreateWebPckg */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END taskCreateWebPckg */
}

/* timerPowerOff_callback function */
__weak void timerPowerOff_callback(void const *argument) {
    /* USER CODE BEGIN timerPowerOff_callback */

    /* USER CODE END timerPowerOff_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
