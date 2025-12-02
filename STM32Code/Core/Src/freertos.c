/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MPU5060ReadTask */
osThreadId_t MPU5060ReadTaskHandle;
const osThreadAttr_t MPU5060ReadTask_attributes = {
  .name = "MPU5060ReadTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HandServoContro */
osThreadId_t HandServoControHandle;
const osThreadAttr_t HandServoContro_attributes = {
  .name = "HandServoContro",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for txDataUART1 */
osMessageQueueId_t txDataUART1Handle;
const osMessageQueueAttr_t txDataUART1_attributes = {
  .name = "txDataUART1"
};
/* Definitions for rxDataUART1 */
osMessageQueueId_t rxDataUART1Handle;
const osMessageQueueAttr_t rxDataUART1_attributes = {
  .name = "rxDataUART1"
};
/* Definitions for MPU6050Mutex */
osMutexId_t MPU6050MutexHandle;
const osMutexAttr_t MPU6050Mutex_attributes = {
  .name = "MPU6050Mutex"
};
/* Definitions for UARTDataMutex */
osMutexId_t UARTDataMutexHandle;
const osMutexAttr_t UARTDataMutex_attributes = {
  .name = "UARTDataMutex"
};
/* Definitions for myEvent01 */
osEventFlagsId_t myEvent01Handle;
const osEventFlagsAttr_t myEvent01_attributes = {
  .name = "myEvent01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMPU5060ReadTask(void *argument);
void StartServoControlTask(void *argument);
void StartUARTTask(void *argument);
void StartHandServoControlTask(void *argument);
void StartMotorControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MPU6050Mutex */
  MPU6050MutexHandle = osMutexNew(&MPU6050Mutex_attributes);

  /* creation of UARTDataMutex */
  UARTDataMutexHandle = osMutexNew(&UARTDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of txDataUART1 */
  txDataUART1Handle = osMessageQueueNew (4, 128, &txDataUART1_attributes);

  /* creation of rxDataUART1 */
  rxDataUART1Handle = osMessageQueueNew (4, 128, &rxDataUART1_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MPU5060ReadTask */
  MPU5060ReadTaskHandle = osThreadNew(StartMPU5060ReadTask, NULL, &MPU5060ReadTask_attributes);

  /* creation of ServoControlTas */
  ServoControlTasHandle = osThreadNew(StartServoControlTask, NULL, &ServoControlTas_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* creation of HandServoContro */
  HandServoControHandle = osThreadNew(StartHandServoControlTask, NULL, &HandServoContro_attributes);

  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(StartMotorControlTask, NULL, &MotorControlTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of myEvent01 */
  myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMPU5060ReadTask */
/**
* @brief Function implementing the MPU5060ReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPU5060ReadTask */
void StartMPU5060ReadTask(void *argument)
{
  /* USER CODE BEGIN StartMPU5060ReadTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMPU5060ReadTask */
}

/* USER CODE BEGIN Header_StartServoControlTask */
/**
* @brief Function implementing the ServoControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoControlTask */
void StartServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartServoControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartServoControlTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUARTTask */
}

/* USER CODE BEGIN Header_StartHandServoControlTask */
/**
* @brief Function implementing the HandServoContro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHandServoControlTask */
void StartHandServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartHandServoControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHandServoControlTask */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

