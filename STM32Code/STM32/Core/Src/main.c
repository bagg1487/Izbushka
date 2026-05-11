/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 FreeRTOS Servos Example
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef* servo_timers[] = {
    &htim2, &htim2, &htim2, &htim2, // PB10, PB11, PB15, PB14
    &htim3, &htim3, &htim3, &htim3, // PE13, PE11, PG1, PE7
    &htim1, &htim1                  // PF13, PF14
};

uint32_t servo_channels[] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
    TIM_CHANNEL_1, TIM_CHANNEL_2
};

/* Example: 1500 = ??????? ????????? ???????????? */
uint16_t servo_positions[10] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void ServoTask(void *argument);

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
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* Start PWM for all servos */
  for(int i=0;i<10;i++){
      HAL_TIM_PWM_Start(servo_timers[i], servo_channels[i]);
  }

  /* FreeRTOS init */
  osKernelInitialize();
  osThreadNew(ServoTask, NULL, NULL);
  osKernelStart();

  /* Infinite loop */
  while (1)
  {
  }
}

/* FreeRTOS task to move servos */
void ServoTask(void *argument)
{
  (void) argument;
  while(1)
  {
      for(int i=0;i<10;i++){
          __HAL_TIM_SET_COMPARE(servo_timers[i], servo_channels[i], servo_positions[i]);
      }
      osDelay(20); // ?????????? ?????? 20 ??
  }
}

/* System Clock Stub */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while(1);
}

/* Timer interrupt callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_IncTick();
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
