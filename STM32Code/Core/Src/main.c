/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include "LegControl.h"
#include "servo.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "L298NDriver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M_PI 3.141592

#define SERVO_COUNT    20

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#define BUFFER_SIZE 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
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
/* Definitions for HeadControlTask */
osThreadId_t HeadControlTaskHandle;
const osThreadAttr_t HeadControlTask_attributes = {
  .name = "HeadControlTask",
  .stack_size = 256 * 4,
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
/* USER CODE BEGIN PV */

// PID Controller structure
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error;
    float prev_error;
    float integral;
    float derivative;
    float output;
    float setpoint;
    float actual;
    float dt;
    float max_integral;
    float max_output;
} PID_Controller;

// PID controllers
PID_Controller pid_motor1 = {0};
PID_Controller pid_motor2 = {0};

uint32_t last_pid_update = 0;
float target_speed1 = 400.0f;
float target_speed2 = 700.0f;

// Motors
Motor motor1 = {
    .GPIO_Port1 = GPIOG,
    .GPIO_Pin1 = GPIO_PIN_1,
    .GPIO_Port2 = GPIOE,
    .GPIO_Pin2 = GPIO_PIN_7,
    .htim = &htim1,
    .channel = TIM_CHANNEL_2
};

Motor motor2 = {
    .GPIO_Port1 = GPIOF,
    .GPIO_Pin1 = GPIO_PIN_13,
    .GPIO_Port2 = GPIOF,
    .GPIO_Pin2 = GPIO_PIN_14,
    .htim = &htim1,
    .channel = TIM_CHANNEL_3
};

// Arm motors
Motor motor_hand1 = {
    .GPIO_Port1 = GPIOE,
    .GPIO_Pin1 = GPIO_PIN_1,
    .GPIO_Port2 = GPIOE,
    .GPIO_Pin2 = GPIO_PIN_15,
};

Motor motor_hand2 = {
    .GPIO_Port1 = GPIOB,
    .GPIO_Pin1 = GPIO_PIN_13,
    .GPIO_Port2 = GPIOB,
    .GPIO_Pin2 = GPIO_PIN_11,
};

// Arm structure (3 servos)
Arm leftHand = {
    .part1Length = 8,
    .part2Length = 8,
    .servoARange = {10, 170},
    .servoBRange = {10, 170},
    .servoCRange = {10, 170},
    .defaultAngle = {90, 90, 90},
    .calibrationAngle = {90, 90, 90},
    .currentAngle = {90, 90, 90}
};

Arm rightHand = {
    .part1Length = 8,
    .part2Length = 8,
    .servoARange = {10, 170},
    .servoBRange = {10, 170},
    .servoCRange = {10, 170},
    .defaultAngle = {90, 90, 90},
    .calibrationAngle = {90, 90, 90},
    .currentAngle = {90, 90, 90}
};

// Head angles (2 servos, C controls tilt)
LegAngles leftHead = {.B = 87, .C = 100};
LegAngles rightHead = {.B = 80, .C = 95};
LegAngles leftHeadDefault = {.B = 87, .C = 100};
LegAngles rightHeadDefault = {.B = 80, .C = 95};
LegAngles leftHeadCurrent = {.B = 87, .C = 100};
LegAngles rightHeadCurrent = {.B = 80, .C = 95};

PCA9685 pca9685_leg;   // for head servos

int headDir = 0;       // -1 up, 1 down, 0 stop

// Hand PCA9685
PCA9685 pca9685_hand;

// Global variables
int calibrationAngles[20];
bool calibration = false;
int turnLeft = 0;
int turnRight = 0;
int hold3mod = 0;
int hold10mod = 0;
int clenchLeft = 0;
int clenchRight = 0;

// UART variables
bool flag_send;
uint8_t queue_message;
uint8_t tx_buffer[BUFFER_SIZE];
uint8_t rx_buffer[1];
uint8_t error_message[] = "tx buffer is crowded\n\r";
uint8_t error_counter;
volatile uint8_t rx_counter;
volatile uint8_t size_message;

// Ultrasonic
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOA

uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;
float dist = 0;
uint32_t icValue1 = 0;
uint32_t icValue2 = 0;
uint8_t isFirstCaptured = 0;
float distance = 0.0f;

// State variable
volatile int state = 6;

// Messages
char msg[40];
int reedModule = 0;
int lightSensor = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartUARTTask(void *argument);
void StartHandServoControlTask(void *argument);
void StartMotorControlTask(void *argument);
void StartHeadControlTask(void *argument);

/* USER CODE BEGIN PFP */

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float dt, float max_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
    pid->setpoint = 0;
    pid->actual = 0;
    pid->max_integral = max_output * 2;
    pid->max_output = max_output;
}

float PID_Update(PID_Controller* pid, float setpoint, float actual) {
    pid->setpoint = setpoint;
    pid->actual = actual;
    
    pid->error = pid->setpoint - pid->actual;
    pid->integral += pid->error * pid->dt;
    
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    
    pid->derivative = (pid->error - pid->prev_error) / pid->dt;
    pid->output = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * pid->derivative);
    
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }
    
    pid->prev_error = pid->error;
    return pid->output;
}

void PID_Reset(PID_Controller* pid) {
    pid->integral = 0;
    pid->prev_error = 0;
    pid->error = 0;
    pid->output = 0;
}

void L298N_move_PID(Motor motor, int dir, float target_pwm, PID_Controller* pid, float current_pwm) {
    switch(dir) {
        case -1:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 1);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
            target_pwm = -target_pwm;
            break;
        case 1:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 1);
            break;
        default:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
            PID_Reset(pid);
            __HAL_TIM_SET_COMPARE(motor.htim, motor.channel, 0);
            return;
    }
    
    float pid_output = PID_Update(pid, target_pwm, current_pwm);
    int pwm_value = (int)fabs(pid_output);
    
    if (pwm_value > 1500) pwm_value = 1500;
    if (pwm_value < 0) pwm_value = 0;
    
    __HAL_TIM_SET_COMPARE(motor.htim, motor.channel, pwm_value);
}

void L298N_move(Motor motor, int dir, int speed) {
    switch(dir) {
        case -1:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 1);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
            break;
        case 1:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 1);
            break;
        default:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
            break;
    }
    __HAL_TIM_SET_COMPARE(motor.htim, motor.channel, speed);
}

void L298N_move_without_PWM(Motor motor, int dir) {
    switch(dir) {
        case -1:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 1);
            break;
        case 1:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 1);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
            break;    
        default:
            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
            break;
    }
}

void HCSR04_Trigger() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

char* getSensorInfo() {
    int fake_temp = 0;   
    int fake_hum = 0;    
    reedModule = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    
    HAL_ADC_Start(&hadc1);  
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  
    lightSensor = HAL_ADC_GetValue(&hadc1);  
    int light = round((lightSensor / 4095.0) * 100);  
    HAL_ADC_Stop(&hadc1);  
    
    dist = distance;
    int smoke_sensor = 0;
    int touch_sensor = 0;
    
    sprintf(msg, "CMD_SENSORS#%d#%d#%d#%d#%.2f#%d#%d\n",
            fake_temp, fake_hum, reedModule, light, dist, smoke_sensor, touch_sensor);
    return msg;
}

void setServoAngle(char leg, char servo, int angle) {
    if(leg == 'H') {  // Left hand
        switch(servo) {
            case 'A': leftHand.calibrationAngle.A = angle; break;
            case 'B': leftHand.calibrationAngle.B = angle; break;
            case 'C': leftHand.calibrationAngle.C = angle; break;
        }
    } else if(leg == 'M') {  // Right hand
        switch(servo) {
            case 'A': rightHand.calibrationAngle.A = angle; break;
            case 'B': rightHand.calibrationAngle.B = angle; break;
            case 'C': rightHand.calibrationAngle.C = angle; break;
        }
    }
}

void allServoHandSpin(PCA9685 pca9685, int speedDelay, ArmAngles left, ArmAngles right, int correctAngle) {
    // Right hand (pins 4, 6, 5)
    PCA9685_SetServoAngle(pca9685, 4, left.A);
    PCA9685_SetServoAngle(pca9685, 6, left.B);
    PCA9685_SetServoAngle(pca9685, 5, (left.C + correctAngle));
    
    // Left hand (pins 2, 1, 3) - inverted
    PCA9685_SetServoAngle(pca9685, 2, 180 - right.A);
    PCA9685_SetServoAngle(pca9685, 1, 180 - right.B);
    PCA9685_SetServoAngle(pca9685, 3, 180 - (right.C + correctAngle));
    
    HAL_Delay(speedDelay);
}

void allServoLegSpin(PCA9685 pca9685, int speedDelay, LegAngles left, LegAngles right, int correctAngle) {
    // Left head servos
    PCA9685_SetServoAngle(pca9685, 6, (left.C + correctAngle));
    PCA9685_SetServoAngle(pca9685, 7, left.B);
    
    // Right head servos 
    PCA9685_SetServoAngle(pca9685, 15, 180 - (right.C + correctAngle));
    PCA9685_SetServoAngle(pca9685, 14, 180 - right.B);
    
    HAL_Delay(speedDelay);
}

// Animation functions
ArmAngles greatingStateLead(int frame, bool reverse) {
    ArmAngles angles = {0, 0, 0};
    switch(frame) {
        case 0: angles = setArmAngle(0, 0, 0); break;
        case 1: angles = setArmAngle(0, -90, -60); break;
        case 2: angles = setArmAngle(0, -90, 60); break;
        case 3: angles = setArmAngle(0, -90, -60); break;
        case 4: angles = setArmAngle(0, 0, 0); break;
    }
    if(!reverse) {
        angles.B = leftHand.defaultAngle.B + angles.B;
        angles.C = leftHand.defaultAngle.C + angles.C;
    } else {
        angles.B = rightHand.defaultAngle.B + angles.B;
        angles.C = rightHand.defaultAngle.C + angles.C;
    }
    return angles;
}

ArmAngles greatingStateSupport(int frame, bool reverse) {
    ArmAngles angles = {0, 0, 0};
    switch(frame) {
        case 0: angles = setArmAngle(0, 0, 50); break;
        case 1: angles = setArmAngle(0, -40, 50); break;
        case 2: angles = setArmAngle(0, 0, 50); break;
        case 3: angles = setArmAngle(0, -40, 50); break;
        case 4: angles = setArmAngle(0, 0, 0); break;
    }
    if(reverse) {
        angles.B = leftHand.defaultAngle.B + angles.B;
        angles.C = leftHand.defaultAngle.C + angles.C;
    } else {
        angles.B = rightHand.defaultAngle.B + angles.B;
        angles.C = rightHand.defaultAngle.C + angles.C;
    }
    return angles;
}

ArmAngles boxAnimLead_Hand(int frame) {
    ArmAngles angles = {0, 0, 0};
    switch(frame) {
        case 0: angles = setArmAngle(0, -62, 60); break;
        case 1: angles = setArmAngle(0, 40, -30); break;
    }
    angles.B = leftHand.defaultAngle.B + angles.B;
    angles.C = leftHand.defaultAngle.C + angles.C;
    return angles;
}

ArmAngles boxAnimSupport_Hand(int frame) {
    ArmAngles angles = {0, 0, 0};
    switch(frame) {
        case 0: angles = setArmAngle(0, 50, -40); break;
        case 1: angles = setArmAngle(0, -65, 65); break;
    }
    angles.B = rightHand.defaultAngle.B + angles.B;
    angles.C = rightHand.defaultAngle.C + angles.C;
    return angles;
}

ArmAngles danceAnimLead_Hand(int frame) {
    ArmAngles angles = {0, 0, 0};
    switch(frame) {
        case 0: case 2: angles = setArmAngle(0, -50, 90); break;
        case 1: case 3: angles = setArmAngle(0, -90, 90); break;
        case 4: angles = setArmAngle(0, -75, -45); break;
        case 5: angles = setArmAngle(0, -75, 90); break;
        case 6: case 8: angles = setArmAngle(0, -90, 70); break;
        case 7: angles = setArmAngle(0, 5, -5); break;
        case 9: case 10: angles = setArmAngle(0, 0, 0); break;
    }
    angles.B = leftHand.defaultAngle.B + angles.B;
    angles.C = leftHand.defaultAngle.C + angles.C;
    return angles;
}

ArmAngles danceAnimSupport_Hand(int frame) {
    ArmAngles angles = {0, 0, 0};
    switch(frame) {
        case 0: case 2: angles = setArmAngle(0, -50, 90); break;
        case 1: case 3: angles = setArmAngle(0, -90, 90); break;
        case 4: angles = setArmAngle(0, -75, 90); break;
        case 5: angles = setArmAngle(0, -75, -45); break;
        case 6: case 8: angles = setArmAngle(0, 0, 0); break;
        case 7: angles = setArmAngle(0, 5, 20); break;
        case 9: angles = setArmAngle(0, -90, 70); break;
        case 10: angles = setArmAngle(0, 0, 0); break;
    }
    angles.B = rightHand.defaultAngle.B + angles.B;
    angles.C = rightHand.defaultAngle.C + angles.C;
    return angles;
}

static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout) {
    uint32_t Tickstart = HAL_GetTick();
    uint8_t ret = 1;
    for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) {
        if(timeout != HAL_MAX_DELAY) {
            if((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) ret = 0;
        }
        __NOP;
    }
    return ret;
}

static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *GPIOPortSCL, GPIO_TypeDef *GPIOPortSDA, uint32_t GPIO_SCl, uint32_t GPIO_SDA, uint32_t timeout) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    HAL_I2C_DeInit(hi2c);
    
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    
    GPIO_InitStructure.Pin = GPIO_SCl;
    HAL_GPIO_Init(GPIOPortSCL, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = GPIO_SDA;
    HAL_GPIO_Init(GPIOPortSDA, &GPIO_InitStructure);
    
    HAL_GPIO_WritePin(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET);
    
    wait_for_gpio_state_timeout(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET, timeout);
    wait_for_gpio_state_timeout(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET, timeout);
    
    HAL_GPIO_WritePin(GPIOPortSDA, GPIO_SDA, GPIO_PIN_RESET);
    wait_for_gpio_state_timeout(GPIOPortSDA, GPIO_SDA, GPIO_PIN_RESET, timeout);
    
    HAL_GPIO_WritePin(GPIOPortSCL, GPIO_SCl, GPIO_PIN_RESET);
    wait_for_gpio_state_timeout(GPIOPortSCL, GPIO_SCl, GPIO_PIN_RESET, timeout);
    
    HAL_GPIO_WritePin(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET);
    wait_for_gpio_state_timeout(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET, timeout);
    
    HAL_GPIO_WritePin(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET);
    wait_for_gpio_state_timeout(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET, timeout);
    
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pin = GPIO_SCl;
    HAL_GPIO_Init(GPIOPortSCL, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = GPIO_SDA;
    HAL_GPIO_Init(GPIOPortSDA, &GPIO_InitStructure);
    
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    __NOP;
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    __NOP;
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
    __NOP;
    HAL_I2C_Init(hi2c);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
  /* USER CODE END 2 */
  
  /* Init scheduler */
  osKernelInitialize();
  
  /* Create the mutex(es) */
  UARTDataMutexHandle = osMutexNew(&UARTDataMutex_attributes);
  
  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */
  
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */
  
  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */
  
  /* Create the queue(s) */
  txDataUART1Handle = osMessageQueueNew (4, 128, &txDataUART1_attributes);
  rxDataUART1Handle = osMessageQueueNew (4, 128, &rxDataUART1_attributes);
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */
  
  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);
  HandServoControHandle = osThreadNew(StartHandServoControlTask, NULL, &HandServoContro_attributes);
  MotorControlTasHandle = osThreadNew(StartMotorControlTask, NULL, &MotorControlTas_attributes);
  HeadControlTaskHandle = osThreadNew(StartHeadControlTask, NULL, &HeadControlTask_attributes);
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
  
  /* creation of myEvent01 */
  myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);
  
  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
  
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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* Peripheral initialization functions */
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C3_Init(void) {
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 127;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 44-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65450-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
  
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (isFirstCaptured == 0) {
            icValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            isFirstCaptured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            icValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_COUNTER(htim, 0);
            
            if (icValue2 > icValue1)
                distance = (icValue2 - icValue1) * 0.0343f / 2.0f;
            else
                distance = ((0xFFFF - icValue1) + icValue2) * 0.0343f / 2.0f;
            
            isFirstCaptured = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_counter < (BUFFER_SIZE - 2)) { 
            if (rx_buffer[0] != '\r') {
                tx_buffer[rx_counter++] = rx_buffer[0];
                size_message = rx_counter;
            } else {
                tx_buffer[rx_counter++] = '\n'; 
                tx_buffer[rx_counter] = '\0';   
                size_message = rx_counter;
                rx_counter = 0;                 
                flag_send = true;               
                error_counter = 0;              
            }
        } else {
            if (error_counter == 0) {
                HAL_UART_Transmit_IT(huart, error_message, sizeof(error_message) - 1);
                error_counter++;
            }
            rx_counter = 0; 
        }
        HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  for(;;) {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
  * @brief Function implementing the UARTTask thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument) {
  /* USER CODE BEGIN StartUARTTask */
  flag_send = 0;
  queue_message = 1;
  rx_counter = 0;
  size_message = 0;
  error_counter = 0;
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
  
  char commandSep[1] = "\n";
  char parameterSep[1] = "#";
  char *command;
  char *parametr;
  
  for(;;) {
    char* check = getSensorInfo();
    HCSR04_Trigger();
    osDelay(1);
    
    if (flag_send == 1 && queue_message != 255) {
      char* commands = (char*)tx_buffer;
      command = strtok(commands, commandSep);
      
      while (command != NULL) {
        parametr = strtok(command, parameterSep);
        
        while (parametr != NULL) {
          if (strcmp(parametr, "CMD_SONIC") == 0) {
            char buffer[10];
            sprintf(buffer, "%.2f\n", distance);
            if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {
              HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer, strlen(buffer));
            }
          }
          else if (strcmp(parametr, "CMD_SENSORS") == 0) {
            char* data = getSensorInfo();
            if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {
              HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, strlen(data));
            }
          }
          else if (strcmp(parametr, "CMD_LED") == 0) {
            int red = 0, green = 0, blue = 0;
            parametr = strtok(NULL, parameterSep);
            if (parametr) red = atoi(parametr);
            parametr = strtok(NULL, parameterSep);
            if (parametr) green = atoi(parametr);
            parametr = strtok(NULL, parameterSep);
            if (parametr) blue = atoi(parametr);
						
						red = constrain(red, 0, 255);
						green = constrain(green, 0, 255);
						blue = constrain(blue, 0, 255);
		
						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, red > 127 ? GPIO_PIN_SET : GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, green > 127 ? GPIO_PIN_SET : GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, blue > 127 ? GPIO_PIN_SET : GPIO_PIN_RESET);
          }
          else if (strcmp(parametr, "CMD_PING") == 0) {
            if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {
              uint8_t str[] = "CMD_PING\n";
              HAL_UART_Transmit_DMA(&huart1, str, 9);
            }
          }
          else if (strcmp(parametr, "CMD_CALIBRATION") == 0) {
            parametr = strtok(NULL, parameterSep);
            char *leg = parametr;
            parametr = strtok(NULL, parameterSep);
            char *servo = parametr;
            parametr = strtok(NULL, parameterSep);
            int angle = atoi(parametr);
            setServoAngle(*leg, *servo, angle);
          }
          else if (strcmp(parametr, "CMD_CALIBRATION_MOD") == 0) {
            parametr = strtok(NULL, parameterSep);
            int mod = atoi(parametr);
            if (mod == 1) {
              osMutexAcquire(UARTDataMutexHandle, osWaitForever);
              state = 8;
              osMutexRelease(UARTDataMutexHandle);
            } else if (mod == 0) {
              osMutexAcquire(UARTDataMutexHandle, osWaitForever);
              leftHand.defaultAngle = leftHand.calibrationAngle;
              rightHand.defaultAngle = rightHand.calibrationAngle;
              state = 6;
              osMutexRelease(UARTDataMutexHandle);
            }
          }
          else if (strcmp(parametr, "CMD_CALIBRATION_ALL") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            for (int i = 10; i < 16; i++) {
              parametr = strtok(NULL, parameterSep);
              if(parametr) calibrationAngles[i] = atoi(parametr);
            }
            parametr = strtok(NULL, parameterSep); if(parametr) calibrationAngles[16] = atoi(parametr);
            parametr = strtok(NULL, parameterSep); if(parametr) calibrationAngles[17] = atoi(parametr);
            state = 9;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_MOVE_FORWARD") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 1;
            turnLeft = 1;
            turnRight = 1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_MOVE_BACKWARD") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 1;
            turnLeft = 0;
            turnRight = 0;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_MOVE_LEFT") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 1;
            turnLeft = 1;
            turnRight = 0;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_MOVE_RIGHT") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 1;
            turnLeft = 0;
            turnRight = 1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_MOVE_STOP") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 6;
            turnLeft = 1;
            turnRight = 1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_CLENCH_RIGHT") == 0) {
            parametr = strtok(NULL, parameterSep);
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            clenchRight = 1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_CLENCH_LEFT") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            clenchLeft = 1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_LOOK_UP") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            headDir = -1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_LOOK_DOWN") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            headDir = 1;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_LOOK_DEFINE") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            headDir = 0;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_HOLD_3") == 0) {
            parametr = strtok(NULL, parameterSep);
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            hold3mod = atoi(parametr);
            state = 2;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_HOLD_10") == 0) {
            parametr = strtok(NULL, parameterSep);
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            hold10mod = atoi(parametr);
            state = 3;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_HANDS_UP") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 5;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_BOX") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 11;
            osMutexRelease(UARTDataMutexHandle);
          }
          else if (strcmp(parametr, "CMD_DANCE") == 0) {
            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
            state = 12;
            osMutexRelease(UARTDataMutexHandle);
          }
          
          parametr = strtok(NULL, parameterSep);
        }
        command = strtok(NULL, commandSep);
      }
      
      queue_message = 255;
      memset(tx_buffer, 0, sizeof(uint8_t) * BUFFER_SIZE);
    } else if (flag_send == 1 && queue_message == 255) {
      queue_message = 1;
      flag_send = 0;
    }
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
void StartHandServoControlTask(void *argument) {
  /* USER CODE BEGIN StartHandServoControlTask */
  osDelay(500);
  
  pca9685_hand.pca9685_i2c = &hi2c3;
  pca9685_hand.PCA9685_ADDRESS = PCA9685_ADDRESS1;
  
  I2C_ClearBusyFlagErratum(&hi2c3, GPIOA, GPIOC, GPIO_PIN_8, GPIO_PIN_9, 1000);
  osDelay(100);
  
  int init_attempts = 0;
  while (PCA9685_Init(pca9685_hand) != 0 && init_attempts < 10) {
    init_attempts++;
    osDelay(50);
  }
  
  PCA9685_ResetAllChannels(pca9685_hand);
  osDelay(50);
  
  allServoHandSpin(pca9685_hand, 20, leftHand.defaultAngle, rightHand.defaultAngle, 0);
  
  for(;;) {
    int current_state = state;
    switch(current_state) {
      case 6:
        if(!targetCheckArm(leftHand.defaultAngle, leftHand.currentAngle)) {
          ArmAngles resultLeft = leftHand.defaultAngle;
          ArmAngles resultRight = rightHand.defaultAngle;
          int maxAngleLeft = getMaxAngleArm(leftHand.currentAngle, resultLeft);
          int maxAngleRight = getMaxAngleArm(rightHand.currentAngle, resultRight);
          int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
          ArmAngles startLeft = leftHand.currentAngle;
          ArmAngles startRight = rightHand.currentAngle;
          
          for(int step = 1; step <= maxAngle; step++) {
            leftHand.currentAngle = getNextStepArm(startLeft, resultLeft, maxAngle, step);
            rightHand.currentAngle = getNextStepArm(startRight, resultRight, maxAngle, step);
            allServoHandSpin(pca9685_hand, 20, leftHand.currentAngle, rightHand.currentAngle, 0);
          }
        }
        break;
        
      case 5:
        {
          bool legSwitch = false;
          for(int i = 0; i < 5; i++) {
            ArmAngles resultLeft, resultRight;
            if(!legSwitch) {
              resultLeft = greatingStateLead(i, false);
              resultRight = greatingStateSupport(i, false);
            } else {
              resultLeft = greatingStateSupport(i, true);
              resultRight = greatingStateLead(i, true);
            }
            
            int maxAngleLeft = getMaxAngleArm(leftHand.currentAngle, resultLeft);
            int maxAngleRight = getMaxAngleArm(rightHand.currentAngle, resultRight);
            int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
            ArmAngles startLeft = leftHand.currentAngle;
            ArmAngles startRight = rightHand.currentAngle;
            
            for(int step = 1; step <= maxAngle; step++) {
              leftHand.currentAngle = getNextStepArm(startLeft, resultLeft, maxAngle, step);
              rightHand.currentAngle = getNextStepArm(startRight, resultRight, maxAngle, step);
              allServoHandSpin(pca9685_hand, 5, leftHand.currentAngle, rightHand.currentAngle, 0);
            }
          }
          osMutexAcquire(UARTDataMutexHandle, osWaitForever);
          state = 6;
          osMutexRelease(UARTDataMutexHandle);
        }
        break;
        
      case 8:
        allServoHandSpin(pca9685_hand, 400, leftHand.calibrationAngle, rightHand.calibrationAngle, 0);
        break;
        
      case 9:
        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
        leftHand.calibrationAngle.A = calibrationAngles[10];
        leftHand.calibrationAngle.B = calibrationAngles[11];
        leftHand.calibrationAngle.C = calibrationAngles[12];
        leftHand.defaultAngle = leftHand.calibrationAngle;
        
        rightHand.calibrationAngle.A = calibrationAngles[15];
        rightHand.calibrationAngle.B = calibrationAngles[16];
        rightHand.calibrationAngle.C = calibrationAngles[17];
        rightHand.defaultAngle = rightHand.calibrationAngle;
        osMutexRelease(UARTDataMutexHandle);
        break;
        
      case 11:
        for(int j = 0; j < 3; j++) {
          for(int k = 0; k < 2; k++) {
            ArmAngles resultLeft = boxAnimLead_Hand(k);
            ArmAngles resultRight = boxAnimSupport_Hand(k);
            
            int maxAngleLeft = getMaxAngleArm(leftHand.currentAngle, resultLeft);
            int maxAngleRight = getMaxAngleArm(rightHand.currentAngle, resultRight);
            int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
            ArmAngles startLeft = leftHand.currentAngle;
            ArmAngles startRight = rightHand.currentAngle;
            
            for(int step = 1; step <= maxAngle; step++) {
              leftHand.currentAngle = getNextStepArm(startLeft, resultLeft, maxAngle, step);
              rightHand.currentAngle = getNextStepArm(startRight, resultRight, maxAngle, step);
              allServoHandSpin(pca9685_hand, 5, leftHand.currentAngle, rightHand.currentAngle, 0);
            }
            osDelay(100);
          }
        }
        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
        state = 6;
        osMutexRelease(UARTDataMutexHandle);
        break;
        
      case 12:
        for(int k = 0; k < 10; k++) {
          ArmAngles resultLeft = danceAnimLead_Hand(k);
          ArmAngles resultRight = danceAnimSupport_Hand(k);
          
          int maxAngleLeft = getMaxAngleArm(leftHand.currentAngle, resultLeft);
          int maxAngleRight = getMaxAngleArm(rightHand.currentAngle, resultRight);
          int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
          ArmAngles startLeft = leftHand.currentAngle;
          ArmAngles startRight = rightHand.currentAngle;
          
          for(int step = 1; step <= maxAngle; step++) {
            leftHand.currentAngle = getNextStepArm(startLeft, resultLeft, maxAngle, step);
            rightHand.currentAngle = getNextStepArm(startRight, resultRight, maxAngle, step);
            allServoHandSpin(pca9685_hand, 10, leftHand.currentAngle, rightHand.currentAngle, 0);
          }
        }
        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
        state = 6;
        osMutexRelease(UARTDataMutexHandle);
        break;
    }
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
void StartMotorControlTask(void *argument) {
  /* USER CODE BEGIN StartMotorControlTask */
  int delayInterval = 8000;
  int timerLeft, timerRight;
  int dirLeft = -1, dirRight = -1;
  
  PID_Init(&pid_motor1, 2.0f, 0.1f, 0.05f, 0.01f, 1500.0f);
  PID_Init(&pid_motor2, 2.0f, 0.1f, 0.05f, 0.01f, 1500.0f);
  
  last_pid_update = HAL_GetTick();
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
  L298N_move_without_PWM(motor_hand1, 0);
  L298N_move_without_PWM(motor_hand2, 0);
  
  for(;;) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_pid_update) / 1000.0f;
    
    float current_pwm1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    float current_pwm2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    
    if (dt >= 0.01f) {
      last_pid_update = current_time;
      
      int current_state = state;
      if (current_state == 1) {
        if(turnLeft == 1 && turnRight == 1) {
          L298N_move_PID(motor1, 1, target_speed1, &pid_motor1, current_pwm1);
          L298N_move_PID(motor2, 1, target_speed2, &pid_motor2, current_pwm2);
        } else if(turnLeft == 0 && turnRight == 0) {
          L298N_move_PID(motor1, -1, target_speed1, &pid_motor1, current_pwm1);
          L298N_move_PID(motor2, -1, target_speed2, &pid_motor2, current_pwm2);
        } else if(turnLeft == 1 && turnRight == 0) {
          L298N_move_PID(motor1, -1, target_speed1, &pid_motor1, current_pwm1);
          L298N_move_PID(motor2, 1, target_speed2, &pid_motor2, current_pwm2);
        } else if(turnLeft == 0 && turnRight == 1) {
          L298N_move_PID(motor1, 1, target_speed1, &pid_motor1, current_pwm1);
          L298N_move_PID(motor2, -1, target_speed2, &pid_motor2, current_pwm2);
        }
      } else if (current_state == 12) {
        osDelay(4000);
        L298N_move(motor1, 1, 400);
        L298N_move(motor2, -1, 700);
        L298N_move(motor1, 1, 400);
        L298N_move(motor2, -1, 700);
        L298N_move(motor1, -1, 400);
        L298N_move(motor2, -1, 700);
        osDelay(4000);
        L298N_move(motor1, 1, 400);
        L298N_move(motor2, 1, 700);
        osDelay(4000);
        osDelay(10000);
      } else if (current_state == 6) {
        L298N_move(motor1, 0, 0);
        L298N_move(motor2, 0, 0);
        PID_Reset(&pid_motor1);
        PID_Reset(&pid_motor2);
      }
    }
    
    if(clenchLeft == 1) {
      timerLeft = HAL_GetTick();
      dirLeft *= -1;
      clenchLeft = -1;
    }
    
    if(clenchRight == 1) {
      timerRight = HAL_GetTick();
      dirRight *= -1;
      clenchRight = -1;
    }
    
    if(HAL_GetTick() - timerLeft < delayInterval) {
      L298N_move_without_PWM(motor_hand1, dirLeft);
    } else {
      L298N_move_without_PWM(motor_hand1, 0);
    }
    
    if(HAL_GetTick() - timerRight < delayInterval) {
      L298N_move_without_PWM(motor_hand2, dirRight);
    } else {
      L298N_move_without_PWM(motor_hand2, 0);
    }
    
    osDelay(10);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartHeadControlTask */
/**
  * @brief Function implementing the HeadControlTask thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeadControlTask */
void StartHeadControlTask(void *argument) {
  osDelay(200);
  pca9685_leg.pca9685_i2c = &hi2c1;
  pca9685_leg.PCA9685_ADDRESS = PCA9685_ADDRESS2;

  while (PCA9685_Init(pca9685_leg) != 0) {
    osDelay(50);
  };
  
  leftHeadCurrent = leftHeadDefault;
  rightHeadCurrent = rightHeadDefault;
  
  static LegAngles targetLeft;
  static LegAngles targetRight;
  static int initialized = 0;

  if (!initialized) {
    targetLeft = leftHeadCurrent;
    targetRight = rightHeadCurrent;
    initialized = 1;
  }
  
  allServoLegSpin(pca9685_leg, 20, leftHeadCurrent, rightHeadCurrent, 0);

  for(;;) {
    static int prevHeadDir = 0;
    int step = 15;
    
    if (headDir != prevHeadDir) {
      if (headDir == -1) { // look up
        targetLeft.C = targetLeft.C - step;
        targetRight.C = targetRight.C - step;
        
        if (targetLeft.C < 20) targetLeft.C = 20;
        if (targetRight.C < 20) targetRight.C = 20;
        
        int maxAngle = abs(targetLeft.C - leftHeadCurrent.C);
        int maxAngleR = abs(targetRight.C - rightHeadCurrent.C);
        if (maxAngleR > maxAngle) maxAngle = maxAngleR;
        
        if (maxAngle > 0) {
          LegAngles startLeft = leftHeadCurrent;
          LegAngles startRight = rightHeadCurrent;
          
          for (int i = 1; i <= maxAngle; i++) {
            leftHeadCurrent.C = startLeft.C + (targetLeft.C - startLeft.C) * i / maxAngle;
            rightHeadCurrent.C = startRight.C + (targetRight.C - startRight.C) * i / maxAngle;
            leftHeadCurrent.B = leftHeadDefault.B;
            rightHeadCurrent.B = rightHeadDefault.B;
            allServoLegSpin(pca9685_leg, 5, leftHeadCurrent, rightHeadCurrent, 0);
          }
        }
        
        leftHeadCurrent = targetLeft;
        rightHeadCurrent = targetRight;
        allServoLegSpin(pca9685_leg, 5, leftHeadCurrent, rightHeadCurrent, 0);
      } 
			else if (headDir == 0) { // default
					targetLeft = leftHeadDefault;
					targetRight = rightHeadDefault;
			}
      else if (headDir == 1) { // look down
        targetLeft.C = targetLeft.C + step;
        targetRight.C = targetRight.C + step;
        
        if (targetLeft.C > 160) targetLeft.C = 160;
        if (targetRight.C > 160) targetRight.C = 160;
        
        int maxAngle = abs(targetLeft.C - leftHeadCurrent.C);
        int maxAngleR = abs(targetRight.C - rightHeadCurrent.C);
        if (maxAngleR > maxAngle) maxAngle = maxAngleR;
        
        if (maxAngle > 0) {
          LegAngles startLeft = leftHeadCurrent;
          LegAngles startRight = rightHeadCurrent;
          
          for (int i = 1; i <= maxAngle; i++) {
            leftHeadCurrent.C = startLeft.C + (targetLeft.C - startLeft.C) * i / maxAngle;
            rightHeadCurrent.C = startRight.C + (targetRight.C - startRight.C) * i / maxAngle;
            leftHeadCurrent.B = leftHeadDefault.B;
            rightHeadCurrent.B = rightHeadDefault.B;
            allServoLegSpin(pca9685_leg, 5, leftHeadCurrent, rightHeadCurrent, 0);
          }
        }
        
        leftHeadCurrent = targetLeft;
        rightHeadCurrent = targetRight;
        allServoLegSpin(pca9685_leg, 5, leftHeadCurrent, rightHeadCurrent, 0);
      }
      
      prevHeadDir = headDir;

      osMutexAcquire(UARTDataMutexHandle, osWaitForever);
      headDir = 0;
      osMutexRelease(UARTDataMutexHandle);
      prevHeadDir = 0;
    }
    osDelay(50);
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */