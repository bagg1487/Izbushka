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
#include <string.h> // for strtok
#include <stdlib.h>
#include <string.h>
#include "L298NDriver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M_PI 3.141592

#define SERVO_COUNT    20 //number of servos used

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#define BUFFER_SIZE 64

servo_t servoLA, servoLB, servoLC, servoLD, servoLE,
                servoRA, servoRB, servoRC, servoRD, servoRE;

/*Temp and liquid*/
    char msg[40];
    /*reed switch*/
    int reedModule=0;
    /*light sensor*/
    int lightSensor = 0;

/* UART */
bool flag_send;                                       // Flag to allow tx_buffer[] output to the serial port
uint8_t queue_message;                                // Message queue (since it works with interrupts or DMA. See lesson 6)
uint8_t tx_buffer[BUFFER_SIZE];                    // tx_buffer to output to the port whatever the MCU receives via rx
uint8_t rx_buffer[1];                                 // Buffer to receive information (one symbol at a time, to constantly enter the interrupt and process the incoming message)
uint8_t error_message[] = "tx buffer is crowded\n\r"; // Message indicating that tx_buffer[size_tx_buffer] is full
uint8_t error_counter;                                // Ensures "tx buffer is crowded\n\r" message displays only once
volatile uint8_t rx_counter;                          // Counter for incoming characters
volatile uint8_t size_message;                        // Message size for port output

/*Ultrasonic*/
// Definition of macros for TRIG and ECHO pins
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOA

// Variable declarations
uint32_t pMillis;       // Time in milliseconds
uint32_t Value1 = 0;    // Value 1
uint32_t Value2 = 0;    // Value 2
uint16_t Distance = 0;  // Distance in centimeters
float dist=0;
uint32_t icValue1 = 0;
uint32_t icValue2 = 0;
uint8_t  isFirstCaptured = 0;
float distance = 0.0f;
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
        float Kp;           // Proportional coefficient
        float Ki;           // Integral coefficient
        float Kd;           // Derivative coefficient
        float error;        // Current error
        float prev_error;   // Previous error
        float integral;     // Integral sum
        float derivative;   // Derivative
        float output;       // Output signal
        float setpoint;     // Setpoint (target speed)
        float actual;       // Current value (current speed)
        float dt;           // Time between updates
        float max_integral; // Integral limit
        float max_output;   // Output signal limit
    } PID_Controller;

    // PID controllers for each motor
    PID_Controller pid_motor1 = {0};
    PID_Controller pid_motor2 = {0};

    uint32_t last_pid_update = 0;

    // Target speeds (can be configured via UART)
    float target_speed1 = 400.0f;  // Target PWM value for motor 1
    float target_speed2 = 700.0f;  // Target PWM value for motor 2

    /* Motor structure initialization */

    /* Leg motors with speed control */
    Motor motor1 = {
            .GPIO_Port1 = GPIOG,            // Port for the first pin
            .GPIO_Pin1 = GPIO_PIN_1,        // Pin number for the first pin
            .GPIO_Port2 = GPIOE,            // Port for the second pin
            .GPIO_Pin2 = GPIO_PIN_7,        // Pin number for the second pin
            .htim = &htim1,                 // Timer for PWM
            .channel = TIM_CHANNEL_2       // PWM channel for the timer
    };
    Motor motor2 = {
            .GPIO_Port1 = GPIOF,            // Port for the first pin
            .GPIO_Pin1 = GPIO_PIN_13,       // Pin number for the first pin
            .GPIO_Port2 = GPIOF,            // Port for the second pin
            .GPIO_Pin2 = GPIO_PIN_14,       // Pin number for the second pin
            .htim = &htim1,                 // Timer for PWM
            .channel = TIM_CHANNEL_3        // PWM channel for the timer
    };

    /* Arm motors without speed control */
    Motor motor_hand1 = {
            .GPIO_Port1 = GPIOE,            // Port for the first pin
            .GPIO_Pin1 = GPIO_PIN_14,       // Pin number for the first pin
            .GPIO_Port2 = GPIOE,            // Port for the second pin
            .GPIO_Pin2 = GPIO_PIN_15,       // Pin number for the second pin
    };
    Motor motor_hand2 = {
            .GPIO_Port1 = GPIOB,            // Port for the first pin
            .GPIO_Pin1 = GPIO_PIN_13,       // Pin number for the first pin
            .GPIO_Port2 = GPIOB,            // Port for the second pin
            .GPIO_Pin2 = GPIO_PIN_11,       // Pin number for the second pin
    };

    double part1 = 10;       // Length of the first part of the leg
    double part2 = 13;       // Length of the second part of the leg
    double result = 0;       // Distance from servo A to the endpoint

    int part1Angle = 0;
    int part2Angle = 0;

    double stepLength = 4;   // Step length (distance from one support point to another)

    double hDefault = 17;    // Default height at which the robot stands

    double lDefault = 9;     // Distance from the center of mass to the rotation axis of servo A

    double footWidth = 6.2;  // Width of the foot

    double deltaLength;

    double projectionX;      // Projection on the X-axis for calculating the deviation angle of servo A

    int calibrationAngles[SERVO_COUNT];

    bool calibration = false;

    int deltaAngle[2][2] = {0};

    int angleAlpha = 52;

    int turnLeft = 0;        // Turn left
    int turnRight = 0;       // Turn right

    int hold3mod = 0;        // One of the idle animations after 3 minutes
    int hold10mod = 0;       // One of the idle animations after 10 minutes

    int clenchLeft = 0;      // Left hand state: open/closed
    int clenchRight = 0;     // Right hand state: open/closed
    int headDir = 0;

    int degree90State = 0;
    int main_mode=1;
    
    /* Left leg configuration - only B and C servos are used */
    Leg leftLeg = {
            .part1Length = 10,
            .part2Length = 13,
            .centerGravity = {0, 0, 9},
            .servoBRange = {10, 170},
            .servoCRange = {10, 170},
    };

    /* Right leg configuration - only B and C servos are used */
    Leg rightLeg = {
            .part1Length = 10,
            .part2Length = 13,
            .centerGravity = {0, 0, 9},
            .servoBRange = {10, 170},
            .servoCRange = {10, 170},
    };

    /* Left hand configuration */
    Leg leftHand = {
            .part1Length = 8,
            .part2Length = 8,
            .centerGravity = {0, 0, 9},
            .servoARange = {10, 170},
            .servoBRange = {10, 170},
            .servoCRange = {10, 170},
    };

    /* Right hand configuration */
    Leg rightHand = {
            .part1Length = 8,
            .part2Length = 8,
            .centerGravity = {0, 0, 9},
            .servoARange = {10, 170},
            .servoBRange = {10, 170},
            .servoCRange = {10, 170},
    };

    /* Creating variables for PCA9685 modules */
    PCA9685 pca9685_leg, pca9685_hand;
    
    volatile int state = 6;                                        // Variable for the robot's state
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
void StartServoControlTask(void *argument);
void StartUARTTask(void *argument);
void StartHandServoControlTask(void *argument);
void StartMotorControlTask(void *argument);

/* USER CODE BEGIN PFP */
    // PID Controller functions
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
        pid->max_integral = max_output * 2; // Integral limit
        pid->max_output = max_output;
    }

    // PID Controller update
    float PID_Update(PID_Controller* pid, float setpoint, float actual) {
        pid->setpoint = setpoint;
        pid->actual = actual;
        
        // Calculate error
        pid->error = pid->setpoint - pid->actual;
        
        // Integral component (with saturation)
        pid->integral += pid->error * pid->dt;
        
        // Anti-windup (integral limit)
        if (pid->integral > pid->max_integral) {
            pid->integral = pid->max_integral;
        } else if (pid->integral < -pid->max_integral) {
            pid->integral = -pid->max_integral;
        }
        
        // Derivative component
        pid->derivative = (pid->error - pid->prev_error) / pid->dt;
        
        // Calculate output signal
        pid->output = (pid->Kp * pid->error) + 
                      (pid->Ki * pid->integral) + 
                      (pid->Kd * pid->derivative);
        
        // Output signal limit
        if (pid->output > pid->max_output) {
            pid->output = pid->max_output;
        } else if (pid->output < -pid->max_output) {
            pid->output = -pid->max_output;
        }
        
        // Save error for next iteration
        pid->prev_error = pid->error;
        
        return pid->output;
    }

    // Reset integral component
    void PID_Reset(PID_Controller* pid) {
        pid->integral = 0;
        pid->prev_error = 0;
        pid->error = 0;
        pid->output = 0;
    }

    // Simplified motor control with PID (without encoders)
    void L298N_move_PID(Motor motor, int dir, float target_pwm, PID_Controller* pid, float current_pwm) {
        // Set direction
        switch(dir) {
            case -1:
                HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 1);
                HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
                target_pwm = -target_pwm; // Negative speed for reverse direction
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
        
        // Update PID and get output PWM value
        float pid_output = PID_Update(pid, target_pwm, current_pwm);
        
        // Convert to absolute PWM value
        int pwm_value = (int)fabs(pid_output);
        
        // PWM limit (max 1500, because timer period is 1500)
        if (pwm_value > 1500) pwm_value = 1500;
        if (pwm_value < 0) pwm_value = 0;
        
        // Set PWM
        __HAL_TIM_SET_COMPARE(motor.htim, motor.channel, pwm_value);
    }

    /* Motor rotation with speed control */
    void L298N_move(Motor motor, int dir, int speed) {
            // Switch depending on the direction of movement
            switch(dir) {
                    case -1:
                            // Move left
                            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 1);
                            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
                            break;
                    case 1:
                            // Move right
                            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
                            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 1);
                            break;
                    default:
                            // Stop
                            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
                            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
                            break;
            }
            // Set speed
            __HAL_TIM_SET_COMPARE(motor.htim, motor.channel, speed);
    }

    /* Motor rotation without speed control */
    void L298N_move_without_PWM(Motor motor, int dir) {
            switch(dir) {
                    case -1:
                            // Move left
                            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
                            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 1);
                            break;
                    case 1:
                            // Move right
                            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 1);
                            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
                            break;    
                    default:
                            // Stop
                            HAL_GPIO_WritePin(motor.GPIO_Port1, motor.GPIO_Pin1, 0);
                            HAL_GPIO_WritePin(motor.GPIO_Port2, motor.GPIO_Pin2, 0);
                            break;
            }
    }
    
    int getServoCAngle(int servoAngleD,int deltaAngleC,int deltaAngleD){
        int result = 180-(servoAngleD-deltaAngleD-angleAlpha+deltaAngleC);
        return result; 
    }

    // Function to send trigger pulse
    void HCSR04_Trigger()
    {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // TRIG - PA0
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    char* getSensorInfo(){

    int fake_temp = 0;   
    int fake_hum = 0;    

    reedModule = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);

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
        
    void setServoAngle(char leg,char servo, int angle){
            if(leg=='L'){
                switch(servo){
                    case 'B': leftLeg.calibrationAngle.B = angle;
                        break;
                    case 'C': leftLeg.calibrationAngle.C = angle;
                        break;
                }
            }else if(leg=='R'){
                switch(servo){
                    case 'B': rightLeg.calibrationAngle.B = angle;
                        break;
                    case 'C': rightLeg.calibrationAngle.C = angle;
                        break;
                }
            }else if(leg=='H'){
                switch(servo){
                    case 'A': leftHand.calibrationAngle.A = angle;
                        break;
                    case 'B': leftHand.calibrationAngle.B = angle;
                        break;
                    case 'C': leftHand.calibrationAngle.C = angle;
                        break;
                    case 'D': leftHand.calibrationAngle.D = angle;
                        break;
                    case 'E': leftHand.calibrationAngle.E = angle;
                        break;
                }
            }else if(leg=='M'){
                switch(servo){
                    case 'A': rightHand.calibrationAngle.A = angle;
                        break;
                    case 'B': rightHand.calibrationAngle.B = angle;
                        break;
                    case 'C': rightHand.calibrationAngle.C = angle;
                        break;
                    case 'D': rightHand.calibrationAngle.D = angle;
                        break;
                    case 'E': rightHand.calibrationAngle.E = angle;
                        break;
                }
            }
    }

    int getMaxAngle(ServosAngle current, ServosAngle target){
        // Only B and C servos are used for legs
        int deltaAngles[2] = {abs(current.B-target.B),abs(current.C-target.C)};
        int maxAngle = deltaAngles[0];    
        for(int i = 1;i<2;i++){
            if(maxAngle<deltaAngles[i])
                maxAngle=deltaAngles[i];
        }
        return maxAngle;
    }

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
		{
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
								// OVERFLOW: Ignore byte, reset counter, send error (once)
								if (error_counter == 0) {
										HAL_UART_Transmit_IT(huart, error_message, sizeof(error_message) - 1);
										error_counter++;
								}
								rx_counter = 0; 
						}

						HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
				}
		}
                                    
    static uint8_t wait_for_gpio_state_timeout(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, uint32_t timeout)
     {
            uint32_t Tickstart = HAL_GetTick();
            uint8_t ret = 1;

            for(;(state != HAL_GPIO_ReadPin(port, pin)) && (1 == ret);) // Wait until flag is set
            {
                    if(timeout != HAL_MAX_DELAY) // Check for the timeout
                    {
                            if((timeout == 0U) || ((HAL_GetTick() - Tickstart) > timeout)) ret = 0;
                    }

                    __NOP;
            }
            return ret;
    }

    /*----------------------------------------------------------------------------------------------------
    --The function to reset the Busy flag for the I2C bus must be executed after initializing each bus.
    ------------------------------------------------------------------------------------------------------*/
    static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *GPIOPortSCL, GPIO_TypeDef *GPIOPortSDA, uint32_t GPIO_SCl, uint32_t GPIO_SDA, uint32_t timeout)
    {
                    // 2.13.7 I2C analog filter may provide wrong value, locking BUSY. STM32F10xx8 STM32F10xxB Errata sheet

            GPIO_InitTypeDef GPIO_InitStructure = {0};

            // 1. Clear PE bit.
            CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_PE);

            //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
            HAL_I2C_DeInit(hi2c);

            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
            GPIO_InitStructure.Pull = GPIO_NOPULL;

            GPIO_InitStructure.Pin = GPIO_SCl; // SCL // if the pin is different, then indicate the desired one
            HAL_GPIO_Init(GPIOPortSCL, &GPIO_InitStructure); // if the port is different, then indicate the desired letter GPIOx, and below there, change all the ports and pins to yours

            GPIO_InitStructure.Pin = GPIO_SDA; // SDA
            HAL_GPIO_Init(GPIOPortSDA, &GPIO_InitStructure);

            // 3. Check SCL and SDA High level in GPIOx_IDR.
            HAL_GPIO_WritePin(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET);

            wait_for_gpio_state_timeout(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET, timeout);
            wait_for_gpio_state_timeout(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET, timeout);

            // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
            HAL_GPIO_WritePin(GPIOPortSDA, GPIO_SDA, GPIO_PIN_RESET);

            // 5. Check SDA Low level in GPIOx_IDR.
            wait_for_gpio_state_timeout(GPIOPortSDA, GPIO_SDA, GPIO_PIN_RESET, timeout);

            // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
            HAL_GPIO_WritePin(GPIOPortSCL, GPIO_SCl, GPIO_PIN_RESET);

            // 7. Check SCL Low level in GPIOx_IDR.
            wait_for_gpio_state_timeout(GPIOPortSCL, GPIO_SCl, GPIO_PIN_RESET, timeout);

            // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
            HAL_GPIO_WritePin(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET);

            // 9. Check SCL High level in GPIOx_IDR.
            wait_for_gpio_state_timeout(GPIOPortSCL, GPIO_SCl, GPIO_PIN_SET, timeout);

            // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
            HAL_GPIO_WritePin(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET);

            // 11. Check SDA High level in GPIOx_IDR.
            wait_for_gpio_state_timeout(GPIOPortSDA, GPIO_SDA, GPIO_PIN_SET, timeout);

            // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
            GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
            //GPIO_InitStructure.Alternate = GPIO_AF4_I2C2; // F4

            GPIO_InitStructure.Pin = GPIO_SCl;
            HAL_GPIO_Init(GPIOPortSCL, &GPIO_InitStructure);

            GPIO_InitStructure.Pin = GPIO_SDA;
            HAL_GPIO_Init(GPIOPortSDA, &GPIO_InitStructure);

            // 13. Set SWRST bit in I2Cx_CR1 register.
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
            __NOP;

            /* 14. Clear SWRST bit in I2Cx_CR1 register. */
            CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
            __NOP;

            /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_PE);
            __NOP;

            // Call initialization function.
            HAL_I2C_Init(hi2c);
    }

    static inline int8_t _sign(int val) {
        if (val <= 0) return -1;
        //if (val==0) return 0;
        return 1;
    }
		
    /*---------------------------------------------------------------------------------------
    --Function of rotating all servos to a given angle of the lower part (legs)
    --Left leg: PIN 6 (C), PIN 7 (B) on PCA9685
    --Right leg: PIN 15 (C), PIN 14 (B) on PCA9685
    ----------------------------------------------------------------------------------------*/
    void allServoLegSpin(PCA9685 pca9685, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle) {
            // Left leg servos (B and C)
            PCA9685_SetServoAngle(pca9685, 14, (left.C + correctAngle));  // Left leg servo C (PIN 6)
            PCA9685_SetServoAngle(pca9685, 15, left.B);                    // Left leg servo B (PIN 7)

            // Right leg servos (B and C)
            PCA9685_SetServoAngle(pca9685, 5, 180 - (right.C + correctAngle));  // Right leg servo C (PIN 15)
            PCA9685_SetServoAngle(pca9685, 4, 180 - right.B);                    // Right leg servo B (PIN 14)

            HAL_Delay(speedDelay);  // Delay sets the speed of rotation
    }
    
    /*---------------------------------------------------------------------------------------
    --Function of rotating all servos to a given angle of the upper part (hands)
    ----------------------------------------------------------------------------------------*/
    void allServoHandSpin(PCA9685 pca9685, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle) {
            PCA9685_SetServoAngle(pca9685, 4, left.A);  // Right hand servo A
            PCA9685_SetServoAngle(pca9685, 6, left.B);  // Right hand servo B
            PCA9685_SetServoAngle(pca9685, 5, (left.C + correctAngle));  // Right hand servo C
            // PCA9685_SetServoAngle(pca9685, 15, left.D);  // Right hand servo D (commented out)

            PCA9685_SetServoAngle(pca9685, 2, 180 - right.A);  // Left hand servo A
            PCA9685_SetServoAngle(pca9685, 1, 180 - right.B);  // Left hand servo B
            PCA9685_SetServoAngle(pca9685, 3, 180 - (right.C + correctAngle));  // Left hand servo C
            // PCA9685_SetServoAngle(pca9685, 15, 180 - right.D);  // Left hand servo D (commented out)

            HAL_Delay(speedDelay);  // Delay sets the speed of rotation
    }

    /*---------------------------------------------------------------------------------------
    --Comprehensive next step calculation for all servos
    ----------------------------------------------------------------------------------------*/
    ServosAngle getResultAngle( Leg leg,ServosAngle deltaResultAngles, int headTilt) {
            ServosAngle resultAngles={0,0};
            resultAngles.B= leg.currentAngle.B+deltaResultAngles.B;
            resultAngles.C= leg.currentAngle.C+deltaResultAngles.C+headTilt;
            return resultAngles;
    }
    
    /*Greeting animation function consisting of 4 frames for the leading hand*/
    ServosAngle greatingStateLead(int frame,bool reverse){
    ServosAngle    servosAngle = {0,0};
        
        switch(frame){
            case 0:                                                            
                servosAngle = setAngle(0,0,0,0,0);
                break;
            case 1:                                                            
                servosAngle = setAngle(0,-90,-60,0,0);
                break;
            case 2:    
                servosAngle = setAngle(0,-90,60,0,0);
                break;
            case 3: 
                servosAngle = setAngle(0,-90,-60,0,0);
                break;                
            case 4: 
                servosAngle = setAngle(0,0,0,0,0);
                break;            
        }
            if(reverse){
                    servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
                    servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
                    return servosAngle;
            }else{
                    servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
                    servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
                    return servosAngle;
            }
    }
    
    /*Greeting animation function consisting of 4 frames for the supporting hand*/
    ServosAngle greatingStateSupport(int frame,bool reverse){
        ServosAngle    servosAngle = {0,0};
        switch(frame){
            case 0: 
                servosAngle = setAngle(0,0,50,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,-40,50,0,0);
                break;
            case 2: 
                servosAngle = setAngle(0,0,50,0,0);
                break;
            case 3: 
                servosAngle = setAngle(0,-40,50,0,0);
                break;        
            case 4: 
                servosAngle = setAngle(0,0,0,0,0);
            break;
        }
            if(!reverse){
                    servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
                    servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
                    return servosAngle;
            }else{    
                    servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
                    servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
                    return servosAngle;        
            }
    }
    
    /*Animation 1 after 3 minutes of downtime for the lead leg*/
    ServosAngle hold3Anim1Lead(int frame, int turnLeft,int turnRight){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,0,-15,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftLeg.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftLeg.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 1 after 3 minutes of inactivity for the support leg*/
    ServosAngle hold3Anim1Support(int frame, int turnLeft,int turnRight){
        ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,0,-15,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightLeg.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightLeg.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 3 after 3 minutes of downtime for the dominant hand*/
    ServosAngle hold3Anim3Lead(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,20,60,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 3 after 3 minutes of downtime for support hand*/
    ServosAngle hold3Anim3Support(int frame){
        ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,20,60,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 1 after 10 minutes of inactivity for the lead leg*/
    ServosAngle hold10Anim1Lead_Leg(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,0,30,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftLeg.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftLeg.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 1 after 10 minutes of inactivity for the support leg*/
    ServosAngle hold10Anim1Support_Leg(int frame){
        ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,30,0,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightLeg.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightLeg.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 1 after 10 minutes of downtime for the dominant hand*/
    ServosAngle hold10Anim1Lead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-30,90,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 1 after 10 minutes of downtime for support hand*/
    ServosAngle hold10Anim1Support_Hand(int frame){
        ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-30,90,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 2 after 10 minutes of downtime for the dominant hand*/
    ServosAngle hold10Anim2Lead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-30,0,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 2 after 10 minutes of downtime for support hand*/
    ServosAngle hold10Anim2Support_Hand(int frame){
        ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-30,0,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*Animation 3 after 10 minutes of downtime for the dominant hand*/
    ServosAngle hold10Anim3Lead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,60,10,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,60,-10,0,0);
                break;
            case 2: 
                servosAngle = setAngle(0,60,10,0,0);
                break;
            case 3: 
                servosAngle = setAngle(0,60,-10,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    /*run animation*/
    ServosAngle runAnimLead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,60,60,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    ServosAngle runAnimSupport_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,60,60,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    ServosAngle runAnimLead_Leg(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,15,-5,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftLeg.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftLeg.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    ServosAngle runAnimSupport_Leg(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-5,20,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightLeg.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightLeg.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }

    /*spin animation*/
    ServosAngle spinAnimLead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-70,80,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    ServosAngle spinAnimSupport_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-85,80,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }

    /*box animation*/
    ServosAngle boxAnimLead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,-62,60,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,40,-30,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    ServosAngle boxAnimSupport_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0: 
                servosAngle = setAngle(0,50,-40,0,0);
                break;
            case 1: 
                servosAngle = setAngle(0,-65,65,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }

    /*dance animation*/
    ServosAngle danceAnimLead_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0:
            case 2:    
                servosAngle = setAngle(0,-50,90,0,0);
                break;
            case 1:
            case 3:    
                servosAngle = setAngle(0,-90,90,0,0);
                break;
            case 4:
                servosAngle = setAngle(0,-75,-45,0,0);
                break;
            case 5:
                servosAngle = setAngle(0,-75,90,0,0);
                break;
            case 6:
                servosAngle = setAngle(0,-90,70,0,0);
                break;
            case 7:
                servosAngle = setAngle(0,5,-5,0,0);
                break;
            case 8:
                servosAngle = setAngle(0,-90,70,0,0);
                break;
            case 9:
                servosAngle = setAngle(0,0,0,0,0);
                break;
            case 10:
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }
    
    ServosAngle danceAnimSupport_Hand(int frame){
    ServosAngle    servosAngle = {0,0};

        switch(frame){
            case 0:
            case 2:
                servosAngle = setAngle(0,-50,90,0,0);
                break;
            case 1: 
            case 3:
                servosAngle = setAngle(0,-90,90,0,0);
                break;
            case 4:
                servosAngle = setAngle(0,-75,90,0,0);
                break;
            case 5:
                servosAngle = setAngle(0,-75,-45,0,0);
                break;
            case 6:
                servosAngle = setAngle(0,0,0,0,0);
                break;
            case 7:
                servosAngle = setAngle(0,5,20,0,0);
                break;
            case 8:
                servosAngle = setAngle(0,0,0,0,0);
                break;
            case 9:
                servosAngle = setAngle(0,-90,70,0,0);
                break;
            case 10:
                servosAngle = setAngle(0,0,0,0,0);
                break;
        }
        servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
        servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
        return servosAngle;
    }

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */

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

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
        while (1)
        {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 127;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 44-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65450-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13 PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 PG2 PG4 PG6
                           PG13 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
    // Input Capture callback for TIM3 Channel 1
    void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
    {
            if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
            {
                    if (isFirstCaptured == 0)
                    {
                            icValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                            isFirstCaptured = 1;

                            // switch on Falling edge
                            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
                    }
                    else
                    {
                            icValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                            __HAL_TIM_SET_COUNTER(htim, 0);

                            if (icValue2 > icValue1)
                                    distance = (icValue2 - icValue1) * 0.0343f / 2.0f;
                            else
                                    distance = ((0xFFFF - icValue1) + icValue2) * 0.0343f / 2.0f;

                            isFirstCaptured = 0;

                            // back to Rising edge
                            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
                    }
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
        /* Infinite loop */
        for(;;)
        {
            osDelay(1);
        }
  /* USER CODE END 5 */
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
            bool legSwitch = false;

        leftLeg.defaultAngle = setAngle(87,100,120,90,90);
        leftLeg.currentAngle = leftLeg.defaultAngle;
        ServosAngle resultLeft = leftLeg.defaultAngle;

        rightLeg.defaultAngle = setAngle(104,80,95,90,90);
        rightLeg.currentAngle = rightLeg.defaultAngle;
        ServosAngle resultRight = rightLeg.defaultAngle;

        pca9685_leg.pca9685_i2c = &hi2c1;
        pca9685_leg.PCA9685_ADDRESS = PCA9685_ADDRESS2;

        while (PCA9685_Init(pca9685_leg) == 1) {};

        allServoLegSpin(pca9685_leg, 20, leftLeg.defaultAngle, rightLeg.defaultAngle, 0);
        
        for(;;)
        {
            degree90State = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
            if(degree90State==1){
                PCA9685_ResetAllChannels(pca9685_leg);
                PCA9685_SetServoAngle(pca9685_leg,10, 90);
                main_mode = 0;
            }
            else 
                if(main_mode==0){
                    PCA9685_ResetAllChannels(pca9685_leg);
                    main_mode = 1;
                }
            if(main_mode==1)    
            {
                int current_state = state;
                switch(current_state){
                case 10:
                    PCA9685_SetServoAngle(pca9685_leg,10, 90);
                break;
                case 6:
                    allServoLegSpin(pca9685_leg,20,leftLeg.defaultAngle,rightLeg.defaultAngle,0);
                break;
                case 1:
                break;                
                
                case 2:    {
                    if (hold3mod == 2){
                        for(int j = 0;j<4;j++){
                            for (int i = 0; i < 2; i++) {
                    
                                resultLeft = hold3Anim1Lead(i,1,0);
                                resultRight = hold3Anim1Support(i,1,0);

                                int movementSpeed = 40;
                                int maxAngleLeft = getMaxAngle(leftLeg.currentAngle,resultLeft);
                                int maxAngleRight = getMaxAngle(rightLeg.currentAngle,resultRight);
                                int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                                ServosAngle startLeftAngle = leftLeg.currentAngle;
                                ServosAngle startRightAngle = rightLeg.currentAngle;
                                for(int i =1;i<=maxAngle;i++){
                                    leftLeg.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                                    rightLeg.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                                    allServoLegSpin(pca9685_leg,movementSpeed,leftLeg.currentAngle,rightLeg.currentAngle,0);
                                }
                            }
                        }
                        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                        state=6;
                        osMutexRelease(UARTDataMutexHandle);
                    }
                break;
                }            

                case 3:    {    
                    if (hold10mod == 1 || hold10mod == 2) {
                            for (int i = 0; i < 2; i++) {
                                    resultLeft = hold10Anim1Lead_Leg(i);
                                    resultRight = hold10Anim1Support_Leg(i);
                                    int movementSpeed = 60;
                                    int maxAngleLeft = getMaxAngle(leftLeg.currentAngle, resultLeft);
                                    int maxAngleRight = getMaxAngle(rightLeg.currentAngle, resultRight);
                                    int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;

                                    ServosAngle startLeftAngle = leftLeg.currentAngle;
                                    ServosAngle startRightAngle = rightLeg.currentAngle;

                                    for (int i = 1; i <= maxAngle; i++) {
                                            leftLeg.currentAngle = getNextStepAll(startLeftAngle, resultLeft, maxAngle, i);
                                            rightLeg.currentAngle = getNextStepAll(startRightAngle, resultRight, maxAngle, i);
                                            allServoLegSpin(pca9685_leg, movementSpeed, leftLeg.currentAngle, rightLeg.currentAngle, 0);
                                    }
                                    osDelay(4000);
                            }
                    }
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    state=6;
                    osMutexRelease(UARTDataMutexHandle);
                break;
                }
                case 7:    {    
                    if((leftLeg.defaultAngle.C>(20-headDir)    && leftLeg.defaultAngle.C<(160-headDir)) &&(rightLeg.defaultAngle.C>(20-headDir)    && rightLeg.defaultAngle.C<(160-headDir))){
                        leftLeg.defaultAngle.C+=headDir;
                        rightLeg.defaultAngle.C+=headDir;
                    }
                    allServoLegSpin(pca9685_leg,100,leftLeg.defaultAngle,rightLeg.defaultAngle,0);
                    
                break;
                }
                case 8:    {    
                    int maxAngleLeft = getMaxAngle(leftLeg.currentAngle, leftLeg.calibrationAngle);
                    int maxAngleRight = getMaxAngle(rightLeg.currentAngle, rightLeg.calibrationAngle);
                    int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;

                    ServosAngle startLeftAngle = leftLeg.currentAngle;
                    ServosAngle startRightAngle = rightLeg.currentAngle;

                    for (int i = 1; i <= maxAngle; i++) {
                            leftLeg.currentAngle = getNextStepAll(startLeftAngle, leftLeg.calibrationAngle, maxAngle, i);
                            rightLeg.currentAngle = getNextStepAll(startRightAngle, rightLeg.calibrationAngle, maxAngle, i);
                            allServoLegSpin(pca9685_leg, 60, leftLeg.currentAngle, rightLeg.currentAngle, 0);
                    }
                break;
                }    
                case 9: {
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    leftLeg.calibrationAngle.B = calibrationAngles[1];
                    leftLeg.calibrationAngle.C = calibrationAngles[2];
                
                    leftLeg.defaultAngle.B = leftLeg.calibrationAngle.B;
                    leftLeg.defaultAngle.C = leftLeg.calibrationAngle.C;
                    leftLeg.currentAngle.B = leftLeg.calibrationAngle.B;
                    leftLeg.currentAngle.C = leftLeg.calibrationAngle.C;

                    rightLeg.calibrationAngle.B = calibrationAngles[6];
                    rightLeg.calibrationAngle.C = calibrationAngles[7];

                    rightLeg.defaultAngle.B = rightLeg.calibrationAngle.B;
                    rightLeg.defaultAngle.C = rightLeg.calibrationAngle.C;
                    rightLeg.currentAngle.B = rightLeg.calibrationAngle.B;
                    rightLeg.currentAngle.C = rightLeg.calibrationAngle.C;
                    
                    state = 6;
                    osMutexRelease(UARTDataMutexHandle);
                break;
            }
            }
        }
            osDelay(1);
            
            static int error_counter_local = 0;	
            if(rightLeg.currentAngle.C > 160 || rightLeg.currentAngle.C < 20) {
                error_counter_local++;
                if(error_counter_local > 10) {
                    PCA9685_ResetAllChannels(pca9685_leg);
                    allServoLegSpin(pca9685_leg,20,leftLeg.defaultAngle,rightLeg.defaultAngle,0);
                    error_counter_local = 0;
                }
            } else {
                error_counter_local = 0;
            }
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
        flag_send = 0;  // Sending messages is still prohibited, because there's nothing to send yet.
        queue_message = 1;  // The message queue is initially set to 1, meaning ready to send.
        rx_counter = 0;  // No characters have been received yet, so the counter is 0.
        size_message = 0;  // The size of the message to be sent is unknown, it is initially 0.
        error_counter = 0;  // No errors have occurred yet, so the error counter is 0.
        HAL_UART_Receive_IT(&huart1, rx_buffer, 1);  // Enable receiving 1 byte of data via UART interrupt.

        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // Set the TRIG pin low (usually for sensor triggering, such as ultrasonic).
        
        // Initialize command and parameter separators
        char commandSep[1] = "\n";  // Separator for commands, typically used for ending a command.
        char parameterSep[1] = "#";  // Separator for parameters, used to distinguish command arguments.
        char *command;  // Pointer to store the command.
        char *parametr;  // Pointer to store the parameter.
        /* Infinite loop */
        for(;;)
        {
            char* check=getSensorInfo();
            HCSR04_Trigger();
            //HAL_Delay(100); // pause

            if (flag_send == 1 && queue_message != 255) {  // If message sending is allowed and the queue is not full

            char* commands = (char*)tx_buffer;  // Copy values from tx_buffer into the 'commands' variable
            command = strtok(commands, commandSep);  // Extract the first command using the command separator
            while (command != NULL)  // Loop continues as long as there are commands
            {
                    parametr = strtok(command, parameterSep);  // Extract the command's name
                    while (parametr != NULL)  // Loop continues as long as there are parameters in the command
                    {
                            // Handling the ultrasonic sensor command
                            if (strcmp(parametr, "CMD_SONIC") == 0) {
                                    char buffer[10];
                                    sprintf(buffer, "%.2f\n", distance);
                                    if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {  // If the port is free and the message queue is ready
                                            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer, strlen(buffer));
                                    }
                            }
                            if (strcmp(parametr, "CMD_SENSORS") == 0) {
                                    char* data = getSensorInfo();  // Get sensors data
                                    if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {  // If the port is free and the message queue is ready
                                            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, strlen(data));
                                    }
                            }
                            if (strcmp(parametr, "CMD_LIGHT") == 0) {
										int red = 0, green = 0, blue = 0;
										parametr = strtok(NULL, parameterSep);  // Extract red parameter
                                    if (parametr) red = atoi(parametr);
                                    parametr = strtok(NULL, parameterSep);  // Extract green parameter
                                    if (parametr) green = atoi(parametr);
                                    parametr = strtok(NULL, parameterSep);  // Extract blue parameter
                                    if (parametr) blue = atoi(parametr);  // Convert angle to integer
										red = constrain (red,0,255);
										green = constrain (green,0,255);
										blue = constrain (blue,0,255);
                                                   
										if (red > 127) 
												HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
										else 
												HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

										
										if (green > 127) 
												HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
										else 
												HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);

										if (blue > 127) 
												HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
										else 
												HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
                            }
                            // Handling the "ping" command
                            else if (strcmp(parametr, "CMD_PING") == 0) {
                                    if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {  // If the port is free and the message queue is ready
                                            uint8_t str[] = "CMD_PING\n";  // Prepare "ping" message
                                            HAL_UART_Transmit_DMA(&huart1, str, 9);  // Send "ping" message via UART
                                            HAL_Delay(10);  // Wait for 10ms before proceeding
                                    }
                            }
                            // Handling the calibration command
                            else if (strcmp(parametr, "CMD_CALIBRATION") == 0) {
                                    parametr = strtok(NULL, parameterSep);  // Extract the leg number parameter
                                    char *leg = parametr;
                                    parametr = strtok(NULL, parameterSep);  // Extract the servo number parameter
                                    char *servo = parametr;
                                    parametr = strtok(NULL, parameterSep);  // Extract the servo angle parameter
                                    int angle = atoi(parametr);  // Convert angle to integer
                                    setServoAngle(*leg, *servo, angle);  // Set servo angle
                            }
                            // Handling calibration mode switch command
                            else if (strcmp(parametr, "CMD_CALIBRATION_MOD") == 0) {
                                    parametr = strtok(NULL, parameterSep);  // Extract the mode parameter
                                    int mod = atoi(parametr);
                                    if (mod == 1) {
                                            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                            state = 8;
                                            osMutexRelease(UARTDataMutexHandle);
                                    } else if (mod == 0) {
                                            // Save default angles for all servos
                                            saveNewDefoultAngle(&leftLeg);
                                            saveNewDefoultAngle(&rightLeg);
                                            saveNewDefoultAngle(&leftHand);
                                            saveNewDefoultAngle(&rightHand);
                                            osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                            state = 6;
                                            osMutexRelease(UARTDataMutexHandle);
                                    }
                            }
                            // Handling all servos default calibration command
                            else if (strcmp(parametr, "CMD_CALIBRATION_ALL") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    for (int i = 0; i < SERVO_COUNT; i++) {
                                            parametr = strtok(NULL, parameterSep);
                                            if(parametr) calibrationAngles[i] = atoi(parametr);
                                    }
                                    state = 9;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling movement forward command
                            else if (strcmp(parametr, "CMD_MOVE_FORWARD") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 1;
                                    turnLeft = 1;
                                    turnRight = 1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling movement backward command
                            else if (strcmp(parametr, "CMD_MOVE_BACKWARD") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 1;
                                    turnLeft = 0;
                                    turnRight = 0;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling movement left command
                            else if (strcmp(parametr, "CMD_MOVE_LEFT") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 1;
                                    turnLeft = 1;
                                    turnRight = 0;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling movement right command
                            else if (strcmp(parametr, "CMD_MOVE_RIGHT") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 1;
                                    turnLeft = 0;
                                    turnRight = 1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling stop movement command
                            else if (strcmp(parametr, "CMD_MOVE_STOP") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 6;
                                    turnLeft = 1;
                                    turnRight = 1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling command to clench right hand
                            else if (strcmp(parametr, "CMD_CLENCH_RIGHT") == 0) {
                                    parametr = strtok(NULL, parameterSep);
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    clenchRight = 1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling command to clench left hand
                            else if (strcmp(parametr, "CMD_CLENCH_LEFT") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    clenchLeft = 1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling command to look up
                            else if (strcmp(parametr, "CMD_LOOK_UP") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 7;
                                    headDir = -1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling command to look down
                            else if (strcmp(parametr, "CMD_LOOK_DOWN") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 7;
                                    headDir = 1;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling command to stop looking
                            else if (strcmp(parametr, "CMD_LOOK_STOP") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 6;
                                    headDir = 0;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling the 3-minute hold animation
                            else if (strcmp(parametr, "CMD_HOLD_3") == 0) {
                                    parametr = strtok(NULL, parameterSep);
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    hold3mod = atoi(parametr);
                                    state = 2;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling the 10-minute hold animation
                            else if (strcmp(parametr, "CMD_HOLD_10") == 0) {
                                    parametr = strtok(NULL, parameterSep);
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    hold10mod = atoi(parametr);
                                    state = 3;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Handling hands-up command
                            else if (strcmp(parametr, "CMD_HANDS_UP") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 5;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // Box animation
                            else if (strcmp(parametr, "CMD_BOX") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 11;
                                    osMutexRelease(UARTDataMutexHandle);
                            }
                            // dance animation
                            else if (strcmp(parametr, "CMD_DANCE") == 0) {
                                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                                    state = 12;
                                    osMutexRelease(UARTDataMutexHandle);
                            }

                            parametr = strtok(NULL, parameterSep);
                    }

                    // Extract the next command
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
void StartHandServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartHandServoControlTask */
        bool legSwitch = false;
        
        osDelay(500);
        
        leftHand.defaultAngle = setAngle(90, 90, 90, 90, 90);
        leftHand.currentAngle = leftHand.defaultAngle;
        ServosAngle resultLeft = leftHand.defaultAngle;

        rightHand.defaultAngle = setAngle(90, 90, 90, 90, 90);
        rightHand.currentAngle = rightHand.defaultAngle;
        ServosAngle resultRight = rightHand.defaultAngle;  

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
        
        for(;;)
        {
            int current_state = state;
            switch(current_state){
                case 1:
                    
                break;
                
                case 10:
                    PCA9685_SetServoAngle(pca9685_hand,10, 90);
                break;
                
                case 6:
                    if(!targetPositionCheck(leftHand.defaultAngle,leftHand.currentAngle)){
                        resultLeft = leftHand.defaultAngle;
                        resultRight = rightHand.defaultAngle;
                        int movementSpeed = 20;
                        int maxAngleLeft = getMaxAngle(leftHand.currentAngle, resultLeft);
                        int maxAngleRight = getMaxAngle(rightHand.currentAngle, resultRight);
                        int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
                        ServosAngle startLeftAngle = leftHand.currentAngle;
                        ServosAngle startRightAngle = rightHand.currentAngle;
                        
                        for (int i = 1; i <= maxAngle; i++) {
                                leftHand.currentAngle = getNextStepAll(startLeftAngle, resultLeft, maxAngle, i);
                                rightHand.currentAngle = getNextStepAll(startRightAngle, resultRight, maxAngle, i);
                                allServoHandSpin(pca9685_hand, movementSpeed, leftHand.currentAngle, rightHand.currentAngle, 0);
                        }
                    }
                break;
                
                case 2:
                    if (hold3mod == 3){
                            for (int i = 0; i < 4; i++) {
                                    resultLeft = hold3Anim3Lead(i);
                                    resultRight = hold3Anim3Support(i);
                                    int movementSpeed = 20;
                                    int maxAngleLeft = getMaxAngle(leftHand.currentAngle, resultLeft);
                                    int maxAngleRight = getMaxAngle(rightHand.currentAngle, resultRight);
                                    int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
                                    ServosAngle startLeftAngle = leftHand.currentAngle;
                                    ServosAngle startRightAngle = rightHand.currentAngle;
                                    
                                    for (int i = 1; i <= maxAngle; i++) {
                                            leftHand.currentAngle = getNextStepAll(startLeftAngle, resultLeft, maxAngle, i);
                                            rightHand.currentAngle = getNextStepAll(startRightAngle, resultRight, maxAngle, i);
                                            allServoHandSpin(pca9685_hand, movementSpeed, leftHand.currentAngle, rightHand.currentAngle, 0);
                                    }
                            }
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    state = 6;
                    osMutexRelease(UARTDataMutexHandle);
                    }
                    break;
                    
                case 3:
                    if (hold10mod == 1){
                        for (int i = 0; i < 2; i++) {
                            resultLeft = hold10Anim1Lead_Hand(i);
                            resultRight = hold10Anim1Support_Hand(i);
                            int movementSpeed = 20;
                            int maxAngleLeft = getMaxAngle(leftHand.currentAngle,resultLeft);
                            int maxAngleRight = getMaxAngle(rightHand.currentAngle,resultRight);
                            int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                            ServosAngle startLeftAngle = leftHand.currentAngle;
                            ServosAngle startRightAngle = rightHand.currentAngle;
                            for(int i =1;i<=maxAngle;i++){
                                leftHand.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                                rightHand.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                                allServoHandSpin(pca9685_hand,movementSpeed,leftHand.currentAngle,rightHand.currentAngle,0);
                            }
                        osDelay(4000);
                        }
                        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                        state=6;
                        osMutexRelease(UARTDataMutexHandle);
                    }
                    if (hold10mod == 2){
                        for (int i = 0; i < 2; i++) {
                            resultLeft = hold10Anim2Lead_Hand(i);
                            resultRight = hold10Anim2Support_Hand(i);
                            int movementSpeed = 20;
                            int maxAngleLeft = getMaxAngle(leftHand.currentAngle,resultLeft);
                            int maxAngleRight = getMaxAngle(rightHand.currentAngle,resultRight);
                            int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                            ServosAngle startLeftAngle = leftHand.currentAngle;
                            ServosAngle startRightAngle = rightHand.currentAngle;
                            for(int i =1;i<=maxAngle;i++){
                                leftHand.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                                rightHand.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                                allServoHandSpin(pca9685_hand,movementSpeed,leftHand.currentAngle,rightHand.currentAngle,0);
                            }
                        osDelay(4000);
                        }
                        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                        state=6;
                        osMutexRelease(UARTDataMutexHandle);
                    }
                    if (hold10mod == 3){
                        for (int i = 0; i < 4; i++) {
                            resultLeft = hold10Anim3Lead_Hand(i);
                            resultRight = rightHand.defaultAngle;
                            int movementSpeed = 20;
                            int maxAngleLeft = getMaxAngle(leftHand.currentAngle,resultLeft);
                            int maxAngleRight = getMaxAngle(rightHand.currentAngle,resultRight);
                            int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                            ServosAngle startLeftAngle = leftHand.currentAngle;
                            ServosAngle startRightAngle = rightHand.currentAngle;
                            for(int i =1;i<=maxAngle;i++){
                                leftHand.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                                rightHand.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                                allServoHandSpin(pca9685_hand,movementSpeed,leftHand.currentAngle,rightHand.currentAngle,0);
                            }
                        }
                        osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                        state=6;
                        osMutexRelease(UARTDataMutexHandle);
                    }
                    break;
                    
                case 5:
                    for (int i = 0; i < 5; i++) {
                        int leftLegDev=0;
                        int rightLegDev = 0;
                        int correctAngleL=0;
                        int correctAngleR=0;    

                        if(!legSwitch){
                            resultLeft = greatingStateLead(i,false);
                            resultRight = greatingStateSupport(i,false);
                        }else{
                            resultLeft = greatingStateSupport(i,true);
                            resultRight = greatingStateLead(i,true);
                        }
                        int movementSpeed = 5;
                        int maxAngleLeft = getMaxAngle(leftHand.currentAngle,resultLeft);
                        int maxAngleRight = getMaxAngle(rightHand.currentAngle,resultRight);
                        int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                        ServosAngle startLeftAngle = leftHand.currentAngle;
                        ServosAngle startRightAngle = rightHand.currentAngle;
                        for(int i =1;i<=maxAngle;i++){
                            leftHand.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                            rightHand.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                            allServoHandSpin(pca9685_hand,movementSpeed,leftHand.currentAngle,rightHand.currentAngle,0);
                        }

                    }
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    state = 6;
                    osMutexRelease(UARTDataMutexHandle);
                    break;
                    
                case 8:
                        allServoHandSpin(pca9685_hand,400,leftHand.calibrationAngle, rightHand.calibrationAngle,0);
                break;
                
                case 9:
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    leftHand.calibrationAngle.A = calibrationAngles[10];
                    leftHand.defaultAngle.A = leftHand.calibrationAngle.A;
                    leftHand.calibrationAngle.B = calibrationAngles[11];
                    leftHand.defaultAngle.B = leftHand.calibrationAngle.B;
                    leftHand.calibrationAngle.C = calibrationAngles[12];
                    leftHand.defaultAngle.C = leftHand.calibrationAngle.C;
                    leftHand.calibrationAngle.D = calibrationAngles[13];
                    leftHand.defaultAngle.D = leftHand.calibrationAngle.D;
                    leftHand.calibrationAngle.E = calibrationAngles[14];
                    leftHand.defaultAngle.E = leftHand.calibrationAngle.E;
                    
                    rightHand.calibrationAngle.A = calibrationAngles[15];
                    rightHand.defaultAngle.A = rightHand.calibrationAngle.A;
                    rightHand.calibrationAngle.B = calibrationAngles[16];
                    rightHand.defaultAngle.B = rightHand.calibrationAngle.B;
                    rightHand.calibrationAngle.C = calibrationAngles[17];
                    rightHand.defaultAngle.C = rightHand.calibrationAngle.C;
                    rightHand.calibrationAngle.D = calibrationAngles[18];
                    rightHand.defaultAngle.D = rightHand.calibrationAngle.D;
                    rightHand.calibrationAngle.E = calibrationAngles[19];
                    rightHand.defaultAngle.E = rightHand.calibrationAngle.E;
                    osMutexRelease(UARTDataMutexHandle);
                break;
                
                case 11:
                    for (int j=0;j<3;j++){
                        for (int k = 0; k < 2; k++) {
                            resultLeft = boxAnimLead_Hand(k);
                            resultRight = boxAnimSupport_Hand(k);
                            int movementSpeed = 5;
                            int maxAngleLeft = getMaxAngle(leftHand.currentAngle,resultLeft);
                            int maxAngleRight = getMaxAngle(rightHand.currentAngle,resultRight);
                            int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                            ServosAngle startLeftAngle = leftHand.currentAngle;
                            ServosAngle startRightAngle = rightHand.currentAngle;
                            for(int i =1;i<=maxAngle;i++){
                                leftHand.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                                rightHand.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                                allServoHandSpin(pca9685_hand,movementSpeed,leftHand.currentAngle,rightHand.currentAngle,0);
                            }
                        osDelay(100);
                        }
                    }
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    state=6;
                    osMutexRelease(UARTDataMutexHandle);
                break;    
                
                case 12:
                    for (int k = 0; k < 10; k++) {
                        resultLeft = danceAnimLead_Hand(k);
                        resultRight = danceAnimSupport_Hand(k);
                        int movementSpeed = 10;
                        int maxAngleLeft = getMaxAngle(leftHand.currentAngle,resultLeft);
                        int maxAngleRight = getMaxAngle(rightHand.currentAngle,resultRight);
                        int maxAngle = maxAngleLeft>maxAngleRight ? maxAngleLeft : maxAngleRight;
                        ServosAngle startLeftAngle = leftHand.currentAngle;
                        ServosAngle startRightAngle = rightHand.currentAngle;
                        for(int i =1;i<=maxAngle;i++){
                            leftHand.currentAngle = getNextStepAll(startLeftAngle,resultLeft,maxAngle,i);
                            rightHand.currentAngle = getNextStepAll(startRightAngle,resultRight,maxAngle,i);
                            allServoHandSpin(pca9685_hand,movementSpeed,leftHand.currentAngle,rightHand.currentAngle,0);
                        }
                    }
                    osMutexAcquire(UARTDataMutexHandle, osWaitForever);
                    state=6;
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
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
        int delayInterval = 8000;
        int timerLeft,timerRight;
        int dirLeft=-1,dirRight=-1;
        
        // PID controllers initialization
        // Coefficients are selected experimentally
        // Kp=2.0, Ki=0.1, Kd=0.05, dt=0.01s (update every 10ms), max_output=1500
        PID_Init(&pid_motor1, 2.0f, 0.1f, 0.05f, 0.01f, 1500.0f);
        PID_Init(&pid_motor2, 2.0f, 0.1f, 0.05f, 0.01f, 1500.0f);
        
        // Last PID update time
        last_pid_update = HAL_GetTick();
        
        // Connecting motors in the caterpillars with speed control
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // PA1 TIM2 CH2
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // PA1 TIM2 CH3
        //L298N_move_without_PWM(motor_hand1, dirLeft);
        //L298N_move_without_PWM(motor_hand2, dirRight);
        osDelay(8000);

        // Connecting motors in the hands
        L298N_move_without_PWM(motor_hand1, 0);
        L298N_move_without_PWM(motor_hand2, 0);

        /* Infinite loop */
        for(;;)
        {
            // Calculate time for PID
            uint32_t current_time = HAL_GetTick();
            float dt = (current_time - last_pid_update) / 1000.0f; // in seconds
            
            // Get current PWM values (if needed for PID)
            float current_pwm1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
            float current_pwm2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
            
            // Update PID every 10ms
            if (dt >= 0.01f) {
                last_pid_update = current_time;
                
                int current_state = state;
                // If state requires movement with PID control
                if (current_state == 1) {
                    // Control with PID
                    // FORWARD
                    if(turnLeft == 1 && turnRight == 1) {
                        L298N_move_PID(motor1, 1, target_speed1, &pid_motor1, current_pwm1);
                        L298N_move_PID(motor2, 1, target_speed2, &pid_motor2, current_pwm2);
                    } 
                    // BACKWARD  
                    else if(turnLeft == 0 && turnRight == 0) {
                        L298N_move_PID(motor1, -1, target_speed1, &pid_motor1, current_pwm1);
                        L298N_move_PID(motor2, -1, target_speed2, &pid_motor2, current_pwm2);
                    }
                    // TURN LEFT
                    else if(turnLeft == 1 && turnRight == 0) {
                        L298N_move_PID(motor1, -1, target_speed1, &pid_motor1, current_pwm1);
                        L298N_move_PID(motor2, 1, target_speed2, &pid_motor2, current_pwm2);
                    }
                    // TURN RIGHT
                    else if(turnLeft == 0 && turnRight == 1) {
                        L298N_move_PID(motor1, 1, target_speed1, &pid_motor1, current_pwm1);
                        L298N_move_PID(motor2, -1, target_speed2, &pid_motor2, current_pwm2);
                    }
                } else {
                    // For other states use normal control
                    if (current_state == 4) {
                        if (hold3mod == 1) {
                            // Rotate in place left
                            if(turnLeft == 1 && turnRight == 0) {
                                L298N_move(motor1, 1, 400);
                                L298N_move(motor2, -1, 700);
                            } 
                            // Rotate in place right
                            else if(turnLeft == 0 && turnRight == 1) {
                                L298N_move(motor1, -1, 400);
                                L298N_move(motor2, 1, 700);
                            }
                        }
                    }
                    else if (current_state == 12) {
                        // For dance use fixed values without PID
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
                    }
                    else if (current_state == 6) {
                        // Stop - reset PID
                        L298N_move(motor1, 0, 0);
                        L298N_move(motor2, 0, 0);
                        PID_Reset(&pid_motor1);
                        PID_Reset(&pid_motor2);
                    }
                }
            }

            // Opening/closing manipulator on the left hand
            if(clenchLeft == 1) {
                    timerLeft = HAL_GetTick();
                    dirLeft *= -1;
                    clenchLeft = -1;
            }

            // Opening/closing manipulator on the right hand
            if(clenchRight == 1) {
                    timerRight = HAL_GetTick();
                    dirRight *= -1;
                    clenchRight = -1;
            }

            // Control left hand motor
            if(HAL_GetTick() - timerLeft < delayInterval) {
                    L298N_move_without_PWM(motor_hand1, dirLeft);
            } else {
                    L298N_move_without_PWM(motor_hand1, 0);
            }

            // Control right hand motor
            if(HAL_GetTick() - timerRight < delayInterval) {
                    L298N_move_without_PWM(motor_hand2, dirRight);
            } else {
                    L298N_move_without_PWM(motor_hand2, 0);
            }

            osDelay(10); // Delay for PID controller (10ms)
        }
  /* USER CODE END StartMotorControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
        
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
        __disable_irq();
        while (1)
        {
        }
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
             ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */