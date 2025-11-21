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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "pca9685.h"
	#include "mpu6050.h"
	#include "LegControl.h"
	#include "servo.h"
	#include <math.h>
	#include <stdlib.h>
	#include <stdbool.h>
	#include <stdio.h>
	#include <string.h> // for strtok
	#include <stdlib.h>

	#include <string.h>
	#include "DHT.h"
	//#include "L298NDriver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M_PI 3.141592


#define SERVO_COUNT	20 //number of servos used

#define firstAddressMPU6050 0xD0 // first i2c address for MPU6050 module
#define secondAddressMPU6050 0xD2 // second i2c address for MPU6050 module

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
#define BUFFER_SIZE 32


typedef struct
{
    GPIO_TypeDef *GPIO_Port1;   // Pointer to GPIO port for the first pin
    uint32_t GPIO_Pin1;         // GPIO pin number for the first pin
    GPIO_TypeDef *GPIO_Port2;   // Pointer to GPIO port for the second pin
    uint32_t GPIO_Pin2;         // GPIO pin number for the second pin
    TIM_HandleTypeDef *htim;    // Pointer to TIM_HandleTypeDef structure for timer operations
    uint32_t channel;           // Timer channel number
} Motor;


servo_t servoLA, servoLB, servoLC, servoLD, servoLE,
				servoRA, servoRB, servoRC, servoRD, servoRE;

int state = 6;                                        // Variable for the hut's state
/*Temp and liquid*/
	DHT_data dht_data;
	static DHT_sensor dht = {GPIOB, GPIO_PIN_4, DHT11, GPIO_NOPULL};
	char msg[40]; 
	/*reed switch*/
	int reedModule=0;
	/*light sensor*/
	int lightSensor = 0;

/* UART */
#define size_tx_buffer 100                            // Length of the tx buffer
bool flag_send;                                       // Flag to allow tx_buffer[] output to the serial port
uint8_t queue_message;                                // Message queue (since it works with interrupts or DMA. See lesson 6)
uint8_t tx_buffer[size_tx_buffer];                    // tx_buffer to output to the port whatever the MCU receives via rx
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

/* USER CODE BEGIN PV */

#define gyroMemorySize 5


MPU6050_t LeftLegGyroData, RightLegGyroData, BodyGyroData;
/* Motor structure initialization */

/* Leg motors with speed control */
Motor motor1 = {
    .GPIO_Port1 = GPIOG,            // Port for the first pin
    .GPIO_Pin1 = GPIO_PIN_1,        // Pin number for the first pin
    .GPIO_Port2 = GPIOE,            // Port for the second pin
    .GPIO_Pin2 = GPIO_PIN_7,        // Pin number for the second pin
    .htim = &htim1,                 // Timer for PWM
    .channel = TIM_CHANNEL_2        // PWM channel for the timer
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
    .GPIO_Pin1 = GPIO_PIN_10,       // Pin number for the first pin
    .GPIO_Port2 = GPIOB,            // Port for the second pin
    .GPIO_Pin2 = GPIO_PIN_11,       // Pin number for the second pin
};

double angleXLocal;      // Local deviation angle on the X-axis
double angleYLocal;      // Local deviation angle on the Y-axis
double hHatch;           // Height of the center of mass when in a falling state
double lStep;            // Step length needed to bring the center of mass to the balance point
double degreeY;
// double accelX;
double accelXLocal;
double pid;
double absY;             // Absolute value of the calculated angle
int Angle = 90;
uint8_t ActiveServo;

float servoAngleChange[6][gyroMemorySize]; // Array to track the change in gyroscope angles (6 = 3 gyroscopes * 2 axes)

uint8_t HTU21D_RX_Data[3];
float HTU21D_Temperature;
uint16_t HTU21D_ADC_Raw;
uint8_t HTU21D_Temp_Cmd = 0xE3;
#define HTU21D_Address (0x40 << 1)

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
/* Left leg configuration */
Leg leftLeg = {
    .part1Length = 10,
    .part2Length = 13,
    .centerGravity = {0, 0, 9},
    .servoARange = {10, 170},
    .servoBRange = {10, 170},
    .servoCRange = {10, 170},
};

/* Right leg configuration */
Leg rightLeg = {
    .part1Length = 10,
    .part2Length = 13,
    .centerGravity = {0, 0, 9},
    .servoARange = {10, 170},
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* Motor rotation with speed control */
void L298N_move(Motor motor, int dir, int speed) {
    // Switch depending on the direction of movement
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
/*uint16_t getDistance(void) {
    // Send pulse to measure distance
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // Set TRIG to HIGH
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (__HAL_TIM_GET_COUNTER(&htim3) < 10);  // Wait 10 µs

    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // Set TRIG to LOW

    pMillis = HAL_GetTick(); // Use this to avoid an infinite loop (for timeout)
    // Wait until ECHO goes HIGH
    while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());

    Value1 = __HAL_TIM_GET_COUNTER(&htim3);

    pMillis = HAL_GetTick(); // Use this to avoid an infinite loop (for timeout)
    // Wait until ECHO goes LOW
    while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());

    Value2 = __HAL_TIM_GET_COUNTER(&htim3);

    // Calculate distance
    Distance = (Value2 - Value1) * 0.034 / 2;
    return Distance;
}*/

// Функция посылки триггер-импульса
void HCSR04_Trigger()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // TRIG - PA0
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

char* getSensorInfo(){
		dht_data = DHT_getData(&dht); 
	
		reedModule = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);

		HAL_ADC_Start(&hadc1);  // ADC start
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  
    lightSensor = HAL_ADC_GetValue(&hadc1);  
		int light = round((lightSensor / 4095.0) * 100);  
    HAL_ADC_Stop(&hadc1);  // ADC stop
		dist = distance;
		int smoke_sensor=0;
		int touch_sensor = 0;
        // Выводим значение в UART
    sprintf(msg, "CMD_SENSORS#%d#%d#%d#%d#%.2f#%d#%d\n",(uint8_t)dht_data.temp, (uint8_t)dht_data.hum, reedModule,light,dist,smoke_sensor,touch_sensor);
		return msg;
}
	
void setServoAngle(char leg,char servo, int angle){
		if(leg=='L'){
			switch(servo){
				case 'A': leftLeg.calibrationAngle.A = angle;
					break;
				case 'B': leftLeg.calibrationAngle.B = angle;
					break;
				case 'C': leftLeg.calibrationAngle.C = angle;
					break;
				case 'D': leftLeg.calibrationAngle.D = angle;
					break;
				case 'E': leftLeg.calibrationAngle.E = angle;
					break;
			}
		}else if(leg=='R'){
			switch(servo){
				case 'A': rightLeg.calibrationAngle.A = angle;
					break;
				case 'B': rightLeg.calibrationAngle.B = angle;
					break;
				case 'C': rightLeg.calibrationAngle.C = angle;
					break;
				case 'D': rightLeg.calibrationAngle.D = angle;
					break;
				case 'E': rightLeg.calibrationAngle.E = angle;
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

	int deltaAngles[5] = {abs(current.A-target.A),abs(current.B-target.B),abs(current.C-target.C),abs(current.D-target.D),abs(current.E-target.E)};
	int maxAngle = deltaAngles[0];	
	for(int i = 1;i<5;i++){
		if(maxAngle<deltaAngles[i])
			maxAngle=deltaAngles[i];
	}
	return maxAngle;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // A character has been received, the interrupt is triggered, and here we will handle the received data.
    if (rx_buffer[0] != '\r') { // If the incoming character is not '\r'
        tx_buffer[rx_counter] = rx_buffer[0]; // Add this character to tx_buffer
        rx_counter++; // Increment the count of incoming characters
        size_message = rx_counter; // Set the message size to be sent to the serial port
        if (size_message >= size_tx_buffer && error_counter == 0) { // If message size exceeds buffer size, display buffer overflow message
            HAL_UART_Transmit_DMA(&huart1, error_message,
                                  sizeof error_message / sizeof error_message[0]);
            error_counter++; // Error message counter to send only once
            flag_send = 0; // Disallow message output to the port due to buffer overflow
        }
    } else if (rx_buffer[0] == '\r') { // If the incoming character is '\r'
        tx_buffer[rx_counter] = '\n'; // Our message is fully received. Add a newline character
        // tx_buffer[rx_counter + 1] = '\r'; // Optionally add carriage return
        // tx_buffer[rx_counter + 2] = '\0'; // Manually add a null terminator since it does not add itself
        size_message = rx_counter + 1; // Increase the size of the message sent to the port
        rx_counter = 0; // Reset the incoming character counter to receive a new message
        if (size_message >= size_tx_buffer) { // Again, if message size exceeds buffer, show buffer overflow message
            flag_send = 0; // Disallow output to the port due to overflow
            HAL_UART_Transmit_DMA(&huart1, error_message,
                                  sizeof error_message / sizeof error_message[0]);
        } else {
            flag_send = 1; // Allow message output to the port if within buffer size
            error_counter = 0; // Reset error counter
        }
    }
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1); // Restart data reception after each interrupt
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
--Conversion of acceleration from raw data to m/s^2
----------------------------------------------------------------------------------------*/
double accelCalc(double accelRaw){
	double acceX = constrain(accelRaw, -16384, 16384)/ 16384.0;    // limit +-1g
  double accel = acceX*9.82 ;           // convert в +-1.0
	return accel;

}

/*---------------------------------------------------------------------------------------
--Conversion function from angles to milliseconds (PWM)
----------------------------------------------------------------------------------------*/
float degToMS(int deg){
	return (float)deg*(2430-550)/180+550;
}
/*---------------------------------------------------------------------------------------
--Function of rotating all servos to a given angle of the lower part
----------------------------------------------------------------------------------------*/
void allServoLegSpin(PCA9685 pca9685, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle) {
    PCA9685_SetServoAngle(pca9685, 12, left.A);  // Right leg servo A
    PCA9685_SetServoAngle(pca9685, 13, left.B);  // Right leg servo B
    PCA9685_SetServoAngle(pca9685, 14, (left.C + correctAngle));  // Right leg servo C
    PCA9685_SetServoAngle(pca9685, 15, left.D);  // Right leg servo D

    PCA9685_SetServoAngle(pca9685, 7, 180 - right.A);  // Left leg servo A
    PCA9685_SetServoAngle(pca9685, 6, 180 - right.B);  // Left leg servo B
    PCA9685_SetServoAngle(pca9685, 5, 180 - (right.C + correctAngle));  // Left leg servo C
    PCA9685_SetServoAngle(pca9685, 4, 180 - right.D);  // Left leg servo D

    HAL_Delay(speedDelay);  // Delay sets the speed of rotation
}
/*---------------------------------------------------------------------------------------
--Function of rotating all servos to a given angle of the upper part
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

/*----------------------------------------------------------------------------------------------------
-- Function for calculating the angle of 3 sides of a triangle in degrees
------------------------------------------------------------------------------------------------------*/
int angleCalc(double a, double b, double c) {

	double acosValue = acos((a*a + b*b - c*c) / (2 * a * b));
	int result = ceil(acosValue * 180 / M_PI);
  return result;
}

/*---------------------------------------------------------------------------------------
--Comprehensive next step calculation for all servos
----------------------------------------------------------------------------------------*/
ServosAngle getResultAngle( Leg leg,ServosAngle deltaResultAngles, int headTilt) {
		ServosAngle resultAngles={0,0,0,0};
		resultAngles.A= leg.currentAngle.A+deltaResultAngles.A;
		resultAngles.B= leg.currentAngle.B+deltaResultAngles.B;
		resultAngles.C= leg.currentAngle.C+deltaResultAngles.C+headTilt;
		resultAngles.D= leg.currentAngle.D+deltaResultAngles.D;
	
	
}
/*Greeting animation function consisting of 4 frames for the leading hand*/
ServosAngle greatingStateLead(int frame,bool reverse){
ServosAngle	servosAngle = {0,0,0,0,0};
	
	switch(frame){
		case 0: 															
			servosAngle = setAngle(0,0,0,0,0);
			break;
		case 1: 															
			servosAngle = setAngle(-90,-60,-45,0,0);
			break;
		case 2: 	
			servosAngle = setAngle(-90,-60,45,0,0);
			break;
		case 3: 
			servosAngle = setAngle(-90,-60,-45,0,0);
			break;				
		case 4: 
			servosAngle = setAngle(0,0,0,0,0);
			break;			
	}
		if(reverse){
				servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
				servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
				servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
			  servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
				return servosAngle;
		}else{
				servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
				servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
				servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
				servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
			  return servosAngle;
		}
}
/*Greeting animation function consisting of 4 frames for the supporting hand*/
ServosAngle greatingStateSupport(int frame,bool reverse){
	ServosAngle	servosAngle = {0,0,0,0,0};
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
				servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
				servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
				servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
				servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
				 //servosAngle1 = getResultAngle(rightLeg,servosAngle,headTilt);
				return servosAngle;
		}else{	
				servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
				servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
				servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
				servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
				//servosAngle1 = getResultAngle(leftLeg,servosAngle,headTilt);
				return servosAngle;		
		}
}
/*Animation 1 after 3 minutes of downtime for the lead leg*/
ServosAngle hold3Anim1Lead(int frame, int turnLeft,int turnRight){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,0,-15,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftLeg.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftLeg.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftLeg.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftLeg.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 1 after 3 minutes of inactivity for the support leg*/
ServosAngle hold3Anim1Support(int frame, int turnLeft,int turnRight){
	ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,0,-15,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightLeg.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightLeg.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightLeg.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightLeg.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 3 after 3 minutes of downtime for the dominant hand*/
ServosAngle hold3Anim3Lead(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,20,60,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 3 after 3 minutes of downtime for support hand*/
ServosAngle hold3Anim3Support(int frame){
	ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,20,60,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 1 after 10 minutes of inactivity for the lead leg*/
ServosAngle hold10Anim1Lead_Leg(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,0,30,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftLeg.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftLeg.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftLeg.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftLeg.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 1 after 10 minutes of inactivity for the support leg*/
ServosAngle hold10Anim1Support_Leg(int frame){
	ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,30,0,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightLeg.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightLeg.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightLeg.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightLeg.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 1 after 10 minutes of downtime for the dominant hand*/
ServosAngle hold10Anim1Lead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(-30,0,90,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 1 after 10 minutes of downtime for support hand*/
ServosAngle hold10Anim1Support_Hand(int frame){
	ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(-30,0,90,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 2 after 10 minutes of downtime for the dominant hand*/
ServosAngle hold10Anim2Lead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(-30,0,0,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 2 after 10 minutes of downtime for support hand*/
ServosAngle hold10Anim2Support_Hand(int frame){
	ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(-30,0,0,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*Animation 3 after 10 minutes of downtime for the dominant hand*/
ServosAngle hold10Anim3Lead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(60,0,10,0,0);
			break;
		case 1: 
			servosAngle = setAngle(60,0,-10,0,0);
			break;
		case 2: 
			servosAngle = setAngle(60,0,10,0,0);
			break;
		case 3: 
			servosAngle = setAngle(60,0,-10,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
/*run animation*/
ServosAngle runAnimLead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(60,5,60,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
ServosAngle runAnimSupport_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(60,5,60,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
ServosAngle runAnimLead_Leg(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(5,15,-5,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftLeg.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftLeg.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftLeg.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftLeg.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
ServosAngle runAnimSupport_Leg(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(5,-5,20,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightLeg.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightLeg.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightLeg.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightLeg.defaultAngle.D+servosAngle.D;
	return servosAngle;
}


/*spin animation*/
ServosAngle spinAnimLead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,-70,80,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
ServosAngle spinAnimSupport_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(0,-85,80,0,0);
			break;
		case 1: 
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}


/*box animation*/
ServosAngle boxAnimLead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(-62,10,60,0,0);
			break;
		case 1: 
			servosAngle = setAngle(40,10,-30,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
ServosAngle boxAnimSupport_Hand(int frame){

ServosAngle	servosAngle = {0,0,0,0,0};

	switch(frame){
		case 0: 
			servosAngle = setAngle(50,0,-40,0,0);
			break;
		case 1: 
			servosAngle = setAngle(-65,5,65,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}

	
/*dance animation*/
ServosAngle danceAnimLead_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

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
			servosAngle = setAngle(-90,0,70,0,0);
			break;
		case 7:
			servosAngle = setAngle(5,15,-5,0,0);
			break;
		case 8:
			servosAngle = setAngle(-90,-75,70,0,0);
			break;
		case 9:
			servosAngle = setAngle(0,0,0,0,0);
			break;
		case 10:
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= leftHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= leftHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= leftHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= leftHand.defaultAngle.D+servosAngle.D;
	return servosAngle;
}
ServosAngle danceAnimSupport_Hand(int frame){
ServosAngle	servosAngle = {0,0,0,0,0};

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
			servosAngle = setAngle(5,-5,20,0,0);
			break;
		case 8:
			servosAngle = setAngle(0,0,0,0,0);
			break;
		case 9:
			servosAngle = setAngle(-90,-75,70,0,0);
			break;
		case 10:
			servosAngle = setAngle(0,0,0,0,0);
			break;
	}
	servosAngle.A= rightHand.defaultAngle.A+servosAngle.A;
	servosAngle.B= rightHand.defaultAngle.B+servosAngle.B;
	servosAngle.C= rightHand.defaultAngle.C+servosAngle.C;
	servosAngle.D= rightHand.defaultAngle.D+servosAngle.D;
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
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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

/* USER CODE BEGIN 4 */
// Callback Input Capture для TIM3 Channel 1
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
