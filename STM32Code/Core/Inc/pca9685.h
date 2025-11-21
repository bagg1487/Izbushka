/*
 * pca9685.h
 *
 *  Created on: 20.01.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/Servos_PWM_STM32_HAL
 */

#ifndef PCA9685_H_
#define PCA9685_H_
//#include "cmsis_os.h"
#include "main.h"
//
//	Enable Servo control mode
//
#define PCA9685_SERVO_MODE

#ifdef PCA9685_SERVO_MODE
//
//	Servo min and max values for TURINGY TG9e Servos
//
#define SERVO_MIN	100
#define SERVO_MAX	460
#define MIN_ANGLE	0.0
#define MAX_ANGLE	180.0
#endif

//
//	Adjustable address 0x80 - 0xFE
//
#define PCA9685_ADDRESS1 0x80//следующий адрес 0x82
#define PCA9685_ADDRESS2 0x82
//
//	Registers
//
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 		0x0
#define PCA9685_PRESCALE 	0xFE

#define PCA9685_LED0_ON_L 	0x6
#define PCA9685_LED0_ON_H	0x7
#define PCA9685_LED0_OFF_L	0x8
#define PCA9685_LED0_OFF_H 	0x9

#define PCA9685_ALLLED_ON_L 	0xFA
#define PCA9685_ALLLED_ON_H 	0xFB
#define PCA9685_ALLLED_OFF_L 	0xFC
#define PCA9685_ALLLED_OFF_H 	0xFD

#define PCA9685_MODE1_ALCALL_BIT	0
typedef enum
{
	PCA9685_MODE1_SUB1_BIT 	= 3,
	PCA9685_MODE1_SUB2_BIT	= 2,
	PCA9685_MODE1_SUB3_BIT	= 1
}SubaddressBit;
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7


typedef enum
{
	PCA9685_OK 		= 0,
	PCA9685_ERROR	= 1
}PCA9685_STATUS;
typedef struct{
	
	I2C_HandleTypeDef *pca9685_i2c;
	uint16_t PCA9685_ADDRESS;
}PCA9685;
PCA9685_STATUS PCA9685_SoftwareReset(PCA9685 pca9685);
PCA9685_STATUS PCA9685_SleepMode(PCA9685 pca9685,uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(PCA9685 pca9685,uint8_t Enable);
PCA9685_STATUS PCA9685_AutoIncrement(PCA9685 pca9685,uint8_t Enable);
void PCA9685_WriteRegister(PCA9685 pca9685,uint8_t reg, uint8_t value);
void PCA9685_ResetAllChannels(PCA9685 pca9685);
#ifndef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetPwmFrequency(PCA9685 pca9685,uint16_t Frequency);
#endif

PCA9685_STATUS PCA9685_SetPwm(PCA9685 pca9685,uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(PCA9685 pca9685,uint8_t Channel, uint16_t Value, uint8_t Invert);
#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(PCA9685 pca9685, uint8_t Channel, float Angle);
#endif

PCA9685_STATUS PCA9685_Init(PCA9685 pca9685);

#endif /* PCA9685_H_ */
