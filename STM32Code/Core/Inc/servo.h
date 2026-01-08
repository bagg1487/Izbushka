/*
 * servo.h
 *
 *		The MIT License.
 *  Created on: 01.02.2019
 *		Author: Mateusz Salamon
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/MAX7219_matrix_STM32_HAL
 */

#ifndef SERVO_H_
#define SERVO_H_
#include "stm32f4xx_hal_tim.h"
//
//	Servo
//
#define TURNIGY_TG9E

//
//	Defines
//

// For Turnigy TG9e
#ifdef	TURNIGY_TG9E
#define SERVO_MIN_GPIO 550
#define SERVO_MAX_GPIO 2430
#define ANGLE_MIN_GPIO 0
#define ANGLE_MAX_GPIO 180
#endif

typedef struct {
    TIM_HandleTypeDef* htim;
    int channel;
} servo_t;

void Servo_Init(servo_t *servo, TIM_HandleTypeDef *_htim, int _channel);
void Servo_SetAngle(servo_t *servo, int angle);
void Servo_SetAngleFine(servo_t *servo, float angle);
void Servo_SetAnglePWM(servo_t *servo, int angle);
#endif /* SERVO_H_ */
