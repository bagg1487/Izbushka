#include "main.h"
#include "L298NDriver.h"
#include "stm32f4xx_hal.h"


void L298N_move_without_PMW(Motor motor, int dir){
	switch(dir){
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

void L298N_move_all(Motor motor1, int dir1, int speed1,Motor motor2, int dir2, int speed2){
	switch(dir1){
		case -1:
			HAL_GPIO_WritePin(motor1.GPIO_Port1, motor1.GPIO_Pin1, 0);
			HAL_GPIO_WritePin(motor1.GPIO_Port2, motor1.GPIO_Pin2, 1);
			break;
		case 1:
			HAL_GPIO_WritePin(motor1.GPIO_Port1, motor1.GPIO_Pin1, 1);
			HAL_GPIO_WritePin(motor1.GPIO_Port2, motor1.GPIO_Pin2, 0);
			break;	
		default:
			HAL_GPIO_WritePin(motor1.GPIO_Port1, motor1.GPIO_Pin1, 0);
			HAL_GPIO_WritePin(motor1.GPIO_Port2, motor1.GPIO_Pin2, 0);
			break;
	}
		switch(dir2){
		case -1:
			HAL_GPIO_WritePin(motor2.GPIO_Port1, motor2.GPIO_Pin1, 0);
			HAL_GPIO_WritePin(motor2.GPIO_Port2, motor2.GPIO_Pin2, 1);
			break;
		case 1:
			HAL_GPIO_WritePin(motor2.GPIO_Port1, motor2.GPIO_Pin1, 1);
			HAL_GPIO_WritePin(motor2.GPIO_Port2, motor2.GPIO_Pin2, 0);
			break;	
		default:
			HAL_GPIO_WritePin(motor2.GPIO_Port1, motor2.GPIO_Pin1, 0);
			HAL_GPIO_WritePin(motor2.GPIO_Port2, motor2.GPIO_Pin2, 0);
			break;
	}
	__HAL_TIM_SET_COMPARE(motor1.htim,motor1.channel, speed1);
	__HAL_TIM_SET_COMPARE(motor2.htim,motor2.channel, speed2);
}