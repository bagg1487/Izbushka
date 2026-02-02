/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */
//#include "cmsis_os.h"
#include "main.h"
#include <stdint.h>
//#include "stm32f0xx_hal_def.h"
/*#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"*/
//#include "stm32f1xx_hal_i2c.h"

// Kalman structure
typedef struct
{
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

		Kalman_t KalmanX, KalmanY;
	
		double KalmanOldDataX;
		double KalmanOldDataY;
	
    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;


uint8_t MPU6050_Init(uint32_t MPU6050_ADDR_num, I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(uint32_t MPU6050_ADDR,I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(uint32_t MPU6050_ADDR,I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(uint32_t MPU6050_ADDR_num, I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(uint32_t MPU6050_ADDR_num, I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
