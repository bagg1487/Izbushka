#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include "config.h"

extern int16_t accX, accY, accZ;
extern int16_t gyroX, gyroY, gyroZ;
extern int16_t temp;
extern bool mpuConnected;

void setupMPU6050();
void readMPU6050();

#endif