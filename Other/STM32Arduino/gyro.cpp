#include "gyro.h"
#include <Wire.h>
#include <Arduino.h>

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temp;
bool mpuConnected = false;

void setupMPU6050() {
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  
  Wire.beginTransmission(MPU6050_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    mpuConnected = true;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
  }
}

void readMPU6050() {
  if (!mpuConnected) {
    accX = accY = accZ = 0;
    gyroX = gyroY = gyroZ = 0;
    temp = 0;
    return;
  }
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  if (Wire.available() >= 14) {
    accX = Wire.read() << 8 | Wire.read();
    accY = Wire.read() << 8 | Wire.read();
    accZ = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
  }
}