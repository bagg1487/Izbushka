#include "gyro.h"

TwoWire myWire(PB9, PB8);

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temp;
bool mpuConnected = false;

void setupMPU6050() {
  myWire.begin();
  myWire.setClock(100000);
  delay(100);
  
  myWire.beginTransmission(MPU6050_ADDR);
  byte error = myWire.endTransmission();
  
  if (error == 0) {
    mpuConnected = true;
    
    myWire.beginTransmission(MPU6050_ADDR);
    myWire.write(0x6B);
    myWire.write(0x00);
    myWire.endTransmission();
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
  
  myWire.beginTransmission(MPU6050_ADDR);
  myWire.write(0x3B);
  myWire.endTransmission(false);
  myWire.requestFrom(MPU6050_ADDR, 14);
  
  if (myWire.available() >= 14) {
    accX = myWire.read() << 8 | myWire.read();
    accY = myWire.read() << 8 | myWire.read();
    accZ = myWire.read() << 8 | myWire.read();
    temp = myWire.read() << 8 | myWire.read();
    gyroX = myWire.read() << 8 | myWire.read();
    gyroY = myWire.read() << 8 | myWire.read();
    gyroZ = myWire.read() << 8 | myWire.read();
  }
}