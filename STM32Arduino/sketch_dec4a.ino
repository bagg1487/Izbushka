
#include <Wire.h>

#define TRIG_PIN PA6
#define ECHO_PIN PA7

#define MPU6050_ADDR 0x68
#define MPU6050_ADDR_ALT 0x69

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temp;

bool mpuConnected = false;

void setup() {
  Serial1.begin(9600);
  delay(1000);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PC13, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(PC13, LOW);
  
  Serial1.println("Инициализация системы...");
  
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  
  checkMPU6050();
  
  if (mpuConnected) {
    initMPU6050();
    Serial1.println("MPU6050 инициализирован успешно");
  } else {
    Serial1.println("MPU6050 не обнаружен!");
  }
  
  Serial1.println("Система: HC-SR04 + MPU6050");
  Serial1.println("==========================");
}

void checkMPU6050() {
  byte address = MPU6050_ADDR;
  
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial1.print("MPU6050 найден по адресу 0x");
    Serial1.println(address, HEX);
    mpuConnected = true;
    return;
  }
  
  address = MPU6050_ADDR_ALT;
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  
  if (error == 0) {
    Serial1.print("MPU6050 найден по альтернативному адресу 0x");
    Serial1.println(address, HEX);
    mpuConnected = true;
  }
}

void initMPU6050() {
  if (!mpuConnected) return;
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); 
  Wire.endTransmission();
  
  delay(100);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); 
  Wire.write(0x00);
  Wire.endTransmission();
  
  delay(100);
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
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int timeout = 1000; 
  while (Wire.available() < 14 && timeout-- > 0) {
    delay(1);
  }
  
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

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long timeout = micros() + 25000;
  while (digitalRead(ECHO_PIN) == LOW && micros() < timeout) {}
  
  unsigned long startTime = micros();
  while (digitalRead(ECHO_PIN) == HIGH && micros() < timeout) {}
  unsigned long travelTime = micros() - startTime;
  
  float distance = travelTime * 0.034 / 2.0;
  
  if (distance < 2.0 || distance > 400.0) {
    return 0.0;
  }
  
  return distance;
}

void loop() {
  static int counter = 0;
  
  digitalWrite(PC13, HIGH);
  
  float distance = measureDistance();
  
  readMPU6050();
  
  digitalWrite(PC13, LOW);
  
  counter++;
  
  Serial1.print("№");
  Serial1.print(counter);
  Serial1.print(" | ");
  
  if (distance > 0) {
    Serial1.print("Расстояние: ");
    Serial1.print(distance, 1);
    Serial1.print(" см [");
    
    if (distance < 10) {
      Serial1.print("ОЧЕНЬ БЛИЗКО");
    } else if (distance < 30) {
      Serial1.print("близко");
    } else if (distance < 100) {
      Serial1.print("нормально");
    } else {
      Serial1.print("далеко");
    }
    Serial1.print("]");
  } else {
    Serial1.print("Нет объекта");
  }
  
  Serial1.print(" | MPU: ");
  
  if (mpuConnected) {
    float accX_g = accX / 16384.0;
    float accY_g = accY / 16384.0;
    float accZ_g = accZ / 16384.0;
    
    float gyroX_dps = gyroX / 131.0;
    float gyroY_dps = gyroY / 131.0;
    float gyroZ_dps = gyroZ / 131.0;
    
    Serial1.print("G(");
    Serial1.print(gyroX_dps, 1);
    Serial1.print(",");
    Serial1.print(gyroY_dps, 1);
    Serial1.print(",");
    Serial1.print(gyroZ_dps, 1);
    Serial1.print(") A(");
    Serial1.print(accX_g, 2);
    Serial1.print(",");
    Serial1.print(accY_g, 2);
    Serial1.print(",");
    Serial1.print(accZ_g, 2);
    Serial1.print(")");
  } else {
    Serial1.print("НЕ ПОДКЛЮЧЕН");
  }
  
  Serial1.println();
  
  delay(500);
}
