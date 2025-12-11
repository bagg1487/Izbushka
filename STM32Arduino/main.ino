#include "config.h"
#include "gyro.h"
#include "ultrasonic.h"
#include "joystick.h"
#include "servoControl.h"

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("=== Система управления ===");
  
  setupUltrasonic();
  setupMPU6050();
  setupJoystick();
  setupServoMotor();
  
  Serial.println("Система готова");
}

void loop() {
  static int counter = 0;
  
  digitalWrite(LED_PIN, HIGH);
  
  float distance = measureDistance();
  readMPU6050();
  
  bool joyConnected = checkJoystick();
  if (joyConnected) {
    processJoystick();
  }
  
  digitalWrite(LED_PIN, LOW);
  
  counter++;
  
  Serial.print("№");
  Serial.print(counter);
  Serial.print(" | Dist: ");
  
  if (distance > 0) {
    Serial.print(distance, 1);
    Serial.print("cm");
  } else {
    Serial.print("---");
  }
  
  Serial.print(" | MPU: ");
  
  if (mpuConnected) {
    float gx = gyroX / 131.0;
    float gy = gyroY / 131.0;
    float gz = gyroZ / 131.0;
    
    float ax = accX / 16384.0;
    float ay = accY / 16384.0;
    float az = accZ / 16384.0;
    
    Serial.print("G(");
    Serial.print(gx, 1);
    Serial.print(",");
    Serial.print(gy, 1);
    Serial.print(",");
    Serial.print(gz, 1);
    Serial.print(") A(");
    Serial.print(ax, 2);
    Serial.print(",");
    Serial.print(ay, 2);
    Serial.print(",");
    Serial.print(az, 2);
    Serial.print(")");
  } else {
    Serial.print("NO MPU");
  }
  
  if (joyConnected) {
    Serial.print(" | Joy: ");
    Serial.print("X:");
    Serial.print(joystickX);
    Serial.print(" Y:");
    Serial.print(joystickY);
    
    if (joystickConnected) {
      updateFromJoystick();
    }
  } else {
    Serial.print(" | NO JOY");
  }
  
  Serial.println();
  
  updateServoMotor();
  
  delay(LOOP_DELAY);
}