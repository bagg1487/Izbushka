#include "config.h"
#include "gyro.h"
#include "ultrasonic.h"
#include "servoControl.h"
#include "joystick.h"

void setup() {
    Serial.begin(115200);
    while(!Serial);
    delay(1000);
    
    Serial.println("=== STM32F4 Robot Shield ===");
    
    setupUltrasonic();
    setupMPU6050();
    setupServoControl();
    
    Serial.println("System ready");
}

void loop() {
    static unsigned long lastSensorRead = 0;
    static unsigned long lastUpdate = 0;
    
    unsigned long now = millis();
    
    if(now - lastSensorRead >= 100) {
        lastSensorRead = now;
        
        float distance = measureDistance();
        readMPU6050();
        
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print(" cm | MPU: ");
        if(mpuConnected) {
            Serial.print(accX);
            Serial.print(",");
            Serial.print(accY);
            Serial.print(",");
            Serial.print(accZ);
        } else {
            Serial.print("NO MPU");
        }
        Serial.print(" | State: ");
        Serial.print(state);
        Serial.print(" | Left C: ");
        Serial.print(leftLeg.currentAngle.C);
        Serial.print(" | Right C: ");
        Serial.print(rightLeg.currentAngle.C);
        Serial.println();
    }
    
    if(now - lastUpdate >= 20) {
        lastUpdate = now;
        updateServoControl();
    }
    
    delay(10);
}