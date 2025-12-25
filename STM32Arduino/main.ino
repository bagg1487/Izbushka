#include "config.h"
#include "gyro.h"
#include "ultrasonic.h"
#include "servoControl.h"
#include "joystick.h"
#include "motorControl.h"

void setup() {
    Serial.begin(115200);
    while(!Serial);
    delay(1000);
    
    Serial.println("=== STM32F4 Robot Shield ===");
    
    setupUltrasonic();
    setupMPU6050();
    setupServoControl();
    setupMotorControl();
    
    Serial.println("System ready");
}

void loop() {
    static unsigned long lastSensorRead = 0;
    
    unsigned long now = millis();
    
    if(now - lastSensorRead >= 1000) {
        lastSensorRead = now;
        
        float distance = measureDistance();
        readMPU6050();
        
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.print(" cm");
        Serial.println();
    }
    
    updateServoControl();
    
    state = 1;
    
}