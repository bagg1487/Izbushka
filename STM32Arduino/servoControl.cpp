#include "servoControl.h"

Adafruit_PWMServoDriver pwm1;
Adafruit_PWMServoDriver pwm2;
int servoAngles[16] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
int motorASpeed = 0;
int motorBSpeed = 0;

void setupServoMotor() {
  pwm1 = Adafruit_PWMServoDriver(PCA9685_ADDR1);
  pwm2 = Adafruit_PWMServoDriver(PCA9685_ADDR2);
  
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  
  for (int i = 0; i < 16; i++) {
    setServoAngle(i, 90);
  }
  
  pinMode(MOTOR_A_IA, OUTPUT);
  pinMode(MOTOR_A_IB, OUTPUT);
  pinMode(MOTOR_B_IA, OUTPUT);
  pinMode(MOTOR_B_IB, OUTPUT);
  
  analogWrite(MOTOR_A_IA, 0);
  analogWrite(MOTOR_A_IB, 0);
  analogWrite(MOTOR_B_IA, 0);
  analogWrite(MOTOR_B_IB, 0);
  
  delay(500);
}

void updateServoMotor() {
  static unsigned long lastServoUpdate = 0;
  static unsigned long lastMotorUpdate = 0;
  unsigned long now = millis();
  
  if (now - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdate = now;
    
    for (int i = 0; i < NUM_SERVOS_I2C1; i++) {
      int pulse = map(servoAngles[i], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
      pwm1.setPWM(i, 0, pulse);
    }
    
    for (int i = 0; i < NUM_SERVOS_I2C2; i++) {
      int pulse = map(servoAngles[i + NUM_SERVOS_I2C1], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
      pwm2.setPWM(i, 0, pulse);
    }
  }
  
  if (now - lastMotorUpdate >= LOOP_DELAY) {
    lastMotorUpdate = now;
    
    int motorAPWM = abs(motorASpeed);
    int motorBPWM = abs(motorBSpeed);
    
    if (motorASpeed > MOTOR_DEADZONE) {
      analogWrite(MOTOR_A_IA, motorAPWM);
      analogWrite(MOTOR_A_IB, 0);
    } else if (motorASpeed < -MOTOR_DEADZONE) {
      analogWrite(MOTOR_A_IA, 0);
      analogWrite(MOTOR_A_IB, motorAPWM);
    } else {
      analogWrite(MOTOR_A_IA, 0);
      analogWrite(MOTOR_A_IB, 0);
    }
    
    if (motorBSpeed > MOTOR_DEADZONE) {
      analogWrite(MOTOR_B_IA, motorBPWM);
      analogWrite(MOTOR_B_IB, 0);
    } else if (motorBSpeed < -MOTOR_DEADZONE) {
      analogWrite(MOTOR_B_IA, 0);
      analogWrite(MOTOR_B_IB, motorBPWM);
    } else {
      analogWrite(MOTOR_B_IA, 0);
      analogWrite(MOTOR_B_IB, 0);
    }
  }
}

void setServoAngle(uint8_t servoNum, int angle) {
  if (servoNum >= 16) return;
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  servoAngles[servoNum] = angle;
}

void setMotorSpeed(uint8_t motorNum, int speed) {
  speed = constrain(speed, -255, 255);
  if (motorNum == 0) {
    motorASpeed = speed;
  } else if (motorNum == 1) {
    motorBSpeed = speed;
  }
}

void updateFromJoystick() {
  static uint8_t activeServo = 0;
  static uint8_t activeMotor = 0;
  static bool lastButtonX = false;
  static bool lastButtonY = false;
  
  if (buttonX && !lastButtonX) {
    activeServo = (activeServo + 1) % 16;
    Serial.print("Сервопривод: ");
    Serial.println(activeServo);
  }
  lastButtonX = buttonX;
  
  if (buttonY && !lastButtonY) {
    activeMotor = (activeMotor + 1) % 2;
    Serial.print("Мотор: ");
    Serial.println(activeMotor);
  }
  lastButtonY = buttonY;
  
  if (joystickY != 0) {
    int delta = map(joystickY, -32767, 32767, -5, 5);
    int newAngle = servoAngles[activeServo] + delta;
    newAngle = constrain(newAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    servoAngles[activeServo] = newAngle;
  }
  
  int speed = 0;
  if (buttonA) {
    speed = 200;
  } else if (buttonB) {
    speed = -200;
  }
  
  setMotorSpeed(activeMotor, speed);
}