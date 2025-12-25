#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>

typedef struct {
    int A, B, C, D, E;
} ServosAngle;

typedef struct {
    ServosAngle defaultAngle;
    ServosAngle currentAngle;
    ServosAngle calibrationAngle;
} Leg;

typedef struct {
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t en_pin;
    int current_speed;
    int current_direction;
} Motor;

extern Motor motor1;
extern Motor motor2;
extern Motor motor_hand1;
extern Motor motor_hand2;

extern Leg leftLeg;
extern Leg rightLeg;
extern Leg leftHand;
extern Leg rightHand;

extern int state;
extern int main_mode;
extern int turnLeft;
extern int turnRight;
extern int hold3mod;
extern int hold10mod;
extern int clenchLeft;
extern int clenchRight;
extern int headDir;
extern int degree90State;

extern Adafruit_PWMServoDriver pca9685_leg;
extern Adafruit_PWMServoDriver pca9685_hand;

extern int calibrationAngles[20];

void setupServoControl();
void updateServoControl();
void L298N_move(Motor motor, int dir, int speed);
void L298N_move_without_PWM(Motor motor, int dir);
ServosAngle setAngle(int A, int B, int C, int D, int E);
void setServoAngle(char leg, char servo, int angle);
int getMaxAngle(ServosAngle current, ServosAngle target);
ServosAngle getNextStepAll(ServosAngle start, ServosAngle target, int maxAngle, int step);
void allServoLegSpin(Adafruit_PWMServoDriver &pwm, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle);
void allServoHandSpin(Adafruit_PWMServoDriver &pwm, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle);
void PCA9685_SetServoAngle(Adafruit_PWMServoDriver &pwm, uint8_t channel, int angle);
void PCA9685_ResetAllChannels(Adafruit_PWMServoDriver &pwm);

#endif