#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "motorControl.h"

typedef struct {
    int A, B, C, D, E;
} ServosAngle;

typedef struct {
    ServosAngle defaultAngle;
    ServosAngle currentAngle;
    ServosAngle calibrationAngle;
} Leg;

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
extern uint32_t timerLeft;
extern uint32_t timerRight;
extern int dirLeft;
extern int dirRight;
extern const uint32_t GRIPPER_INTERVAL;

void setupServoControl();
void updateServoControl();

void PCA9685_SetServoAngle(Adafruit_PWMServoDriver &pwm, uint8_t channel, float angle);
void PCA9685_ResetAllChannels(Adafruit_PWMServoDriver &pwm);
void setTrackTilt(Adafruit_PWMServoDriver &pwm, int offset);

void allServoLegSpin(
    Adafruit_PWMServoDriver &pwm,
    int speedDelay,
    ServosAngle left,
    ServosAngle right,
    int correctAngle
);

void allServoHandSpin(
    Adafruit_PWMServoDriver &pwm,
    int speedDelay,
    ServosAngle left,
    ServosAngle right,
    int correctAngle
);

ServosAngle setAngle(int A, int B, int C, int D, int E);

#endif