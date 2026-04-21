#include "servoControl.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TRACK_CENTER_LEFT   92
#define TRACK_CENTER_RIGHT  88
#define TRACK_MIN_OFFSET   -25
#define TRACK_MAX_OFFSET    25
#define ANGLE_CORRECTION -17

extern int turnLeft;
extern int turnRight;
extern int state;
extern int clenchLeft;
extern int clenchRight;
extern uint32_t timerLeft;
extern uint32_t timerRight;
extern int dirLeft;
extern int dirRight;


Leg leftLeg;
Leg rightLeg;
Leg leftHand;
Leg rightHand;

int main_mode = 1;
int hold3mod = 0;
int hold10mod = 0;
int headDir = 0;
int degree90State = 0;
int state = 0;

Adafruit_PWMServoDriver pca9685_leg(PCA9685_ADDR2);
Adafruit_PWMServoDriver pca9685_hand(PCA9685_ADDR1);

int calibrationAngles[20];
const uint32_t GRIPPER_INTERVAL = 8000;

void PCA9685_SetServoAngle(Adafruit_PWMServoDriver &pwm, uint8_t channel, float angle) {
    angle = constrain(angle, 0.0, 180.0);
    #define PCA9685_SERVO_MIN 150
    #define PCA9685_SERVO_MAX 490
    float value_f = PCA9685_SERVO_MIN + (angle * (PCA9685_SERVO_MAX - PCA9685_SERVO_MIN) / 180.0);
    int value = (int)value_f;
    value = constrain(value, 0, 4095);
    pwm.setPWM(channel, 0, value);
}

void PCA9685_ResetAllChannels(Adafruit_PWMServoDriver &pwm) {
    for(int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);
    }
}

void setTrackTilt(Adafruit_PWMServoDriver &pwm, int offset) {
    offset = constrain(offset, TRACK_MIN_OFFSET, TRACK_MAX_OFFSET);
    PCA9685_SetServoAngle(pwm, 7, TRACK_CENTER_LEFT + offset);
    PCA9685_SetServoAngle(pwm, 14, TRACK_CENTER_RIGHT + offset);
}

void allServoLegSpin(
    Adafruit_PWMServoDriver &pwm,
    int speedDelay,
    ServosAngle left,
    ServosAngle right,
    int correctAngle
) {
    PCA9685_SetServoAngle(pwm, 12, right.A);
    PCA9685_SetServoAngle(pwm, 13, right.B);
    PCA9685_SetServoAngle(pwm, 15, right.D);
    PCA9685_SetServoAngle(pwm, 6, left.B);
    PCA9685_SetServoAngle(pwm, 5, left.C + correctAngle);
    PCA9685_SetServoAngle(pwm, 4, left.D);
    delay(speedDelay);
}

void allServoHandSpin(
    Adafruit_PWMServoDriver &pwm,
    int speedDelay,
    ServosAngle left,
    ServosAngle right,
    int correctAngle
) {
    PCA9685_SetServoAngle(pwm, 4, left.A);
    PCA9685_SetServoAngle(pwm, 6, left.B);
    PCA9685_SetServoAngle(pwm, 5, left.C + correctAngle);
    PCA9685_SetServoAngle(pwm, 2, right.A);
    PCA9685_SetServoAngle(pwm, 1, right.B);
    PCA9685_SetServoAngle(pwm, 3, right.C + correctAngle);
    delay(speedDelay);
}

ServosAngle setAngle(int A, int B, int C, int D, int E) {
    ServosAngle a;
    a.A = A;
    a.B = B;
    a.C = C;
    a.D = D;
    a.E = E;
    return a;
}

void setupServoControl() {
    Wire.begin();
    Wire.setSDA(PB_7);
    Wire.setSCL(PB_6);
    pca9685_leg.begin();
    pca9685_leg.setPWMFreq(48);
    pca9685_hand.begin();
    pca9685_hand.setPWMFreq(48);
    delay(100);
    for(int i = 0; i < 16; i++) {
        PCA9685_SetServoAngle(pca9685_leg, i, 90 + ANGLE_CORRECTION);
        PCA9685_SetServoAngle(pca9685_hand, i, 90 + ANGLE_CORRECTION);
    }
    delay(800);
    leftLeg.defaultAngle = setAngle(88 + ANGLE_CORRECTION, 
                                    94 + ANGLE_CORRECTION, 
                                    140 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION);
    leftLeg.currentAngle = leftLeg.defaultAngle;
    leftLeg.calibrationAngle = leftLeg.defaultAngle;
    rightLeg.defaultAngle = setAngle(97 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION, 
                                    96 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION);
    rightLeg.currentAngle = rightLeg.defaultAngle;
    rightLeg.calibrationAngle = rightLeg.defaultAngle;
    leftHand.defaultAngle = setAngle(90 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION, 
                                    20 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION, 
                                    90 + ANGLE_CORRECTION);
    leftHand.currentAngle = leftHand.defaultAngle;
    leftHand.calibrationAngle = leftHand.defaultAngle;
    rightHand.defaultAngle = setAngle(165 + ANGLE_CORRECTION, 
                                    50 + ANGLE_CORRECTION, 
                                    20 + ANGLE_CORRECTION, 
                                    50 + ANGLE_CORRECTION, 
                                    50 + ANGLE_CORRECTION);
    rightHand.currentAngle = rightHand.defaultAngle;
    rightHand.calibrationAngle = rightHand.defaultAngle;
}

void updateServoControl() {
    if(main_mode != 1) return;
    if(state == 6) {
        allServoLegSpin(pca9685_leg, 20, leftLeg.currentAngle, rightLeg.currentAngle, 0);
        allServoHandSpin(pca9685_hand, 20, leftHand.currentAngle, rightHand.currentAngle, 0);
    }
    if(state == 7) {
        static int tilt = 0;
        tilt += headDir * 2;
        setTrackTilt(pca9685_leg, tilt);
    }
}