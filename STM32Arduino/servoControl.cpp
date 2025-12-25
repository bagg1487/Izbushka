#include "servoControl.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Motor motor1 = {PG_1, PE_7, PE_11, 0, 0};
Motor motor2 = {PF_13, PF_14, PE_13, 0, 0};
Motor motor_hand1 = {PE_14, PE_15, 255, 0, 0};
Motor motor_hand2 = {PB_10, PB_11, 255, 0, 0};

Leg leftLeg;
Leg rightLeg;
Leg leftHand;
Leg rightHand;

int state = 6;
int main_mode = 1;
int turnLeft = 0;
int turnRight = 0;
int hold3mod = 0;
int hold10mod = 0;
int clenchLeft = 0;
int clenchRight = 0;
int headDir = 0;
int degree90State = 0;

Adafruit_PWMServoDriver pca9685_leg(PCA9685_ADDR2);
Adafruit_PWMServoDriver pca9685_hand(PCA9685_ADDR1);

int calibrationAngles[20];

uint32_t timerLeft = 0;
uint32_t timerRight = 0;
int dirLeft = -1;
int dirRight = -1;
const uint32_t GRIPPER_INTERVAL = 8000;

void setupServoControl() {
    Wire.begin();
    Wire.setSDA(PB_7);
    Wire.setSCL(PB_6);
    
    pca9685_leg.begin();
    pca9685_leg.setPWMFreq(50);
    
    pca9685_hand.begin();
    pca9685_hand.setPWMFreq(50);
    
    pinMode(PG_1, OUTPUT);
    pinMode(PE_7, OUTPUT);
    pinMode(PF_13, OUTPUT);
    pinMode(PF_14, OUTPUT);
    pinMode(PE_11, OUTPUT);
    pinMode(PE_13, OUTPUT);
    pinMode(PE_14, OUTPUT);
    pinMode(PE_15, OUTPUT);
    pinMode(PB_10, OUTPUT);
    pinMode(PB_11, OUTPUT);
    
    digitalWrite(PG_1, LOW);
    digitalWrite(PE_7, LOW);
    digitalWrite(PF_13, LOW);
    digitalWrite(PF_14, LOW);
    digitalWrite(PE_11, LOW);
    digitalWrite(PE_13, LOW);
    digitalWrite(PE_14, LOW);
    digitalWrite(PE_15, LOW);
    digitalWrite(PB_10, LOW);
    digitalWrite(PB_11, LOW);
    
    for(int i = 0; i < 16; i++) {
        PCA9685_SetServoAngle(pca9685_leg, i, 90);
        PCA9685_SetServoAngle(pca9685_hand, i, 90);
    }
    
    delay(1000);
    
    leftLeg.defaultAngle = setAngle(88, 94, 140, 90, 90);
    leftLeg.currentAngle = leftLeg.defaultAngle;
    leftLeg.calibrationAngle = setAngle(88, 94, 140, 90, 90);
    
    rightLeg.defaultAngle = setAngle(97, 90, 90, 96, 90);
    rightLeg.currentAngle = rightLeg.defaultAngle;
    rightLeg.calibrationAngle = setAngle(97, 90, 90, 96, 90);
    
    leftHand.defaultAngle = setAngle(90, 90, 20, 90, 90);
    leftHand.currentAngle = leftHand.defaultAngle;
    leftHand.calibrationAngle = setAngle(90, 90, 20, 90, 90);
    
    rightHand.defaultAngle = setAngle(165, 50, 20, 50, 50);
    rightHand.currentAngle = rightHand.defaultAngle;
    rightHand.calibrationAngle = setAngle(165, 50, 20, 50, 50);
    
    delay(1000);
    
    Serial.println("Setup complete");
}

void updateServoControl() {
    degree90State = digitalRead(PB_12);
    
    if(degree90State == 1) {
        PCA9685_ResetAllChannels(pca9685_leg);
        PCA9685_SetServoAngle(pca9685_leg, 10, 90);
        main_mode = 0;
    } else if(main_mode == 0) {
        PCA9685_ResetAllChannels(pca9685_leg);
        main_mode = 1;
    }
    
    if(main_mode == 1) {
        switch(state) {
            case 10:
                PCA9685_SetServoAngle(pca9685_leg, 10, 90);
                break;
            case 6:
                allServoLegSpin(pca9685_leg, 20, leftLeg.currentAngle, rightLeg.currentAngle, 0);
                allServoHandSpin(pca9685_hand, 20, leftHand.currentAngle, rightHand.currentAngle, 0);
                L298N_move(motor1, 1, 0);
                L298N_move(motor2, 1, 0);
                break;
            case 1:
                if(turnLeft == 0 && turnRight == 0) {
                    L298N_move(motor1, 1, 700);
                    L298N_move(motor2, 1, 700);
                } else if(turnLeft == 1 && turnRight == 0) {
                    L298N_move(motor1, -1, 600);
                    L298N_move(motor2, 1, 700);
                } else if(turnLeft == 0 && turnRight == 1) {
                    L298N_move(motor1, 1, 700);
                    L298N_move(motor2, -1, 600);
                } else if(turnLeft == 1 && turnRight == 1) {
                    L298N_move(motor1, -1, 700);
                    L298N_move(motor2, -1, 700);
                }
                break;
            case 4:
                if(hold3mod == 1) {
                    if(turnLeft == 1 && turnRight == 0) {
                        L298N_move(motor1, -1, 400);
                        L298N_move(motor2, 1, 400);
                    } else if(turnLeft == 0 && turnRight == 1) {
                        L298N_move(motor1, 1, 400);
                        L298N_move(motor2, -1, 400);
                    }
                }
                break;
            case 7:
                if((leftLeg.defaultAngle.C > (20 - headDir) && leftLeg.defaultAngle.C < (160 - headDir)) &&
                   (rightLeg.defaultAngle.C > (20 - headDir) && rightLeg.defaultAngle.C < (160 - headDir))) {
                    leftLeg.defaultAngle.C += headDir;
                    rightLeg.defaultAngle.C += headDir;
                }
                allServoLegSpin(pca9685_leg, 100, leftLeg.defaultAngle, rightLeg.defaultAngle, 0);
                break;
            case 8:
                {
                    int maxAngleLeft = getMaxAngle(leftLeg.currentAngle, leftLeg.calibrationAngle);
                    int maxAngleRight = getMaxAngle(rightLeg.currentAngle, rightLeg.calibrationAngle);
                    int maxAngle = maxAngleLeft > maxAngleRight ? maxAngleLeft : maxAngleRight;
                    
                    ServosAngle startLeftAngle = leftLeg.currentAngle;
                    ServosAngle startRightAngle = rightLeg.currentAngle;
                    
                    for(int i = 1; i <= maxAngle; i++) {
                        leftLeg.currentAngle = getNextStepAll(startLeftAngle, leftLeg.calibrationAngle, maxAngle, i);
                        rightLeg.currentAngle = getNextStepAll(startRightAngle, rightLeg.calibrationAngle, maxAngle, i);
                        allServoLegSpin(pca9685_leg, 60, leftLeg.currentAngle, rightLeg.currentAngle, 0);
                    }
                }
                break;
            case 9:
                leftLeg.calibrationAngle.A = calibrationAngles[0];
                leftLeg.calibrationAngle.B = calibrationAngles[1];
                leftLeg.calibrationAngle.C = calibrationAngles[2];
                leftLeg.calibrationAngle.D = calibrationAngles[3];
                leftLeg.calibrationAngle.E = calibrationAngles[4];
                leftLeg.defaultAngle = leftLeg.calibrationAngle;
                leftLeg.currentAngle = leftLeg.calibrationAngle;
                
                rightLeg.calibrationAngle.A = calibrationAngles[5];
                rightLeg.calibrationAngle.B = calibrationAngles[6];
                rightLeg.calibrationAngle.C = calibrationAngles[7];
                rightLeg.calibrationAngle.D = calibrationAngles[8];
                rightLeg.calibrationAngle.E = calibrationAngles[9];
                rightLeg.defaultAngle = rightLeg.calibrationAngle;
                rightLeg.currentAngle = rightLeg.calibrationAngle;
                
                leftHand.calibrationAngle.A = calibrationAngles[10];
                leftHand.calibrationAngle.B = calibrationAngles[11];
                leftHand.calibrationAngle.C = calibrationAngles[12];
                leftHand.calibrationAngle.D = calibrationAngles[13];
                leftHand.calibrationAngle.E = calibrationAngles[14];
                leftHand.defaultAngle = leftHand.calibrationAngle;
                leftHand.currentAngle = leftHand.calibrationAngle;
                
                rightHand.calibrationAngle.A = calibrationAngles[15];
                rightHand.calibrationAngle.B = calibrationAngles[16];
                rightHand.calibrationAngle.C = calibrationAngles[17];
                rightHand.calibrationAngle.D = calibrationAngles[18];
                rightHand.calibrationAngle.E = calibrationAngles[19];
                rightHand.defaultAngle = rightHand.calibrationAngle;
                rightHand.currentAngle = rightHand.calibrationAngle;
                
                state = 6;
                break;
            case 12:
                delay(4000);
                L298N_move(motor1, -1, 500);
                L298N_move(motor2, 1, 500);
                delay(1000);
                L298N_move(motor1, 1, 0);
                L298N_move(motor2, 1, 0);
                delay(100);
                L298N_move(motor1, -1, 500);
                L298N_move(motor2, 1, 500);
                delay(1000);
                L298N_move(motor1, 1, 0);
                L298N_move(motor2, 1, 0);
                delay(100);
                L298N_move(motor1, 1, 300);
                L298N_move(motor2, 1, 300);
                delay(2000);
                L298N_move(motor1, -1, 300);
                L298N_move(motor2, -1, 300);
                delay(2000);
                L298N_move(motor1, -1, 0);
                L298N_move(motor2, -1, 0);
                delay(10000);
                break;
        }
        
        if(clenchLeft == 1) {
            timerLeft = millis();
            dirLeft *= -1;
            clenchLeft = -1;
        }
        
        if(clenchRight == 1) {
            timerRight = millis();
            dirRight *= -1;
            clenchRight = -1;
        }
        
        if(millis() - timerLeft < GRIPPER_INTERVAL) {
            L298N_move_without_PWM(motor_hand1, dirLeft);
        } else {
            L298N_move_without_PWM(motor_hand1, 0);
        }
        
        if(millis() - timerRight < GRIPPER_INTERVAL) {
            L298N_move_without_PWM(motor_hand2, dirRight);
        } else {
            L298N_move_without_PWM(motor_hand2, 0);
        }
    }
}

void L298N_move(Motor motor, int dir, int speed) {
    speed = constrain(speed, 0, 255);
    
    switch(dir) {
        case -1:
            digitalWrite(motor.in1_pin, LOW);
            digitalWrite(motor.in2_pin, HIGH);
            break;
        case 1:
            digitalWrite(motor.in1_pin, HIGH);
            digitalWrite(motor.in2_pin, LOW);
            break;
        default:
            digitalWrite(motor.in1_pin, LOW);
            digitalWrite(motor.in2_pin, LOW);
            break;
    }
    
    if(motor.en_pin != 255) {
        analogWrite(motor.en_pin, speed);
    }
    motor.current_speed = speed;
    motor.current_direction = dir;
}

void L298N_move_without_PWM(Motor motor, int dir) {
    switch(dir) {
        case -1:
            digitalWrite(motor.in1_pin, LOW);
            digitalWrite(motor.in2_pin, HIGH);
            break;
        case 1:
            digitalWrite(motor.in1_pin, HIGH);
            digitalWrite(motor.in2_pin, LOW);
            break;
        default:
            digitalWrite(motor.in1_pin, LOW);
            digitalWrite(motor.in2_pin, LOW);
            break;
    }
    motor.current_direction = dir;
}

ServosAngle setAngle(int A, int B, int C, int D, int E) {
    ServosAngle angle;
    angle.A = A;
    angle.B = B;
    angle.C = C;
    angle.D = D;
    angle.E = E;
    return angle;
}

void setServoAngle(char leg, char servo, int angle) {
    ServosAngle* targetAngle = NULL;
    
    if(leg == 'L') targetAngle = &leftLeg.calibrationAngle;
    else if(leg == 'R') targetAngle = &rightLeg.calibrationAngle;
    else if(leg == 'H') targetAngle = &leftHand.calibrationAngle;
    else if(leg == 'M') targetAngle = &rightHand.calibrationAngle;
    
    if(targetAngle) {
        switch(servo) {
            case 'A': targetAngle->A = angle; break;
            case 'B': targetAngle->B = angle; break;
            case 'C': targetAngle->C = angle; break;
            case 'D': targetAngle->D = angle; break;
            case 'E': targetAngle->E = angle; break;
        }
    }
}

int getMaxAngle(ServosAngle current, ServosAngle target) {
    int deltaAngles[5] = {
        abs(current.A - target.A),
        abs(current.B - target.B),
        abs(current.C - target.C),
        abs(current.D - target.D),
        abs(current.E - target.E)
    };
    int maxAngle = deltaAngles[0];
    for(int i = 1; i < 5; i++) {
        if(maxAngle < deltaAngles[i])
            maxAngle = deltaAngles[i];
    }
    return maxAngle;
}

ServosAngle getNextStepAll(ServosAngle start, ServosAngle target, int maxAngle, int step) {
    ServosAngle result;
    if(maxAngle == 0) return start;
    
    float ratio = (float)step / (float)maxAngle;
    result.A = start.A + (target.A - start.A) * ratio;
    result.B = start.B + (target.B - start.B) * ratio;
    result.C = start.C + (target.C - start.C) * ratio;
    result.D = start.D + (target.D - start.D) * ratio;
    result.E = start.E + (target.E - start.E) * ratio;
    
    return result;
}

void PCA9685_SetServoAngle(Adafruit_PWMServoDriver &pwm, uint8_t channel, int angle) {
    angle = constrain(angle, 0, 180);
    int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
}

void PCA9685_ResetAllChannels(Adafruit_PWMServoDriver &pwm) {
    for(int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);
    }
}

void allServoLegSpin(Adafruit_PWMServoDriver &pwm, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle) {
    PCA9685_SetServoAngle(pwm, 12, right.A);
    PCA9685_SetServoAngle(pwm, 13, right.B);
    PCA9685_SetServoAngle(pwm, 14, right.C + correctAngle);
    PCA9685_SetServoAngle(pwm, 15, right.D);
    
    PCA9685_SetServoAngle(pwm, 7, 180 - left.A);
    PCA9685_SetServoAngle(pwm, 6, 180 - left.B);
    PCA9685_SetServoAngle(pwm, 5, 180 - (left.C + correctAngle));
    PCA9685_SetServoAngle(pwm, 4, 180 - left.D);
    
    delay(speedDelay);
}

void allServoHandSpin(Adafruit_PWMServoDriver &pwm, int speedDelay, ServosAngle left, ServosAngle right, int correctAngle) {
    PCA9685_SetServoAngle(pwm, 4, left.A);
    PCA9685_SetServoAngle(pwm, 6, left.B);
    PCA9685_SetServoAngle(pwm, 5, left.C + correctAngle);
    
    PCA9685_SetServoAngle(pwm, 2, 180 - right.A);
    PCA9685_SetServoAngle(pwm, 1, 180 - right.B);
    PCA9685_SetServoAngle(pwm, 3, 180 - (right.C + correctAngle));
    
    delay(speedDelay);
}