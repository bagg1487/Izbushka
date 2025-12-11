#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

void setupServoMotor();
void updateServoMotor();
void setServoAngle(uint8_t servoNum, int angle);
void setMotorSpeed(uint8_t motorNum, int speed);
void updateFromJoystick();

#endif