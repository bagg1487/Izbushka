#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

typedef struct {
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t en_pin;
    int current_speed;
    int current_direction;
} Motor;

extern Motor motor1;
extern Motor motor2;

void setupMotorControl();
void L298N_move(Motor &motor, int dir, int speed);

#endif
