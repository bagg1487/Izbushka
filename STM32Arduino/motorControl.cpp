#include "motorControl.h"

Motor motor1 = {115, 71, 75, 0, 0}; 
Motor motor2 = {125, 126, 77, 0, 0}; 

void L298N_move(Motor &motor, int dir, int speed) {
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

    if (motor.en_pin != 255) {
        analogWrite(motor.en_pin, speed); 
    }

    motor.current_speed = speed;
    motor.current_direction = dir;
}

void setupMotorControl() {
    pinMode(motor1.in1_pin, OUTPUT);
    pinMode(motor1.in2_pin, OUTPUT);
    if (motor1.en_pin != 255) pinMode(motor1.en_pin, OUTPUT);

    pinMode(motor2.in1_pin, OUTPUT);
    pinMode(motor2.in2_pin, OUTPUT);
    if (motor2.en_pin != 255) pinMode(motor2.en_pin, OUTPUT);

    L298N_move(motor1, 0, 0);
    L298N_move(motor2, 0, 0);
}
