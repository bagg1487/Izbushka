#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define MOTOR_ENA PE_11
#define MOTOR_ENB PE_13
#define MOTOR_IN1 PG_1
#define MOTOR_IN2 PE_7
#define MOTOR_IN3 PF_13
#define MOTOR_IN4 PF_14

#define TRIG_PIN PA7
#define ECHO_PIN PA6
#define LED_PIN PC13
#define DEGREE90_PIN PB12

#define I2C1_SDA PB7
#define I2C1_SCL PB6
#define I2C2_SDA PB11
#define I2C2_SCL PB10

#define MOTOR_A_IA PB10
#define MOTOR_A_IB PB11
#define MOTOR_B_IA PE14
#define MOTOR_B_IB PE15

#define MPU6050_ADDR 0x68
#define PCA9685_ADDR1 0x40
#define PCA9685_ADDR2 0x41

#define JOYSTICK_DEADZONE 1000
#define MOTOR_DEADZONE 20
#define LOOP_DELAY 50
#define SERVO_UPDATE_INTERVAL 20
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600
#define NUM_SERVOS_I2C1 8
#define NUM_SERVOS_I2C2 8

#endif