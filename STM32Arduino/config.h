#ifndef CONFIG_H
#define CONFIG_H

#define TRIG_PIN PA6
#define ECHO_PIN PA7
#define LED_PIN PC13

#define I2C1_SDA PA9
#define I2C1_SCL PA8
#define I2C2_SDA PB7
#define I2C2_SCL PB6

#define MOTOR_A_IA PB10
#define MOTOR_A_IB PB11
#define MOTOR_B_IA PE14
#define MOTOR_B_IB PE15

#define MPU6050_ADDR 0x68

#define PCA9685_ADDR1 0x40
#define PCA9685_ADDR2 0x41
#define PCA9685_ADDR3 0x42

#define JOYSTICK_DEADZONE 20
#define MOTOR_DEADZONE 1000

#define LOOP_DELAY 50
#define SERVO_UPDATE_INTERVAL 20

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600

#define NUM_SERVOS_I2C1 8
#define NUM_SERVOS_I2C2 8

#endif