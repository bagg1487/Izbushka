#ifndef L298NDRIVER_H_
#define L298NDRIVER_H_

#include "main.h"

typedef struct
{
    GPIO_TypeDef *GPIO_Port1;
    uint32_t GPIO_Pin1;
    GPIO_TypeDef *GPIO_Port2;
    uint32_t GPIO_Pin2;
	  TIM_HandleTypeDef *htim;
    uint32_t channel;
} Motor;

void L298N_move(Motor motor, int dir,int speed);
void L298N_move_without_PMW(Motor motor, int dir);
void L298N_move_all(Motor motor1, int dir1, int speed1, Motor motor2, int dir2, int speed2);
#endif /* LEGCONTROL_H_ */