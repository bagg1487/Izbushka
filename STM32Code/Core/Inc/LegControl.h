#ifndef LEGCONTROL_H_
#define LEGCONTROL_H_


#include "main.h"
#include <stdbool.h>

#define M_PI 3.141592


typedef struct  {
		float A;
    float B;
    float C;
		float D;
		float E;
}ServosAngle;
typedef struct  {
    double x;
    double y;
    double z;
}Coordinate;
typedef struct{
	int currentAngle;
	int speed;
	int accel;
}ServoSmooth;

typedef struct {


    int servoARange[2]; //= { 10, 170 };
    int servoBRange[2]; //= { 10, 170 };
    int servoCRange[2];// = { 10, 170 };

		ServosAngle currentAngle;
	
		ServosAngle defaultAngle;
	
		ServosAngle calibrationAngle;


    double part1Length;// = 10;
    double part2Length;// = 10;

		Coordinate servoAPosition; //{0,0,9}
    Coordinate centerGravity;//{0, 0, 0};
    Coordinate defaultPosition;//{ 0, hDefault, 0 };


}Leg;

static inline int8_t _sign(int val);

float getNextStep(float currentStep, float finalStep); //пошаговое (по градусу) движение всех серв сразу

ServosAngle getNextStepAll(ServosAngle currentAngle, ServosAngle target, int maxAngle, float step ); //одновременный старт и одновременный финиш движения серв вне зависимости от целевого угла

ServoSmooth getNextStepSmooth(ServoSmooth self, int _target);  //пошаговое (с ускорением) движение всех серв

ServosAngle setAngle(float A,float B, float C, float D, float E ); //мгновенное вращение до конкретного угла

void saveNewDefoultAngle(Leg *leg);
bool targetPositionCheck(ServosAngle current, ServosAngle result); //совпадает ли текущее значение серв с целевым (использовалось для выхода из цикла while)
#endif /* LEGCONTROL_H_ */
