#include "main.h"
#include "LegControl.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

static inline int8_t _sign(int val) {
  if (val <= 0) return -1;
  //if (val==0) return 0;
  return 1;
}

void saveNewDefoultAngle(Leg *leg){
	leg->defaultAngle=leg->calibrationAngle;
}
/*----------------------------------------------------------------------------------------------------
-- Функция вычисления следующего шага относительно текущей позии и позиции к которой стремится система
------------------------------------------------------------------------------------------------------*/
float getNextStep(float currentStep, float finalStep) {
  int deltaStep = 1;															// значение 1 шага в градусах
  if (currentStep != finalStep) {									// если текущее значение не равно заданному
    if (abs(currentStep - finalStep) < 2) {				// Проверка, если требуемый угол меньше дельты
      deltaStep = abs(currentStep - finalStep);
    }
    if (currentStep < finalStep)
    {
      currentStep += deltaStep;
    }
    else {
      currentStep -= deltaStep;
    }
  }
  return currentStep;															// возвращаем следующий угол
}
/*----------------------------------------------------------------------------------------------------
-- Функция вычисления следующего шага относительно текущей позии и позиции к которой стремится система
------------------------------------------------------------------------------------------------------*/
ServosAngle getNextStepAll(ServosAngle currentAngle, ServosAngle target, int maxAngle, float step ) {
  ServosAngle newStep;
	newStep.A = (currentAngle.A + step*(target.A-currentAngle.A)/maxAngle);
	newStep.B = (currentAngle.B + step*(target.B-currentAngle.B)/maxAngle);
	newStep.C = (currentAngle.C + step*(target.C-currentAngle.C)/maxAngle);
	newStep.D = (currentAngle.D + step*(target.D-currentAngle.D)/maxAngle);
	newStep.E = (currentAngle.E + step*(target.E-currentAngle.E)/maxAngle);
  return newStep;															// возвращаем следующий угол
}
/*-------------------------------------------------------------------------------------------------------------------------------
-- Функция вычисления следующего шага относительно текущей позии и позиции к которой стремится система с ускорением и замедлением
---------------------------------------------------------------------------------------------------------------------------------*/
ServoSmooth getNextStepSmooth(ServoSmooth self, int _target) {
	int _maxSpeed =10;
	int delta = 1;
	float err = _target - self.currentAngle;
	if (abs(err) > 1) {
		bool thisDir = ((self.speed+1) * (self.speed+1) / self.accel / 2.0 >= abs(err));  // пора тормозить
		//if((self.speed * self.speed / self.accel / 2.0)-abs(err)> abs(self.speed)+self.accel){
			self.speed += self.accel * delta * (thisDir ? -_sign(self.speed) : _sign(err));
			self.speed = constrain(self.speed, -_maxSpeed, _maxSpeed);
		//}
		self.currentAngle += self.speed * delta;
}else 
{
		self.speed = 0;
		self.currentAngle = _target;
}
  return self;															// возвращаем следующий угол
}

/*-------------------------------------------------------------------------------------------------------------------------------
-- Функция задает углы для 4 сервоприводов, Если стоит -1 то угол не изменяется
---------------------------------------------------------------------------------------------------------------------------------*/
ServosAngle setAngle(float A,float B, float C, float D,float E ){
	ServosAngle servo;
	if(A!=-1)
		servo.A=A;
	if(B!=-1)
		servo.B=B;
	if(C!=-1)
		servo.C=C;
	if(D!=-1)
		servo.D=D;
	if(E!=-1)
		servo.E=E;
	return servo;
}

/*-------------------------------------------------------------------------------------------------------------------------------
-- Функция проверки попадания текущих углов в требуемые
---------------------------------------------------------------------------------------------------------------------------------*/
bool targetPositionCheck(ServosAngle current, ServosAngle result){
	if(current.A==result.A && current.B==result.B && current.C==result.C && current.D==result.D && current.E==result.E)
		return true;
	return false;
}



