/*
 * pca9685.c
 *
 *  Created on: 20.01.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/Servos_PWM_STM32_HAL
 */

#include "main.h"
//#include "i2c.h"

#include "pca9685.h"
#include "math.h"


PCA9685_STATUS PCA9685_SetBit(PCA9685 pca9685,uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if(Value) Value = 1;

	if(HAL_OK != HAL_I2C_Mem_Read(pca9685.pca9685_i2c, pca9685.PCA9685_ADDRESS, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}
	tmp &= ~((1<<PCA9685_MODE1_RESTART_BIT)|(1<<Bit));
	tmp |= (Value&1)<<Bit;

	if(HAL_OK != HAL_I2C_Mem_Write(pca9685.pca9685_i2c, pca9685.PCA9685_ADDRESS, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SoftwareReset(PCA9685 pca9685)
{
	uint8_t cmd = 0x6;
	if(HAL_OK == HAL_I2C_Master_Transmit(pca9685.pca9685_i2c, 0x00, &cmd, 1, 10))
	{
		return PCA9685_OK;
	}
	return PCA9685_ERROR;
}

PCA9685_STATUS PCA9685_SleepMode(PCA9685 pca9685,uint8_t Enable)
{
	return PCA9685_SetBit(pca9685,PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, Enable);
}

PCA9685_STATUS PCA9685_RestartMode(PCA9685 pca9685,uint8_t Enable)
{
	return PCA9685_SetBit(pca9685,PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, Enable);
}

PCA9685_STATUS PCA9685_AutoIncrement(PCA9685 pca9685,uint8_t Enable)
{
	return PCA9685_SetBit(pca9685,PCA9685_MODE1, PCA9685_MODE1_AI_BIT, Enable);
}

PCA9685_STATUS PCA9685_SubaddressRespond(PCA9685 pca9685,SubaddressBit Subaddress, uint8_t Enable)
{
	return PCA9685_SetBit(pca9685,PCA9685_MODE1, Subaddress, Enable);
}

PCA9685_STATUS PCA9685_AllCallRespond(PCA9685 pca9685,uint8_t Enable)
{
	return PCA9685_SetBit(pca9685,PCA9685_MODE1, PCA9685_MODE1_ALCALL_BIT, Enable);
}

//
//	Frequency - Hz value
//
PCA9685_STATUS PCA9685_SetPwmFrequency(PCA9685 pca9685,uint16_t Frequency)
{
	float PrescalerVal;
	uint8_t Prescale;

	if(Frequency >= 1526)
	{
		Prescale = 0x03;
	}
	else if(Frequency <= 24)
	{
		Prescale = 0xFF;
	}
	else
	{
		PrescalerVal = (25000000 / (4096.0 * (float)Frequency)) - 1;
		Prescale = floor(PrescalerVal + 0.5);
	}

	//
	//	To change the frequency, PCA9685 have to be in Sleep mode.
	//
	PCA9685_SleepMode(pca9685,1);
	HAL_I2C_Mem_Write(pca9685.pca9685_i2c, pca9685.PCA9685_ADDRESS, PCA9685_PRESCALE, 1, &Prescale, 1, 10); // Write Prescale value
	PCA9685_SleepMode(pca9685,0);
	PCA9685_RestartMode(pca9685,1);
	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SetPwm(PCA9685 pca9685,uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
	uint8_t RegisterAddress;
	uint8_t Message[4];

	RegisterAddress = PCA9685_LED0_ON_L + (4 * Channel);
	Message[0] = OnTime & 0xFF;
	Message[1] = OnTime>>8;
	Message[2] = OffTime & 0xFF;
	Message[3] = OffTime>>8;

	if(HAL_OK != HAL_I2C_Mem_Write(pca9685.pca9685_i2c, pca9685.PCA9685_ADDRESS, RegisterAddress, 1, Message, 4, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SetPin(PCA9685 pca9685,uint8_t Channel, uint16_t Value, uint8_t Invert)
{
  if(Value > 4095) Value = 4095;

  if (Invert) {
    if (Value == 0) {
      // Special value for signal fully on.
      return PCA9685_SetPwm(pca9685,Channel, 4096, 0);
    }
    else if (Value == 4095) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(pca9685,Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(pca9685,Channel, 0, 4095-Value);
    }
  }
  else {
    if (Value == 4095) {
      // Special value for signal fully on.
    	return PCA9685_SetPwm(pca9685,Channel, 4096, 0);
    }
    else if (Value == 0) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(pca9685,Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(pca9685,Channel, 0, Value);
    }
  }
}
// Функция записи в регистр PCA9685
void PCA9685_WriteRegister(PCA9685 pca9685,uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    HAL_I2C_Master_Transmit(pca9685.pca9685_i2c, pca9685.PCA9685_ADDRESS, data, 2, HAL_MAX_DELAY);
}

// Функция установки всех каналов PWM в ноль
void PCA9685_ResetAllChannels(PCA9685 pca9685) {
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t reg = PCA9685_LED0_ON_L + 4 * i;
        PCA9685_WriteRegister(pca9685,reg, 0x00);     // LEDx_ON_L
        PCA9685_WriteRegister(pca9685,reg + 1, 0x00); // LEDx_ON_H
        PCA9685_WriteRegister(pca9685,reg + 2, 0x00); // LEDx_OFF_L
        PCA9685_WriteRegister(pca9685,reg + 3, 0x00); // LEDx_OFF_H
    }
}
#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(PCA9685 pca9685, uint8_t Channel, float Angle)
{
	float Value;
	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;

	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;

	return PCA9685_SetPin(pca9685,Channel, (uint16_t)Value, 0);
}
#endif

PCA9685_STATUS PCA9685_Init(PCA9685 pca9685)
{
	//pca9685_i2c = hi2c;

	PCA9685_SoftwareReset(pca9685);
#ifdef PCA9685_SERVO_MODE
	PCA9685_SetPwmFrequency(pca9685,48);
#else
	PCA9685_SetPwmFrequency(pca9685,1000);
#endif
	PCA9685_AutoIncrement(pca9685,1);

	return PCA9685_OK;
}
