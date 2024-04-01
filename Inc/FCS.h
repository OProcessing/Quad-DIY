#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"
#include "MPU6050.h"



typedef struct
{
	float desired;
	float prev_error;
	float error;
	float Kp, Ki, Kd;
	float P, I, D;
	float value;

}PIDdata;

uint16_t FCS_Regulation (float Input);

void FCS_USER_Motor_input(uint16_t Input_motor1, uint16_t Input_motor2, uint16_t Input_motor3, uint16_t Input_motor4);

void FCS_ESC_Starter();

void FCS_PID_Calculate(mpu6050* MPU6050);

void FCS_RPYA_MV(PIDdata* R,PIDdata* P,PIDdata* Y,PIDdata* A);

void FCS_PID_Calibration();

