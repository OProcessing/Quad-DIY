/*FCS.c*/
#include "FCS.h"
#include <stdio.h>
#include <math.h>

#define Motor_Front TIM2->CCR2	//PA0  Yellow
#define Motor_Back TIM2->CCR1	//PB3 purple
#define Motor_Left TIM2->CCR3	//PB10 Blue
#define Motor_Right TIM2->CCR4	//PA3 Green
#define PWM_MIN 1000
#define PWM_MAX 2000
#define M_dt 0.001
#define M_den_dt 1000


extern TIM_HandleTypeDef htim2;
extern void Delay_ms(uint32_t ms);
extern PIDdata* parameter[];
extern float Avg_Alt;
extern float P_Gain[4];
extern float I_Gain[4];
extern float D_Gain[4];

uint16_t FCS_Regulation(float Input)
{
	Input = (uint16_t)(1000 + Input);
	if(Input>PWM_MAX) Input = 2000;
	else if(Input<PWM_MIN) Input = 1000;
	return Input;
}

void FCS_ESC_Starter() 	//ESC has minimum pulse 1ms, maximum pulse 2ms
{

	TIM2->ARR = 19999;

	Turn_LED(1,0,1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	FCS_USER_Motor_input(1000,1000,1000,1000);
	Delay_ms(3000);
	FCS_USER_Motor_input(0, 0, 0, 0);									//1ms signal, stop throttle
	Delay_ms(3000);
	Toggle_LED(1,1,1);
}

void FCS_PID_Calibration()
{
	for(int i=0;i<4;i++){
		if(i==2) i++;
		parameter[i]->desired = 0;
		parameter[i]->error = 0;
		parameter[i]->P = 0;
		parameter[i]->I = 0;
		parameter[i]->D = 0;
		parameter[i]->Kp = P_Gain[i];
		parameter[i]->Ki = I_Gain[i];
		parameter[i]->Kd = D_Gain[i];
		parameter[i]->prev_error = 0;
		parameter[i]->value = 0;
	}
}

void FCS_USER_Motor_input(uint16_t Input_motor1, uint16_t Input_motor2, uint16_t Input_motor3, uint16_t Input_motor4)
{
	Motor_Front = FCS_Regulation(Input_motor1);
	Motor_Back = FCS_Regulation(Input_motor2);
	Motor_Left = FCS_Regulation(Input_motor3);
	Motor_Right = FCS_Regulation(Input_motor4);
}

void FCS_PID_Calculate(mpu6050* MPU6050)
{
	for(int i=0;i<4;i++){

		switch (i)
		{
		case 0:
			parameter[i]->error = parameter[i]->desired - MPU6050->Angle_x;
			break;
		case 1:
			parameter[i]->error = parameter[i]->desired - MPU6050->Angle_y;
			break;
		case 2:
			parameter[i]->error = parameter[i]->desired - MPU6050->Angle_z;
			break;
		case 3:
			parameter[i]->error = parameter[i]->desired - Avg_Alt;
			break;
		}
		if(i==2) i++;
    	parameter[i]->P = parameter[i]->Kp * parameter[i]->error;
    	parameter[i]->I += parameter[i]->Ki * parameter[i]->error * M_dt;
    	parameter[i]->D = parameter[i]->Kd * (parameter[i]->error - parameter[i]->prev_error) * M_den_dt;
    	parameter[i]->prev_error = parameter[i]->error;
    	parameter[i]->value = parameter[i]->P + parameter[i]->I + parameter[i]->D;
	}
}

void FCS_RPYA_MV(PIDdata* Roll,PIDdata* Pitch,PIDdata* Yaw,PIDdata* Altitude)
{
	Altitude->value=300;
	Motor_Front = FCS_Regulation(Altitude->value + Pitch->value + Yaw->value);
	Motor_Back = FCS_Regulation(Altitude->value - Pitch->value + Yaw->value);
	Motor_Left = FCS_Regulation(Altitude->value + Roll->value - Yaw->value);
	Motor_Right = FCS_Regulation(Altitude->value - Roll->value - Yaw->value);
}
