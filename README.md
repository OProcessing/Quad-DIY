# Quadrotor DIY project 
![drone_frontview](https://github.com/user-attachments/assets/d48bb2ab-f787-4ad7-bfd2-0e73390102b7)

&nbsp;

## Info
Language : C\
Program : IAREWARM, VSCode, STM32CubeIDE, STM32CubeMx, ST-Link, Matlab

&nbsp;
## Hardware configuration
Frame : length 500mm\
Board : Nucleo-F439Zi (ARM Cortex-M4) \
BLDC motor : dys G-Power Series\
ESC : 30A ESC\
IMU sensor : MPU6050 GY521\
Alt sensor : BMP180\
Sample rate : Sensors 1kHz, ESC 40Hz

&nbsp;
## PinOut
ESC : PWM signal PA0(front), PB3(back), PB10(left), PA3(right)\
BMP180 : I2C SCL PB9, SDA PB6\
IMU : I2C SCL PF1, SDA PF0

&nbsp;
## example code
```C
void IMU_read_data(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050){

	MPU6050->Gyro_x = MPU6050->Angle_x + MPU6050->gyro_dps_x * dt;
	MPU6050->Gyro_y = MPU6050->Angle_y + MPU6050->gyro_dps_x * dt;

	MPU6050->Accel_x = atan2(MPU6050->acc_raw_y, sqrt(pow(MPU6050->acc_raw_x, 2) + pow(MPU6050->acc_raw_z, 2)))* Rad_to_deg;
	MPU6050->Accel_y = atan2(-1 * MPU6050->acc_raw_x,sqrt(pow(MPU6050->acc_raw_y, 2) + pow(MPU6050->acc_raw_z, 2)))* Rad_to_deg;

	MPU6050->Angle_x = alpha * MPU6050->Gyro_x + (1 - alpha) * MPU6050->Accel_x;
	MPU6050->Angle_y = alpha * MPU6050->Gyro_y + (1 - alpha) * MPU6050->Accel_y;
}
```

&nbsp;
## Reference
[1] Eunhye Seok (2021.02.20.). STM32_HAL_MPU6050_lib. https://github.com/mokhwasomssi. \
[2] Fanni, M., & Khalifa, A. (2017). A new 6-DOF quadrotor manipulation system: Design, kinematics, dynamics, and control. IEEE/ASME Transactions On Mechatronics, 22(3), 1315-1326.\
[3] 김진석, 임영도, 허재영. (2012). 센서 결합을 통한 쿼드콥터의 자세제어 시뮬레이터 구현. 한국정보기술학회논문지, 10(7), 1-11.\
[4] 류성진. (2016). 드론의 상용화(常用化)에 따른 안전과 법적 문제. 법제연구, 51, 241-282.\
[5] 서민우. (2021). ESP32 아두이노 드론 만들고 직접 코딩으로 PID 제어하기. 고양: 앤써북.\
[6] 양현수, 이동준. (2016). 드론 비행제어 및 상태추정 기초. 한국통신학회지(정보와통신), 33(2), 86-92.\
[7] 윤덕용. (2017). (ARM Cortex-M0) STM32F091 정복:OK-STM091 키트. 서울 : Ohm사.\
[8] 홍봉조. (2022). (기초에서 응용까지) STM32Fx Cortex ARM 프로그래밍 : 기초편. 서울 : 지식과감성#
