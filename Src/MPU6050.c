#include <stdio.h>
#include <math.h>
#include <MPU6050.h>


#define PI 3.141592
#define Rad_to_deg 180 / PI
#define alpha 0.98	//gain
#define dt 0.001

/*자이로와 가속도의 측정 범위 설정*/
void IMU_Init(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050)
{
	uint8_t slave_address = 0xD0;		//0x68 왼쪽으로 한칸 쉬프팅
	uint8_t register_to_access = ADDRESS_WHO_AM_I; //접근할 레지스터(WHO_AM_I)의 주소
	uint8_t temp = 0; 					//WHO_AM_I에 저장된 값을 이 변수로 받음

	HAL_I2C_Master_Transmit(hi2c, slave_address, &register_to_access, 1, 1000); //읽으려는 레지스터의 주소값 보내고
	HAL_I2C_Master_Receive(hi2c, slave_address, &temp, 1, 1000); //WHO_AM_I에 저장되있는값(0x68) temp로 받음

	while(temp!=0x68) Turn_LED(1,1,1);
	Turn_LED(0,1,0);

	uint8_t data_to_write[3]; //레지스터 주소와 그 레지스터에 새로 넣어줄 값를 저장하는 배열

	data_to_write[0] = ADDRESS_PWR_MGMT_1; //접근할 레지스터(PWR_MGMT_1)의 주소
	data_to_write[1] = 0x00; //초기 값 0x40 -> 0x00으로 바꿔줌 : wake_up

	//배열을 이용해서 레지스터 주소와 해당되는 레지스터의 새로운 값를 한번에 보냄
	HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 2, 1000);
	//해당 레지스터 값 바뀜

	// Initial 값 초기화
	MPU6050->Angle_x = 0;
	MPU6050->Angle_y = 0;
	MPU6050->Angle_z = 0;

}

/*원하는 샘플 레이트 설정*/
void IMU_set_sample_rate(I2C_HandleTypeDef* hi2c, uint16_t sample_rate_you_want)
{
	uint8_t slave_address = 0xD0; //0x68 왼쪽으로 한칸 쉬프팅
	uint8_t data_to_write[2];
    uint8_t calculated_SMPLRT_DIV = (gyro_output_rate/sample_rate_you_want) - 1;
        
	data_to_write[0] = ADDRESS_SMPLRT_DIV; //접근할 레지스터(SMPLRT_DIV)의 주소
	data_to_write[1] = calculated_SMPLRT_DIV; //해당되는 레지스터의 새로운 값

    //배열을 이용해서 레지스터 주소와 해당되는 레지스터의 새로운 값를 한번에 보냄
	HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 2, 1000); 
}

/*자이로와 가속도의 측정 범위 설정*/
void IMU_set_sensitivity(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, gyro_full_scale_range gyro_range_you_want, accel_full_scale_range accel_range_you_want)
{
	//자이로 엑셀로 민감도 정하기
	uint8_t slave_address = 0xD0; //0x68 왼쪽으로 한칸 쉬프팅
	uint8_t data_to_write[3];

	data_to_write[0] = ADDRESS_GYRO_CONFIG; //접근할 레지스터(GYRO_CONFIG)의 주소
	data_to_write[1] = gyro_range_you_want <<3; //해당되는 레지스터의 새로운 값
	data_to_write[2] = accel_range_you_want <<3; //해당되는 레지스터의 새로운 값

	HAL_I2C_Master_Transmit(hi2c, slave_address, data_to_write, 3, 1000);


	//자이로 측정 범위에 따라 gyro_change_unit_factor 값 바꾸기
	switch (gyro_range_you_want) 
	{
	case gyro_full_scale_range_250 :
		MPU6050->gyro_change_unit_factor = 131;
		break;

	case gyro_full_scale_range_500 :
		MPU6050->gyro_change_unit_factor = 65.5;
		break;

	case gyro_full_scale_range_1000:
		MPU6050->gyro_change_unit_factor = 32.8;
		break;

	case gyro_full_scale_range_2000:
		MPU6050->gyro_change_unit_factor = 16.4;
		break;

	default :
		break;
	}

	//가속도 측정 범위에 따라 accel_change_unit_factor 값 바꾸기
	switch (accel_range_you_want) 
	{
	case accel_full_scale_range_2g :
		MPU6050->accel_change_unit_factor = 16384;
		break;

	case accel_full_scale_range_4g:
		MPU6050->accel_change_unit_factor = 8192;
		break;

	case accel_full_scale_range_8g:
		MPU6050->accel_change_unit_factor = 4096;
		break;

	case accel_full_scale_range_16g:
		MPU6050->accel_change_unit_factor = 2048;
		break;

	default :
		break;
	}

}

/*자이로값 읽기*/
void IMU_read_gyro(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, unit unit_you_want)
{
	// x y z roll pitch yaw
	uint8_t slave_address = 0xD0; //0x68 왼쪽으로 한칸 쉬프팅f
	uint8_t register_to_access = ADDRESS_GYRO_XOUT_H; //접근할 레지스터(GYRO_XOUT_H)의 주소
	uint8_t data_to_read[6] = {0,}; //자이로 값 저장하는 배열

	HAL_I2C_Master_Transmit(hi2c, slave_address, &register_to_access, 1, 1000); 


	HAL_I2C_Master_Receive(hi2c, slave_address, data_to_read, 6, 1000);

	switch (unit_you_want) //센서 값을 원하는 단위로 바꿔줌
	{
	case raw : //바로 저장
		MPU6050->gyro_raw_x = (int16_t)(data_to_read[0] << 8 | data_to_read[1]);
		MPU6050->gyro_raw_y = (int16_t)(data_to_read[2] << 8 | data_to_read[3]);
		MPU6050->gyro_raw_z = (int16_t)(data_to_read[4] << 8 | data_to_read[5]);
		//break;

	case dps : //단위 변환을 위해 raw data를 gyro_change_unit_factor로 나눈 값을 저장
		MPU6050->gyro_dps_x = (int16_t)(data_to_read[0] << 8 | data_to_read[1]) / MPU6050->gyro_change_unit_factor;
		MPU6050->gyro_dps_y = (int16_t)(data_to_read[2] << 8 | data_to_read[3]) / MPU6050->gyro_change_unit_factor;
		MPU6050->gyro_dps_z = (int16_t)(data_to_read[4] << 8 | data_to_read[5]) / MPU6050->gyro_change_unit_factor;
		break;

	default :
		break;
	}
}

/*가속도 값 읽기*/
void IMU_read_accel(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, unit unit_you_want)
{
	uint8_t slave_address = 0xD0; //0x68 왼쪽으로 한칸 쉬프팅
	uint8_t register_to_access = ADDRESS_ACCEL_XOUT_H;//접근할 레지스터(ACCEL_XOUT_H)의 주소
	uint8_t data_to_read[6] = {0,}; //가속도 값 저장하는 배열


	HAL_I2C_Master_Transmit(hi2c, slave_address, &register_to_access, 1, 1000);


	HAL_I2C_Master_Receive(hi2c, slave_address, data_to_read, 6, 1000);


	switch (unit_you_want) //센서 값을 원하는 단위로 바꿔줌
	{
	case raw : //바로 저장
		MPU6050->acc_raw_x = (int16_t)(data_to_read[0] << 8 | data_to_read[1]) - MPU6050->Init_ac_x;
		MPU6050->acc_raw_y = (int16_t)(data_to_read[2] << 8 | data_to_read[3]) - MPU6050->Init_ac_y;
		MPU6050->acc_raw_z = (int16_t)(data_to_read[4] << 8 | data_to_read[5]) - MPU6050->Init_ac_z;
		break;

	case gra : //단위 변환을 위해 raw data를 accel_change_unit_factor로 나눈 값을 저장
		MPU6050->acc_gra_x = ((int16_t)(data_to_read[0] << 8 | data_to_read[1]) - MPU6050->Init_ac_x) / MPU6050->accel_change_unit_factor;
		MPU6050->acc_gra_y = ((int16_t)(data_to_read[2] << 8 | data_to_read[3]) - MPU6050->Init_ac_y) / MPU6050->accel_change_unit_factor;
		MPU6050->acc_gra_z = ((int16_t)(data_to_read[4] << 8 | data_to_read[5]) - MPU6050->Init_ac_z) / MPU6050->accel_change_unit_factor;
		break;

	default :
		break;
	}
}

// calibrate
void IMU_Calibration(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, uint8_t sample)
{

	//평균값 구하기 위한 변수 선언
	double Sum_gy_x = 0;
	double Sum_gy_y = 0;
	double Sum_gy_z = 0;

	// sample 수 만큼의 합 산
	for(uint8_t n = 0; n < sample ; n++)
	{
		IMU_read_gyro(hi2c, MPU6050, raw);
		Sum_gy_x += MPU6050->gyro_raw_x;
		Sum_gy_y += MPU6050->gyro_raw_y;
		Sum_gy_z += MPU6050->gyro_raw_z;

	}
	// sample 만큼 더한 값을 나누어 평균값 도출
	MPU6050->Init_gy_x =(int16_t) (Sum_gy_x / sample);
	MPU6050->Init_gy_y =(int16_t) (Sum_gy_y / sample);
	MPU6050->Init_gy_z =(int16_t) (Sum_gy_z / sample);
}

void IMU_read_data(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050){

	MPU6050->Gyro_x = MPU6050->Angle_x + MPU6050->gyro_dps_x * dt;
	MPU6050->Gyro_y = MPU6050->Angle_y + MPU6050->gyro_dps_x * dt;

	MPU6050->Accel_x = atan2(MPU6050->acc_raw_y, sqrt(pow(MPU6050->acc_raw_x, 2) + pow(MPU6050->acc_raw_z, 2)))* Rad_to_deg;
	MPU6050->Accel_y = atan2(-1 * MPU6050->acc_raw_x,sqrt(pow(MPU6050->acc_raw_y, 2) + pow(MPU6050->acc_raw_z, 2)))* Rad_to_deg;

	MPU6050->Angle_x = alpha * MPU6050->Gyro_x + (1 - alpha) * MPU6050->Accel_x;
	MPU6050->Angle_y = alpha * MPU6050->Gyro_y + (1 - alpha) * MPU6050->Accel_y;
}
