#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_gpio.h"

/*MPU6050 registers address*/
/*대부분 레지스터 초기값 0x00 (예외 PWR_MGMT_1, WHO_AM_I)*/


/**마스터에서 write 하는 레지스터**/

#define ADDRESS_SMPLRT_DIV 0x19    //샘플링 레이트 설정 -> 자이로 4Hz ~ 8kHz, 가속도 4Hz ~ 1kHz
#define ADDRESS_GYRO_CONFIG 0x1B   //자이로 스코프 측정범위 설정
#define ADDRESS_ACCEL_CONFIG 0x1C  //엑셀로미터 측정범위 설정
#define ADDRESS_PWR_MGMT_1 0x6B    //초기 값 0x40, 파워 모드 설정

/**마스터에서 read 하는 레지스터**/

#define ADDRESS_WHO_AM_I 0x75 //초기 값 0x68, mpu6050의 주소가 저장 되어 있음

/*가속도 값 저장되는 레지스터*/
#define ADDRESS_ACCEL_XOUT_H 0x3B  //ACCEL_XOUT[15:8]
#define ADDRESS_ACCEL_XOUT_L 0x3C  //ACCEL_XOUT[7:0]
#define ADDRESS_ACCEL_YOUT_H 0x3D  //ACCEL_YOUT[15:8]
#define ADDRESS_ACCEL_YOUT_L 0x3E  //ACCEL_YOUT[7:0]
#define ADDRESS_ACCEL_ZOUT_H 0x3F  //ACCEL_ZOUT[15:8]
#define ADDRESS_ACCEL_ZOUT_L 0x40  //ACCEL_ZOUT[7:0]

/*각속도 값 저장되는 레지스터*/
#define ADDRESS_GYRO_XOUT_H 0x43  //GYRO_XOUT[15:8]
#define ADDRESS_GYRO_XOUT_L 0x44  //GYRO_XOUT[7:0]
#define ADDRESS_GYRO_YOUT_H 0x45  //GYRO_YOUT[15:8]
#define ADDRESS_GYRO_YOUT_L 0x46  //GYRO_YOUT[7:0]
#define ADDRESS_GYRO_ZOUT_H 0x47  //GYRO_ZOUT[15:8]
#define ADDRESS_GYRO_ZOUT_L 0x48  //GYRO_ZOUT[7:0]

#define gyro_output_rate 8000 //8kHz(digital low pass filter 안쓴 상태)


typedef struct //센서값과 단위변환 인자를 멤버로 갖는 구조체
{	
    //raw data에서 다른 단위로 바꿔주는 인자
	//(raw data) / (factor) -> 단위 변환
	float gyro_change_unit_factor; //raw data -> deg/sec
	float accel_change_unit_factor; //raw data -> g (중력가속도)

	int16_t Init_gy_x;
	int16_t Init_gy_y;
	int16_t Init_gy_z;
	int16_t Init_ac_x;
	int16_t Init_ac_y;
	int16_t Init_ac_z;

	//read_gyro, read_accel 함수를 사용하면 값이 업데이트 됨
	int16_t gyro_raw_x;	 //센서 raw data 저장되는 아날로그 변수
	int16_t gyro_raw_y;
	int16_t gyro_raw_z;
	int16_t acc_raw_x;
	int16_t acc_raw_y;
	int16_t acc_raw_z;

    float gyro_dps_x; //단위가 deg/sec 일때, 센서 값 저장되는 변수
    float gyro_dps_y;
    float gyro_dps_z;
    float acc_gra_x;   //단위가 중력 가속도 일때, 센서 값 저장되는 변수
    float acc_gra_y;
    float acc_gra_z;

	//누산 데이터 현재 각도 means roll,pitch,yaw
	float Angle_x;
	float Angle_y;
	float Angle_z;

	float Accel_x;
	float Accel_y;
	
	float Gyro_x;
	float Gyro_y;

}mpu6050;

typedef enum //센서 값 단위 종류
{
    raw = 0x00, // 16비트 -32768 ~ 32767
    dps = 0x01, //1초 당 움직인 각도
    gra = 0x02 //중력 가속도
          
}unit;

typedef enum //자이로 측정 가능한 범위 종류
{
	gyro_full_scale_range_250 = 0x00,  //±250 deg/sec
	gyro_full_scale_range_500 = 0x01,  //±500 deg/sec
	gyro_full_scale_range_1000 = 0x02, //±1000 deg/sec
	gyro_full_scale_range_2000 = 0x03, //±2000 deg/sec

}gyro_full_scale_range;

typedef enum //가속도 측정 가능한 범위 종류
{
	accel_full_scale_range_2g = 0x00,  //±2g (단위 : 중력가속도)
	accel_full_scale_range_4g = 0x01,  //±4g
	accel_full_scale_range_8g = 0x02,  //±8g
	accel_full_scale_range_16g = 0x03  //±16g

}accel_full_scale_range;

void IMU_Init(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050);

/*원하는 샘플 레이트 설정*/
void IMU_set_sample_rate(I2C_HandleTypeDef* hi2c, uint16_t sample_rate_you_want);

/*자이로와 가속도의 측정 범위 설정*/
void IMU_set_sensitivity(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, gyro_full_scale_range gyro_range_you_want, accel_full_scale_range accel_range_you_want);

/*자이로 센서 값 읽기*/
void IMU_read_gyro(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, unit unit_you_want);

/*가속도 센서 값 읽기*/
void IMU_read_accel(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050, unit unit_you_want);

/* 센서 calibration*/
void IMU_Calibration(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050,uint8_t sample);

/* 데이터 출력*/
void IMU_read_data(I2C_HandleTypeDef* hi2c, mpu6050* MPU6050);
