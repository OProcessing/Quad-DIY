## Quadrotor DIY project 
![drone_frontview](https://github.com/user-attachments/assets/d48bb2ab-f787-4ad7-bfd2-0e73390102b7)

### Info
Language : C\
IAREWARM, VSCode, STM32CubeIDE, STM32CubeMx, ST-Link, Matlab

### Hardware configuration
Frame : length 500mm\
Board : Nucleo-F439Zi (ARM Cortex-M4) \
BLDC motor : dys G-Power Series\
ESC : 30A ESC\
IMU sensor : MPU6050 GY521\
Alt sensor : BMP180\
Sample rate : Sensors 1kHz, ESC 40Hz

### PinOut
||ESC-forward|ESC-backward|ESC-left|ESC-right|BMP180|BMP180|GY-521|GY-521|
|:---|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|Signal|PWM|PWM|PWM|PWM|SCL|SDA|SCL|SDA|
|Pin|PA0|PB3|PB10|PA3|PB9|PB6|PF1|PF0|
|Channel|TIM2_CH2|TIM2_CH1|TIM2_CH3|TIM2_CH4|I2C1|I2C1|I2C2|I2C2|

### Reference
[1] Eunhye Seok (2021.02.20.). STM32_HAL_MPU6050_lib. https://github.com/mokhwasomssi. \
[2] Fanni, M., & Khalifa, A. (2017). A new 6-DOF quadrotor manipulation system: Design, kinematics, dynamics, and control. IEEE/ASME Transactions On Mechatronics, 22(3), 1315-1326.\
[3] 김진석, 임영도, 허재영. (2012). 센서 결합을 통한 쿼드콥터의 자세제어 시뮬레이터 구현. 한국정보기술학회논문지, 10(7), 1-11.\
[4] 류성진. (2016). 드론의 상용화(常用化)에 따른 안전과 법적 문제. 법제연구, 51, 241-282.\
[5] 서민우. (2021). ESP32 아두이노 드론 만들고 직접 코딩으로 PID 제어하기. 고양: 앤써북.\
[6] 양현수, 이동준. (2016). 드론 비행제어 및 상태추정 기초. 한국통신학회지(정보와통신), 33(2), 86-92.\
[7] 윤덕용. (2017). (ARM Cortex-M0) STM32F091 정복:OK-STM091 키트. 서울 : Ohm사.\
[8] 홍봉조. (2022). (기초에서 응용까지) STM32Fx Cortex ARM 프로그래밍 : 기초편. 서울 : 지식과감성#
