## Quadrotor DIY project 
쿼드로터 1인 제작
### Abstract
### Project Infomation
2023년 8월 ~ 2023년 12월
### Project Participant
[한영철](https://github.com/OProcessing)

### d
Lang : C, C++\
Tool : IAREWARM, STM32IDE, STM32Cube, ST-Link

### Hardware configuration
||Processor|Actuator|Controller|Sensor|Sensor|
|-|:--:|:--:|:--:|:--:|:--:|
|Name|Embedded Board|BLDC motor|ESC|Gyro Sensor|Alt sensor|
|Model|Nucloe-F439ZI|dys G-Power Series|30A ESC|GY521|BMP180|
|Specification|cortex-m4, 168MHz|NL 360000rpm|continuos 30A|acc, gyro 6-DOF|altitude|
### PinOut
|ESC-forward|ESC-backward|ESC-left|ESC-right|BMP180|BMP180|GY-521|GY-521|
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|PWM|PWM|PWM|PWM|SCL|SDA|SCL|SDA|
|PA0|PB3|PB10|PA3|PB9|PB6|PF1|PF0|
|TIM2_CH2|TIM2_CH1|TIM2_CH3|TIM2_CH4|-|-|-|-|
### Software configuration
### Source
initial setup\
main\
IMU\
Alt
### Error Handle
IMU Sensor preprocessing\
Altitude sensor accuracy issue\
Embedded board malfunction\
Soldering issue\
Motor power
### Results
### Limit
### Reference
[1] Eunhye Seok (2021.02.20.). STM32_HAL_MPU6050_lib. https://github.com/mokhwasomssi. \
[2] Fanni, M., & Khalifa, A. (2017). A new 6-DOF quadrotor manipulation system: Design, kinematics, dynamics, and control. IEEE/ASME Transactions On Mechatronics, 22(3), 1315-1326.\
[3] 김진석, 임영도, 허재영. (2012). 센서 결합을 통한 쿼드콥터의 자세제어 시뮬레이터 구현. 한국정보기술학회논문지, 10(7), 1-11.\
[4] 류성진. (2016). 드론의 상용화(常用化)에 따른 안전과 법적 문제. 법제연구, 51, 241-282.\
[5] 서민우. (2021). ESP32 아두이노 드론 만들고 직접 코딩으로 PID 제어하기. 고양: 앤써북.\
[6] 양현수, 이동준. (2016). 드론 비행제어 및 상태추정 기초. 한국통신학회지(정보와통신), 33(2), 86-92.\
[7] 윤덕용. (2017). (ARM Cortex-M0) STM32F091 정복:OK-STM091 키트. 서울 : Ohm사.\
[8] 홍봉조. (2022). (기초에서 응용까지) STM32Fx Cortex ARM 프로그래밍 : 기초편. 서울 : 지식과감성#\
