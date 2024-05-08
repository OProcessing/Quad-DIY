# Quadrotor DIY project
졸업 프로젝트, 쿼드로터 1인 제작

## Abstract

## Project Infomation
기간 2023년 8월 ~ 2023년 12월\
맨땅에 헤딩하며 쿼드로터 DIY하기

## Project Participant
[한영철](https://github.com/OProcessing)

## 수행목록

## stack
STM32 IDE, STM32 Cube, STM32 monitoring, Git

## System configuration

## Hardware configuration
||Processor|Actuator|Controller|Sensor|Sensor|
|-|:--:|:--:|:--:|:--:|:--:|
||Dev Board|BLDC motor|ESC|Gyro Sensor|Alt sensor|
|Model Name|Nucloe-F439ZI|dys G-Power Series|30A ESC|GY521|BMP180|
|spec|cortex-m4, 168MHz|NL 360000rpm|continuos 30A|acc, gyro 6-DOF|altitude|

## PinOut
|ESC-forward|ESC-backward|ESC-left|ESC-right|BMP180|BMP180|GY-521|GY-521|
|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
|PWM|PWM|PWM|PWM|SCL|SDA|SCL|SDA|
|PA0|PB3|PB10|PA3|PB9|PB6|PF1|PF0|
|TIM2_CH2|TIM2_CH1|TIM2_CH3|TIM2_CH4|-|-|-|-|

## Software configuration

## Source
initial setup\
main\
IMU\
Alt

## challenging
자이로 데이터 전처리\
대기압 부정확\
보드고장\
납땜불량\
모터출력\
전원출력

## Result

## Extend

## Reference
