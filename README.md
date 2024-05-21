# Quadrotor DIY project
쿼드로터 1인 제작

## Abstract

## Project Infomation
2023년 8월 ~ 2023년 12월\

## Project Participant
[한영철](https://github.com/OProcessing)

## d
Lan : C, C++\
Tool : IAREWARM, STM32IDE, STM32Cube, ST-Link

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

## Error Handle
IMU Sensor preprocessing\
Altitude sensor accuracy issue\
Embedded board malfunction\
Soldering issue\
Motor power

## Results

## Limit

## Reference
