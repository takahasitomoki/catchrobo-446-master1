/*
 * servomotor.h
 *
 *  Created on: 2019/08/26
 *      Author: tomok
 */

#ifndef SERVOMOTOR_H_
#define SERVOMOTOR_H_

#include "stm32f4xx_hal.h"
#include "math.h"

long map(long x,long in_min,long in_max,long out_min,long out_max);
void ServoSetAngle(int angle);
void ServoDisable(void);
void ServoEnable(void);

//サーボの可動範囲(0.5ms ~ 2.0ms)
#define SERVO_LOW 			500
#define SERVO_HIGH 			1000
#define SERVO1_MIN_ANGLE 	-60
#define SERVO1_MAX_ANGLE 	60

//サーボのタイマーハンドラ
#define SERVO1_TIM_HANDLER	htim9

//サーボのチャンネル
#define SERVO1_TIM_CH		TIM_CHANNEL_1

//サーボの初期位置
#define SERVO1_ZERO_ANGLE 	0

#endif /* SERVOMOTOR_H_ */
