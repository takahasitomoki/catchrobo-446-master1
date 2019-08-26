/*
 * servomotor.c
 *
 *  Created on: 2019/08/26
 *      Author: tomok
 */

#include "servomotor.h"

extern TIM_HandleTypeDef htim9;

//引数ｘで与えられた角度に相当するPWMの値を返す（MAXは10000）
long map(long x,long in_min,long in_max,long out_min,long out_max){
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//引数で与えられた角度にサーボを動かす
void ServoSetAngle(int angle){
	long PWM;

	//角度範囲を制限
	angle  = fmaxf(fminf(angle, SERVO1_MAX_ANGLE), SERVO1_MIN_ANGLE);

	//対応するPWMの値を取得
	PWM = map(angle,SERVO1_MIN_ANGLE,SERVO1_MAX_ANGLE,SERVO_LOW,SERVO_HIGH);

	__HAL_TIM_SET_COMPARE(&SERVO1_TIM_HANDLER,SERVO1_TIM_CH,PWM);

}

/*
 * タイマENABLE
 * @param
 * @return
 */
void ServoEnable(void) {
	HAL_TIM_PWM_Start(&SERVO1_TIM_HANDLER, SERVO1_TIM_CH);
	ServoSetAngle(SERVO1_ZERO_ANGLE);
}

/*
 * タイマDISABLE
 * @param
 * @return
 */
void ServoDisable(void) {
	HAL_TIM_PWM_Stop(&SERVO1_TIM_HANDLER, SERVO1_TIM_CH);
}
