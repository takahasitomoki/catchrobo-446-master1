/*
 * servomotor.c
 *
 *  Created on: 2019/08/26
 *      Author: tomok
 */

#include "servomotor.h"

extern TIM_HandleTypeDef htim9;

//�������ŗ^����ꂽ�p�x�ɑ�������PWM�̒l��Ԃ��iMAX��10000�j
long map(long x,long in_min,long in_max,long out_min,long out_max){
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//�����ŗ^����ꂽ�p�x�ɃT�[�{�𓮂���
void ServoSetAngle(int angle){
	long PWM;

	//�p�x�͈͂𐧌�
	angle  = fmaxf(fminf(angle, SERVO1_MAX_ANGLE), SERVO1_MIN_ANGLE);

	//�Ή�����PWM�̒l���擾
	PWM = map(angle,SERVO1_MIN_ANGLE,SERVO1_MAX_ANGLE,SERVO_LOW,SERVO_HIGH);

	__HAL_TIM_SET_COMPARE(&SERVO1_TIM_HANDLER,SERVO1_TIM_CH,PWM);

}

/*
 * �^�C�}ENABLE
 * @param
 * @return
 */
void ServoEnable(void) {
	HAL_TIM_PWM_Start(&SERVO1_TIM_HANDLER, SERVO1_TIM_CH);
	ServoSetAngle(SERVO1_ZERO_ANGLE);
}

/*
 * �^�C�}DISABLE
 * @param
 * @return
 */
void ServoDisable(void) {
	HAL_TIM_PWM_Stop(&SERVO1_TIM_HANDLER, SERVO1_TIM_CH);
}
