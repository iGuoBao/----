#include "MG513.h"
#include "tim.h"

void MG513_SetRight(int pwm){
	pwm = pwm>800? 800: pwm;
	pwm = pwm<-800? -800: pwm;
	if(pwm<0){
		pwm *= -1;
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm);
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	}else if(pwm>0){
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm);
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	}else{
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
	}
}

void MG513_SetLeft(int pwm){
	pwm = pwm>800? 800: pwm;
	pwm = pwm<-800? -800: pwm;
	if(pwm<0){
		pwm *= -1;
		HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, pwm);
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
		return;
	}else if(pwm>0){
		HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwm);
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	}else{
		HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
	}
}


void MG513_Set(int pwm_l, int pwm_r){
	MG513_SetLeft(pwm_l);
	MG513_SetRight(pwm_r);
}

