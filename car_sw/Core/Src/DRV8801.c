/*
 * DRV8801.c
 *
 *  Created on: Apr 13, 2021
 *      Author: joseph
 */

#include "DRV8801.h"

void drv8801_begin(TIM_HandleTypeDef *m_tim, TIM_HandleTypeDef *m0_tim,
		TIM_HandleTypeDef *m1_tim, TIM_HandleTypeDef *m2_tim) {

	drv8801.m_tim = m_tim;
	drv8801.m0_tim = m0_tim;
	drv8801.m1_tim = m1_tim;
	drv8801.m2_tim = m2_tim;

	// Fast-decay synchronous rectification
	HAL_GPIO_WritePin(M_MODE1_GPIO_Port, M_MODE1_Pin, RESET);

	// start timers
	HAL_TIM_PWM_Start(drv8801.m_tim, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Start(drv8801.m_tim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(drv8801.m_tim, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(drv8801.m0_tim);
	HAL_TIM_Base_Start(drv8801.m1_tim);
	HAL_TIM_Base_Start(drv8801.m2_tim);

	// clear encoder count
	getCount(0);
	getCount(1);
	getCount(2);

	// init motors
	setDirection(0, 0);
	setDirection(1, 0);
	setDirection(2, 0);

	setPower(0, 0);
	setPower(1, 0);
	setPower(2, 0);

	drv8801_enable(1);
}

void setDirection(uint8_t id, drv8801_direction dir) {
	GPIO_PinState state = (dir == CLOCKWISE) ? RESET : SET;
	switch (id) {
	case 0:
		HAL_GPIO_WritePin(M0_PHASE_GPIO_Port, M0_PHASE_Pin, state);
		break;
	case 1:
		HAL_GPIO_WritePin(M1_PHASE_GPIO_Port, M1_PHASE_Pin, state);
		break;
	case 2:
		HAL_GPIO_WritePin(M2_PHASE_GPIO_Port, M2_PHASE_Pin, state);
		break;

	}
}

void drv8801_enable(uint8_t sleep) {
	if (sleep)
		HAL_GPIO_WritePin(M_nSLEEP_GPIO_Port, M_nSLEEP_Pin, SET);
	else
		HAL_GPIO_WritePin(M_nSLEEP_GPIO_Port, M_nSLEEP_Pin, RESET);
}

void setPower(uint8_t id, uint32_t power) {
	if (power > 1000) {
		power = 1000;
	}

	/*
	 * Channel 2 and 3 are using complementary output
	 */

	switch (id) {
	case 0:
		drv8801.m_tim->Instance->CCR4 = power;
		break;
	case 1:
		power = 1000 - power;
		drv8801.m_tim->Instance->CCR2 = power;
		break;
	case 2:
		power = 1000 - power;
		drv8801.m_tim->Instance->CCR3 = power;
		break;
	}
}

uint32_t getCount(uint8_t id) {
	uint32_t v;
	switch (id) {
	case 0:
		v = __HAL_TIM_GET_COUNTER(drv8801.m0_tim);
		__HAL_TIM_SET_COUNTER(drv8801.m0_tim, 0);
		return v;
	case 1:
		v = __HAL_TIM_GET_COUNTER(drv8801.m1_tim);
		__HAL_TIM_SET_COUNTER(drv8801.m1_tim, 0);
		return v;
	case 2:
		v = __HAL_TIM_GET_COUNTER(drv8801.m2_tim);
		__HAL_TIM_SET_COUNTER(drv8801.m2_tim, 0);
		return v;
	}
	return 0;
}
