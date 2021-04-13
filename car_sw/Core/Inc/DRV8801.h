/*
 * DRV8801.h
 *
 *  Created on: Apr 13, 2021
 *      Author: joseph
 */

#ifndef INC_DRV8801_H_
#define INC_DRV8801_H_

#include "main.h"

typedef enum {
	CLOCKWISE, ANTICLOCKWISE
} drv8801_direction;

struct {
	TIM_HandleTypeDef* m_tim;
	TIM_HandleTypeDef* m0_tim;
	TIM_HandleTypeDef* m1_tim;
	TIM_HandleTypeDef* m2_tim;
} drv8801;

void drv8801_begin(
		TIM_HandleTypeDef *m_tim, TIM_HandleTypeDef *m0_tim,
		TIM_HandleTypeDef *m1_tim, TIM_HandleTypeDef *m2_tim);

/*
 * Set 1 for enable
 */
void drv8801_enable(uint8_t sleep);

/*
 * Set direction, 0: clockwise, 1: anti-clockwise
 */
void setDirection(uint8_t id, drv8801_direction dir);

/*
 * Set motor power, range: 0 -> 1000
 */
void setPower(uint8_t id, uint32_t power);

/*
 * Get and reset to zero
 */
uint32_t getCount(uint8_t id);

#endif /* INC_DRV8801_H_ */
