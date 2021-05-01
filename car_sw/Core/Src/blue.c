/*
 * blue.c
 *
 *  Created on: Apr 17, 2021
 *      Author: jasoni111
 */

#include "usart.h"
#include "blue.h"

void sendfloat(char idx, float data) {
	char c = 0b10101010;
	HAL_UART_Transmit(&huart2, &c, 1, 300);
	HAL_UART_Transmit(&huart2, &idx, 1, 300);
	HAL_UART_Transmit(&huart2, &data, 4, 300);
}
