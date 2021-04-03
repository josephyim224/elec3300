/*
 * W2812B.h
 *
 *  Created on: Apr 3, 2021
 *      Author: joseph
 *
 *  Implemented by bit bang
 */

#ifndef INC_WS2812B_H_
#define INC_WS2812B_H_

#include "main.h"

#define writeLowReg (uint32_t)WS2812B_Pin << 16u
#define writeHigh WS2812B_GPIO_Port->BSRR = WS2812B_Pin;
#define writeLow WS2812B_GPIO_Port->BSRR = writeLowReg;

//#define T0H delayNs(400);
//#define T0L delayNs(800);
//#define T1H delayNs(800);
//#define T1L delayNs(400);

#define T0H for (uint8_t t = 0; t < 1; ++t) {}
#define T0L for (uint8_t t = 0; t < 2; ++t) {}
#define T1H for (uint8_t t = 0; t < 2; ++t) {}
#define T1L for (uint8_t t = 0; t < 1; ++t) {}

/*
 * c: uint8_t array, [G1, R1, B1]
 */
void writeRGB(uint8_t *c) {
	for (int grb = 0; grb < 3; ++grb) {
		for (int bit = 0; bit < 8; ++bit) {
			if ((c[grb] << bit) & 0b10000000) {
				writeHigh
				T1H
				writeLow
				T1L
			} else {
				writeHigh
				T0H
				writeLow
				T0L
			}
		}
	}
	writeLow
}

#endif /* INC_WS2812B_H_ */
