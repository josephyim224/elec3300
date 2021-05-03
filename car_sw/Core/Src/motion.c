/*
 * motion.c
 *
 *  Created on: Apr 16, 2021
 *      Author: jasoni111
 */
#define PI 3.141592654
#include "motion.h"
#include "DRV8801.h"
#include "math.h"
//#include "usart.h"
#include "blue.h"
#include <stdlib.h>

typedef struct MotorDirAndSpeed {
	int motor0Dir;
	float motor0Speed;
	int motor1Dir;
	float motor1Speed;
	int motor2Dir;
	float motor2Speed;
} MotorDirAndSpeed;

MotorDirAndSpeed storedDirAndSpeed = { 0, 0, 0, 0, 0, 0 };

/**
 *
 */
MotorDirAndSpeed getMotorDirAndSpeed(float x, float y, float w) {
	MotorDirAndSpeed dirAndSpeed;
	int d = 1;
	dirAndSpeed.motor0Speed = w * d + x;
	dirAndSpeed.motor1Speed = w * d - x / 2 - y * sin(PI / 3);
	dirAndSpeed.motor2Speed = w * d - x / 2 + y * sin(PI / 3);

	dirAndSpeed.motor0Dir = dirAndSpeed.motor0Speed > 0;
	dirAndSpeed.motor1Dir = dirAndSpeed.motor1Speed > 0;
	dirAndSpeed.motor2Dir = dirAndSpeed.motor2Speed > 0;
	dirAndSpeed.motor0Speed = abs(dirAndSpeed.motor0Speed);
	dirAndSpeed.motor1Speed = abs(dirAndSpeed.motor1Speed);
	dirAndSpeed.motor2Speed = abs(dirAndSpeed.motor2Speed);

	return dirAndSpeed;
}

void move(Direction dir) {
	float x, y, w = 0;

	switch (dir) {
	case MOTION_FORWARD: {
		x = 12;
		y = 0;
		w = 0;
		break;
	}
	case MOTION_BACKWARD: {
		x = -12;
		y = 0;
		w = 0;
		break;
	}
	case MOTION_TRANSLATE_LEFT: {
		x = 0;
		y = 12;
		w = 0;
		break;
	}
	case MOTION_TRANSLATE_RIGHT: {
		x = 0;
		y = -12;
		w = 0;
		break;
	}
	case MOTION_ROTATE_LEFT: {
		x = 0;
		y = 0;
		w = 8;
		break;
	}
	case MOTION_ROTATE_RIGHT: {
		x = 0;
		y = 0;
		w = -8;
		break;
	}
	case MOTION_STOP: {
		x = 0;
		y = 0;
		w = 0;
		break;
	}
	default: {
		x = 0;
		y = 0;
		w = 0;
		break;
	}
	}
	storedDirAndSpeed = getMotorDirAndSpeed(x, y, w);
}

void MotorUpdate() {
	static last_tick = 0;
	uint32_t tick = HAL_GetTick();
	uint32_t dt = tick - last_tick;
	if (dt < 10) {
		return;
	}

	char c[20];
	sendfloat(0, 12.0f);


	static float I_error[3] = { 0, 0, 0 };
	static uint32_t power_value[3] = { 0, 0, 0 };
	int dir[3] = { storedDirAndSpeed.motor0Dir, storedDirAndSpeed.motor1Dir,
			storedDirAndSpeed.motor2Dir };
	float target_speed[3] = { storedDirAndSpeed.motor0Speed,
			storedDirAndSpeed.motor1Speed, storedDirAndSpeed.motor2Speed };
	float last_error[3] = { 0, 0, 0 };

	for (uint8_t i = 0; i < 3; ++i) {

		float speed = (float) getCount(i) / dt * 100.0;

		float error = (float) target_speed[i] - speed;
		I_error[i] = I_error[i] + error;
		if (speed < 1) {
			I_error[i] += error*4;
		}

		sendfloat(10 + i, speed);

		if (target_speed[i] == 0) {
			I_error[i] = 0;
			power_value[i] = 0;
			setPower(i, power_value[i]);
			continue;
		}

		if (target_speed[i] == 0) {
			I_error[i] = 0;
			power_value[i] = 0;
			setPower(i, power_value[i]);
			continue;
		}
		float derror = last_error[i] - error;

		int p = (error * 35 * (5+target_speed[i])/target_speed[i] + I_error[i] * 8 + derror * 3);
		power_value[i] = p > 0 ? p : 0;

		last_error[i] = error;
		setDirection(i, dir[i]);
		setPower(i, power_value[i]);

	}
	last_tick = tick;

}

