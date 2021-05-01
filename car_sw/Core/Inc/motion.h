/*
 * motion.h
 *
 *  Created on: Apr 16, 2021
 *      Author: jasoni111
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

typedef enum {
	MOTION_FORWARD,
	MOTION_BACKWARD,
	MOTION_TRANSLATE_LEFT,
	MOTION_TRANSLATE_RIGHT,
	MOTION_ROTATE_LEFT,
	MOTION_ROTATE_RIGHT,
	MOTION_STOP
} Direction;

void move(Direction dir);

#endif /* INC_MOTION_H_ */
