/*
 * MPU6050.h
 *
 *  Created on: Apr 13, 2021
 *      Author: joseph
 *
 *  Code referenced from https://raw.githubusercontent.com/adafruit/Adafruit_MPU6050/master/Adafruit_MPU6050.h
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "stm8s.h"

#define MPU6050_I2CADDR_DEFAULT (0xD0) ///< MPU6050 default i2c address w/ AD0 high

#define MPU6050_SELF_TEST_X 0x0D	   ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y 0x0E	   ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z 0x0F	   ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A 0x10	   ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 0x19		   ///< sample rate divisor register
#define MPU6050_CONFIG 0x1A			   ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B	   ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C	   ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 0x37	   ///< Interrupt pin configuration register
#define MPU6050_WHO_AM_I 0x75		   ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 0x6A		   ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 0x6B		   ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2 0x6C		   ///< Secondary power/sleep control register
#define MPU6050_TEMP_H 0x41			   ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42			   ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT 0x3B		   ///< base address for sensor data reads

void mpu6050_begin();
void mpu6050_readData(void);

struct MPU6050
{
	int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
	float accel_scale, gyro_scale;
	uint8_t buffer[14];
};

#endif /* INC_MPU6050_H_ */
