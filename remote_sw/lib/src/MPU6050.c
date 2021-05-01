/*
 * MPU6050.c
 *
 *  Created on: Apr 13, 2021
 *      Author: joseph
 *
 *  Code references from
 *  https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.cpp
 */

#include <stdlib.h>
#include <string.h>

#include "MPU6050.h"
#include "softi2c.h"

struct MPU6050 mpu6050;

void mpu6050_read(uint8_t addr, uint8_t *d, uint8_t n)
{
	eeI2C_Start();
	eeI2C_Write(MPU6050_I2CADDR_DEFAULT & 0b11111110);
	eeI2C_Write(addr);
	eeI2C_Start();
	eeI2C_Write(MPU6050_I2CADDR_DEFAULT | 0x01);
	for (uint8_t i = 0; i < n - 1; ++i)
		d[i] = eeI2C_Read(1);
	d[n - 1] = eeI2C_Read(0);
	eeI2C_Stop();
}

void mpu6050_write(uint8_t addr, uint8_t *d, uint8_t n)
{
	eeI2C_Start();
	eeI2C_Write(MPU6050_I2CADDR_DEFAULT & 0b11111110);
	eeI2C_Write(addr);
	for (uint8_t i = 0; i < n; ++i)
		eeI2C_Write(d[i]);
	eeI2C_Stop();
}

void mpu6050_delay(uint8_t delay)
{
	uint16_t nCount = delay * 100;
	while (nCount != 0)
		nCount--;
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
void mpu6050_begin()
{
	{
		// mpu6050_reset();
		uint8_t power_mgmt_1;
		mpu6050_read(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);
		power_mgmt_1 |= 0b10000000;
		mpu6050_write(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);
		do
		{
			mpu6050_delay(1);
			mpu6050_read(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);
		} while ((power_mgmt_1 & 0b10000000) != 0);
		mpu6050_delay(100);
		uint8_t sig_path_reset = 0x7;
		mpu6050_write(MPU6050_SIGNAL_PATH_RESET, &sig_path_reset, 1);
		mpu6050_delay(100);
	}
	{
		// setSampleRateDivisor(0);
		uint8_t sample_rate_div = 0;
		mpu6050_write(MPU6050_SMPLRT_DIV, &sample_rate_div, 1);
	}
	{
		// setFilterBandwidth(MPU6050_BAND_260_HZ);
		uint8_t config;
		mpu6050_read(MPU6050_CONFIG, &config, 1);
		config = (config & 0b11111000);
		mpu6050_write(MPU6050_CONFIG, &config, 1);
	} {
		// setGyroRange(MPU6050_RANGE_500_DEG);
		uint8_t gyro_config;
		mpu6050_read(MPU6050_GYRO_CONFIG, &gyro_config, 1);
		gyro_config = (gyro_config & 0b11100111) | (0x01 << 3);
		mpu6050_write(MPU6050_GYRO_CONFIG, &gyro_config, 1);
	}
	{
		// setAccelerometerRange(MPU6050_RANGE_2_G); // already the default
		uint8_t accel_config;
		mpu6050_read(MPU6050_ACCEL_CONFIG, &accel_config, 1);
		accel_config = (accel_config & 0b11100111) | (0b00 << 3);
		mpu6050_write(MPU6050_ACCEL_CONFIG, &accel_config, 1);
	}

	uint8_t power_mgmt_1 = 0x00;
	mpu6050_write(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);
	mpu6050_delay(100);
}

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void mpu6050_readData(void)
{
	uint8_t *buffer = mpu6050.buffer;
	mpu6050_read(MPU6050_ACCEL_OUT, buffer, 14);

	// mpu6050.rawAccX = buffer[0] << 8 | buffer[1];
	// mpu6050.rawAccY = buffer[2] << 8 | buffer[3];
	// mpu6050.rawAccZ = buffer[4] << 8 | buffer[5];

	// mpu6050.rawTemp = buffer[6] << 8 | buffer[7];

	// mpu6050.rawGyroX = buffer[8] << 8 | buffer[9];
	// mpu6050.rawGyroY = buffer[10] << 8 | buffer[11];
	// mpu6050.rawGyroZ = buffer[12] << 8 | buffer[13];

	// mpu6050.accel_scale = 16384;
	// mpu6050.gyro_scale = 65.5;

	// mpu6050.temperature = (mpu6050.rawTemp / 340.0) + 36.53;
	// mpu6050.accX = ((float)mpu6050.rawAccX) / accel_scale;
	// mpu6050.accY = ((float)mpu6050.rawAccY) / accel_scale;
	// mpu6050.accZ = ((float)mpu6050.rawAccZ) / accel_scale;
	// mpu6050.gyroX = ((float)mpu6050.rawGyroX) / gyro_scale;
	// mpu6050.gyroY = ((float)mpu6050.rawGyroY) / gyro_scale;
	// mpu6050.gyroZ = ((float)mpu6050.rawGyroZ) / gyro_scale;
}
