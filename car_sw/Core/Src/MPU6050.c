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

void mpu6050_read(uint8_t addr, uint8_t *d, uint8_t n) {
	HAL_I2C_Master_Transmit(mpu6050.MPU6050_hi2c, mpu6050.i2caddr, &addr, 1,
			1000);
	HAL_I2C_Master_Receive(mpu6050.MPU6050_hi2c, mpu6050.i2caddr, d, n, 1000);
}

void mpu6050_write(uint8_t addr, uint8_t *d, uint8_t n) {
	uint8_t v[2];
	v[0] = addr;
	v[1] = *d;
	HAL_I2C_Master_Transmit(mpu6050.MPU6050_hi2c, mpu6050.i2caddr, v, 2, 1000);
}

void mpu6050_delay(uint32_t delay) {
	HAL_Delay(delay);
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
uint8_t mpu6050_begin(I2C_HandleTypeDef *MPU1306_hi2c) {
	mpu6050.i2caddr = MPU6050_I2CADDR_DEFAULT;
	mpu6050.i2caddr = 0xD0 | 0x00; // 0x02
	mpu6050.MPU6050_hi2c = MPU1306_hi2c;

	if (HAL_I2C_IsDeviceReady(mpu6050.MPU6050_hi2c, mpu6050.i2caddr, 2, 5)
			!= HAL_OK) {
		return 0;
	}

	uint8_t who_am_i = 0;
	mpu6050_read(MPU6050_WHO_AM_I, &who_am_i, 1);

	// make sure we're talking to the right chip
	if (who_am_i != MPU6050_DEVICE_ID) {
		return 0;
	}

	mpu6050_reset();
	setSampleRateDivisor(0);
	setFilterBandwidth(MPU6050_BAND_260_HZ);
	setGyroRange(MPU6050_RANGE_500_DEG);
	setAccelerometerRange(MPU6050_RANGE_2_G); // already the default

	uint8_t power_mgmt_1 = 0x00;
	mpu6050_write(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);

	mpu6050_delay(100);

	return 1;
}

/**************************************************************************/
/*!
 @brief Resets registers to their initial value and resets the sensors'
 analog and digital signal paths.
 */
/**************************************************************************/
void mpu6050_reset(void) {
	uint8_t power_mgmt_1;
	mpu6050_read(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);
	power_mgmt_1 |= 0b10000000;
	mpu6050_write(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);

	do {
		mpu6050_delay(1);
		mpu6050_read(MPU6050_PWR_MGMT_1, &power_mgmt_1, 1);
	} while ((power_mgmt_1 & 0b10000000) != 0);
	mpu6050_delay(100);

	uint8_t sig_path_reset = 0x7;
	mpu6050_write(MPU6050_SIGNAL_PATH_RESET, &sig_path_reset, 1);
	mpu6050_delay(100);
}

/**************************************************************************/
/*!
 @brief  Sets the divisor used to divide the base clock rate into a
 measurement rate
 @param  divisor
 The new clock divisor
 */
/**************************************************************************/
void setSampleRateDivisor(uint8_t divisor) {
	uint8_t sample_rate_div = divisor;
	mpu6050_write(MPU6050_SMPLRT_DIV, &sample_rate_div, 1);
}

/**************************************************************************/
/*!
 @brief Gets the sample rate divisor.
 @return  The sample rate divisor
 */
/**************************************************************************/
uint8_t getSampleRateDivisor(void) {
	uint8_t sample_rate_div;
	mpu6050_read(MPU6050_SMPLRT_DIV, &sample_rate_div, 1);
	return sample_rate_div;
}

/**************************************************************************/
/*!
 *    @brief Sets the bandwidth of the Digital Low-Pass Filter
 *    @param bandwidth the new `mpu6050_bandwidth_t` bandwidth
 */
/**************************************************************************/
void setFilterBandwidth(mpu6050_bandwidth_t bandwidth) {
	uint8_t config;
	mpu6050_read(MPU6050_CONFIG, &config, 1);
	config = (config & 0b11111000) | (bandwidth << 0);
	mpu6050_write(MPU6050_CONFIG, &config, 1);
}

/**************************************************************************/
/*!
 *     @brief  Gets bandwidth of the Digital Low Pass Filter
 *     @return  The current `mpu6050_bandwidth_t` filter bandwidth
 */
/**************************************************************************/
mpu6050_bandwidth_t getFilterBandwidth(void) {
	uint8_t config;
	mpu6050_read(MPU6050_CONFIG, &config, 1);
	return (mpu6050_bandwidth_t) (config & 0b00000111) >> 0;
}

/**************************************************************************/
/*!
 @brief Sets the gyroscope measurement range
 @param  new_range
 The new range to set. Must be a `mpu6050_gyro_range_t`
 */
/**************************************************************************/
void setGyroRange(mpu6050_gyro_range_t new_range) {
	uint8_t gyro_config;
	mpu6050_read(MPU6050_GYRO_CONFIG, &gyro_config, 1);
	gyro_config = (gyro_config & 0b11100111) | (new_range << 3);
	mpu6050_write(MPU6050_GYRO_CONFIG, &gyro_config, 1);
}

/**************************************************************************/
/*!
 @brief Gets the gyroscope measurement range
 @return  The `mpu6050_gyro_range_t` gyroscope measurement range
 */
/**************************************************************************/
mpu6050_gyro_range_t getGyroRange(void) {
	uint8_t gyro_config;
	mpu6050_read(MPU6050_GYRO_CONFIG, &gyro_config, 1);
	return (gyro_config & 0b00011000) >> 3;
}

/**************************************************************************/
/*!
 @brief Sets the accelerometer measurement range
 @param  new_range
 The new range to set. Must be a `mpu6050_accel_range_t`
 */
/**************************************************************************/
void setAccelerometerRange(mpu6050_accel_range_t new_range) {
	uint8_t accel_config;
	mpu6050_read(MPU6050_ACCEL_CONFIG, &accel_config, 1);
	accel_config = (accel_config & 0b11100111) | (new_range << 3);
	mpu6050_write(MPU6050_ACCEL_CONFIG, &accel_config, 1);
}

/**************************************************************************/
/*!
 @brief Gets the acceleration measurement range.
 @return  The acceleration measurement range
 */
/**************************************************************************/
mpu6050_accel_range_t getAccelerometerRange(void) {
	uint8_t accel_config;
	mpu6050_read(MPU6050_ACCEL_CONFIG, &accel_config, 1);
	return (mpu6050_accel_range_t) (accel_config & 0b00011000) >> 3;
}

/**************************************************************************/
/*!
 *     @brief  Controls the sleep state of the sensor
 *     @param  enable
 If `true` the sensor is put into a low power state
 and measurements are halted until sleep mode is deactivated
 Setting `false` wakes up the sensor from sleep mode,
 resuming measurements.
 @returns True or false on successful write
 */
/**************************************************************************/
void enableSleep(uint8_t enable) {
	uint8_t pwr_mgmt;
	mpu6050_read(MPU6050_PWR_MGMT_1, &pwr_mgmt, 1);
	pwr_mgmt |= (enable & 0b1) << 6;
	mpu6050_write(MPU6050_PWR_MGMT_1, &pwr_mgmt, 1);
}

/**************************************************************************/
/*!
 *     @brief  Controls sensor's 'Cycle' measurement mode
 *     @param  enable
 If `true` the sensor will take measurements at the rate
 set by calling `setCycleRate`, sleeping between measurements.
 *Setting the sensor into 'Cycle' mode will have no effect
 if the sensor has been put into a sleep state with `enableSleep`
 Setting `false` returns the sensor to the normal
 measurement mode.
 @returns True or false on successful write
 */
/**************************************************************************/
void enableCycle(uint8_t enable) {
	uint8_t pwr_mgmt;
	mpu6050_read(MPU6050_PWR_MGMT_1, &pwr_mgmt, 1);
	pwr_mgmt |= (enable & 0b1) << 5;
	mpu6050_write(MPU6050_PWR_MGMT_1, &pwr_mgmt, 1);
}

/**************************************************************************/
/*!
 *     @brief  Sets the frequency of measurement in 'Cycle' mode
 *     @param  rate
 *              The `mpu6050_cycle_rate_t` specifying the desired
 *              measurement rate
 */
/**************************************************************************/
void setCycleRate(mpu6050_cycle_rate_t rate) {
	uint8_t pwr_mgmt_2;
	mpu6050_read(MPU6050_PWR_MGMT_2, &pwr_mgmt_2, 1);
	pwr_mgmt_2 = (pwr_mgmt_2 & 0b00111111) | ((rate & 0b11) << 6);
	mpu6050_write(pwr_mgmt_2, &pwr_mgmt_2, 1);
}

/**************************************************************************/
/*!
 *     @brief  Gets the frequencey of measurements in 'Cycle' mode
 *     @return  The current 'Cycle' measurement frequency
 */
/**************************************************************************/
mpu6050_cycle_rate_t getCycleRate(void) {
	uint8_t pwr_mgmt_2;
	mpu6050_read(MPU6050_PWR_MGMT_2, &pwr_mgmt_2, 1);
	return (mpu6050_cycle_rate_t) (pwr_mgmt_2 & 0b11000000) >> 6;
}

/******************* Adafruit_Sensor functions *****************/
/*!
 *     @brief  Updates the measurement data for all sensors simultaneously
 */
/**************************************************************************/
void mpu6050_readData(void) {
	// get raw readings
	uint8_t buffer[14];
	mpu6050_read(MPU6050_ACCEL_OUT, buffer, 14);

	mpu6050.rawAccX = buffer[0] << 8 | buffer[1];
	mpu6050.rawAccY = buffer[2] << 8 | buffer[3];
	mpu6050.rawAccZ = buffer[4] << 8 | buffer[5];

	mpu6050.rawTemp = buffer[6] << 8 | buffer[7];

	mpu6050.rawGyroX = buffer[8] << 8 | buffer[9];
	mpu6050.rawGyroY = buffer[10] << 8 | buffer[11];
	mpu6050.rawGyroZ = buffer[12] << 8 | buffer[13];

	mpu6050.temperature = (mpu6050.rawTemp / 340.0) + 36.53;

	mpu6050_accel_range_t accel_range = getAccelerometerRange();

	float accel_scale = 1;
	if (accel_range == MPU6050_RANGE_16_G)
		accel_scale = 2048;
	if (accel_range == MPU6050_RANGE_8_G)
		accel_scale = 4096;
	if (accel_range == MPU6050_RANGE_4_G)
		accel_scale = 8192;
	if (accel_range == MPU6050_RANGE_2_G)
		accel_scale = 16384;

	// setup range dependent scaling
	mpu6050.accX = ((float) mpu6050.rawAccX) / accel_scale;
	mpu6050.accY = ((float) mpu6050.rawAccY) / accel_scale;
	mpu6050.accZ = ((float) mpu6050.rawAccZ) / accel_scale;

	mpu6050_gyro_range_t gyro_range = getGyroRange();

	float gyro_scale = 1;
	if (gyro_range == MPU6050_RANGE_250_DEG)
		gyro_scale = 131;
	if (gyro_range == MPU6050_RANGE_500_DEG)
		gyro_scale = 65.5;
	if (gyro_range == MPU6050_RANGE_1000_DEG)
		gyro_scale = 32.8;
	if (gyro_range == MPU6050_RANGE_2000_DEG)
		gyro_scale = 16.4;

	mpu6050.gyroX = ((float) mpu6050.rawGyroX) / gyro_scale;
	mpu6050.gyroY = ((float) mpu6050.rawGyroY) / gyro_scale;
	mpu6050.gyroZ = ((float) mpu6050.rawGyroZ) / gyro_scale;
}
