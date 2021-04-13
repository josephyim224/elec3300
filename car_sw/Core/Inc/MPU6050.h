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

#define MPU6050_I2CADDR_DEFAULT 0x68 ///< MPU6050 default i2c address w/ AD0 high
#define MPU6050_DEVICE_ID 0x68 ///< The correct MPU6050_WHO_AM_I value

#define MPU6050_SELF_TEST_X 0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y 0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z 0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A 0x10 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 0x19  ///< sample rate divisor register
#define MPU6050_CONFIG 0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 0x37    ///< Interrupt pin configuration register
#define MPU6050_WHO_AM_I 0x75          ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 0x6A         ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 0x6B        ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2 0x6C ///< Secondary power/sleep control register
#define MPU6050_TEMP_H 0x41     ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42     ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT 0x3B  ///< base address for sensor data reads

/**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
typedef enum fsync_out {
	MPU6050_FSYNC_OUT_DISABLED,
	MPU6050_FSYNC_OUT_TEMP,
	MPU6050_FSYNC_OUT_GYROX,
	MPU6050_FSYNC_OUT_GYROY,
	MPU6050_FSYNC_OUT_GYROZ,
	MPU6050_FSYNC_OUT_ACCELX,
	MPU6050_FSYNC_OUT_ACCELY,
	MPU6050_FSYNC_OUT_ACCEL_Z,
} mpu6050_fsync_out_t;

/**
 * @brief Clock source options
 *
 * Allowed values for `setClock`.
 */
typedef enum clock_select {
	MPU6050_INTR_8MHz,
	MPU6050_PLL_GYROX,
	MPU6050_PLL_GYROY,
	MPU6050_PLL_GYROZ,
	MPU6050_PLL_EXT_32K,
	MPU6050_PLL_EXT_19MHz,
	MPU6050_STOP = 7,
} mpu6050_clock_select_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
	MPU6050_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
	MPU6050_RANGE_4_G = 0b01,  ///< +/- 4g
	MPU6050_RANGE_8_G = 0b10,  ///< +/- 8g
	MPU6050_RANGE_16_G = 0b11, ///< +/- 16g
} mpu6050_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum {
	MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
	MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
	MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
	MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum {
	MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
	MPU6050_BAND_184_HZ, ///< 184 Hz
	MPU6050_BAND_94_HZ,  ///< 94 Hz
	MPU6050_BAND_44_HZ,  ///< 44 Hz
	MPU6050_BAND_21_HZ,  ///< 21 Hz
	MPU6050_BAND_10_HZ,  ///< 10 Hz
	MPU6050_BAND_5_HZ,   ///< 5 Hz
} mpu6050_bandwidth_t;

/**
 * @brief Periodic measurement options
 *
 * Allowed values for `setCycleRate`.
 */
typedef enum {
	MPU6050_CYCLE_1_25_HZ, ///< 1.25 Hz
	MPU6050_CYCLE_5_HZ,    ///< 5 Hz
	MPU6050_CYCLE_20_HZ,   ///< 20 Hz
	MPU6050_CYCLE_40_HZ,   ///< 40 Hz
} mpu6050_cycle_rate_t;

uint8_t mpu6050_begin(I2C_HandleTypeDef *SSD1306_hi2c);

mpu6050_accel_range_t getAccelerometerRange(void);
void setAccelerometerRange(mpu6050_accel_range_t);

mpu6050_gyro_range_t getGyroRange(void);
void setGyroRange(mpu6050_gyro_range_t);

void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);
mpu6050_bandwidth_t getFilterBandwidth(void);

void setSampleRateDivisor(uint8_t);
uint8_t getSampleRateDivisor(void);

void enableSleep(uint8_t enable);
void enableCycle(uint8_t enable);

void setCycleRate(mpu6050_cycle_rate_t rate);
mpu6050_cycle_rate_t getCycleRate(void);
void mpu6050_reset(void);

void mpu6050_readData(void);

struct MPU6050 {
	int8_t i2caddr;
	float temperature, 	///< Last reading's temperature (C)
			accX,		///< Last reading's accelerometer X axis m/s^2
			accY,		///< Last reading's accelerometer Y axis m/s^2
			accZ,		///< Last reading's accelerometer Z axis m/s^2
			gyroX,		///< Last reading's gyro X axis in rad/s
			gyroY,		///< Last reading's gyro Y axis in rad/s
			gyroZ;		///< Last reading's gyro Z axis in rad/s
	int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
	I2C_HandleTypeDef *MPU6050_hi2c;
} mpu6050;

#endif /* INC_MPU6050_H_ */
