/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SSD1306.h"
#include "WS2812B.h"
#include "MPU6050.h"
#include "motion.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct MPU6050 m_mpu6050_readData(uint8_t *buffer) {
	// get raw readings

	int16_t rawAccX = buffer[0] << 8 | buffer[1];
	int16_t rawAccY = buffer[2] << 8 | buffer[3];
	int16_t rawAccZ = buffer[4] << 8 | buffer[5];

	int16_t rawGyroX = buffer[8] << 8 | buffer[9];
	int16_t rawGyroY = buffer[10] << 8 | buffer[11];
	int16_t rawGyroZ = buffer[12] << 8 | buffer[13];

	float accel_scale = 16384;
	// setup range dependent scaling
	float accX = ((float) rawAccX) / accel_scale;
	float accY = ((float) rawAccY) / accel_scale;
	float accZ = ((float) rawAccZ) / accel_scale;

	float gyro_scale = 65.5;

	float gyroX = ((float) rawGyroX) / gyro_scale;
	float gyroY = ((float) rawGyroY) / gyro_scale;
	float gyroZ = ((float) rawGyroZ) / gyro_scale;

	struct MPU6050 received_mpu;
	received_mpu.accX = accX;
	received_mpu.accY = accY;
	received_mpu.accZ = accZ;
	received_mpu.gyroX = gyroX;
	received_mpu.gyroY = gyroY;
	received_mpu.gyroZ = gyroZ;

	return received_mpu;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	ssd1306_begin(&hi2c1);

	mpu6050_begin(&hi2c1);
	drv8801_begin(&htim1, &htim2, &htim4, &htim3);

	uint8_t grb[] = { 0xff, 0x00, 0x00 };
	uint32_t last_tick = HAL_GetTick();

	/* motor test code */
	if (0) {
		setPower(0, 1000);
		setPower(1, 1000);
		setPower(2, 1000);

		{
			setDirection(0, 0);
			setDirection(1, 0);
			setDirection(2, 0);
			HAL_Delay(2000);
		}
		{
			setDirection(0, 1);
			setDirection(1, 1);
			setDirection(2, 1);
			HAL_Delay(2000);
		}

		{
			setPower(0, 0);
			setDirection(1, 0);
			setDirection(2, 1);
			HAL_Delay(2000);
		}
		{
			setPower(0, 0);
			setDirection(1, 1);
			setDirection(2, 0);
			HAL_Delay(2000);
		}
	}

	setDirection(0, 0);
	setDirection(1, 1);

//	setPower(0, 700);
//	setPower(1, 700);
//	setPower(2, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int last_x_count = 0;
	uint8_t MPU_real_buffer[14];
	int count = 0;
//	struct MPU6050 received_mpu;

//	float AccErrorX = 0;
//	float AccErrorY = 0;
//	float GyroErrorX = 0;
//	float GyroErrorY = 0;
//	float GyroErrorZ = 0;

	float AccErrorX = -10.75;
	float AccErrorY = -0;
	float GyroErrorX = 60;
	float GyroErrorY = 60;
	float GyroErrorZ = 20;

	float gyroAngleX = 0; // deg/s * s = deg
	float gyroAngleY = 0;

	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	float recorded_roll = 0;
	float recorded_pitch = 0;
	float recorded_yaw = 0;

	uint8_t UART2_rxBuffer[3];
	while (1) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

//		writeRGB(grb);
//		writeRGB(grb);
//		writeRGB(grb);
//		writeRGB(grb);
//		writeRGB(grb);
//		writeRGB(grb);

//		grb[2] += 16;
//		grb[1] += 16;

//		mpu6050_readData();
//
		uint8_t c[20];

		clearDisplay();

		uint8_t UART2_rx;
		static uint8_t uart2_count = 0;

		mpu6050_readData();
		uint8_t calibrate_error = 1;
//		if (calibrate_error) {
//			if (count < 200) {
//				AccErrorX = AccErrorX
//						+ ((atan(
//								(mpu6050.accY)
//										/ sqrt(
//												pow((mpu6050.accX), 2)
//														+ pow((mpu6050.accZ),
//																2))) * 180
//								/ M_PI));
//				AccErrorY = AccErrorY
//						+ ((atan(
//								-1 * (mpu6050.accX)
//										/ sqrt(
//												pow((mpu6050.accY), 2)
//														+ pow((mpu6050.accZ),
//																2))) * 180
//								/ M_PI));
//				GyroErrorX = GyroErrorX + mpu6050.gyroX;
//				GyroErrorY = GyroErrorY + mpu6050.gyroY;
//				GyroErrorZ = GyroErrorZ + mpu6050.gyroZ;
//				count++;
//
//				sprintf(c, "%f      ", AccErrorX / count);
//				drawString(0, 0, c, 9, SSD1306_WHITE,
//				SSD1306_BLACK);
//				sprintf(c, "%f      ", AccErrorY / count);
//				drawString(0, 8, c, 9, SSD1306_WHITE,
//				SSD1306_BLACK);
//				sprintf(c, "%f      ", GyroErrorX / count);
//				drawString(0, 16, c, 9, SSD1306_WHITE,
//				SSD1306_BLACK);
//				sprintf(c, "%f      ", GyroErrorY / count);
//				drawString(0, 24, c, 9, SSD1306_WHITE,
//				SSD1306_BLACK);
//				sprintf(c, "%f      ", GyroErrorZ / count);
//				drawString(60, 0, c, 9, SSD1306_WHITE,
//				SSD1306_BLACK);
//				sprintf(c, "%d      ", count);
//				drawString(60, 8, c, 9, SSD1306_WHITE,
//				SSD1306_BLACK);
//				if (count == 200) {
//					HAL_Delay(1000000);
//				}
//			}
//		}

//		static int encoder_counter = -1;
//
//		if (HAL_UART_Receive(&huart2, &UART2_rxBuffer, 1, 100) == HAL_OK) {
//			drawString(0, 24, &UART2_rxBuffer, 1, SSD1306_WHITE, SSD1306_BLACK);
//			switch (UART2_rxBuffer) {
//			case 'W': {
//				move(MOTION_FORWARD);
//				break;
//			}
//			case 'A': {
//				move(MOTION_TRANSLATE_LEFT);
//				break;
//			}
//			case 'S': {
//				move(MOTION_BACKWARD);
//				break;
//			}
//			case 'D': {
//				move(MOTION_TRANSLATE_RIGHT);
//				break;
//			}
//			case 'Q': {
//				move(MOTION_ROTATE_LEFT);
//				break;
//			}
//			case 'E': {
//				move(MOTION_ROTATE_RIGHT);
//				break;
//			}
//			case 'K': {
//				move(MOTION_STOP);
//				break;
//			}
//
//			}
//		}
//		MotorUpdate();
		uint8_t MPU_buffer[43];
		if (HAL_UART_Receive(&huart2, MPU_buffer, 43, 1000) == HAL_OK) {
			uint8_t i = 0;
			for (; i < 43 - 16; ++i) {
				if (MPU_buffer[i] == 'x' && MPU_buffer[i + 1] == 'x'
						&& MPU_buffer[i + 2] == 'x') {
					struct MPU6050 received_mpu = m_mpu6050_readData(
							MPU_buffer+ i + 3);
					uint8_t is_pressing = MPU_buffer[i + 3 + 14];
					static uint8_t was_pressing = 0;

					sprintf(c, "pres %u     ", is_pressing);
					drawString(60, 0, c, 9, SSD1306_WHITE, SSD1306_BLACK);
//					sprintf(c, "%f      ", received_mpu.accX);
//					drawString(0, 0, c, 9, SSD1306_WHITE, SSD1306_BLACK);
//					sprintf(c, "%f      ", received_mpu.accY);
//					drawString(0, 8, c, 9, SSD1306_WHITE, SSD1306_BLACK);
//					sprintf(c, "%f      ", received_mpu.accZ);
//					drawString(0, 16, c, 9, SSD1306_WHITE, SSD1306_BLACK);

					uint8_t calibrate_error = 1;

					//source: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

					if (calibrate_error) {
						if (count < 200) {
							AccErrorX =
									AccErrorX
											+ ((atan(
													(received_mpu.accY)
															/ sqrt(
																	pow(
																			(received_mpu.accX),
																			2)
																			+ pow(
																					(received_mpu.accZ),
																					2)))
													* 180 / M_PI));
							AccErrorY =
									AccErrorY
											+ ((atan(
													-1 * (received_mpu.accX)
															/ sqrt(
																	pow(
																			(received_mpu.accY),
																			2)
																			+ pow(
																					(received_mpu.accZ),
																					2)))
													* 180 / M_PI));
							GyroErrorX = GyroErrorX + received_mpu.gyroX;
							GyroErrorY = GyroErrorY + received_mpu.gyroY;
							GyroErrorZ = GyroErrorZ + received_mpu.gyroZ;
							count++;
//							sprintf(c, "X%f      ", received_mpu.gyroX);
//							drawString(0, 0, c, 9, SSD1306_WHITE,
//							SSD1306_BLACK);
//							sprintf(c, "Y%f      ", received_mpu.gyroY);
//							drawString(0, 8, c, 9, SSD1306_WHITE,
//							SSD1306_BLACK);
//							sprintf(c, "Z%f      ", received_mpu.gyroZ);
//							drawString(0, 16, c, 9, SSD1306_WHITE,
//							SSD1306_BLACK);

							sprintf(c, "%f      ", AccErrorX / count);
							drawString(0, 0, c, 9, SSD1306_WHITE,
							SSD1306_BLACK);
							sprintf(c, "%f      ", AccErrorY / count);
							drawString(0, 8, c, 9, SSD1306_WHITE,
							SSD1306_BLACK);
							sprintf(c, "%f      ", GyroErrorX / count);
							drawString(0, 16, c, 9, SSD1306_WHITE,
							SSD1306_BLACK);
							sprintf(c, "%f      ", GyroErrorY / count);
							drawString(0, 24, c, 9, SSD1306_WHITE,
							SSD1306_BLACK);
							sprintf(c, "%f      ", GyroErrorZ / count);
							drawString(60, 0, c, 9, SSD1306_WHITE,
							SSD1306_BLACK);
							sprintf(c, "%d      ", count);
							drawString(60, 8, c, 9, SSD1306_WHITE,
							SSD1306_BLACK);
							if (count == 200) {
								HAL_Delay(1000000);
							}
						}
					} else {
						float accAngleX =
								((atan(
										(received_mpu.accY)
												/ sqrt(
														pow((received_mpu.accX),
																2)
																+ pow(
																		(received_mpu.accZ),
																		2)))
										* 180 / M_PI)) - AccErrorX;

						float accAngleY =
								((atan(
										-1 * (received_mpu.accX)
												/ sqrt(
														pow((received_mpu.accY),
																2)
																+ pow(
																		(received_mpu.accZ),
																		2)))
										* 180 / M_PI)) - AccErrorY;

						float GyroX = received_mpu.gyroX - GyroErrorX;
						float GyroY = received_mpu.gyroY - GyroErrorY;
						float GyroZ = received_mpu.gyroZ - GyroErrorZ;
						// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
						static uint32_t last_MPU_tick = 0;
						uint32_t MPU_tick = HAL_GetTick();
						float elapsedTime = ((float) (MPU_tick - last_MPU_tick))
								/ 1000.0f;
						if (last_MPU_tick != 0) {

							gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
							gyroAngleY = gyroAngleY + GyroY * elapsedTime;
							yaw = yaw + GyroZ * elapsedTime;
							roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
							pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
						}
						last_MPU_tick = MPU_tick;

//						if (is_pressing && !was_pressing) {
//							recorded_roll = roll;
//							recorded_pitch = pitch;
//							recorded_yaw = yaw;
//						}
//						if (is_pressing) {
						sprintf(c, "r:%f      ", roll);
						drawString(0, 0, c, 9, SSD1306_WHITE,
						SSD1306_BLACK);
						sprintf(c, "p:%f      ", pitch);
						drawString(0, 8, c, 9, SSD1306_WHITE,
						SSD1306_BLACK);
						sprintf(c, "y:%f      ", yaw);
						drawString(0, 16, c, 9, SSD1306_WHITE,
						SSD1306_BLACK);
//						}
						was_pressing = is_pressing;

//						sprintf(c, "gX:%f      ", gyroAngleX);
//						drawString(60, 0, c, 9, SSD1306_WHITE,
//						SSD1306_BLACK);
//						sprintf(c, "gY:%f      ", gyroAngleY);
//						drawString(60, 8, c, 9, SSD1306_WHITE,
//						SSD1306_BLACK);

						sprintf(c, "s:%f      ", elapsedTime);
						drawString(60, 16, c, 9, SSD1306_WHITE,
						SSD1306_BLACK);

					}

					break;
				} else {
				}
			}
			if (i == 37) {
				sprintf(c, "Not OK!", i);
				drawString(0, 16, c, 9, SSD1306_WHITE, SSD1306_BLACK);
			}

		} else {
			sprintf(c, "no 37 buffer");
			drawString(0, 24, c, 9, SSD1306_WHITE, SSD1306_BLACK);
		}

		display();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
