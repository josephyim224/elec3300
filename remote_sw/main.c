/**
  ******************************************************************************
  * @file    GPIO_Toggle\main.c
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    26-April-2018
  * @brief   This file contains the main function for GPIO Toggle example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "SSD1306.h"
#include "MPU6050.h"

/**
  * @addtogroup GPIO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern struct MPU6050 mpu6050;
/* Private function prototypes -----------------------------------------------*/
void Delay(uint16_t nCount);
void BT_Send(uint8_t *, uint8_t);
/* Private functions ---------------------------------------------------------*/

void initPeripherals();

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  initPeripherals();

  uint8_t tick = 0;
  uint8_t mode = 0;
  uint8_t topStart = 0;
  uint8_t topped = 0;
  while (1)
  {
    uint8_t top = !GPIO_ReadInputPin(TOP_BUTTON_GPIO_PORT, TOP_BUTTON_GPIO_PIN);
    uint8_t side = !GPIO_ReadInputPin(SIDE_BUTTON_GPIO_PORT, SIDE_BUTTON_GPIO_PIN);

    if (tick % 10 == 0)
    {
      GPIO_WriteReverse(LED0_GPIO_PORT, LED0_GPIO_PIN);

      if (top)
      {
        if (!topped)
          topStart = tick;
        topped = 1;
      }
      else if (topped)
      {
        topped = 0;
        if (tick - topStart < 20)
          mode = mode ? 0 : 1;
      }

      if (top == mode || side)
      {
        uint16_t volt = (uint16_t)((uint32_t)100 * ADC1_GetConversionValue() * 4 / 696);

        clearDisplay();
        drawString(0, 0, "ti", 2);
        drawUint16(24, 0, tick);
        drawString(0, 8, "vo", 2);
        drawUint16(24, 8, volt);
        drawUint16(0, 16, mpu6050.buffer[0]);

        drawUint16(72, 0, mpu6050.who_am_i);

        if (volt < 360)
          drawString(0, 24, "LOW BATT", 8);
        if (top)
          drawString(54, 24, "TOP", 3);
        if (side)
          drawString(78, 24, "SIDE", 4);

        display();
      }
    }

    mpu6050_readData();

    for (uint8_t i = 0; i < 2; ++i)
    {
      BT_Send("xxx", 3);
      BT_Send(mpu6050.buffer, 14);
      BT_Send(&top, 1);
      BT_Send(&side, 1);
      BT_Send("yyy", 3);
    }

    tick += 1;
  }
}

void initPeripherals()
{
  // wait for clock to settle
  for (uint16_t i = 0; i < 0xffff; ++i)
    ;

  disableInterrupts();

  // clk init
  CLK->ICKR = CLK_ICKR_RESET_VALUE;
  CLK->ECKR = CLK_ECKR_RESET_VALUE;
  CLK->SWR = CLK_SWR_RESET_VALUE;
  CLK->SWCR = CLK_SWCR_RESET_VALUE;
  CLK->CKDIVR = CLK_CKDIVR_RESET_VALUE;
  CLK->PCKENR1 = CLK_PCKENR1_RESET_VALUE;
  CLK->PCKENR2 = CLK_PCKENR2_RESET_VALUE;
  CLK->CSSR = CLK_CSSR_RESET_VALUE;
  CLK->CCOR = CLK_CCOR_RESET_VALUE;
  while ((CLK->CCOR & CLK_CCOR_CCOEN) != 0)
    ;
  CLK->CCOR = CLK_CCOR_RESET_VALUE;
  CLK->HSITRIMR = CLK_HSITRIMR_RESET_VALUE;
  CLK->SWIMCCR = CLK_SWIMCCR_RESET_VALUE;
  CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
  CLK->CKDIVR |= (uint8_t)0;

  // gpio init
  GPIO_Init(TOP_BUTTON_GPIO_PORT, TOP_BUTTON_GPIO_PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(SIDE_BUTTON_GPIO_PORT, SIDE_BUTTON_GPIO_PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  // i2c init
  GPIO_Init(SCL_GPIO_PORT, SCL_GPIO_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
  GPIO_Init(SDA_GPIO_PORT, SDA_GPIO_PIN, GPIO_MODE_OUT_OD_HIZ_FAST);

  // uart
  UART1_DeInit();
  UART1_Init(38400, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
             (UART1_SyncMode_TypeDef)(UART1_SYNCMODE_CLOCK_ENABLE | UART1_SYNCMODE_CPOL_LOW | UART1_SYNCMODE_CPHA_MIDDLE | UART1_SYNCMODE_LASTBIT_ENABLE),
             UART1_MODE_TXRX_ENABLE);
  UART1_Cmd(ENABLE);

  // ssd1306
  ssd1306_begin();
  clearDisplay();

  // mpu6050
  mpu6050_begin();

  // adc1
  ADC1_DeInit();
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_4, ADC1_PRESSEL_FCPU_D8,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL4,
            DISABLE);
  ADC1_StartConversion();

  enableInterrupts();
}

/**
  * @brief Delay
  * @param nCount
  * @retval None
  */
void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

void BT_Send(uint8_t *data, uint8_t n)
{
  for (uint8_t i = 0; i < n; ++i)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    UART1_SendData8(data[i]);
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
