#ifndef __MAIN_H
#define __MAIN_H

#include "stm8s.h"

#define LED0_GPIO_PORT  (GPIOC)
#define LED0_GPIO_PIN  (GPIO_PIN_7)

#define TOP_BUTTON_GPIO_PORT (GPIOA)
#define TOP_BUTTON_GPIO_PIN (GPIO_PIN_1)

#define SIDE_BUTTON_GPIO_PORT (GPIOA)
#define SIDE_BUTTON_GPIO_PIN (GPIO_PIN_3)

#define SCL_GPIO_PORT (GPIOB)
#define SCL_GPIO_PIN (GPIO_PIN_4)

#define SDA_GPIO_PORT (GPIOB)
#define SDA_GPIO_PIN (GPIO_PIN_5)

#endif