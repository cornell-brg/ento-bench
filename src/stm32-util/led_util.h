#ifndef LED_UTIL_H
#define LED_UTIL_H

#if defined STM32G4
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_rcc.h>
#define USER_LED_PORT GPIOA
#define USER_LED_PIN  LL_GPIO_PIN_5
#define USER_LED_PORT_CLK_ENABLE() { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; }
#endif


void UserLED_Init(void);
void UserLED_Toggle(void);

#endif
