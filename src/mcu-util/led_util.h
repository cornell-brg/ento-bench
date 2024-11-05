#ifndef LED_UTIL_H
#define LED_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#if defined(STM32G4)
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_rcc.h>
#define USER_LED_PORT GPIOA
#define USER_LED_PIN  LL_GPIO_PIN_5
#define USER_LED_PORT_CLK_ENABLE() { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; }
#elif defined(STM32H7)
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_cortex.h>
#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_bus.h>
#define USER_LED_PORT GPIOB
#define USER_LED_PIN  LL_GPIO_PIN_0
#define USER_LED_PORT_CLK_ENABLE() LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB)
#endif


void UserLED_Init(void);
void UserLED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif // LED_UTIL_H
