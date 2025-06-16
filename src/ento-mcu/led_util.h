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

#define USER_LED_GREEN_PORT GPIOA
#define USER_LED_GREEN_PIN  LL_GPIO_PIN_5
#define USER_LED_PORT_CLK_ENABLE() { RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; }

#elif defined(STM32H7)
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_cortex.h>
#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_bus.h>

#define USER_LED_GREEN_PORT GPIOB
#define USER_LED_GREEN_PIN  LL_GPIO_PIN_0
#define USER_LED_PORT_CLK_ENABLE() LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB)

#elif defined(STM32F7)
#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_cortex.h>
#include <stm32f7xx_ll_rcc.h>
#include <stm32f7xx_ll_bus.h>

#define USER_LED_GREEN_PORT      GPIOB
#define USER_LED_GREEN_PIN LL_GPIO_PIN_0
#define USER_LED_RED_PIN   LL_GPIO_PIN_7
#define USER_LED_BLUE_PIN  LL_GPIO_PIN_14

#define USER_LED_PORT_CLK_ENABLE()   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

#elif defined(STM32G0)
#include <stm32g0xx_ll_gpio.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_bus.h>

#define USER_LED_GREEN_PORT      GPIOA
#define USER_LED_GREEN_PIN LL_GPIO_PIN_5

#define USER_LED_PORT_CLK_ENABLE() LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

#elif defined(STM32C0)
// C0 family configuration - using C0 specific headers
#include <stm32c0xx_ll_gpio.h>
#include <stm32c0xx_ll_cortex.h>
#include <stm32c0xx_ll_rcc.h>
#include <stm32c0xx_ll_bus.h>

// C092RC LED configuration (assuming PA5 like many STM32 boards)
#define USER_LED_GREEN_PORT      GPIOA
#define USER_LED_GREEN_PIN LL_GPIO_PIN_5

#define USER_LED_PORT_CLK_ENABLE() LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
 
#elif defined(STM32U5)
#include <stm32u5xx_ll_gpio.h>
#include <stm32u5xx_ll_cortex.h>
#include <stm32u5xx_ll_rcc.h>
#include <stm32u5xx_ll_bus.h>

#define USER_LED_RED_PORT      GPIOG
#define USER_LED_GREEN_PORT    GPIOC
#define USER_LED_BLUE_PORT     GPIOB
#define USER_LED_RED_PIN       LL_GPIO_PIN_2
#define USER_LED_GREEN_PIN     LL_GPIO_PIN_7
#define USER_LED_BLUE_PIN      LL_GPIO_PIN_7

#define USER_LED_PORT_CLK_ENABLE()   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

#endif

void UserLED_Init(void);
void UserLED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif // LED_UTIL_H
