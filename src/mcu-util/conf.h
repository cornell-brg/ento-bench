#ifndef CONF_H 
#define CONF_H

#if defined(STM32G4)

#include <stm32g4xx.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_utils.h>
#include <stm32g4xx_ll_pwr.h>

#include <core_cm4.h>

inline const GPIO_TypeDef* latency_gpio_port = GPIOA;
inline const uint32_t      latency_gpio_pin  = 8;
inline const uint32_t      gpio_port_a_rcc   = RCC_AHB2ENR_GPIOAEN;
inline const uint32_t      gpio_port_b_rcc   = RCC_AHB2ENR_GPIOBEN;
inline const uint32_t      gpio_port_c_rcc   = RCC_AHB2ENR_GPIOCEN;
inline const uint32_t      gpio_port_d_rcc   = RCC_AHB2ENR_GPIODEN;
inline const uint32_t      gpio_port_e_rcc   = RCC_AHB2ENR_GPIOEEN;
inline const uint32_t      gpio_port_f_rcc   = RCC_AHB2ENR_GPIOGEN;
inline const uint32_t      gpio_port_g_rcc   = RCC_AHB2ENR_GPIOGEN;

#elif defined(STM32H7)

#include <stm32h7xx.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_system.h>
#include <stm32h7xx_ll_utils.h>
#include <stm32h7xx_ll_pwr.h>

#include <core_cm7.h>

inline const GPIO_TypeDef* latency_gpio_port = GPIOG:
inline const uint32_t      latency_gpio_pin  = 12;
inline const uint32_t      gpio_port_a_rcc   = RCC_AHB4ENR_GPIOAEN;
inline const uint32_t      gpio_port_b_rcc   = RCC_AHB4ENR_GPIOBEN;
inline const uint32_t      gpio_port_c_rcc   = RCC_AHB4ENR_GPIOCEN;
inline const uint32_t      gpio_port_d_rcc   = RCC_AHB4ENR_GPIODEN;
inline const uint32_t      gpio_port_e_rcc   = RCC_AHB4ENR_GPIOEEN;
inline const uint32_t      gpio_port_f_rcc   = RCC_AHB4ENR_GPIOFEN;
inline const uint32_t      gpio_port_g_rcc   = RCC_AHB4ENR_GPIOGEN;
inline const uint32_t      gpio_port_h_rcc   = RCC_AHB4ENR_GPIOHEN;
inline const uint32_t      gpio_port_i_rcc   = RCC_AHB4ENR_GPIOIEN;
inline const uint32_t      gpio_port_j_rcc   = RCC_AHB4ENR_GPIOJEN;
inline const uint32_t      gpio_port_k_rcc   = RCC_AHB4ENR_GPIOKEN;

#elif defined(STM32F7)

#include <stm32f7xx.h>
#include <stm32f7xx_ll_bus.h>
#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_rcc.h>
#include <stm32f7xx_ll_system.h>
#include <stm32f7xx_ll_cortex.h>
#include <stm32f7xx_ll_utils.h>
#include <stm32f7xx_ll_pwr.h>

#include <core_cm7.h>

inline const GPIO_TypeDef* latency_gpio_port = GPIOF;
inline const uint32_t      latency_gpio_pin  = 13;
inline const uint32_t      gpio_port_a_rcc    = RCC_AHB1ENR_GPIOAEN;
inline const uint32_t      gpio_port_b_rcc    = RCC_AHB1ENR_GPIOBEN;
inline const uint32_t      gpio_port_c_rcc    = RCC_AHB1ENR_GPIOCEN;
inline const uint32_t      gpio_port_d_rcc    = RCC_AHB1ENR_GPIODEN;
inline const uint32_t      gpio_port_e_rcc    = RCC_AHB1ENR_GPIOEEN;
inline const uint32_t      gpio_port_f_rcc    = RCC_AHB1ENR_GPIOFEN;
inline const uint32_t      gpio_port_g_rcc    = RCC_AHB1ENR_GPIOGEN;
inline const uint32_t      gpio_port_h_rcc    = RCC_AHB1ENR_GPIOHEN;
inline const uint32_t      gpio_port_i_rcc    = RCC_AHB1ENR_GPIOIEN;
inline const uint32_t      gpio_port_j_rcc    = RCC_AHB1ENR_GPIOJEN;
inline const uint32_t      gpio_port_k_rcc    = RCC_AHB1ENR_GPIOKEN;

#elif defined(STM32G0)

#include <stm32g0xx.h>
#include <stm32g0xx_ll_bus.h>
#include <stm32g0xx_ll_gpio.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_system.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_utils.h>
#include <stm32g0xx_ll_pwr.h>

#include <core_cm4.h>

inline const GPIO_TypeDef* latency_gpio_port = GPIOA;
inline const uint32_t      latency_gpio_pin  = 8;
inline const uint32_t      gpio_port_a_rcc    = RCC_IOPENR_GPIOAEN;
inline const uint32_t      gpio_port_b_rcc    = RCC_IOPENR_GPIOBEN;
inline const uint32_t      gpio_port_c_rcc    = RCC_IOPENR_GPIOCEN;
inline const uint32_t      gpio_port_d_rcc    = RCC_IOPENR_GPIODEN;
inline const uint32_t      gpio_port_e_rcc    = RCC_IOPENR_GPIOEEN;
inline const uint32_t      gpio_port_f_rcc    = RCC_IOPENR_GPIOFEN;
                            
#elif defined(STM32U5)      
                            
#include <stm32u5xx.h>      
#include <stm32u5xx_ll_bus.h>
#include <stm32u5xx_ll_gpio.h>
#include <stm32u5xx_ll_rcc.h>
#include <stm32u5xx_ll_system.h>
#include <stm32u5xx_ll_cortex.h>
#include <stm32u5xx_ll_utils.h>
#include <stm32u5xx_ll_pwr.h>

#include <core_cm33.h>

inline GPIO_TypeDef*       latency_gpio_port  = GPIOF;
inline uint32_t            latency_gpio_pin   = LL_GPIO_PIN_13;
inline const uint32_t      gpio_port_a_rcc    = RCC_AHB2ENR1_GPIOAEN;
inline const uint32_t      gpio_port_b_rcc    = RCC_AHB2ENR1_GPIOBEN;
inline const uint32_t      gpio_port_c_rcc    = RCC_AHB2ENR1_GPIOCEN;
inline const uint32_t      gpio_port_d_rcc    = RCC_AHB2ENR1_GPIODEN;
inline const uint32_t      gpio_port_e_rcc    = RCC_AHB2ENR1_GPIOEEN;
inline const uint32_t      gpio_port_f_rcc    = RCC_AHB2ENR1_GPIOFEN;
inline const uint32_t      gpio_port_g_rcc    = RCC_AHB2ENR1_GPIOGEN;
inline const uint32_t      gpio_port_h_rcc    = RCC_AHB2ENR1_GPIOHEN;
inline const uint32_t      gpio_port_i_rcc    = RCC_AHB2ENR1_GPIOIEN;

#endif // if defined({STM_FAMILY})

#endif // CONF_H
