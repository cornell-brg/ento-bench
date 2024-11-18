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

#define latency_gpio_port GPIOB
#define  latency_gpio_pin LL_GPIO_PIN_5

#define trigger_gpio_port GPIOA
#define  trigger_gpio_pin LL_GPIO_PIN_8

#define   gpio_port_a_rcc RCC_AHB2ENR_GPIOAEN
#define   gpio_port_b_rcc RCC_AHB2ENR_GPIOBEN
#define   gpio_port_c_rcc RCC_AHB2ENR_GPIOCEN
#define   gpio_port_d_rcc RCC_AHB2ENR_GPIODEN
#define   gpio_port_e_rcc RCC_AHB2ENR_GPIOEEN
#define   gpio_port_f_rcc RCC_AHB2ENR_GPIOGEN
#define   gpio_port_g_rcc RCC_AHB2ENR_GPIOGEN

#elif defined(STM32H7)

#include <stm32h7xx.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_system.h>
#include <stm32h7xx_ll_utils.h>
#include <stm32h7xx_ll_pwr.h>

#include <core_cm7.h>

#define latency_gpio_port GPIOE
#define  latency_gpio_pin LL_GPIO_PIN_14

#define trigger_gpio_port GPIOG
#define  trigger_gpio_pin LL_GPIO_PIN_12

#define   gpio_port_a_rcc RCC_AHB4ENR_GPIOAEN
#define   gpio_port_b_rcc RCC_AHB4ENR_GPIOBEN
#define   gpio_port_c_rcc RCC_AHB4ENR_GPIOCEN
#define   gpio_port_d_rcc RCC_AHB4ENR_GPIODEN
#define   gpio_port_e_rcc RCC_AHB4ENR_GPIOEEN
#define   gpio_port_f_rcc RCC_AHB4ENR_GPIOFEN
#define   gpio_port_g_rcc RCC_AHB4ENR_GPIOGEN
#define   gpio_port_h_rcc RCC_AHB4ENR_GPIOHEN
#define   gpio_port_i_rcc RCC_AHB4ENR_GPIOIEN
#define   gpio_port_j_rcc RCC_AHB4ENR_GPIOJEN
#define   gpio_port_k_rcc RCC_AHB4ENR_GPIOKEN

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

#define latency_gpio_port GPIOF;
#define  latency_gpio_pin LL_GPIO_PIN_14

#define trigger_gpio_port GPIOF
#define  trigger_gpio_pin LL_GPIO_PIN_13

#define   gpio_port_a_rcc RCC_AHB1ENR_GPIOAEN
#define   gpio_port_b_rcc RCC_AHB1ENR_GPIOBEN
#define   gpio_port_c_rcc RCC_AHB1ENR_GPIOCEN
#define   gpio_port_d_rcc RCC_AHB1ENR_GPIODEN
#define   gpio_port_e_rcc RCC_AHB1ENR_GPIOEEN
#define   gpio_port_f_rcc RCC_AHB1ENR_GPIOFEN
#define   gpio_port_g_rcc RCC_AHB1ENR_GPIOGEN
#define   gpio_port_h_rcc RCC_AHB1ENR_GPIOHEN
#define   gpio_port_i_rcc RCC_AHB1ENR_GPIOIEN
#define   gpio_port_j_rcc RCC_AHB1ENR_GPIOJEN
#define   gpio_port_k_rcc RCC_AHB1ENR_GPIOKEN

#elif defined(STM32G0)

#include <stm32g0xx.h>
#include <stm32g0xx_ll_bus.h>
#include <stm32g0xx_ll_gpio.h>
#include <stm32g0xx_ll_rcc.h>
#include <stm32g0xx_ll_system.h>
#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_utils.h>
#include <stm32g0xx_ll_pwr.h>

#include <core_cm0plus.h>

#define latency_gpio_port GPIOB
#define  latency_gpio_pin LL_GPIO_PIN_5

#define trigger_gpio_port GPIOA
#define  trigger_gpio_pin LL_GPIO_PIN_8

#define   gpio_port_a_rcc RCC_IOPENR_GPIOAEN
#define   gpio_port_b_rcc RCC_IOPENR_GPIOBEN
#define   gpio_port_c_rcc RCC_IOPENR_GPIOCEN
#define   gpio_port_d_rcc RCC_IOPENR_GPIODEN
#define   gpio_port_e_rcc RCC_IOPENR_GPIOEEN
#define   gpio_port_f_rcc RCC_IOPENR_GPIOFEN
                            
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

#define latency_gpio_port GPIOF
#define  latency_gpio_pin LL_GPIO_PIN_14

#define trigger_gpio_port GPIOF
#define  trigger_gpio_pin LL_GPIO_PIN_13

#define   gpio_port_a_rcc RCC_AHB2ENR1_GPIOAEN
#define   gpio_port_b_rcc RCC_AHB2ENR1_GPIOBEN
#define   gpio_port_c_rcc RCC_AHB2ENR1_GPIOCEN
#define   gpio_port_d_rcc RCC_AHB2ENR1_GPIODEN
#define   gpio_port_e_rcc RCC_AHB2ENR1_GPIOEEN
#define   gpio_port_f_rcc RCC_AHB2ENR1_GPIOFEN
#define   gpio_port_g_rcc RCC_AHB2ENR1_GPIOGEN
#define   gpio_port_h_rcc RCC_AHB2ENR1_GPIOHEN
#define   gpio_port_i_rcc RCC_AHB2ENR1_GPIOIEN

#endif // if defined({STM_FAMILY})

#endif // CONF_H
