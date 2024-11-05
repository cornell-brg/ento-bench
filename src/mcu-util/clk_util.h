#ifndef CLK_UTIL_H
#define CLK_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined(STM32G4)
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_pwr.h>
#include <stm32g4xx_ll_utils.h>
#include <stm32g4xx_ll_bus.h>
#elif defined(STM32H7)
#include <stm32h7xx.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_system.h>
#include <stm32h7xx_ll_utils.h>
#include <stm32h7xx_ll_pwr.h>
#endif


uint32_t get_sys_clk_freq(void);
uint32_t get_AHB_clk_freq(void);
uint32_t get_APB2_clk_freq(void);
uint32_t get_APB2_clk_freq(void);
void     sys_clk_cfg();

#ifdef __cplusplus
}
#endif

#endif // CLK_UTIL_H
