#ifndef FLASH_UTIL_H
#define FLASH_UTIL_H

#include <stdint.h>

#if defined(STM32G4)
#include <stm32g4xx_ll_system.h>
#elif defined(STM32H7)
#include <stm32h7xx_ll_system.h>
#elif defined(STM32F7)
#include <stm32f7xx_ll_system.h>
#elif defined(STM32G0)
#include <stm32g0xx_ll_system.h>
#elif defined(STM32U5)
#include <stm32u5xx_ll_system.h>
#endif

// @TODO: Convert enable icache below to be enable
//  art instead
void     enable_art();
void     disable_art();
bool     is_art_enabled();
void     enable_art_prefetch();
void     disable_art_prefetch();
bool     is_art_prefetch_enabled();


void     enable_instruction_cache();
void     disable_instruction_cache();
void     enable_instruction_cache_prefetch();
void     disable_instruction_cache_prefetch();
bool     is_instruction_cache_prefetch_enabled();
uint32_t get_flash_latency();


#endif // FLASH_UTIL_H
