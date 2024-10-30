#ifndef FLASH_UTIL_H
#define FLASH_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef STM32G4

#include <stm32g4xx_ll_system.h>

#endif

void     enable_instruction_cache();
void     disable_instruction_cache();
void     enable_instruction_cache_prefetch();
void     disable_instruction_cache_prefetch();
uint32_t is_instruction_cache_prefetch_enabled();
uint32_t get_flash_latency();


#ifdef __cplusplus
}
#endif



#endif // FLASH_UTIL_H
