#ifndef PWR_UTIL_H
#define PWR_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef STM32G4
#include <stm32g4xx_ll_pwr.h>
#endif

// Function to get the current Vcore voltage scaling range (VOS)
uint32_t get_vcore_range(void);

#ifdef __cplusplus
}
#endif

#endif // PWR_UTIL_H
