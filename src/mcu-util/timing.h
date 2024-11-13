#ifndef TIMING_H
#define TIMING_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include <cstdint>
#include <stdint.h>
#if defined(STM32G4)

#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_rcc.h>

#include <core_cm4.h>

#elif defined(STM32H7)

#include <stm32h7xx_ll_cortex.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_rcc.h>

//#ifndef __CORE_CM7_H_GENERIC
#include <core_cm7.h>
//#endif

#elif defined(STM32F7)

#include <stm32f7xx_ll_cortex.h>
#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_rcc.h>

//#ifndef __CORE_CM7_H_GENERIC
#include <core_cm7.h>

#elif defined(STM32G0)

#include <stm32g0xx_ll_cortex.h>
#include <stm32g0xx_ll_gpio.h>
#include <stm32g0xx_ll_rcc.h>

#include <core_cm4.h>

#elif defined(STM32U5)

#include <stm32u5xx_ll_cortex.h>
#include <stm32u5xx_ll_gpio.h>
#include <stm32u5xx_ll_rcc.h>

#include <core_cm33.h>

//#endif

#endif // defined(STM{STM_FAMILY})

// Initialization functions to enable counters
static inline void init_cycle_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CYCCNT = 0;                                // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
}

static inline void init_cpi_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CPICNT = 0;                                // Reset CPI counter
    DWT->CTRL |= DWT_CTRL_CPIEVTENA_Msk;            // Enable CPI tracking
}

static inline void init_fold_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->FOLDCNT = 0;                               // Reset folded instruction counter
    DWT->CTRL |= DWT_CTRL_FOLDEVTENA_Msk;           // Enable folded instruction tracking
}

static inline void init_lsu_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->LSUCNT = 0;                                // Reset LSU counter
    DWT->CTRL |= DWT_CTRL_LSUEVTENA_Msk;            // Enable LSU tracking
}

static inline void init_sleep_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->SLEEPCNT = 0;                              // Reset sleep counter
    DWT->CTRL |= DWT_CTRL_SLEEPEVTENA_Msk;          // Enable sleep tracking
}

static inline void init_exc_counter() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->EXCCNT = 0;                                // Reset exception counter
    DWT->CTRL |= DWT_CTRL_EXCEVTENA_Msk;            // Enable exception tracking
}

// Check if each counter is enabled
static inline bool is_cycle_counter_enabled() {
    return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
}

static inline bool is_cpi_counter_enabled() {
    return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CPIEVTENA_Msk);
}

static inline bool is_fold_counter_enabled() {
    return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_FOLDEVTENA_Msk);
}

static inline bool is_lsu_counter_enabled() {
    return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_LSUEVTENA_Msk);
}

static inline bool is_sleep_counter_enabled() {
    return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_SLEEPEVTENA_Msk);
}

static inline bool is_exc_counter_enabled() {
    return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_EXCEVTENA_Msk);
}

// Functions to read current metric values
static inline uint32_t get_cycle_count() { return DWT->CYCCNT; }
static inline uint32_t get_cpi_count() { return DWT->CPICNT; }
static inline uint32_t get_fold_count() { return DWT->FOLDCNT; }
static inline uint32_t get_lsu_count() { return DWT->LSUCNT; }
static inline uint32_t get_sleep_count() { return DWT->SLEEPCNT; }
static inline uint32_t get_exc_count() { return DWT->EXCCNT; }

// Calculate elapsed cycles between two values
static inline uint32_t calculate_elapsed(uint32_t start, uint32_t end) {
    return (end >= start) ? (end - start) : (UINT32_MAX - start + end + 1);
}
//#ifdef __cplusplus
//}
//#endif

#endif // TIMING_H
