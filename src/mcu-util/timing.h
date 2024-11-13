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

#include <core_cm0plus.h>

#elif defined(STM32U5)

#include <stm32u5xx_ll_cortex.h>
#include <stm32u5xx_ll_gpio.h>
#include <stm32u5xx_ll_rcc.h>

#include <core_cm33.h>

//#endif

#endif // defined(STM{STM_FAMILY})

// Check if each counter is enabled
static inline bool is_dwt_enabled()
{
#if !defined(STM32G0)
  bool en = (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk);
#if defined(STM32F7)
  en &= ( DWT->LAR == 0xC5ACCE55 ); 
#endif
  return en;
#else
  return false;
#endif
}

static inline bool is_cycle_counter_enabled()
{
#if !defined(STM32G0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
#else
  return false;
#endif
}

static inline bool is_cpi_counter_enabled()
{
#if !defined(STM32G0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CPIEVTENA_Msk);
#else
  return false;
#endif
}

static inline bool is_fold_counter_enabled()
{
#if !defined(STM32G0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_FOLDEVTENA_Msk);
#else
  return false;
#endif
}

static inline bool is_lsu_counter_enabled()
{
#if !defined(STM32G0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_LSUEVTENA_Msk);
#else
  return false;
#endif
}

static inline bool is_sleep_counter_enabled()
{
#if !defined(STM32G0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_SLEEPEVTENA_Msk);
#else
  return false;
#endif
}

static inline bool is_exc_counter_enabled()
{
#if !defined(STM32G0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_EXCEVTENA_Msk);
#else
  return false;
#endif
}

// Initialization functions to enable counters
//

static inline void disable_dwt()
{
#if !defined(STM32G0)
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
#endif
}

static inline void enable_dwt()
{
#if !defined(STM32G0)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(STM32F7)
  DWT->LAR = 0xC5ACCE55; 
#endif
#endif
}

static inline void init_cycle_counter() {
#if !defined(STM32G0)
  enable_dwt();
  DWT->CYCCNT = 0;                                // Reset cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
#endif
}

static inline void init_cpi_counter() {
#if !defined(STM32G0)
  enable_dwt();
  DWT->CPICNT = 0;                                // Reset CPI counter
  DWT->CTRL |= DWT_CTRL_CPIEVTENA_Msk;            // Enable CPI tracking
#endif
}

static inline void init_fold_counter() {
#if !defined(STM32G0)
  enable_dwt();
  DWT->FOLDCNT = 0;                               // Reset folded instruction counter
  DWT->CTRL |= DWT_CTRL_FOLDEVTENA_Msk;           // Enable folded instruction tracking
#endif
}

static inline void init_lsu_counter() {
#if !defined(STM32G0)
  enable_dwt();
  DWT->LSUCNT = 0;                                // Reset LSU counter
  DWT->CTRL |= DWT_CTRL_LSUEVTENA_Msk;            // Enable LSU tracking
#endif
}

static inline void init_sleep_counter() {
#if !defined(STM32G0)
  enable_dwt();
  DWT->SLEEPCNT = 0;                              // Reset sleep counter
  DWT->CTRL |= DWT_CTRL_SLEEPEVTENA_Msk;          // Enable sleep tracking
#endif
}

static inline void init_exc_counter() {
#if !defined(STM32G0)
  enable_dwt();
  DWT->EXCCNT = 0;                                // Reset exception counter
  DWT->CTRL |= DWT_CTRL_EXCEVTENA_Msk;            // Enable exception tracking
#endif
}


// Functions to read current metric values
static inline uint32_t get_cycle_count()
{
#if !defined(STM32G0)
  return DWT->CYCCNT;
#else
  return 0;
#endif
}
static inline uint32_t get_cpi_count()
{
#if !defined(STM32G0)
  return DWT->CPICNT;
#else
  return 0;
#endif
}
static inline uint32_t get_fold_count()
{ 
#if !defined(STM32G0)
  return DWT->FOLDCNT;
#else
  return 0;
#endif
}
static inline uint32_t get_lsu_count()
{
#if !defined(STM32G0)
  return DWT->LSUCNT;
#else
  return 0;
#endif
}
static inline uint32_t get_sleep_count()
{
#if !defined(STM32G0)
  return DWT->SLEEPCNT;
#else
  return 0;
#endif
}
static inline uint32_t get_exc_count()
{
#if !defined(STM32G0)
  return DWT->EXCCNT;
#else
  return 0;
#endif
}

static inline void reset_lsu_count()
{
#if !defined(STM32G0)
  DWT->LSUCNT = 0;
#endif
}

// Calculate elapsed cycles between two values
static inline uint32_t calculate_elapsed(uint32_t start, uint32_t end) {
  return (end >= start) ? (end - start) : (UINT32_MAX - start + end + 1);
}
//#ifdef __cplusplus
//}
//#endif

#endif // TIMING_H
