#ifndef TIMING_H
#define TIMING_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "systick_config.h"
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

constexpr uint32_t SYSTICK_INTERVAL_US = 100;
extern volatile uint32_t lsu_overflow_count;
extern volatile uint32_t cpi_overflow_count;
extern volatile uint32_t sleep_overflow_count;
extern volatile uint32_t fold_overflow_count;
extern volatile uint32_t exc_overflow_count;

// Initialization functions to enable counters
static inline void enable_dwt()
{
#if !defined(STM32G0)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(STM32F7)
  DWT->LAR = 0xC5ACCE55; 
#endif
#endif
}

static inline void disable_dwt()
{
#if !defined(STM32G0)
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
#endif
}

static inline void enable_all_counters()
{
  enable_dwt();
  DWT->CTRL = (DWT_CTRL_CYCCNTENA_Msk  |
               DWT_CTRL_EXCEVTENA_Msk  |
               DWT_CTRL_CPIEVTENA_Msk  |
               DWT_CTRL_LSUEVTENA_Msk  |
               DWT_CTRL_FOLDEVTENA_Msk |
               DWT_CTRL_SLEEPEVTENA_Msk);
}

static inline void disable_all_counters()
{
#if !defined(STM32G0)
  DWT->CTRL = 0;
#endif
}

static inline void disable_and_reset_all_counters()
{
#if !defined(STM32G0)
  DWT->CTRL = 0;
  DWT->CPICNT = 0;
  DWT->CYCCNT = 0;
  DWT->EXCCNT = 0;
  DWT->LSUCNT = 0;
  DWT->SLEEPCNT = 0;
  DWT->FOLDCNT = 0;
#endif
}

inline void init_systick(void)
{
  uint32_t ticks = (SystemCoreClock / 1000000) * SYSTICK_INTERVAL_US;
  SysTick_Config(ticks);
  NVIC_SetPriority(SysTick_IRQn, 0);
}

inline void SysTick_Handler()
{
  static uint8_t last_lsu_count = 0;
  static uint8_t last_cpi_count = 0;
  static uint8_t last_exc_count = 0;
  static uint8_t last_fold_count = 0;
  static uint8_t last_sleep_count = 0;

  disable_all_counters();

  uint8_t current_lsu_count   = DWT->LSUCNT;
  uint8_t current_cpi_count   = DWT->CPICNT;
  uint8_t current_exc_count   = DWT->EXCCNT;
  uint8_t current_fold_count  = DWT->FOLDCNT;
  uint8_t current_sleep_count = DWT->SLEEPCNT;

  if (current_lsu_count < last_lsu_count) lsu_overflow_count++;
  if (current_cpi_count < last_cpi_count) cpi_overflow_count++;
  if (current_exc_count < last_exc_count) exc_overflow_count++;
  if (current_fold_count < last_fold_count) fold_overflow_count++;
  if (current_sleep_count < last_sleep_count) sleep_overflow_count++;

  last_lsu_count   = DWT->LSUCNT;
  last_cpi_count   = DWT->CPICNT;
  last_exc_count   = DWT->EXCCNT;
  last_fold_count  = DWT->FOLDCNT;
  last_sleep_count = DWT->SLEEPCNT;

  enable_all_counters();
}

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

static inline void init_sleep_counter()
{
#if !defined(STM32G0)
  enable_dwt();
  DWT->SLEEPCNT = 0;                              // Reset sleep counter
  DWT->CTRL |= DWT_CTRL_SLEEPEVTENA_Msk;          // Enable sleep tracking
#endif
}

static inline void init_exc_counter()
{
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

static inline uint32_t get_total_lsu_count(void)
{
  return (lsu_overflow_count * 256) + DWT->LSUCNT;
}

static inline uint32_t get_total_cpi_count(void)
{
  return (cpi_overflow_count * 256) + DWT->CPICNT;
}

static inline uint32_t get_total_fold_count(void)
{
  return (fold_overflow_count * 256) + DWT->FOLDCNT;
}

static inline uint32_t get_total_sleep_count(void)
{
  return (sleep_overflow_count * 256) + DWT->SLEEPCNT;
}

static inline uint32_t get_total_exc_count(void)
{
  return (exc_overflow_count * 256) + DWT->EXCCNT;
}

static inline void reset_lsu_count()
{
#if !defined(STM32G0)
  DWT->LSUCNT = 0;
#endif
}

static inline void reset_cpi_count()
{
#if !defined(STM3G0)
  DWT->CPICNT = 0;
#endif
}

// Calculate elapsed cycles between two values
static inline uint32_t calculate_elapsed(uint32_t start, uint32_t end) {
  return (end >= start) ? (end - start) : (UINT32_MAX - start + end + 1);
}

static inline uint8_t calculate_elapsed(uint8_t start, uint8_t end) 
{
  return (end >= start) ? (end - start) : (UINT8_MAX - start + end + 1);
}

#endif // TIMING_H
