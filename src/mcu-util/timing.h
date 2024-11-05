#ifndef TIMING_H
#define TIMING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#if defined(STM32G4)
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_rcc.h>
#elif defined(STM32H7)
#include <stm32h7xx_ll_cortex.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_rcc.h>
#ifndef __CORE_CM7_H_GENERIC
#include <core_cm7.h>
#endif
#endif

static volatile uint32_t global_last_cycle_count;

static inline void     init_cycle_counter(void) __attribute__((always_inline));
static inline int      is_cycle_counter_init(void) __attribute__((always_inline));
static inline uint32_t read_cycle_counter(void) __attribute__((always_inline));
static inline void     update_last_cycle_count(void) __attribute__((always_inline));
static inline uint32_t get_elapsed_cycles(void) __attribute__((always_inline));

// Inline Implementations
static inline
void init_cycle_counter() {
  global_last_cycle_count = 0; // Initialize the global variable
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
  DWT->CYCCNT = 0; // Reset cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable cycle counter
}

static inline
int is_cycle_counter_init()
{
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk);
}

static inline
uint32_t read_cycle_counter() {
  return DWT->CYCCNT;
}

static inline
void update_last_cycle_count() {
  global_last_cycle_count = DWT->CYCCNT; // Update the global variable with the current cycle count
}

static inline
uint32_t get_elapsed_cycles()
{
  uint32_t current_cycle_count = DWT->CYCCNT;
  uint32_t elapsed_cycles;
  
  if (current_cycle_count >= global_last_cycle_count) {
      elapsed_cycles = current_cycle_count - global_last_cycle_count;
  } else {
      // Handle rollover
      elapsed_cycles = (UINT32_MAX - global_last_cycle_count) + current_cycle_count + 1;
  }

  global_last_cycle_count = current_cycle_count; // Update last cycle count for next measurement
  return elapsed_cycles;
}

#ifdef __cplusplus
}
#endif

#endif // TIMING_H
