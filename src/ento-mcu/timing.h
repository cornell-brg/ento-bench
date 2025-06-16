#ifndef TIMING_H
#define TIMING_H

//#include "systick_config.h"
#include <cstdint>
#include <stdint.h>

#include <ento-mcu/conf.h>
#include <ento-mcu/gpio_util.h>
#include <ento-mcu/systick_config.h>


constexpr uint32_t SYSTICK_INTERVAL_US = 100;
extern volatile uint32_t lsu_overflow_count;
extern volatile uint32_t cpi_overflow_count;
extern volatile uint32_t sleep_overflow_count;
extern volatile uint32_t fold_overflow_count;
extern volatile uint32_t exc_overflow_count;

// Initialization functions to enable counters
static inline void enable_dwt()
{
#if !defined(STM32G0) && !defined(STM32C0)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(STM32F7)
  DWT->LAR = 0xC5ACCE55; 
#endif
#endif
}

static inline void disable_dwt()
{
#if !defined(STM32G0) && !defined(STM32C0)
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
#endif
}

static inline void enable_all_counters()
{
#if !defined(STM32G0) && !defined(STM32C0)
  __asm__ volatile ("");
  enable_dwt();
  __asm__ volatile ("");
  DWT->CTRL = (DWT_CTRL_CYCCNTENA_Msk  |
               DWT_CTRL_EXCEVTENA_Msk  |
               DWT_CTRL_CPIEVTENA_Msk  |
               DWT_CTRL_LSUEVTENA_Msk  |
               DWT_CTRL_FOLDEVTENA_Msk |
               DWT_CTRL_SLEEPEVTENA_Msk);
  __asm__ volatile ("");
#endif
}

static inline void disable_all_counters()
{
  __asm__ volatile ("");
#if !defined(STM32G0) && !defined(STM32C0)
  DWT->CTRL = 0;
  __asm__ volatile ("");
#endif
}

static inline void disable_and_reset_all_counters()
{
#if !defined(STM32G0) && !defined(STM32C0)
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

//inline void SysTick_Handler()
//{
//#if !defined(STM32G0)
//  static uint8_t last_lsu_count = 0;
//  static uint8_t last_cpi_count = 0;
//  static uint8_t last_exc_count = 0;
//  static uint8_t last_fold_count = 0;
//  static uint8_t last_sleep_count = 0;
//
//  disable_all_counters();
//
//  uint8_t current_lsu_count   = DWT->LSUCNT;
//  uint8_t current_cpi_count   = DWT->CPICNT;
//  uint8_t current_exc_count   = DWT->EXCCNT;
//  uint8_t current_fold_count  = DWT->FOLDCNT;
//  uint8_t current_sleep_count = DWT->SLEEPCNT;
//
//  if (current_lsu_count < last_lsu_count) lsu_overflow_count++;
//  if (current_cpi_count < last_cpi_count) cpi_overflow_count++;
//  if (current_exc_count < last_exc_count) exc_overflow_count++;
//  if (current_fold_count < last_fold_count) fold_overflow_count++;
//  if (current_sleep_count < last_sleep_count) sleep_overflow_count++;
//
//  last_lsu_count   = DWT->LSUCNT;
//  last_cpi_count   = DWT->CPICNT;
//  last_exc_count   = DWT->EXCCNT;
//  last_fold_count  = DWT->FOLDCNT;
//  last_sleep_count = DWT->SLEEPCNT;
//
//  enable_all_counters();
//#endif
//}

// Check if each counter is enabled
static inline
bool is_dwt_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  bool en = (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk);
#if defined(STM32F7)
  en &= ( DWT->LAR == 0xC5ACCE55 ); 
#endif
  return en;
#else
  return false;
#endif
}

static inline
bool is_cycle_counter_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
#else
  return false;
#endif
}

static inline
bool is_cpi_counter_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_CPIEVTENA_Msk);
#else
  return false;
#endif
}

static inline
bool is_fold_counter_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_FOLDEVTENA_Msk);
#else
  return false;
#endif
}

static inline
bool is_lsu_counter_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_LSUEVTENA_Msk);
#else
  return false;
#endif
}

static inline
bool is_sleep_counter_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_SLEEPEVTENA_Msk);
#else
  return false;
#endif
}

static inline
bool is_exc_counter_enabled()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) && (DWT->CTRL & DWT_CTRL_EXCEVTENA_Msk);
#else
  return false;
#endif
}

static inline
void init_cycle_counter() {
#if !defined(STM32G0) && !defined(STM32C0)
  enable_dwt();
  DWT->CYCCNT = 0;                                // Reset cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
#endif
}

static inline
void init_cpi_counter() {
#if !defined(STM32G0) && !defined(STM32C0)
  enable_dwt();
  DWT->CPICNT = 0;                                // Reset CPI counter
  DWT->CTRL |= DWT_CTRL_CPIEVTENA_Msk;            // Enable CPI tracking
#endif
}

static inline
void init_fold_counter() {
#if !defined(STM32G0) && !defined(STM32C0)
  enable_dwt();
  DWT->FOLDCNT = 0;                               // Reset folded instruction counter
  DWT->CTRL |= DWT_CTRL_FOLDEVTENA_Msk;           // Enable folded instruction tracking
#endif
}

static inline
void init_lsu_counter() {
#if !defined(STM32G0) && !defined(STM32C0)
  enable_dwt();
  DWT->LSUCNT = 0;                                // Reset LSU counter
  DWT->CTRL |= DWT_CTRL_LSUEVTENA_Msk;            // Enable LSU tracking
#endif
}

static inline
void init_sleep_counter()
{
#if !defined(STM32G0) && !defined(STM32C0)
  enable_dwt();
  DWT->SLEEPCNT = 0;                              // Reset sleep counter
  DWT->CTRL |= DWT_CTRL_SLEEPEVTENA_Msk;          // Enable sleep tracking
#endif
}

static inline
void init_exc_counter()
{
#if !defined(STM32G0) && !defined(STM32C0)
  enable_dwt();
  DWT->EXCCNT = 0;                                // Reset exception counter
  DWT->CTRL |= DWT_CTRL_EXCEVTENA_Msk;            // Enable exception tracking
#endif
}

// Functions to read current metric values
static inline
uint32_t get_cycle_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return DWT->CYCCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_cpi_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return DWT->CPICNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_fold_count()
{ 
#if !defined(STM32G0) && !defined(STM32C0)
  return DWT->FOLDCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_lsu_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return DWT->LSUCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_sleep_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return DWT->SLEEPCNT;
#else
  return 0;
#endif
}
static inline
uint32_t get_exc_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  return DWT->EXCCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_total_lsu_count(void)
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (lsu_overflow_count * 256) + DWT->LSUCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_total_cpi_count(void)
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (cpi_overflow_count * 256) + DWT->CPICNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_total_fold_count(void)
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (fold_overflow_count * 256) + DWT->FOLDCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_total_sleep_count(void)
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (sleep_overflow_count * 256) + DWT->SLEEPCNT;
#else
  return 0;
#endif
}

static inline
uint32_t get_total_exc_count(void)
{
#if !defined(STM32G0) && !defined(STM32C0)
  return (exc_overflow_count * 256) + DWT->EXCCNT;
#else
  return 0;
#endif
}

static inline
void reset_lsu_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  DWT->LSUCNT = 0;
#endif
}

static inline
void reset_cpi_count()
{
#if !defined(STM32G0) && !defined(STM32C0)
  DWT->CPICNT = 0;
#endif
}

// Calculate elapsed cycles between two values
static inline
uint32_t calculate_elapsed(uint32_t start, uint32_t end) {
  return (end >= start) ? (end - start) : (UINT32_MAX - start + end + 1);
}

static inline
uint8_t calculate_elapsed(uint8_t start, uint8_t end) 
{
  return (end >= start) ? (end - start) : (UINT8_MAX - start + end + 1);
}

// ================================================
// GPIO Latency/Trigger Pin Setup

void latency_pin_enable();
void latency_pin_disable();
void trigger_pin_enable();
void trigger_pin_disable();

inline void latency_pin_toggle()
{
  latency_gpio_port->ODR ^= latency_gpio_pin;
}

inline void latency_pin_high()
{
  latency_gpio_port->BSRR = latency_gpio_pin;
}

inline void latency_pin_low()
{
  latency_gpio_port->BSRR = latency_gpio_pin << 16;
}

void software_delay_cycles(uint32_t cycles);

inline void trigger_pin_toggle()
{
  trigger_gpio_port->ODR ^= trigger_gpio_pin;
}

inline void trigger_pin_high()
{
  trigger_gpio_port->BSRR = trigger_gpio_pin;
}

inline void trigger_pin_low()
{
  trigger_gpio_port->BSRR = trigger_gpio_pin << 16;
}

class Delay
{
public:
  static void ms(uint32_t millis)
  {
#ifdef STM32_BUILD
    __asm__ volatile("" ::: "memory");
#ifndef DEBUG
    uint32_t start = g_systick_ms_counter;
    while ((g_systick_ms_counter - start) < millis)
    {
      __WFI();
    }
#endif // ifndef DEBUG
    __asm__ volatile("" ::: "memory");
#endif // ifdef STM32_BUILD
  }
};

#endif // TIMING_H
