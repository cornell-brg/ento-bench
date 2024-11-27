#include <ento-mcu/cache_util.h>

void icache_enable()
{
#if defined(STM32U5)
  LL_ICACHE_Enable();
#elif defined(STM32F7) || defined(STM32H7)
  SCB_EnableICache();
#endif
}

bool icache_is_enabled()
{
#if defined(STM32U5)
  return LL_ICACHE_IsEnabled();
#elif defined(STM32F7) || defined(STM32H7)
  return (SCB->CCR & SCB_CCR_IC_Msk) != 0;
#endif
  return 0;
}


void icache_disable()
{
#if defined(STM32U5)
  LL_ICACHE_Disable();
#elif defined(STM32F7) || defined(STM32H7)
  SCB_DisableICache();
#endif
}

CacheMode icache_get_mode()
{
#if defined(STM32U5)
  int32_t mode_value = static_cast<int32_t>(LL_ICACHE_GetMode());
  return static_cast<CacheMode>(mode_value); 
#elif defined(STM32F7) || defined(STM32H7)
  // L1 I$ is 2-way set assoc. on M7
  return CacheMode::TwoWayCache;   
#else
  return CacheMode::NoCache;
#endif
}

void icache_set_mode(CacheMode mode)
{
#if defined(STM32U5)
  if (mode != NoCache)
    LL_ICACHE_SetMode((uint32_t) mode);
#endif
}

void icache_invalidate()
{
#if defined(STM32U5)
  LL_ICACHE_Invalidate(); 
#elif defined(STM32F7) || defined(STM32H7)
  SCB_InvalidateICache();
#endif
}

uint32_t icache_get_hit_monitor(void)
{
#if defined(STM32U5)
  return LL_ICACHE_GetHitMonitor();
#else
  return 0;
#endif
}

uint32_t icache_get_miss_monitor(void)
{
#if defined(STM32U5)
  return LL_ICACHE_GetMissMonitor();
#else
  return 0;
#endif
}

void icache_enable_monitors(void)
{
#if defined(STM32U5)
  LL_ICACHE_EnableMonitors(LL_ICACHE_MONITOR_ALL);
#endif
}

void icache_enable_hit_monitor(void)
{
#if defined(STM32U5)
  LL_ICACHE_EnableMonitors(LL_ICACHE_MONITOR_HIT);
#endif
}

void icache_enable_miss_monitor(void)
{
#if defined(STM32U5)
  LL_ICACHE_EnableMonitors(LL_ICACHE_MONITOR_MISS);
#endif
}

void icache_disable_monitors(void)
{
#if defined(STM32U5)
  LL_ICACHE_DisableMonitors(LL_ICACHE_MONITOR_ALL);
#endif
}

void icache_disable_miss_monitor(void)
{
#if defined(STM32U5)
  LL_ICACHE_DisableMonitors(LL_ICACHE_MONITOR_MISS);
#endif
}

void icache_disable_hit_monitor(void)
{
#if defined(STM32U5)
  LL_ICACHE_DisableMonitors(LL_ICACHE_MONITOR_HIT);
#endif
}

void icache_reset_monitors(void)
{
#if defined(STM32U5)
  LL_ICACHE_ResetMonitors(LL_ICACHE_MONITOR_ALL);
#endif
}

void icache_reset_hit_monitor(void)
{
#if defined(STM32U5)
  LL_ICACHE_ResetMonitors(LL_ICACHE_MONITOR_HIT);
#endif
}

void icache_reset_miss_monitor(void)
{
#if defined(STM32U5)
  LL_ICACHE_ResetMonitors(LL_ICACHE_MONITOR_MISS);
#endif
}

bool icache_is_monitors_enabled(void)
{
#if defined(STM32U5)
  return LL_ICACHE_IsEnabledMonitors(LL_ICACHE_MONITOR_ALL);
#else
  return false;
#endif
}

bool icache_is_hit_monitor_enabled(void)
{
#if defined(STM32U5)
  return LL_ICACHE_IsEnabledMonitors(LL_ICACHE_MONITOR_HIT);
#else
  return false;
#endif
}

bool icache_is_miss_monitor_enabled(void)
{
#if defined(STM32U5)
  return LL_ICACHE_IsEnabledMonitors(LL_ICACHE_MONITOR_MISS);
#else
  return false;
#endif
}


void dcache_enable()
{
#if defined(STM32U5)
  // We do not use the DCACHE1 on U5 because
  // it only services external memories
  return;
#elif defined(STM32F7) || defined(STM32H7)
  SCB_EnableDCache();
#else
  return;
#endif
}

void dcache_disable()
{
#if defined(STM32U5)
  // We do not use the DCACHE1 on U5 because
  // it only services external memories
  return;
#elif defined(STM32F7) || defined(STM32H7)
  SCB_DisableDCache();
#else
  return;
#endif
}

bool dcache_is_enabled()
{
#if defined(STM32U5)
  // We do not use the DCACHE1 on U5 because
  // it only services external memories
  return false;
#elif defined(STM32F7) || defined(STM32H7)
  return (SCB->CCR & SCB_CCR_DC_Msk) != 0;
#else
  return false;
#endif
}

void dcache_invalidate()
{
#if defined(STM32U5)
  // We do not use the DCACHE1 on U5 because
  // it only services external memories
  return;
#elif defined(STM32F7) || defined(STM32H7)
  SCB_InvalidateDCache();
#else
  return;
#endif
}

void branch_predictor_enable()
{
#if defined(STM32F7) || defined(STM32H7)
  constexpr uintptr_t ACTLR_ADDRESS = 0xE000E008U;  // Address of the ACTLR register
  volatile uint32_t &ACTLR = *reinterpret_cast<volatile uint32_t *>(ACTLR_ADDRESS);

  ACTLR &= ~((1 << 13) | (1 << 14));  // Clear DISBTACREAD and DISBTACALLOC bits
  __DSB();                            // Data Synchronization Barrier
  __ISB();                            // Instruction Synchronization Barrier
#endif
}

void branch_predictor_disable()
{
#if defined(STM32F7) || defined(STM32H7)
  constexpr uintptr_t ACTLR_ADDRESS = 0xE000E008U;  // Address of the ACTLR register
  volatile uint32_t &ACTLR = *reinterpret_cast<volatile uint32_t *>(ACTLR_ADDRESS);

  // Clear is set to 1
  ACTLR |= ((1 << 13) | (1 << 14));   // Set DISBTACREAD and DISBTACALLOC bits
  __DSB();                            // Data Synchronization Barrier
  __ISB();                            // Instruction Synchronization Barrier
#endif
}
