#ifndef CACHE_UTIL_H
#define CACHE_UTIL_H

#include <cstdint>

#if defined(STM32G4)
    #include <stm32g4xx.h>
    #include <stm32g4xx_ll_bus.h>
    #include <stm32g4xx_ll_cortex.h>
#elif defined(STM32H7)
    #include <stm32h7xx.h>
    #include <stm32h7xx_ll_cortex.h>
    #include <core_cm7.h>
#elif defined(STM32F7)
    #include <stm32f7xx.h>
    #include <stm32f7xx_ll_cortex.h>
    #include <core_cm7.h>
#elif defined(STM32G0)
    #include <stm32g0xx.h>
    #include <stm32g0xx_ll_cortex.h>
    // STM32G0 might not have a cache, but we can still provide stubs if needed
#elif defined(STM32U5)
    #include <stm32u5xx.h>
    #include <stm32u5xx_ll_icache.h>
    #include <stm32u5xx_ll_dcache.h>
    #include <stm32u5xx_ll_cortex.h>
#endif

enum CacheMode
{
  NoCache = -1,
  DirectMappedCache = 0,
  TwoWayCache = 1
};


// ICache utility functions. Most of these only available
// on U5/F7/H7...
void      icache_enable(void);
void      icache_disable(void);
bool      icache_is_enabled(void);
CacheMode icache_get_mode(void);
void      icache_set_mode(CacheMode mode);
void      icache_invalidate(void);
void      icache_clean(void);
uint32_t  icache_get_hit_monitor(void);
uint32_t  icache_get_miss_monitor(void);
void      icache_enable_monitors(void);
void      icache_enable_hit_monitor(void);
void      icache_enable_miss_monitor(void);
void      icache_disable_monitors(void);
void      icache_disable_miss_monitor(void);
void      icache_disable_hit_monitor(void);
void      icache_reset_monitors(void);
void      icache_reset_hit_monitor(void);
void      icache_reset_miss_monitor(void);
bool      icache_is_monitors_enabled(void);
bool      icache_is_hit_monitor_enabled(void);
bool      icache_is_miss_monitor_enabled(void);

// DCache Utility Functions.
void      dcache_enable(void);
void      dcache_disable(void);
bool      dcache_is_enabled(void);
void      dcache_invalidate(void);

// Branch Predictor Functions
void      branch_predictor_enable(void);
void      branch_predictor_disable(void);

// Generic cache enablement function that abstracts MCU-specific sequences
void      enable_all_caches(void);
void      disable_all_caches(void);

#endif // CACHE_UTIL_H
