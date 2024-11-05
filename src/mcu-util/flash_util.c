#include "flash_util.h"

void enable_instruction_cache()
{
#if   defined(STM32G4)
  LL_FLASH_EnableInstCache();
#elif defined(STM32H7)
#if   defined(STM32H745xx) || defined(STM32H757xx)
  LL_ART_Enable();
#endif
#endif
}

void disable_instruction_cache()
{
#if defined(STM32G4)
  LL_FLASH_DisableInstCache();
#elif defined(STM32H7)
#if   defined(STM32H745xx) || defined(STM32H757xx)
  LL_ART_Enable();
#endif
#endif
}

void enable_instruction_cache_prefetch()
{
#if defined(STM32G4)
 LL_FLASH_EnablePrefetch();
#elif defined(STM32H7)
#endif
}

void disable_instruction_cache_prefetch()
{
#if defined(STM32G4)
  LL_FLASH_DisablePrefetch();
#elif defined(STM32H7)
#endif
}

uint32_t is_instruction_cache_prefetch_enabled()
{
#if defined(STM32G4)
  return LL_FLASH_IsPrefetchEnabled();
#elif defined(STM32H7)
  //@TODO: Add enum for return values? Or just use the LL_ENUM?
  return 0;
#else
  return 0;
#endif
}

uint32_t get_flash_latency()
{
#if defined(STM32G4)
  return LL_FLASH_GetLatency();
#elif defined(STM32H7)
#if   defined(STM32H745xx) || defined(STM32H757xx)
  return LL_ART_IsEnable();
#endif
  //@TODO: Add enum for return values? Or just use the LL_ENUM?
  return 0;
#else
  return 0;
#endif
}
