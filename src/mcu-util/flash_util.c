#include "flash_util.h"

void enable_instruction_cache()
{
  LL_FLASH_EnableInstCache();

}

void disable_instruction_cache()
{
  LL_FLASH_DisableInstCache();
}

void enable_instruction_cache_prefetch()
{
 LL_FLASH_EnablePrefetch();
}

void disable_instruction_cache_prefetch()
{
  LL_FLASH_DisablePrefetch();
}

uint32_t is_instruction_cache_prefetch_enabled()
{
  return LL_FLASH_IsPrefetchEnabled();
}

uint32_t get_flash_latency()
{
  return LL_FLASH_GetLatency();
}
