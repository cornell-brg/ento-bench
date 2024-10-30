#include "clk_util.h"

uint32_t get_sys_clk_freq(void)
{
  uint32_t sysclk_freq = 0;

  // Determine the current system clock source
  switch (LL_RCC_GetSysClkSource())
  {
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:
      sysclk_freq = HSI_VALUE;  // HSI is 16 MHz by default
      break;
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:
      sysclk_freq = HSE_VALUE;  // HSE is typically 8 MHz or user-defined
      break;
    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL:
      // Calculate the frequency based on PLL settings
      if (LL_RCC_PLL_GetMainSource() == LL_RCC_PLLSOURCE_HSI) {
        sysclk_freq = (HSI_VALUE / LL_RCC_PLLM_DIV_4) * LL_RCC_PLL_GetN() / LL_RCC_PLL_GetR();
      } else if (LL_RCC_PLL_GetMainSource() == LL_RCC_PLLSOURCE_HSE) {
        sysclk_freq = (HSE_VALUE) * LL_RCC_PLL_GetN() / LL_RCC_PLL_GetR();
      }
      break;
    default:
      sysclk_freq = 0;  // Unknown clock source
      break;
  }

  return sysclk_freq;
}

uint32_t get_AHB_clk_freq(void)
{
  // Get system clock frequency
  uint32_t sysclk_freq = get_sys_clk_freq();

  // Get AHB prescaler and calculate AHB clock frequency
  uint32_t ahb_prescaler = LL_RCC_GetAHBPrescaler();
  return sysclk_freq >> ahb_prescaler;  // Right-shift by prescaler value
}

uint32_t get_APB1_clk_freq(void)
{
  // Get AHB clock frequency
  uint32_t ahb_freq = get_AHB_clk_freq();

  // Get APB1 prescaler and calculate APB1 clock frequency
  uint32_t apb1_prescaler = LL_RCC_GetAPB1Prescaler();
  return ahb_freq >> apb1_prescaler;  // Right-shift by prescaler value
}

uint32_t get_APB2_clk_freq(void)
{
  // Get AHB clock frequency
  uint32_t ahb_freq = get_AHB_clk_freq();

  // Get APB2 prescaler and calculate APB2 clock frequency
  uint32_t apb2_prescaler = LL_RCC_GetAPB2Prescaler();
  return ahb_freq >> apb2_prescaler;  // Right-shift by prescaler value
}

void sys_clk_cfg()
{
#ifdef STM32G4
  if (LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_SYSCFG) == 0) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  }

  if (LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_PWR) == 0) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  }

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1�s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
#endif
}
