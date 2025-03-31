#include <ento-mcu/systick_config.h>
#include <ento-mcu/conf.h>

volatile unsigned int g_systick_ms_counter = 0;

void SysTick_Setup()
{
  SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000UL) - 1UL;
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                | SysTick_CTRL_TICKINT_Msk
                | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
  g_systick_ms_counter++;
}

