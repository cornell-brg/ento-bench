#include <systick_config.h>

void SysTick_Setup(SysTick_Callback callback, unsigned int update_freq)
{
  gSysTickHandle.callback = callback;
  gSysTickHandle.update_freq = update_freq;
}

void SysTick_Handler(void)
{
  gSysTickCount++;
  if (gSysTickHandle.callback && 
      gSysTickCount % gSysTickHandle.update_freq == 0)
  {
    gSysTickHandle.callback();
  }
}

