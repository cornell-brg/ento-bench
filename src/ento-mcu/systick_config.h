#ifndef SYSTICK_CONFIG_H
#define SYSTICK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ento-mcu/conf.h>

typedef void (*SysTick_Callback)(void);

typedef struct {
  SysTick_Callback callback;
  unsigned int update_freq;
} SysTick_Handle;

extern volatile unsigned int g_systick_ms_counter;

void SysTick_Setup();
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // SYSTICK_CONFIG_H
