#ifndef SYSTICK_CONFIG_H
#define SYSTICK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined SMT32G4
#include <stm32g4xx.h>
#endif


typedef void (*SysTick_Callback)(void);

typedef struct {
  SysTick_Callback callback;
  unsigned int update_freq;
} SysTick_Handle;

static volatile SysTick_Handle gSysTickHandle;
static volatile unsigned int gSysTickCount;

void SysTick_Setup(SysTick_Callback callback, unsigned int update_freq);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // SYSTICK_CONFIG_H
