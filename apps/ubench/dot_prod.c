#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <blas.h>

#if defined STM32G4
#include <stm32g4xx.h>
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_rcc.h>
#include <systick_config.h>
#include <led_util.h>
#endif

#ifdef RISCV_BRG_GEM5
// From include/gem5/asm/generic/m5ops.h
#define M5OP_DUMP_RESET_STATS 0x42
 
// Adapted from util/m5/src/abi/riscv/m5op.S
//
// We must remember to move the arguments into a0/a1 before calling
void m5_dump_reset_stats(uint64_t ns_delay, uint64_t ns_period) {
    register unsigned delay_reg __asm__("a0") = ns_delay;
    register unsigned period_reg __asm__("a1") = ns_period;
 
    asm volatile(
        // .insn r opcode6, func3, func7, rd, rs1, rs2
        ".insn r 0x7b, 0x0, %[dump_reset], x0, x0, x0"
      :
      : [dump_reset] "i" (M5OP_DUMP_RESET_STATS) ,
        "r" (delay_reg),
        "r" (period_reg)
      :
    );
}
#endif

__attribute__ ((noinline))
uint32_t
dot_prod(uint32_t* a, uint32_t* b, int32_t M)
{
  int32_t i;
  uint32_t sum = 0;
  for (i = 0; i < M; i++)
  {
		sum += a[i] * b[i];
  }
  return sum;
}

__attribute__ ((noinline))
float
dot_prod_f32(float* a, float* b, int32_t M)
{
  int32_t i;
  float sum = 0;
  for (i = 0; i < M; i++)
  {
    sum += a[i] * b[i];
  }
  return sum;
}


__attribute__ ((noinline))
int clear_cache(uint32_t cache_sz)
{
  volatile uint8_t* data;
  data = (volatile uint8_t*) malloc(cache_sz);
  if (!data) return 0;

  uint32_t i, sum;
  sum = 0;
  for (i = 0; i < cache_sz; i++)
  {
    sum += data[i];
  }

  free((void*) data);
  return 1;
}

int main(int argc, char** argv)
{
  //static volatile uint32_t a[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  //static volatile uint32_t b[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  //static volatile float c[10] = {  1.0, -1.0,  5.0,  5.0,  10.0, -10.0, 20.0, -20.0, 2.0, -2.0 };
  //static volatile float d[10] = {  1.0,  1.0, -5.0,  5.0, -10.0, -10.0, 20.0,  20.0, 2.0,  2.0 };

  static uint32_t a[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  static uint32_t b[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  static float c[10] = {  1.0, -1.0,  5.0,  5.0,  10.0, -10.0, 20.0, -20.0, 2.0, -2.0 };
  static float d[10] = {  1.0,  1.0, -5.0,  5.0, -10.0, -10.0, 20.0,  20.0, 2.0,  2.0 };

  static uint32_t result1;
  static uint32_t expected1 = 385;
  static float result2;
  static float expected2 = 0.0;

#if defined STM32G4
  SystemCoreClockUpdate();
#endif

#ifdef RISCV_BRG_GEM5
  printf("Setting up ROI!\n");
  //__asm__ volatile ("csrw 0x7C1, %0" :: "r"(1));
  m5_dump_reset_stats(0, 0);
#endif
  result1 = dot_prod(a, b, 10);
#ifdef RISCV_BRG_GEM5
  //__asm__ volatile ("csrw 0x7C1, %0" :: "r"(0));
  m5_dump_reset_stats(0, 0);
#endif

#ifdef RISCV_BRG_GEM5
  printf("Setting up ROI!\n");
  //__asm__ volatile ("csrw 0x7C1, %0" :: "r"(1));
  m5_dump_reset_stats(0, 0);
#endif
  result2 = dot_prod_f32(c, d, 10);
#ifdef RISCV_BRG_GEM5
  //__asm__ volatile ("csrw 0x7C1, %0" :: "r"(0));
  m5_dump_reset_stats(0, 0);
#endif



#if defined STM32G4
  UserLED_Init();
  SystemCoreClockUpdate();
  int success = result1 == expected1;
  success &= (result2 == expected2);
  if (success) SysTick_Setup(UserLED_Toggle, 1000);
  else SysTick_Setup(UserLED_Toggle, 125);
  SysTick_Config(SystemCoreClock / 1000);
#elif GEM5
  printf("Finished sum reduction ubenchmark.\n");
  printf("Passed uint32 dot_prod: %b", result1 == expected1);
  printf("Passed float32 dot_prod: %b", result2 == expected2);
#endif
  while (1);
}
