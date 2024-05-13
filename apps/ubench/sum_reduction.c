#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

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
uint32_t sum_reduction(uint32_t* buffer, uint32_t M, uint32_t N)
{
  uint32_t i, j, sum = 0;
  for (i = 0; i < M; i++)
  {
    sum = 0;
    for (j = 0; j < N; j++)
    {
      sum += buffer[j];
    }
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
  uint32_t M, N, cache_sz;
  if (argc != 3 && argc != 4)
  {
    M = 1000;
    N = 512;
    cache_sz = 4096;
  }
  else if (argc == 3)
  {
    M = atoi(argv[1]);
    N = atoi(argv[2]);
    cache_sz = 4096;
  }
  else if (argc == 4)
  {
    M = atoi(argv[1]);
    N = atoi(argv[2]);
    cache_sz = atoi(argv[3]);
  }

  printf("Running sum reduction ubenchmark.\n");
  printf("Using parameters: M=%i, N=%i\n", M, N); 
  uint32_t buffer[N];

  uint32_t i;
  uint32_t expected = 0;
  for (i = 0; i < N; i++)
  {
    buffer[i] = i;
    expected += i;
  }

  if (cache_sz)
  {
    printf("Clearing cache!\n");
    int status = clear_cache(cache_sz);
  }

  uint32_t result;
#ifdef RISCV_BRG_GEM5
  printf("Setting up ROI!\n");
  //__asm__ volatile ("csrw 0x7C1, %0" :: "r"(1));
  m5_dump_reset_stats(0, 0);
#endif
  result = sum_reduction(buffer, M, N);
#ifdef RISCV_BRG_GEM5
  //__asm__ volatile ("csrw 0x7C1, %0" :: "r"(0));
  m5_dump_reset_stats(0, 0);
#endif


  printf("Finished sum reduction ubenchmark.\n");
  printf("Result: %i, Expected: %i\n", result, expected);
  return 0;
}
