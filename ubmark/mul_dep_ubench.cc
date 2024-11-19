#include <stdlib.h>
#include <stdio.h>
#include "bench/harness.hh"

extern "C" void initialise_monitor_handles(void);

template <int N>

inline void __attribute__((always_inline)) mul_dep_recursion_always_inline() {
  //for (int i = 0; i < N; i++)
  if constexpr (N > 0)
  {
    asm volatile (
      "mul r0, r8, r0 \n\t"
      "mul r1, r0, r1 \n\t"
      "mul r2, r1, r2 \n\t"
      "mul r3, r2, r3 \n\t"
      "mul r4, r3, r4 \n\t"
      "mul r5, r4, r5 \n\t"
      "mul r6, r5, r6 \n\t"
      "mul r8, r6, r8 \n\t"
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r8" // Clobber list
      //: "r0", "r1", "r2", "r3", "r4", "r5", "r6"

    );
    mul_dep_recursion_always_inline<N-1>();
  }
}

template<int N>
void __attribute__((noinline)) mul_dep_recursion() {
  //for (int i = 0; i < N; i++)
  if constexpr (N > 0)
  {
    asm volatile (
      "mul r0, r0, r0 \n\t"
      "mul r1, r1, r1 \n\t"
      "mul r2, r2, r2 \n\t"
      "mul r3, r3, r3 \n\t"
      "mul r4, r4, r4 \n\t"
      "mul r5, r5, r5 \n\t"
      "mul r6, r6, r6 \n\t"
      "mul r8, r8, r8 \n\t"
      : // No outputs
      : // No inputs
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r8" // Clobber list
      //: "r0", "r1", "r2", "r3", "r4", "r5", "r6"

    );
    mul_dep_recursion_always_inline<N-1>();
  }
}



int main()
{
  using namespace bench;
  initialise_monitor_handles();

  constexpr int reps = 10; //change 10 to 100

  Harness harness(mul_dep_recursion<reps>, "Dependent Mul Benchmark Example", 1);
  harness.run();

  printf("Finished running dependent mul benchmark example!\n");

  exit(1);

  return 0;
}





