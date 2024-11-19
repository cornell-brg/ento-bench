#include <cstdio>
#include <stdio.h>
#include "bench/harness.hh"
#include <cstdlib>

extern "C" void initialise_monitor_handles(void);

template<int N>
inline void __attribute__((always_inline)) ldr_template_recursion_always_inline(int** base_address) {
  if constexpr (N > 0) {
    asm volatile (
      "ldr r0, [%0] \n\t"    
      "ldr r1, [%0, #4] \n\t"  
      "ldr r2, [%0, #8] \n\t" 
      "ldr r3, [%0, #12] \n\t" 
      "ldr r4, [%0, #16] \n\t" 
      "ldr r5, [%0, #20] \n\t" 
      "ldr r6, [%0, #24] \n\t" 
      "ldr r8, [%0, #28] \n\t" 
      : // No outputs
      : "r"(base_address) 
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r8" // Clobber list
    );
    ldr_template_recursion_always_inline<N-1>(base_address);  // Recursively call
  }
}

// Wrapper for recursion
template<int N>
void __attribute__((noinline)) ldr_template_recursion(int** base_address) {
  if constexpr (N > 0) {
    ldr_template_recursion_always_inline<N>(base_address);  // Correct recursion
  }
}


int main()
{
  using namespace bench;
  initialise_monitor_handles();
  
  volatile int array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  // Initialize array as volatile to avoid optimization
  
  int* array_of_pointers[10];
  
  // Initialize array_of_pointers to point to each element of the array
  for (int i = 0; i < 10; ++i) {
    array_of_pointers[i] = const_cast<int*>(&array[i]);
  }

  constexpr int reps = 800;  // Change from 10 to 100

  // Use lambda or function wrapper to avoid passing template instantiation directly
  auto benchmark_function = [&]() {
    ldr_template_recursion<reps>(array_of_pointers);
  };

  Harness harness(benchmark_function, "Independent LDR Benchmark Example", 3);
  harness.run();

  printf("Finished running independent load benchmark example!\n");

  exit(1);

  return 0;
}



