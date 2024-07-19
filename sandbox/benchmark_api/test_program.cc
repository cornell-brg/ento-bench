#include "armv7e-m_ubench.hh"

#define COUNT 64

template void assembly::mul<COUNT>();

template<int N>
inline void add_recursive() __attribute__((always_inline));

template<int N>
inline void add_recursive() 
{
  if constexpr (N>0) {
    asm volatile (
      "add r0, r0, r0 \n\t"
      "add r1, r1, r1 \n\t"
      "add r2, r2, r2 \n\t"
      "add r3, r3, r3 \n\t"
      "add r4, r4, r4 \n\t"
      "add r5, r5, r5 \n\t"
      "add r6, r6, r6 \n\t"
      "add r7, r7, r7 \n\t"
      : // No outputs
      : // No inputs
      : "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8" // Clobber list
    );
    add_recursive<N-1>();
  }
}

template<int N>
inline void mul_constexpr()
{
  if constexpr (N>0)
  {
    for (int i = 0; i < N; i++)
      asm volatile (
        "add r0, r0, r0 \n\t"
        "add r1, r1, r1 \n\t"
        "add r2, r2, r2 \n\t"
        "add r3, r3, r3 \n\t"
        "add r4, r4, r4 \n\t"
        "add r5, r5, r5 \n\t"
        "add r6, r6, r6 \n\t"
        "add r7, r7, r7 \n\t"
        : // No outputs
        : // No inputs
        : "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8" // Clobber list
      );
  }
}

int main()
{
  constexpr int REPS = 64;
  int a = 0;
  int b = 1;
  int c = 2;
  int d = a + b + c; 
  assembly::mul<8>();
  add_recursive<REPS>();
  mul_constexpr<REPS>();
  return 0;
}
