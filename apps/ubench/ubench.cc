#include <bench/harness.h>

#include <cstdio>
#include <variant>

#if defined(RV32) || defined(RV64)
#include "riscv_ubench.hh"
#elif defined(ARMV7E_M)
#include <workload/microbench/assembly/armv7e-m_ubenchmarks.hh>
#elif defined(X86)
//#include "x86_ubench.hh"
#endif

#define REPETITIONS 64
constexpr int REPS = 64;

void __attribute__ ((noinline)) nop(int n)
{
  for (int i = 0; i < n; i++)
  asm volatile(
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
  );
}

int main()
{
  using namespace bench;
  using namespace assembly;

  printf("Running microbenchmarks!\n");
  //printf("Iprintf test: %i", 10);
  auto lambdaMul = []() {
    mul<8>();
  };

  auto lambdaAdd = []() {
    add<8>();
  };

  /*MultiHarness assembly_harness(lambdaMul);
  assembly_harness.run();

  auto lambdaNop = []() -> void {
    asm volatile(
      ".rept 100  \n\t"
      "nop        \n\t"
      "nop        \n\t"
      "nop        \n\t"
      "nop        \n\t"
      "nop        \n\t"
      "nop        \n\t"
      "nop        \n\t"
      "nop        \n\t"
      ".endr      \n\t"
    );
  };*/

  // (Single) Lambda Multiply Harness
  //Harness mul_harness(lambdaMul, "Mul<64> Lambda");
  //mul_harness.run();

  // (Single) Function Pointer Multiply Harness
  //Harness mul_harness_fp(mul<8>, "Mul<8> Function Pointer", 10);
  //mul_harness_fp.run();

  //Harness add_harness_fp(add<8>, "Add<8> Function Pointer", 10);
  //add_harness_fp.run();

  
  Harness mul_harness_lmbda(lambdaMul, "Mul<8> Lambda", 10);
  mul_harness_lmbda.run();

  Harness add_harness_lmbda(lambdaAdd, "Add<8> Lambda", 10);
  add_harness_lmbda.run();

  return 0;
}

