#include <bench/harness.hh>
#include <workload/microbench/assembly/armv7e-m_ubenchmarks.hh>

/* Benchmark Harness with lambda callable with no inputs, no outputs.
 *
 */
int main()
{
  using namespace bench;

  auto funcA = []() -> void __attribute__((noinline)) {
    int x, y, z;
    x = 1;
    y = 2;
    z = x + y;
    x = z + 2;
  };

  constexpr int REPS = 10;
  auto funcB = []() -> void __attribute__((noinline)) {
    assembly::add<REPS>();
  };

  auto funcC = []() -> void __attribute__((noinline)) {
    assembly::add<10>();
  };

  auto funcD = []() -> void {
    int x, y, z;
    x = 1;
    y = 2;
    z = x + y;
    x = z + 2;
  };
  
  auto funcE = []() -> void {
    volatile int x, y, z;
    x = 1;
    y = 2;
    z = x + y;
    x = z + 2;
  };

  auto funcF = []() -> void __attribute__((always_inline)) {
    int x, y, z;
    x = 1;
    y = 2;
    z = x + y;
    x = z + 2;
  };

  MultiHarness harness1(funcA, funcB, funcC, funcD, funcE, funcF);
  harness1.run();

  return 0;
}


