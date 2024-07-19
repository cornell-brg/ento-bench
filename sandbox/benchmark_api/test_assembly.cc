#include "bench_funcptr.hh"

template<int N>
inline void __attribute__((always_inline)) add()
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
    :
    :
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

int main() {
  using Scalar = float;
  //using namespace assembly;
  constexpr int MaxRows = 100;
  constexpr int REPS = 8;
  constexpr int REPS2 = 16;

  auto ladd = []() { add<REPS>(); };
  auto ladd2 = []() { add<REPS2>(); };
  //auto lmul = []() { mul<REPS>(); };
  //using Harness = AssemblHarness<void(*)()>

  //add<8>();
  //AssemblyHarness singleMicroBenchmark(add<REPETITIONS>);
  auto lambdaAssemblyHarness = make_assembly_harness(ladd);
  lambdaAssemblyHarness.run();

  std::array microbenchmarks = make_no_io_lambda(add<REPS2>, add<REPS>, add<REPS2>);
  auto multiAssemblyHarness = make_assembly_harness(microbenchmarks);
  multiAssemblyHarness.run();

  // Array of functions
  std::array functions = { add<REPS2>, add<REPS>, add<REPS2> };
  auto microBenchmarkHarnessMulti = make_assembly_harness(functions);
  microBenchmarkHarnessMulti.run();

  return 0;
}

