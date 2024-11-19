#include "/Users/amyle/entomoton-bench/external/eigen/Eigen/Dense"
#include <stdlib.h>
#include <stdio.h>
#include "bench/harness.hh"

extern "C" void initialise_monitor_handles(void);

// Template to perform vector addition recursively using Eigen
template <int N>
inline void __attribute__((always_inline)) vector_add_template_recursion_always_inline(Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if constexpr (N > 0) {
    // Perform element-wise vector addition using Eigen's API
    a.noalias() += b;
    vector_add_template_recursion_always_inline<N-1>(a, b);
  }
}

template <int N>
void __attribute__((noinline)) vector_add_template_recursion(Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if constexpr (N > 0) {
    vector_add_template_recursion_always_inline<N>(a, b);
  }
}

int main()
{
  using namespace bench;
  initialise_monitor_handles();

  constexpr int reps = 800; // Adjust the number of repetitions
  constexpr int vector_size = 1000; // Size of the vectors

  // Initialize two vectors with random values
  Eigen::VectorXd a = Eigen::VectorXd::Random(vector_size);
  Eigen::VectorXd b = Eigen::VectorXd::Random(vector_size);

  // Benchmark the vector addition
  Harness harness([&](){ vector_add_template_recursion<reps>(a, b); }, "Eigen Vector Add Benchmark Example", 3);
  harness.run();

  printf("Finished running Eigen vector add benchmark example!\n");

  exit(1);

  return 0;
}
