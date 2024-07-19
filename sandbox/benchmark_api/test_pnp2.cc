#include "bench_funcptr.hh"
#include <Eigen/Dense>

// Example PnP function
template <typename Scalar, int maxN, int StorageOrder>
Eigen::Matrix<Scalar, 4, 4>
example_pnp_function_dynamic(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2,
                                         StorageOrder, maxN, 2>& xprime, 
                             const Eigen::Matrix<Scalar, Eigen::Dynamic, 3,
                                         StorageOrder, maxN, 3>& x) {
  // Dummy implementation
  return Eigen::Matrix<Scalar, 4, 4>::Zero(4, 4);
}

template <typename Scalar, int N, int StorageOrder>
Eigen::Matrix<Scalar, 4, 4>
example_pnp_function_fixed(const Eigen::Matrix<Scalar, N, 2,
                                               StorageOrder, N, 2>& xprime,
                           const Eigen::Matrix<Scalar, N, 3,
                                               StorageOrder, N, 3>& x) {
  // Dummy implementation
  return Eigen::Matrix<Scalar, 4, 4>::Zero(4, 4);
}

int main() {
  // Create example data

  using Scalar = float;
  constexpr int maxN = 100;
  constexpr int storageOrder = Eigen::ColMajor;


  Eigen::Matrix<Scalar, Eigen::Dynamic, 2, storageOrder, maxN, 2> xprime_max = Eigen::Matrix<Scalar, maxN, 2>::Zero();
  Eigen::Matrix<Scalar, Eigen::Dynamic, 3, storageOrder, maxN, 3> x_max = Eigen::Matrix<Scalar, maxN, 3>::Zero();

  // Determine the number of rows to use at runtime
  volatile int rows_to_use = 50;  // Example value; in a real scenario, this would be determined at runtime

  // Create blocks with the actual number of rows to use
  auto xprime = xprime_max.topRows(rows_to_use);
  auto x = x_max.topRows(rows_to_use);

  // Array of PnP functions with dynamic rows
  auto functions = std::array{example_pnp_function_dynamic<Scalar, maxN, storageOrder>,
                              example_pnp_function_dynamic<Scalar, maxN, storageOrder>};
  auto multipleHarness = make_pnp_harness<Scalar, Eigen::Dynamic, maxN, storageOrder>(functions);
  multipleHarness.run_impl(xprime, x);

  Eigen::Matrix<Scalar, 50, 2> x50prime = Eigen::Matrix<Scalar, 50, 2>::Zero();
  Eigen::Matrix<Scalar, 50, 3> x50 = Eigen::Matrix<Scalar, 50, 3>::Zero();
  // Single PnP function with fixed rows
  auto singleHarnessFixed = make_pnp_harness<Scalar, 50, 50, storageOrder>(example_pnp_function_fixed<50, storageOrder>);
  singleHarnessFixed.run_impl(x50prime, x50);

  return 0;
}
