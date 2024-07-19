#include <Eigen/Dense>
#include "harness2.hh"

template <typename Scalar, int maxN, int StorageOrder>
Eigen::Matrix<Scalar, 4, 4>
example_pnp_function_dynamic(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2,
                                         StorageOrder, maxN, 2>& xprime, 
                             const Eigen::Matrix<Scalar, Eigen::Dynamic, 3,
                                         StorageOrder, maxN, 3>& x) {
  return Eigen::Matrix<Scalar, 4, 4>::Zero(4, 4);
}

template <typename Scalar, int N, int StorageOrder>
Eigen::Matrix<Scalar, 4, 4>
example_pnp_function_fixed(const Eigen::Matrix<Scalar, N, 2,
                                               StorageOrder, N, 2>& xprime,
                           const Eigen::Matrix<Scalar, N, 3,
                                               StorageOrder, N, 3>& x) {
  return Eigen::Matrix<Scalar, 4, 4>::Zero(4, 4);
}

template <typename Scalar, int N, int StorageOrder>
Eigen::Matrix<Scalar, 5, 5>
test_function(const Eigen::Matrix<Scalar, N, 2, StorageOrder, N, 2>& xprime)
{
  return Eigen::Matrix<Scalar, 5, 5>::Zero(5, 5);
}

int main() {
  using Scalar = float;
  constexpr int maxN = 100;
  constexpr int N = 50;  
  constexpr int storageOrder = Eigen::ColMajor;

  using Matrix_100dr_2c = Eigen::Matrix<Scalar, Eigen::Dynamic, 2, storageOrder, maxN, 2>;
  using Matrix_100dr_3c = Eigen::Matrix<Scalar, Eigen::Dynamic, 3, storageOrder, maxN, 3>;
  using Matrix_50r_2c  = Eigen::Matrix<Scalar, N, 2, storageOrder, 50, 2>;
  using Matrix_50r_3c  = Eigen::Matrix<Scalar, N, 3, storageOrder, 50, 3>;

  // Create example data
  //Eigen::Matrix<Scalar, Eigen::Dynamic, 2, storageOrder, maxN, 2> xprime_max = Eigen::Matrix<Scalar, maxN, 2>::Zero();
  //Eigen::Matrix<Scalar, Eigen::Dynamic, 3, storageOrder, maxN, 3> x_max = Eigen::Matrix<Scalar, maxN, 3>::Zero();
  auto x_100max = Matrix_100dr_3c::Zero(maxN, 3);
  auto xprime_100max = Matrix_100dr_2c::Zero(maxN, 2);
  auto x_runtime = x_100max.topRows(N);
  auto xprime_runtime = xprime_100max.topRows(N);
  auto x50 = Matrix_50r_3c::Zero();
  auto xprime50 = Matrix_50r_2c::Zero();

  // Single PnP function with fixed rows
  auto func1 = example_pnp_function_fixed<Scalar, 50, storageOrder>;
  auto func2 = example_pnp_function_fixed<Scalar, 20, storageOrder>;
  auto func3 = example_pnp_function_fixed<Scalar, 30, storageOrder>;

  //auto functions = std::array{example_pnp_function_dynamic<Scalar, maxN, storageOrder>,
  //                            example_pnp_function_dynamic<Scalar, maxN, storageOrder>};
                              //test_function<Scalar, maxN, storageOrder>};
  //auto harness_singleFuncFixedRows = PnPHarness(func);
  
  BenchmarkHarness harness(func1);
  
  //BenchmarkHarness harnessArray(functions);
  //BenchmarkHarness harnessArgs(example_pnp_function_dynamic<Scalar, maxN, storageOrder>,
  //                             example_pnp_function_dynamic<Scalar, maxN, storageOrder>);
  BenchmarkHarness harness2(func1, func2);

  // Create the BenchmarkHarness with multiple functions
  //BenchmarkHarness harness(computeSVD, computeCholesky, computeMultiplication, computeInverse);
  //harness.run(matrix);

  return 0;
}
