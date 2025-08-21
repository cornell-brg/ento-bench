#include <cstdio>
#include <string>
#include <sstream>
#include <vector>
#include <ento-ubmark/MatMulProblem.h>
#include <ento-ubmark/mat_mul_kernels.h>
#include <ento-bench/problem.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

using namespace EntoUtil;

// Tolerance for floating point comparisons
constexpr float TOLERANCE = 1e-5f;

// Mock kernel for testing
template <typename T, int M, int N, int P>
struct MockKernel {
  using T_ = T;
  static constexpr bool RequiresScratch_ = false;
  void operator()(const T* A,
                  const T* B,
                  T* C) {
    // Simple implementation that just copies values from B to C
    // (simulating A as identity matrix)
    for (size_t i = 0; i < M; i++) {
      for (size_t j = 0; j < P; j++) {
        // In a real implementation, we'd compute the dot product
        // For testing, just copy B's values since A is identity
        C[i * P + j] = B[i * P + j];
      }
    }
  }
};

void checkCondition(bool condition, const char* testName) {
  if (condition) {
    printf("%s passed.\n", testName);
  } else {
    printf("%s failed.\n", testName);
  }
}

//------------------------------------------------------------------------------
// Test MatMulProblem with EntoContainer
//------------------------------------------------------------------------------
void test_matmul_problem_entocontainer()
{
  ENTO_DEBUG("Running test_matmul_problem_entocontainer...");

  // Constants for matrix dimensions
  constexpr size_t M = 3;
  constexpr size_t N = 3;
  constexpr size_t P = 3;
  
  // Create a MatMulProblem using a simple kernel
  using Kernel = NaiveMatMulKernel<float, M, N, P>;
  using Problem = EntoBench::MatMulProblem<Kernel, float, M, N, P, false, false>;
  
  Kernel kernel;
  Problem problem(kernel);
  
  // Create test data - linear algebra a*b = c
  std::string test_data = "3,3,3,";
  // Matrix A (identity matrix)
  test_data += "1,0,0,0,1,0,0,0,1,";
  // Matrix B (some values)
  test_data += "2,3,4,5,6,7,8,9,10,";
  // Matrix C_gt (expected result = B since A is identity)
  test_data += "2,3,4,5,6,7,8,9,10";
  
  // Deserialize the test data
  bool deserialize_success = problem.deserialize(test_data.c_str());
  ENTO_TEST_CHECK_TRUE(deserialize_success);
  
  // Solve the problem
  problem.solve();
  
  // Create expected result matrix
  EntoUtil::EntoContainer<float, M * P> expected_result;
  bool push_back_success = false;
  push_back_success |= expected_result.push_back(2);
  push_back_success |= expected_result.push_back(3);
  push_back_success |= expected_result.push_back(4);
  push_back_success |= expected_result.push_back(5);
  push_back_success |= expected_result.push_back(6);
  push_back_success |= expected_result.push_back(7);
  push_back_success |= expected_result.push_back(8);
  push_back_success |= expected_result.push_back(9);
  push_back_success |= expected_result.push_back(10);
  ENTO_TEST_CHECK_TRUE(push_back_success);
  
  // Validate the result
  bool validate_success = problem.validate();
  ENTO_TEST_CHECK_TRUE(validate_success);
  
  ENTO_DEBUG("test_matmul_problem_entocontainer PASSED!");
}

//------------------------------------------------------------------------------
// Test MatMulProblem with Eigen
//------------------------------------------------------------------------------
void test_matmul_problem_eigen()
{
  ENTO_DEBUG("Running test_matmul_problem_eigen...");
  
  // Constants for matrix dimensions
  constexpr size_t M = 3;
  constexpr size_t N = 3;
  constexpr size_t P = 3;
  
  // Create a MatMulProblem using an Eigen-based kernel
  using Kernel = EigenStaticMatMulKernel<float, M, N, P, Eigen::RowMajor>;
  using Problem = EntoBench::MatMulProblem<Kernel, float, M, N, P, true, false, Eigen::RowMajor>;
  
  Kernel kernel;
  Problem problem(kernel);
  
  // Create test data - same as before
  std::string test_data = "3,3,3,";
  // Matrix A (identity matrix)
  test_data += "1,0,0,0,1,0,0,0,1,";
  // Matrix B (some values)
  test_data += "2,3,4,5,6,7,8,9,10,";
  // Matrix C_gt (expected result = B since A is identity)
  test_data += "2,3,4,5,6,7,8,9,10";
  
  // Deserialize the test data
  bool deserialize_success = problem.deserialize(test_data.c_str());
  ENTO_TEST_CHECK_TRUE(deserialize_success);
  
  // Solve the problem
  problem.solve();
  
  // Validate the result
  bool validate_success = problem.validate();
  ENTO_TEST_CHECK_TRUE(validate_success);
  
  ENTO_DEBUG("test_matmul_problem_eigen PASSED!");
}

//------------------------------------------------------------------------------
// Test with MockKernel
//------------------------------------------------------------------------------
void test_matmul_problem_with_mock()
{
  ENTO_DEBUG("Running test_matmul_problem_with_mock...");
  
  // Constants for matrix dimensions
  constexpr size_t M = 3;
  constexpr size_t N = 3;
  constexpr size_t P = 3;
  
  using Problem = EntoBench::MatMulProblem<MockKernel<float, M, N, P>, float, M, N, P, false, false>;
  
  MockKernel<float, 3, 3, 3> kernel;
  Problem problem(kernel);
  
  // Create test data - same as before
  std::string test_data = "3,3,3,";
  // Matrix A (identity matrix)
  test_data += "1,0,0,0,1,0,0,0,1,";
  // Matrix B (some values)
  test_data += "2,3,4,5,6,7,8,9,10,";
  // Matrix C_gt (expected result = B since A is identity)
  test_data += "2,3,4,5,6,7,8,9,10";
  
  // Deserialize the test data
  bool deserialize_success = problem.deserialize(test_data.c_str());
  ENTO_TEST_CHECK_TRUE(deserialize_success);
  
  // Solve the problem
  problem.solve();
  
  // Validate the result
  bool validate_success = problem.validate();
  ENTO_TEST_CHECK_TRUE(validate_success);
  
  ENTO_DEBUG("test_matmul_problem_with_mock PASSED!");
}

//------------------------------------------------------------------------------
// Test Column-Major Eigen Storage
//------------------------------------------------------------------------------
void test_matmul_problem_column_major()
{
  ENTO_DEBUG("Running test_matmul_problem_column_major...");
  
  // Constants for matrix dimensions
  constexpr size_t M = 3;
  constexpr size_t N = 3;
  constexpr size_t P = 3;
  
  // Create a MatMulProblem using an Eigen-based kernel with column-major storage
  using Kernel = EigenStaticMatMulKernel<float, M, N, P, Eigen::ColMajor>;
  using Problem = EntoBench::MatMulProblem<Kernel, float, M, N, P, true, false, Eigen::ColMajor>;
  
  Kernel kernel;
  Problem problem(kernel);
  
  // Create test data - same as before
  std::string test_data = "3,3,3,";
  // Matrix A (identity matrix)
  test_data += "1,0,0,0,1,0,0,0,1,";
  // Matrix B (some values)
  test_data += "2,3,4,5,6,7,8,9,10,";
  // Matrix C_gt (expected result = B since A is identity)
  test_data += "2,3,4,5,6,7,8,9,10";
  
  // Deserialize the test data
  bool deserialize_success = problem.deserialize(test_data.c_str());
  ENTO_TEST_CHECK_TRUE(deserialize_success);
  
  // Solve the problem
  problem.solve();
  
  // Validate the result
  bool validate_success = problem.validate();
  ENTO_TEST_CHECK_TRUE(validate_success);
  
  ENTO_DEBUG("test_matmul_problem_column_major PASSED!");
}

//------------------------------------------------------------------------------
// Test Chunked Deserialization with Large Matrices
//------------------------------------------------------------------------------
void test_matmul_problem_large_matrices()
{
  ENTO_DEBUG("Running test_matmul_problem_large_matrices...");
  
  // Constants for larger matrix dimensions
  constexpr size_t M = 20;
  constexpr size_t N = 30; 
  constexpr size_t P = 25;
  
  // Create a MatMulProblem using a simple kernel
  using Kernel = NaiveMatMulKernel<float, M, N, P>;
  using Problem = EntoBench::MatMulProblem<Kernel, float, M, N, P, false, false>;
  
  Kernel kernel;
  Problem problem(kernel);
  
  // Create test data for large matrices
  std::stringstream ss;
  ss << M << "," << N << "," << P << ",";
  
  // Matrix A (simple pattern for testing)
  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < N; j++) {
      ss << (i * 0.1f + j * 0.01f);
      if (i < M-1 || j < N-1) ss << ",";
    }
  }
  ss << ",";
  
  // Matrix B (simple pattern for testing)
  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < P; j++) {
      ss << (i * 0.1f + j * 0.02f);
      if (i < N-1 || j < P-1) ss << ",";
    }
  }
  ss << ",";
  
  // Matrix C_gt (precomputed result)
  for (size_t i = 0; i < M; i++) {
    for (size_t j = 0; j < P; j++) {
      // For simplicity, just use a placeholder value
      // In a real test, this would be the actual expected result
      float value = i * j * 0.01f;
      ss << value;
      if (i < M-1 || j < P-1) ss << ",";
    }
  }
  
  std::string test_data = ss.str();
  ENTO_DEBUG("Generated test data of size: %zu bytes", test_data.size());
  
  // Test chunked deserialization with various chunk sizes suitable for embedded systems
  const size_t chunk_sizes[] = {16, 32, 64, 128, 256};
  
  for (size_t chunk_size : chunk_sizes) {
    ENTO_DEBUG("Testing with chunk size: %zu bytes", chunk_size);
    
    // Reset the problem
    problem.reset_deserialize_state();
    
    // Split the data into chunks
    std::vector<std::string> chunks;
    for (size_t i = 0; i < test_data.size(); i += chunk_size) {
      chunks.push_back(test_data.substr(i, std::min(chunk_size, test_data.size() - i)));
    }
    
    ENTO_DEBUG("Split data into %zu chunks", chunks.size());
    
    // Feed chunks one by one
    bool complete = false;
    for (const auto& chunk : chunks) {
      auto result = problem.deserialize_chunk(chunk.c_str(), chunk.size());
      
      if (result == EntoBench::DeserializeResult::Complete) {
        complete = true;
        break;
      } else if (result == EntoBench::DeserializeResult::Error) {
        ENTO_DEBUG("Error during chunked deserialization with chunk size %zu!", chunk_size);
        break;
      }
      // Continue if NeedMore
    }
    
    ENTO_TEST_CHECK_TRUE(complete);
    
    // No need to solve or validate since we're just testing deserialization
  }
  
  ENTO_DEBUG("test_matmul_problem_large_matrices PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_DEBUG("N: %i", __n);
  ENTO_TEST_START();

  if (__ento_test_num(__n, 1)) test_matmul_problem_entocontainer();
  if (__ento_test_num(__n, 2)) test_matmul_problem_eigen();
  if (__ento_test_num(__n, 3)) test_matmul_problem_with_mock();
  if (__ento_test_num(__n, 4)) test_matmul_problem_column_major();
  if (__ento_test_num(__n, 5)) test_matmul_problem_large_matrices();

  ENTO_TEST_END();
} 