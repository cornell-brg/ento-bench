#include <Eigen/Dense>
#include <cassert>
#include <cstdio>
#include "matrix_reader.h"  // Include the matrix_from_file function and MatrixType enum
#include "ento-util/debug.h"


void check_condition(bool condition, const char* testName)
{
  if (condition) {
    printf("%s passed.\n", testName);
  } else {
    printf("%s not passed.\n", testName);
  }
}

// Helper function to test matrix loading
template <typename MatrixType>
bool test_matrix_loading(const char* file_path, const MatrixType& expected_matrix)
{
  MatrixType matrix;
  bool success = matrix_from_file(file_path, matrix);
  if (!success)
  {
    std::fprintf(stderr, "Test failed: Could not load matrix from %s\n", file_path);
    return false;
  }

  bool status = true;

  // Ensure matrix dimensions are the same
  status &= (matrix.rows() == expected_matrix.rows());
  check_condition(matrix.rows() == expected_matrix.rows(), "Row dimension matching");
  if (!status) return status;

  status &= (matrix.cols() == expected_matrix.cols());
  check_condition(status, "Cols dimension matching");
  if (!status) return status;


  // Ensure matrix content is correct
  //auto diff = matrix - expected_matrix;
  status &= matrix.isApprox(expected_matrix, 1e-6);
  check_condition(status, "Matrices match within epsilon");

  printf("Test passed for file: %s\n\n", file_path);
  return true;
}

int main()
{
  // Test case 1: FLOAT matrix
  {
    printf("Testing float matrix.\n");
    Eigen::Matrix<float, 3, 3> expected_matrix(3, 3);
    expected_matrix << 1.1f, 2.2f, 3.3f,
                       4.4f, 5.5f, 6.6f,
                       7.7f, 8.8f, 9.9f;

    test_matrix_loading("/home/ddo26/workspace/entomoton-bench/datasets/unittest/matrix_float.txt",
                        expected_matrix);
  }

  // Test case 2: DOUBLE matrix
  {
    printf("Testing double matrix.\n");
    Eigen::Matrix<double, 3, 3> expected_matrix(3, 3);
    expected_matrix << 1.111, 2.222, 3.333,
                       4.444, 5.555, 6.666,
                       7.777, 8.888, 9.999;

    test_matrix_loading("/home/ddo26/workspace/entomoton-bench/datasets/unittest/matrix_double.txt",
                        expected_matrix);
  }

  // Test case 3: INT32 matrix
  {
    printf("Testing int32_t matrix.\n");
    Eigen::Matrix<int32_t, 3, 3> expected_matrix(3, 3);
    expected_matrix << 1, 2, 3,
                       4, 5, 6,
                       7, 8, 9;

    test_matrix_loading("/home/ddo26/workspace/entomoton-bench/datasets/unittest/matrix_int32.txt",
                        expected_matrix);
  }

  // Test case 4: UINT32 matrix
  {
    printf("Testing uint32_t matrix.\n");
    Eigen::Matrix<uint32_t, 3, 3> expected_matrix(3, 3);
    expected_matrix << 1u, 2u, 3u,
                       4u, 5u, 6u,
                       7u, 8u, 9u;

    test_matrix_loading("/home/ddo26/workspace/entomoton-bench/datasets/unittest/matrix_uint32.txt",
                        expected_matrix);
  }

  // Test case 5: Dimension mismatch (should return false)
  {
    Eigen::Matrix<float, 2, 2> matrix(2, 2);
    bool success = test_matrix_loading("/home/ddo26/workspace/entomoton-bench/datasets/unittest/matrix_float.txt",
                        matrix);
    assert(!success && "Expected failure due to dimension mismatch");
    printf("Test passed: dimension mismatch\n\n");
  }

  return 0;
}
