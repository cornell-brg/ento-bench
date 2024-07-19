#include <cstdio>
#include <math.h>
#include "FixedPoint.hh"
#include "EigenFixedPoint.hh"
#include "EigenSpecializations.hh"

using Q1_7 = FixedPoint<1, 7, uint8_t>;

bool is_close(float a, float b, float tol = 1e-6) {
  return fabs(a - b) < tol;
}

int main() {
  // Define statically sized Eigen matrices with Q1_7
  Eigen::Matrix<Q1_7, 4, 4> matA;
  Eigen::Matrix<Q1_7, 4, 4> matB;
  Eigen::Matrix<Q1_7, 4, 4> matC;

  // Initialize matrices matA and matB with example values
  matA << Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8),
          Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8),
          Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8),
          Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8);

  matB << Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8),
          Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8),
          Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8),
          Q1_7::from_raw(64), Q1_7::from_raw(32), Q1_7::from_raw(16), Q1_7::from_raw(8);

  // Perform matrix multiplication
  matC = matA * matB;

// Print and check result
  printf("Result matrix C:\n");
  uint8_t expected_fixed_values[4][4] = {
    {128, 64, 32, 16},
    {128, 64, 32, 16},
    {128, 64, 32, 16},
    {128, 64, 32, 16}
  };
  float expected_float_values[4][4] = {
    {1.0, 0.5, 0.25, 0.125},
    {1.0, 0.5, 0.25, 0.125},
    {1.0, 0.5, 0.25, 0.125},
    {1.0, 0.5, 0.25, 0.125}
  };

  bool passed = true;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float result_float = matC(i, j).to_float();
      uint8_t result_raw = matC(i, j).raw();
      printf("C[%d][%d] = %.6f (raw: %u)\n", i, j, result_float, result_raw);
      if (result_raw != expected_fixed_values[i][j]) {
        printf("Mismatch at C[%d][%d]: expected raw %u, got %u\n", i, j, expected_fixed_values[i][j], result_raw);
        passed = false;
      }
      if (!is_close(result_float, expected_float_values[i][j])) {
        printf("Float mismatch at C[%d][%d]: expected %.6f, got %.6f\n", i, j, expected_float_values[i][j], result_float);
        passed = false;
      }
    }
  }

  if (passed) {
    printf("All checks passed!\n");
  } else {
    printf("Some checks failed.\n");
  }
  return 0;
}
