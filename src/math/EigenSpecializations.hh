#ifndef EIGEN_SPECIALIZATIONS_HH
#define EIGEN_SPECIALIZATIONS_HH

#include <Eigen/Core>
#include "EigenConcepts.hh"
#include "FixedPoint.hh"
#include "EigenFixedPoint.hh"
#include "arm_math.h"

// Q1.7
template <typename T>
concept FixedPoint1_7 = std::is_same_v<T, FixedPoint<1, 7, uint8_t>>;

template<typename T>
concept FixedPoint1_15 = std::is_same_v<T, FixedPoint<1, 15, uint16_t>>;

template<typename T>
concept FixedPoint1_31 = std::is_same_v<T, FixedPoint<1,31, uint32_t>>;

// Specialized matrix multiplication for Eigen matrices with FixedPoint<1, 15, uint32_t>
template <FixedPoint1_7 Q1_7, int Rows, int Cols, int Inner>
requires CompatibleMatrixMultiplication<Eigen::Matrix<Q1_7, Rows, Inner>,
                                        Eigen::Matrix<Q1_7, Inner, Cols>>
Eigen::Matrix<Q1_7, Rows, Cols> operator*(const Eigen::Matrix<Q1_7, Rows, Inner>& lhs,
                                          const Eigen::Matrix<Q1_7, Inner, Cols>& rhs) {
  Eigen::Matrix<Q1_7, Rows, Cols> result(lhs.rows(), rhs.cols());
  int numRowsA = lhs.rows();
  int numColsA = lhs.cols();
  int numColsB = rhs.cols();

  for (int i = 0; i < numRowsA; ++i) {
    for (int j = 0; j < numColsB; ++j) {
      int32_t sum = 0;
      for (int k = 0; k < numColsA; k += 4) {
        // Load four 8-bit elements from lhs and rhs
        uint32_t lhs_val = *(reinterpret_cast<const uint32_t*>(&lhs(i, k)));
        uint32_t rhs_val = *(reinterpret_cast<const uint32_t*>(&rhs(k, j)));
        
        // Perform SIMD multiplication and accumulation
        sum = __SMLAD(lhs_val, rhs_val, sum);
      }
      // Store the accumulated sum back into the result matrix
      result(i, j) = Q1_7::from_raw(__SSAT(sum >> 7, 8)); // Adjust for fixed-point scaling
    }
  }
  return result;
}



// Specialized matrix addition for Eigen matrices with FixedPoint<1, 15, uint32_t>
template <FixedPoint1_7 Q1_7, int Rows, int Cols>
Eigen::Matrix<Q1_7, Rows, Cols> operator+(const Eigen::Matrix<Q1_7, Rows, Cols>& lhs,
                                          const Eigen::Matrix<Q1_7, Rows, Cols>& rhs) {
  Eigen::Matrix<Q1_7, Rows, Cols> result(lhs.rows(), lhs.cols());
  for (int i = 0; i < lhs.rows(); ++i) {
    for (int j = 0; j < lhs.cols(); ++j) {
      result(i, j) = lhs(i, j) + rhs(i, j);
    }
  }
  return result;
}

template <FixedPoint1_15 Q1_15, int Rows, int Cols, int Inner>
requires CompatibleMatrixMultiplication<Eigen::Matrix<Q1_15, Rows, Inner>,
                                        Eigen::Matrix<Q1_15, Inner, Cols>>
Eigen::Matrix<Q1_15, Rows, Cols> operator*(const Eigen::Matrix<Q1_15, Rows, Inner>& lhs,
                                           const Eigen::Matrix<Q1_15, Inner, Cols>& rhs) {
  Eigen::Matrix<Q1_15, Rows, Cols> result(lhs.rows(), rhs.cols());
  int numRowsA = lhs.rows();
  int numColsA = lhs.cols();
  int numColsB = rhs.cols();

  // Transpose the rhs matrix
  Eigen::Matrix<Q1_15, Cols, Rows> rhsTransposed = rhs.transpose();

  for (int i = 0; i < numRowsA; ++i) {
    for (int j = 0; j < numColsB; ++j) {
      int64_t sum = 0;
      const uint16_t* pInA = reinterpret_cast<const uint16_t*>(lhs.data() + i * numColsA);
      const uint16_t* pInB = reinterpret_cast<const uint16_t*>(rhsTransposed.data() + j * numColsA);

      // Compute 2 MACs simultaneously
      int colCnt = numColsA >> 2U;
      while (colCnt > 0U) {
        uint32_t inA1 = *reinterpret_cast<const uint32_t*>(pInA);
        uint32_t inB1 = *reinterpret_cast<const uint32_t*>(pInB);
        uint32_t inA2 = *reinterpret_cast<const uint32_t*>(pInA + 2);
        uint32_t inB2 = *reinterpret_cast<const uint32_t*>(pInB + 2);

        sum = __SMLALD(inA1, inB1, sum);
        sum = __SMLALD(inA2, inB2, sum);

        pInA += 4;
        pInB += 4;
        colCnt--;
      }

      // Process remaining column samples
      colCnt = numColsA % 0x4U;
      while (colCnt > 0U) {
        sum += static_cast<int32_t>(*pInA++) * static_cast<int32_t>(*pInB++);
        colCnt--;
      }

      // Saturate and store result in the destination buffer
      result(i, j) = Q1_15::from_raw(__SSAT(sum >> 15, 16));
    }
  }
  return result;
}

#endif // EIGEN_SPECIALIZATIONS_HH

