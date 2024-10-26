#ifndef ENTO_UTIL_POSE_EST_READER_H
#define ENTO_UTIL_POSE_EST_READER_H

#include <cstdio>
#include <cstring>
#include <type_traits>

#include <Eigen/Dense>

#include "ento-util/debug.h"

enum DataLoaderExitCode
{
  END = 0,
  SUCCESS = 1,
  ERROR = 2,
};

enum PoseEstimationType
{
  ABSOLUTE = 1,
  RELATIVE = 2,
  NVIEW    = 3,
};

class PoseEstimationDataLoader
{
private:
  FILE* file_;

  // Experiment Specifications
  int num_points_;
  int num_experiments_;
  float noise_level_;
  int coplanar_;
  PoseEstimationType problem_type_;
public:
  PoseEstimationDataLoader(const char *filename)
  {
    file_ = fopen(filename, "r");
    if (file_ == nullptr)
    {
      DPRINTF("Error opening file: %s\n", filename);
      fprintf(stderr, "Error opening file.\n");
    }

    fscanf(file_, 
           "%i %i %i %f %i",
           (int*)&problem_type_,
           &num_experiments_,
           &num_points_, 
           &noise_level_, 
           &coplanar_);
    
  }

  int   num_points() const { return num_points_; }
  int   num_experiments() const { return num_experiments_; }
  float noise_level() const { return noise_level_; }
  int   coplanar() const { return coplanar_; }

  template<typename DerivedXPrime, typename DerivedX, typename DerivedT>
  bool get_next_line(Eigen::DenseBase<DerivedXPrime>& xprime,
                     Eigen::DenseBase<DerivedX>& x,
                     Eigen::DenseBase<DerivedT>& T)
  {
    using ScalarXprime = typename Eigen::DenseBase<DerivedXPrime>::Scalar;
    using ScalarX      = typename Eigen::DenseBase<DerivedX>::Scalar;
    using ScalarT      = typename Eigen::DenseBase<DerivedT>::Scalar;

    // Use constexpr to determine the appropriate float or double for temporary storage
    constexpr bool is_double_Xprime = std::is_same<ScalarXprime, double>::value;
    constexpr bool is_double_X = std::is_same<ScalarX, double>::value;
    constexpr bool is_double_T = std::is_same<ScalarT, double>::value;

    // Use either float or double for the temporary variables based on the matrix scalar type
    using TempTypeXprime = std::conditional_t<is_double_Xprime, double, float>;
    using TempTypeX = std::conditional_t<is_double_X, double, float>;
    using TempTypeT = std::conditional_t<is_double_T, double, float>;

    constexpr const char* xprime_format = std::is_same<ScalarXprime, double>::value ? "%lf,%lf,%lf," : "%f,%f,%f,";
    constexpr const char* x_format      = std::is_same<ScalarX, double>::value ? "%lf,%lf," : "%f,%f,";
    constexpr const char* T_format      = std::is_same<ScalarT, double>::value ? 
                                        "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf" : "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f";
    DataLoaderExitCode ret;

    if (xprime.rows() != 3 && xprime.cols() != num_points_)
    {
      DPRINTF("Xprime does not have correct dimensions. Got %i, %i. Expected %i, %i.\n",
              xprime.rows(), xprime.cols(), 3, num_points_);
      ret = ERROR;
      return ret;
    }
    if (x.rows() != 2 && x.cols() == num_points_)
    {
      // DPRINTF and bool
      ENTO_DEBUG("X does not have correct dimensions. Got %ti, %ti. Expected %i, %i.\n",
              x.rows(), x.cols(), 2, num_points_);
      ret = ERROR;
      return ret;
    }
    if (T.rows() != 3 && T.cols() != 4)
    {
      DPRINTF("T is not a valid transformation matrix by dimensions. Got shape %i, %i. Expected %i, %i",
              T.rows(), T.cols(), 3, 4);
      ret = ERROR;
      return ret;
    }

    // Read xprime data
    for (int i = 0; i < num_points_; ++i) {
      if (fscanf(file_, xprime_format, &xprime(0, i), &xprime(1, i), &xprime(2, i)) != 3) {
        fprintf(stderr, "Error reading xprime data at point %d.\n", i);
        return false;
      }
    }

    // Read x data
    for (int i = 0; i < num_points_; ++i) {
      if (fscanf(file_, x_format, &x(0, i), &x(1, i)) != 2) {
        fprintf(stderr, "Error reading x data at point %d.\n", i);
        return false;
      }
    }

    // Read T matrix in one line
    if (fscanf(file_, T_format,
               &T(0, 0), &T(0, 1), &T(0, 2), &T(0, 3),
               &T(1, 0), &T(1, 1), &T(1, 2), &T(1, 3),
               &T(2, 0), &T(2, 1), &T(2, 2), &T(2, 3)) != 12) {
      fprintf(stderr, "Error reading T matrix.\n");
      return false;
    }

    if (feof(file_))
    {
      ret = END;
      return ret;
    }

    ret = SUCCESS;
    return ret;
  }
};

#endif
