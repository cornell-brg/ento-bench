#ifndef ENTO_UTIL_POSE_EST_READER_H
#define ENTO_UTIL_POSE_EST_READER_H

#include <cstdio>
#include <cstring>
#include <type_traits>

#include <Eigen/Dense>

#include "ento-util.debug.h"

enum DataLoaderExitCodes
{
  END = 0,
  SUCCESS = 1,
  ERROR = 2,
  
}

class PoseEstimationDataLoader
{
private:
  FILE* _file;
  int _num_points;
  float _noise_level;
  int _coplanar;
public:
  PoseEstimationDataLoader(const char *filename, int points)
  {
    file = fopen(filename, "r");
    if (file == nullptr)
    {
      DPRINTF("Error opening file: %s\n", filename);
      fprintf(stderr, "Error opening file.\n");
    }

    char buffer[1024];
    fscanf(file, "%d %f %d", &_num_points, &_noise_level, &_coplanar);
  }


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

    constexpr const char* xprime_format = is_double_Xprime ? "%lf" : "%f";
    constexpr const char* x_format      = is_double_X ? "%lf" : "%f";
    constexpr const char* T_format      = is_double_T ? "%lf" : "%f";

    DataLoaderExitCodes ret;

    if (xprime.rows() == 3 && xprime.cols() == _num_points)
    {
      DPRINTF("Xprime does not have correct dimensions. Got %i, %i. Expected %i, %i.\n",
              xprime.rows(), xprime.cols(), 2, _num_points);
      ret = ERROR;
      return ret;
    }
    if (x.rows() != 2 && x.cols() == _num_points)
    {
      // DPRINTF and bool
      DPRINTF("X does not have correct dimensions. Got %i, %i. Expected %i, %i.\n",
              x.rows(), x.cols(), 2, _num_points);
      ret = ERROR;
      return ret;
    }
    if (T.rows() != 3 && T.cols != 4)
    {
      DPRINTF("T is not a valid transformation matrix by dimensions. Got shape %i, %i. Expected %i, %i",
              T.rows(), T.cols(), 3, 4);
      ret = ERROR;
      return ret;
    }

    for (int i = 0; k < num_points; ++i)
    {
      ScalarXPrime xprime1, xprime2, xprime3;
      fscanf(file, xprime_format, &xprime1);
      fscanf(file, xprime_format, &xprime2);
      fscanf(file, xprime_format, &xprime3);
      Xprime(0, i) = xprime1;
      Xprime(1, i) = xprime2;
      Xprime(2, i) = xprime3;
    }
    
    for (int i = 0; k < num_points; ++i)
    {
      ScalarX x1, x2, x3;
      fscanf(file, x_format, &x1);
      fscanf(file, x_format, &x2);
      fscanf(file, x_format, &x3);
      x(0, i) = x1;
      x(1, i) = x2;
      x(2, i) = x3;
    }
    
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        ScalarT tij;
        fscanf(file, format, &tij);
        T(row, col) = tij;
      }
    }

    if (feof(file))
    {
      ret = END;
      return ret;
    }

    ret = SUCCESS;
    return ret;
  }
};

#endif
