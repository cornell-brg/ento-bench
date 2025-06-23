//========================================================================
// unittest.h
//========================================================================
// Unit testing framework for EntoBench. Modified from
// ECE2400/4750 testing framework.
//
// Author: Yanghui Ou, Peitian Pan, Derin Ozturk
//   Date: Oct 13, 2020

#ifndef ENTO_UNITTEST_H
#define ENTO_UNITTEST_H

#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace EntoUtil 
{

#define __RED    "\033[31m"
#define __GREEN  "\033[32m"
#define __YELLOW "\033[33m"
#define __RESET  "\033[0m"

constexpr float __rel_tol_pct = 0.01;
template <typename PixelT>
constexpr PixelT __ento_image_default_tol();

template <>
constexpr uint8_t __ento_image_default_tol<uint8_t>() { return 0; }

template <>
constexpr int16_t __ento_image_default_tol<int16_t>() { return 0; }

template <>
constexpr float __ento_image_default_tol<float>() { return 1e-2f; }

template <>
constexpr double __ento_image_default_tol<double>() { return 1e-6; }

//template <>
//constexpr FixedPoint<8, 8, int16_t> __ento_image_default_tol<FixedPoint<8, 8, int16_t>>() {
//  return FixedPoint<8, 8, int16_t>(1);  // = 1/256
//}

typedef unsigned int uint_t;

//========================================================================
// ENTO_TEST_CHECK_* macros
//========================================================================

// __n  > 0: display full [ passed ] line
// __n == 0: do not display anything for passed case
// __n  < 0: display a dot for passed case
extern int __n;

// The status of the current test file. Any failed check sets
// this variable to 1.
extern int __failed;

// Temporary variable to save the condition so that we don't
// evaluate the given expressions multiple times.
extern int   __failure_condition;
extern int   __int_expr0;
extern int   __int_expr1;
extern float __float_expr0;
extern float __float_expr1;

// Check macro helper functions
const char* __ento_debug_get_file_name(const char*);
void __ento_test_start(const char*);
void __ento_test_end(const char*);
void  __ento_test_fail(const char*, int, const char*);
void  __ento_test_check_and_print_uniop(const char*, int, const char*);
void  __ento_test_check_and_print_int_binop(const char*, int, const char*, const char*);
void  __ento_test_check_and_print_float_binop(const char*, int, const char*, const char*);

// Other helper functions
bool  __ento_test_num(int, int);
int   __ento_get_test_num_from_file(const char*);
bool  __ento_is_mcu_test();
void  __ento_replace_file_suffix(const char*, const char*);

// Constexpr Helpers
// Define a global buffer within the header for internal use
inline char __ento_cmdline_args_path_buffer[256] = {};

//------------------------------------------------------------------------
// ENTO_TEST_END()
//------------------------------------------------------------------------
// Handle end of a test file. Print to terminal depending on cmdline args.
#define ENTO_TEST_START()    \
  __ento_test_start( __FILE__ ); \

//------------------------------------------------------------------------
// ENTO_TEST_END()
//------------------------------------------------------------------------
// Handle end of a test file. Print to terminal depending on cmdline args.
#define ENTO_TEST_END()        \
  __ento_test_end( __FILE__ ); \
  return __failed;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_FAIL()
//------------------------------------------------------------------------
// Unconditionally fail a test case.

#define ENTO_TEST_CHECK_FAIL() \
  __ento_test_fail( __FILE__, __LINE__, "ENTO_TEST_CHECK_FAIL" ); \
  return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_TRUE( expr_ )
//------------------------------------------------------------------------
// Checks to see if the expression is true.

#define ENTO_TEST_CHECK_TRUE(expr_) \
  EntoUtil::__int_expr0 = expr_; \
  EntoUtil::__failure_condition = !EntoUtil::__int_expr0; \
  EntoUtil::__ento_test_check_and_print_uniop( __FILE__, __LINE__, #expr_ ); \
  if ( EntoUtil::__failure_condition ) return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_FALSE( expr_ )
//------------------------------------------------------------------------
// Checks to see if the expression is false.

#define ENTO_TEST_CHECK_FALSE(expr_) \
  EntoUtil::__int_expr0 = expr_; \
  EntoUtil::__failure_condition = EntoUtil::__int_expr0; \
  EntoUtil::__ento_test_check_and_print_uniop( __FILE__, __LINE__, #expr_ ); \
  if ( EntoUtil::__failure_condition ) return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_INT_EQ( expr0_, expr1_ )
//------------------------------------------------------------------------
// Checks to see if the two expressions are equal using the != operator.

#define ENTO_TEST_CHECK_INT_EQ(expr0_, expr1_) {\
  EntoUtil::__int_expr0 = (int)(expr0_); \
  EntoUtil::__int_expr1 = (int)(expr1_); \
  EntoUtil::__failure_condition = EntoUtil::__int_expr0 != EntoUtil::__int_expr1; \
  EntoUtil::__ento_test_check_and_print_int_binop( __FILE__, __LINE__, #expr0_, #expr1_ ); \
  if ( EntoUtil::__failure_condition ) return;}

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_APPROX_EQ( expr0_, expr1_, pct_ )
//------------------------------------------------------------------------
// Checks to see if the two expressions are within percent of each other.

#define ENTO_TEST_CHECK_APPROX_EQ(expr0_, expr1_, pct_) \
  EntoUtil::__float_expr0 = expr0_; \
  EntoUtil::__float_expr1 = expr1_; \
  EntoUtil::__failure_condition = fabsf( EntoUtil::__float_expr0 - EntoUtil::__float_expr1 ) > fabs( (float)(pct_) * EntoUtil::__float_expr1 ); \
  EntoUtil::__ento_test_check_and_print_float_binop( __FILE__, __LINE__, #expr0_, #expr1_ ); \
  if ( EntoUtil::__failure_condition ) return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_FLOAT_EQ( expr0_, expr1_ )
//------------------------------------------------------------------------
// Checks to see if two float experssions are within the predefined rel.
// tolerance of each other.

#define ENTO_TEST_CHECK_FLOAT_EQ(expr0_, expr1_)                      \
  ENTO_TEST_CHECK_APPROX_EQ(expr0_, expr1_, EntoUtil::__rel_tol_pct)  


template <typename Derived1, typename Derived2>
inline void __ento_test_check_matrix_eq(const char* file, int lineno, 
                                        const char* expr1, const char* expr2,
                                        const float rel_tol,
                                        const Eigen::MatrixBase<Derived1>& mat1,
                                        const Eigen::MatrixBase<Derived2>& mat2)
{
  file = __ento_debug_get_file_name(file);
  if ((mat1.rows() != mat2.rows()) || (mat1.cols() != mat2.cols()))
  {
    std::printf(__RED "Matrix size mismatch at %s:%d: %s is %ldx%ld, %s is %ldx%ld\n" __RESET,
                file, lineno, expr1, mat1.rows(), mat1.cols(), expr2, mat2.rows(), mat2.cols());
    __failed = 1;
    if (__n < 0 ) std::printf( __RED "." __RESET );
    return;
  }
  bool has_mismatch = false;
  for (int i = 0; i < mat1.rows(); ++i)
  {
    for (int j = 0; j < mat1.cols(); ++j)
    {
      if (std::fabs(mat1(i, j) - mat2(i, j)) > rel_tol)
      {
        has_mismatch = true;
      }
    }
  }
  if (has_mismatch)
  {
    std::printf(__RED "Matrix values mismatch at %s:%d: %s ≠ %s\n" __RESET,
                file, lineno, expr1, expr2);
    __failed = 1;
    if (__n < 0 ) std::printf( __RED "." __RESET );
    return;
  }
  else
  {
    if (__n > 0 )
      std::printf(" - [ " __GREEN "passed" __RESET " ] File %s:%d:  %s == %s\n",
                  file, lineno, expr1, expr2);
    if (__n < 0 ) std::printf( __GREEN "." __RESET );
  }
}

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_EIGEN_MATRIX_EQ
//------------------------------------------------------------------------
// Checks to see if two float experssions are within the predefined rel.
// tolerance of each other.
#define ENTO_TEST_CHECK_EIGEN_MATRIX_EQ(matrix0_, matrix1_) \
  EntoUtil::__ento_test_check_matrix_eq(__FILE__, __LINE__, #matrix0_, #matrix1_, EntoUtil::__rel_tol_pct, matrix0_, matrix1_)

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_EIGEN_MATRIX_EQ_TOL
//------------------------------------------------------------------------
// Checks to see if two float experssions are within user defined rel.
// tolerance of each other.

#define ENTO_TEST_CHECK_EIGEN_MATRIX_EQ_TOL(matrix0_, matrix1_, rel_tol_) \
  EntoUtil::__ento_test_check_matrix_eq(__FILE__, __LINE__, #matrix0_, #matrix1_, rel_tol_, matrix0_, matrix1_)


template <typename Derived1, typename Derived2>
inline void __ento_test_check_quat_eq(const char* file, int lineno, 
                                      const char* expr1, const char* expr2,
                                      const float rel_tol,
                                      const Eigen::QuaternionBase<Derived1>& q1,
                                      const Eigen::QuaternionBase<Derived2>& q2)
{
  file = __ento_debug_get_file_name(file);
  auto dot = std::abs(q1.dot(q2));
  if(dot > 1.0) dot = 1.0;  // Clamp to 1.0 for safety
  // Compute the angular difference (in radians)
  double angle_diff = 2 * std::acos(dot);
  if(angle_diff > rel_tol)
  {
    if ( __n > 0 )
    {
      std::printf(__RED "Quaternion mismatch at %s:%d:  %s != %s (angle difference = %.10e)\n" __RESET,
                  file, lineno, expr1, expr2, angle_diff);
      __failed = 1;
    }
    else if (__n < 0)
      std::printf( __GREEN "." __RESET );
  }
  else
  {
    if ( __n > 0 )
      std::printf(" - [ " __GREEN "passed" __RESET " ] File %s:%d, Function %s:  %s == %s (angle difference = %.10e)\n",
                  file, lineno, expr1, expr2, angle_diff);
    else if ( __n < 0 )
      std::printf( __GREEN "." __RESET );
  }
}


//------------------------------------------------------------------------
// ENTO_TEST_CHECK_ARRAY_INT_EQ( expr0_, expr1_, size_ )
//------------------------------------------------------------------------
// Checks to see if the two arrays of integers are equal using the
// != operator.

#define ENTO_TEST_CHECK_ARRAY_INT_EQ(expr0_, expr1_, size_)             \
  for ( size_t i = 0; i < size_; i++ ) {                                \
    ENTO_TEST_CHECK_INT_EQ( expr0_[i], expr1_[i] );                     \
  }

template <typename Image1, typename Image2>
inline void __ento_test_check_image_eq(const char* file, int lineno, 
                                       const char* expr1, const char* expr2,
                                       typename Image1::pixel_type_ tol,
                                       const Image1& A, const Image2& B)
{
  static_assert(std::is_same<typename Image1::pixel_type_, typename Image2::pixel_type_>::value,
                "Pixel types must match");
  if (A.rows() != B.rows() || A.cols() != B.cols())
  {
    std::printf(__RED "Images are not same size. %s (%d, %d) vs %s (%d,%d)\n" __RESET,
                expr1, A.rows(), A.cols(), expr2, B.rows(), B.cols());
  }

  file = __ento_debug_get_file_name(file);
  using PixelT = typename Image1::pixel_type_;

  bool has_mismatch = false;
  int rows = A.rows();
  int cols = A.cols();
  for (int y = 0; y < rows; ++y)
  {
    for (int x = 0; x < cols; ++x)
    {
      PixelT a = A(y, x);
      PixelT b = B(y, x);
      PixelT diff = (a > b) ? (a - b) : (b - a);
      if (diff > tol)
      {
        if constexpr (std::is_floating_point_v<typename Image1::pixel_type_>)
        {
          std::printf(__RED "Mismatch at (%d, %d): %s = %f, %s = %f\n" __RESET,
                      y, x, expr1, static_cast<float>(a), expr2, static_cast<float>(b));
        }
        else
        {
          std::printf(__RED "Mismatch at (%d, %d): %s = %d, %s = %d\n" __RESET,
                      y, x, expr1, static_cast<int>(a), expr2, static_cast<int>(b));
        }
        has_mismatch = true;
      }
    }
  }

  if (has_mismatch)
  {
    std::printf(__RED "Image mismatch at %s:%d: %s ≠ %s\n" __RESET, file, lineno, expr1, expr2);
    __failed = 1;
    if (__n < 0 ) std::printf(__RED "." __RESET);
    return;
  }

  if (__n > 0)
    std::printf(" - [ " __GREEN "passed" __RESET " ] File %s:%d:  %s == %s\n", file, lineno, expr1, expr2);
  if (__n < 0)
    std::printf(__GREEN "." __RESET);
}

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_IMAGE_EQ( img0_, img1_ )
//------------------------------------------------------------------------
// Checks to see if the two images are equal by comparing each pixel.
// Uses a default tolerance value based on the image pixel type.
// Supports uint8_t, int16_t, float, double, and FixedPoint pixels.
// Reports mismatches with full pixel coordinates.

#define ENTO_TEST_CHECK_IMAGE_EQ(img0_, img1_) \
  EntoUtil::__ento_test_check_image_eq( \
    __FILE__, __LINE__, #img0_, #img1_, \
    EntoUtil::__ento_image_default_tol< \
      typename std::remove_reference<decltype(img0_)>::type::pixel_type_>(), \
    img0_, img1_)

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_IMAGE_EQ_TOL( img0_, img1_, tol_ )
//------------------------------------------------------------------------
// Checks to see if the two images are equal by comparing each pixel,
// allowing for a user-specified tolerance. Pixel types must match.
// Reports mismatches with full pixel coordinates.
#define ENTO_TEST_CHECK_IMAGE_EQ_TOL(img0_, img1_, tol_) \
  EntoUtil::__ento_test_check_image_eq(__FILE__, __LINE__, #img0_, #img1_, tol_, img0_, img1_)


}

#endif  // ENTO_UNITTEST_H
