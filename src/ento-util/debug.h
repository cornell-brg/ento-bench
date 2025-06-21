#ifndef ENTO_DEBUG_H__
#define ENTO_DEBUG_H__

// ENTO_DEBUG_ ( ... )
// Print out debug info when in DEBUG build (not RELEASE). Debug
// info is only dumped to stdout when __n > 0 (i.e., we are looking
// at a specific test function)

#ifdef DEBUG

#include <ento-util/unittest.h>
#include <type_traits>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>

#define DPRINTF(...) std::printf(__VA_ARGS__)

// Helper function for general debug printing (includes file, line, and function name)
template <typename... Args>
inline void __ento_debug_printf(const char* file, int line, const char* func, const char* fmt, Args... args)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s: ", 
                EntoUtil::__ento_debug_get_file_name(file), line, func);
    std::printf(fmt, args...);
    std::printf("\n");
  }
}
#define ENTO_DEBUG(...) __ento_debug_printf(__FILE__, __LINE__, __func__, __VA_ARGS__)

// Helper function for error printing (includes file, line, and function name)
template <typename... Args>
inline void __ento_error_printf(const char* file, int line, const char* func, const char* fmt, Args... args)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __RED "-ERROR-" __RESET " ] File %s:%d, Function %s: ", 
                EntoUtil::__ento_debug_get_file_name(file), line, func);
    std::printf(fmt, args...);
    std::printf("\n");
  }
}
#define ENTO_ERROR(...) __ento_error_printf(__FILE__, __LINE__, __func__, __VA_ARGS__)

// Helper function for printing an array of integers (includes function name)
inline void __ento_debug_print_array_int(const char* file, int line, const char* func,
                                          const char* name, const int* array, size_t size)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s: %s = { ", 
                EntoUtil::__ento_debug_get_file_name(file), line, func, name);
    for (size_t i = 0; i < size; ++i)
    {
      std::printf("%d", array[i]);
      if (i != size - 1)
        std::printf(", ");
    }
    std::printf(" }\n");
  }
}
#define ENTO_DEBUG_ARRAY_INT(array_, size_) \
  __ento_debug_print_array_int(__FILE__, __LINE__, __func__, #array_, array_, size_)

// Helper function for printing Eigen matrices (includes function name)
template <typename Derived>
inline void __ento_debug_print_eigen_matrix(const char* file, int line, const char* func,
                                            const char* name, const Eigen::MatrixBase<Derived>& matrix)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s:\n%s =\n", 
                EntoUtil::__ento_debug_get_file_name(file), line, func, name);
    constexpr int width = 9;
    constexpr const char* indent = "\t";
    constexpr int precision = 4;
    for (int i = 0; i < matrix.rows(); ++i)
    {
      std::printf("%s[ ", indent);
      for (int j = 0; j < matrix.cols(); ++j)
      {
        if constexpr (std::is_integral_v<typename Derived::Scalar>)
        {
          std::printf("%*d", width, matrix(i, j));
        }
        else if constexpr (std::is_floating_point_v<typename Derived::Scalar>)
        {
          std::printf("%*.*e", width, precision, matrix(i, j));
        }
        if (j != matrix.cols() - 1)
          std::printf(", ");
      }
      std::printf(" ]\n");
    }
  }
}

#define ENTO_DEBUG_EIGEN_MATRIX(matrix_) \
  __ento_debug_print_eigen_matrix(__FILE__, __LINE__, __func__, #matrix_, matrix_)


template <typename Derived>
inline void __ento_debug_print_eigen_quaternion(const char* file, int line, const char* func,
                                                const char* name, const Eigen::QuaternionBase<Derived>& quat)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s:\n%s (WXYZ) =\n", 
                EntoUtil::__ento_debug_get_file_name(file), line, func, name);
    constexpr int width = 8;
    constexpr const char* indent = "\t";
    constexpr int precision = 4;
       std::printf("%s|w: ", indent);
    if constexpr (std::is_integral_v<typename Derived::Scalar>)
      std::printf("%*d", width, quat.w());
    else if constexpr (std::is_floating_point_v<typename Derived::Scalar>)
      std::printf("%*.*e", width, precision, quat.w());
    std::printf("|\n");
    
    std::printf("%s|x: ", indent);
    if constexpr (std::is_integral_v<typename Derived::Scalar>)
      std::printf("%*d", width, quat.x());
    else if constexpr (std::is_floating_point_v<typename Derived::Scalar>)
      std::printf("%*.*e", width, precision, quat.x());
    std::printf("|\n");
    
    std::printf("%s|y: ", indent);
    if constexpr (std::is_integral_v<typename Derived::Scalar>)
      std::printf("%*d", width, quat.y());
    else if constexpr (std::is_floating_point_v<typename Derived::Scalar>)
      std::printf("%*.*e", width, precision, quat.y());
    std::printf("|\n");
    
    std::printf("%s|z: ", indent);
    if constexpr (std::is_integral_v<typename Derived::Scalar>)
      std::printf("%*d", width, quat.z());
    else if constexpr (std::is_floating_point_v<typename Derived::Scalar>)
      std::printf("%*.*e", width, precision, quat.z());
    std::printf("|\n");
  }
}

#define ENTO_DEBUG_EIGEN_QUATERNION(q) __ento_debug_print_eigen_quaternion(__FILE__, __LINE__, __func__, #q, q)

// Helper function for printing Eigen quaternions side by side
template <typename Derived1, typename Derived2>
inline void __ento_debug_print_eigen_quaternions_side_by_side(const char* file, int line, const char* func,
                                                              const char* name1, const Eigen::QuaternionBase<Derived1>& quat1,
                                                              const char* name2, const Eigen::QuaternionBase<Derived2>& quat2)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s:\n",
                EntoUtil::__ento_debug_get_file_name(file), line, func);

    constexpr int width = 8;
    constexpr int precision = 4;
    constexpr const char* indent = "\t";
    // Mapping for printing in order: w, x, y, z.
    static const int mapping[4] = { 0, 1, 2, 3 };
    static const char* labels[4] = { "w", "x", "y", "z" };

    auto coeffs1 = quat1.coeffs();
    auto coeffs2 = quat2.coeffs();

    constexpr int headerWidth = width + precision + 1;
    std::printf("%s%-*s   %-*s\n", indent, headerWidth, name1, headerWidth, name2);

    for (int i = 0; i < 4; ++i)
    {
      std::printf("%s[%s: ", indent, labels[i]);
      // Print component from first quaternion
      if constexpr (std::is_integral_v<typename Derived1::Scalar>)
        std::printf("%*d", width, coeffs1[mapping[i]]);
      else if constexpr (std::is_floating_point_v<typename Derived1::Scalar>)
        std::printf("%*.*e", width, precision, coeffs1[mapping[i]]);
      std::printf("]   ");

      std::printf("[%s: ", labels[i]);
      // Print component from second quaternion
      if constexpr (std::is_integral_v<typename Derived2::Scalar>)
        std::printf("%*d", width, coeffs2[mapping[i]]);
      else if constexpr (std::is_floating_point_v<typename Derived2::Scalar>)
        std::printf("%*.*e", width, precision, coeffs2[mapping[i]]);
      std::printf("]\n");
    }
  }
}

#define ENTO_DEBUG_EIGEN_QUAT2(q1, q2) \
  __ento_debug_print_eigen_quaternions_side_by_side(__FILE__, __LINE__, __func__, #q1, q1, #q2, q2)

// Helper function for printing Eigen matrices side by side
template <typename Derived1, typename Derived2>
inline void __ento_debug_print_matrix_comparison(const char* file, int line, const char* func,
                                                 const char* name1, const Eigen::MatrixBase<Derived1>& mat1,
                                                 const char* name2, const Eigen::MatrixBase<Derived2>& mat2,
                                                 float tolerance = 1e-5)
{
  using Scalar1 = typename Derived1::Scalar;
  using Scalar2 = typename Derived2::Scalar;

  if (EntoUtil::__n <= 0) return; 

  constexpr int width = 8;
  constexpr int precision = 4;
  constexpr const char* indent = "\t";
  file = EntoUtil::__ento_debug_get_file_name(file);

  if ((mat1.rows() != mat2.rows()) || (mat1.cols() != mat2.cols()))
  {
    std::printf(" - [ " __RED "-ERROR-" __RESET " ] Matrix size mismatch at %s:%d, Function %s:\n", file, line, func);
    std::printf("%s  %s (%ldx%ld) != %s (%ldx%ld)\n", indent, name1, mat1.rows(), mat1.cols(), name2, mat2.rows(), mat2.cols());
    return;
  }

  std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s:\n", file, line, func);
  std::printf("%s%-*s   %-*s\n", indent, width, name1, width, name2);

  for (int i = 0; i < mat1.rows(); ++i)
  {
    std::printf("%s[", indent);
    for (int j = 0; j < mat1.cols(); ++j)
    {
      bool is_float = std::is_floating_point_v<Scalar1> || std::is_floating_point_v<Scalar2>;
      bool within_tol = is_float ? (std::fabs(mat1(i, j) - mat2(i, j)) <= tolerance) : (mat1(i, j) == mat2(i, j));

      const char* color = within_tol ? __GREEN : __RED;
      std::printf("%s", color);
      
      if constexpr (std::is_integral_v<Scalar1> && std::is_integral_v<Scalar2>)
      {
        std::printf("%*d", width, mat1(i, j));
      }
      else
      {
        std::printf("%*.*e", width, precision, mat1(i, j));
      }
      
      std::printf(__RESET);
      if (j != mat1.cols() - 1) std::printf(", ");
    }

    std::printf("]   [");
    for (int j = 0; j < mat2.cols(); ++j)
    {
      bool is_float = std::is_floating_point_v<Scalar1> || std::is_floating_point_v<Scalar2>;
      bool within_tol = is_float ? (std::fabs(mat1(i, j) - mat2(i, j)) <= tolerance) : (mat1(i, j) == mat2(i, j));

      const char* color = within_tol ? __GREEN : __RED;
      std::printf("%s", color);
      
      if constexpr (std::is_integral_v<Scalar1> && std::is_integral_v<Scalar2>)
      {
        std::printf("%*d", width, mat2(i, j));
      }
      else
      {
        std::printf("%*.*e", width, precision, mat2(i, j));
      }
      
      std::printf(__RESET);
      if (j != mat2.cols() - 1) std::printf(", ");
    }
    std::printf("]\n");
  }
}

#define ENTO_DEBUG_EIGEN_MATRIX_COMPARISON(matrix0_, matrix1_, tol_) \
  __ento_debug_print_matrix_comparison(__FILE__, __LINE__, __func__, #matrix0_, matrix0_, #matrix1_, matrix1_, tol_)

// Generic debug print for any array-like container or pointer+size
// Prints each element as int
// Usage: ENTO_DEBUG_ARRAY(container) or ENTO_DEBUG_ARRAY(ptr, size)
template <typename T>
inline void __ento_debug_print_array(const char* file, int line, const char* func,
                                    const char* name, const T& container)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s: %s = { ",
                EntoUtil::__ento_debug_get_file_name(file), line, func, name);
    for (size_t i = 0; i < container.size(); ++i)
    {
      std::printf("%d", static_cast<int>(container[i]));
      if (i != container.size() - 1)
        std::printf(", ");
    }
    std::printf(" }\n");
  }
}

template <typename T>
inline void __ento_debug_print_array(const char* file, int line, const char* func,
                                    const char* name, const T* array, size_t size)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s: %s = { ",
                EntoUtil::__ento_debug_get_file_name(file), line, func, name);
    for (size_t i = 0; i < size; ++i)
    {
      std::printf("%d", static_cast<int>(array[i]));
      if (i != size - 1)
        std::printf(", ");
    }
    std::printf(" }\n");
  }
}

#define ENTO_DEBUG_ARRAY(array_) \
  __ento_debug_print_array(__FILE__, __LINE__, __func__, #array_, array_)

#define ENTO_DEBUG_ARRAY_PTR(array_, size_) \
  __ento_debug_print_array(__FILE__, __LINE__, __func__, #array_, array_, size_)

#define ENTO_DEBUG_NEWLINE \
  if (EntoUtil::__n > 0) { std::printf("\n"); }

//------------------------------------------------------------------------
// ENTO_DEBUG_IMAGE
//------------------------------------------------------------------------
// Print a small image (templated EntoBench Image class) with optional label.
// Pixels printed as integers or floats depending on type.

#include <type_traits> // for std::is_floating_point

template <typename ImageT>
inline void __ento_debug_print_image(const char* file, int line, const char* func,
                                     const char* name, const ImageT& img)
{
  using PixelT = typename ImageT::pixel_type_;
  constexpr int width = 12;
  constexpr int precision = 8;
  constexpr const char* indent = "\t";

  const int W = img.rows();
  const int H = img.cols();
  if (EntoUtil::__n <= 0) return;

  std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s:\n%s =\n",
              EntoUtil::__ento_debug_get_file_name(file), line, func, name);

  for (int y = 0; y < H; ++y)
  {
    std::printf("%s[ ", indent);
    for (int x = 0; x < W; ++x)
    {
      if constexpr (std::is_floating_point_v<PixelT>)
        std::printf("%*.*f", width, precision, static_cast<float>(img(y, x)));
      else
        std::printf("%*d", width, int(img(y, x)));

      if (x != W - 1)
        std::printf(", ");
    }
    std::printf(" ]\n");
  }
}

#define ENTO_DEBUG_IMAGE(image_) \
  __ento_debug_print_image(__FILE__, __LINE__, __func__, #image_, image_)


#else

#define DPRINTF(...) // no-op

#define ENTO_DPRINTF(...) // no-op

#define ENTO_DEBUG(...) // no-op
                        
#define ENTO_ERROR(...) // no-op

#define ENTO_DEBUG_ARRAY_INT(...) // no-op

#define ENTO_DEBUG_NEWLINE  // no-op

#define ENTO_DEBUG_EIGEN_MATRIX(...) // no-op

#define ENTO_DEBUG_ARRAY_FLOAT(...) // no-op

#define ENTO_DEBUG_EIGEN_QUATERNION(...) // no-op

#define ENTO_DEBUG_EIGEN_QUAT2(...) // no-op
                                    //
#define ENTO_DEBUG_EIGEN_MATRIX_COMPARISON(...) \

#define ENTO_DEBUG_IMAGE(...) // no-op


#endif

#endif // ENTO_DEBUG_H__
