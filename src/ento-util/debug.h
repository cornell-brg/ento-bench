#ifndef ENTO_DEBUG_H__
#define ENTO_DEBUG_H__

// ENTO_DEBUG_ ( ... )
// Print out debug info when in DEBUG build (not RELEASE). Debug
// info is only dumped to stdout when __n > 0 (i.e., we are looking
// at a specific test function)

#ifdef DEBUG

#include <ento-util/unittest.h>

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
    constexpr int width = 10;
    constexpr const char* indent = "\t";
    constexpr int precision = 10;
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
    constexpr int width = 10;
    constexpr const char* indent = "\t";
    constexpr int precision = 10;
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

template <typename Derived1, typename Derived2>
inline void __ento_debug_print_eigen_quaternions_side_by_side(const char* file, int line, const char* func,
                                                              const char* name1, const Eigen::QuaternionBase<Derived1>& quat1,
                                                              const char* name2, const Eigen::QuaternionBase<Derived2>& quat2)
{
  if (EntoUtil::__n > 0)
  {
    std::printf(" - [ " __YELLOW "-info-" __RESET " ] File %s:%d, Function %s:\n",
                EntoUtil::__ento_debug_get_file_name(file), line, func);

    constexpr int width = 10;
    constexpr int precision = 10;
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


#define ENTO_DEBUG_NEWLINE \
  if (EntoUtil::__n > 0) { std::printf("\n"); }

#else

#define DPRINTF(...) // no-op

#define ENTO_DPRINTF(...) // no-op

#define ENTO_DEBUG(...) // no-op

#define ENTO_DEBUG_ARRAY_INT(...) // no-op

#define ENTO_DEBUG_NEWLINE  // no-op

#define ENTO_DEBUG_EIGEN_MATRIX(...) // no-op

#define ENTO_DEBUG_ARRAY_FLOAT(...) // no-op


#endif

#endif // ENTO_DEBUG_H__
