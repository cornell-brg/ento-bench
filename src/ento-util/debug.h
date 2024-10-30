#ifndef ENTO_DEBUG_H__
#define ENTO_DEBUG_H__

#include <cstdio>

// ENTO_DEBUG_ ( ... )
// Print out debug info when in DEBUG build (not RELEASE). Debug
// info is only dumped to stdout when __n > 0 (i.e., we are looking
// at a specific test function)

#ifdef DEBUG

#include <ento-util/unittest.h>

#define DPRINTF(...) std::printf(__VA_ARGS__)

#define ENTO_DEBUG( ... ) \
  if ( EntoUtil::__n > 0 ) { \
    std::printf(" - [ " YELLOW "-info-" RESET " ] File %s:%d: ",        \
        EntoUtil::__ento_debug_get_file_name(__FILE__), __LINE__);      \
    std::printf(__VA_ARGS__); \
    std::printf("\n"); \
  }

#define ENTO_DEBUG_ARRAY_INT(array_, size_)                         \
  if ( EntoUtil::__n > 0 ) {                                                      \
    std::printf(" - [ " YELLOW "-info-" RESET " ] %s:%d: %s = { ",      \
      EntoUtil::__ento_debug_get_file_name(__FILE__), __LINE__, #array_);            \
    for ( size_t i = 0; i < size_; i++ ) {                              \
      std::printf( "%d", array_[i] );                                   \
      if ( i != size_-1 )                                               \
        printf( ", " );                                                 \
    }                                                                   \
    std::printf(" }\n");                                                \
  }

#define ENTO_DEBUG_NEWLINE \
  if ( __n > 0 ) { std::printf("\n"); }

#define ENTO_DEBUG_EIGEN_MATRIX(matrix_, rows_, cols_) \
  if ( EntoUtil::__n > 0 ) { \
    std::printf(" - [ " YELLOW "-info-" RESET " ] %s:%d: \n%s =",      \
      EntoUtil::__ento_debug_get_file_name(__FILE__), __LINE__, #matrix_);\
    constexpr int width = 10;                                           \
    constexpr const char* indent = "\t";                          \
    using ScalarType = typename Eigen::MatrixBase<decltype(matrix_)>::Scalar; \
    for ( size_t i = 0; i < rows_; i++ ) {                              \
      printf("%s[ ", indent);                                         \
      for (size_t j = 0; j < cols_; j++ ) {                             \
        if constexpr (std::is_integral_v<ScalarType>) {                 \
          printf("%*d", width, matrix_(i, j));                          \
        } else if constexpr (std::is_floating_point_v<ScalarType>) {    \
          printf("%*.*f", width, 3, matrix_(i, j));                     \
        }                                                               \
        if ( j != cols_ - 1 )                         \
          printf( ", ", width, 3);                                               \
        else if ( i != rows_ - 1 )                                      \
          printf( "\t]\n" );                                               \
      }                                                                 \
    }                                                                   \
    std::printf("\t]\n");                                                \
  }

#else

#define DPRINTF(...) // no-op

#define ENTO_DPRINTF(...) // no-op

#define ENTO_DEBUG(...) // no-op

#define ENTO_DEBUG_ARRAY_INT(...) // no-op

#define ENTO_DEBUG_NEWLINE  // no-op

#define ENTO_DEBUG_MATRIX(...) // no-op

#define ENTO_DEBUG_ARRAY_FLOAT(...) // no-op

#define ENTO_DEBUG_EIGEN_MATRIX(...) // no-op

#endif

#endif // ENTO_DEBUG_H__
