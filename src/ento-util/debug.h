#include <cstdio>

#define DEBUG

// ENTO_DEBUG_ ( ... )
// Print out debug info when in DEBUG build (not RELEASE). Debug
// info is only dumped to stdout when __n > 0 (i.e., we are looking
// at a specific test function)

#ifdef DEBUG

#define DPRINTF(...) std::printf(__VA_ARGS__)
#define ENTO_DEBUG( ... ) \
  if ( __n > 0 ) { \
    std::printf(" - [ " YELLOW "-info-" RESET " ] File %s:%d: ", __ece2400_get_file_name(__FILE__), __LINE__); \
    std::printf(__VA_ARGS__); \
    std::printf("\n"); \
  }

#define ENTO_DEBUG_ARRAY_INT( array_, size_ )                        \
  if ( __n > 0 ) {                                                      \
    std::printf(" - [ " YELLOW "-info-" RESET " ] %s:%d: %s = { ",      \
      __ento_debug_get_file_name(__FILE__), __LINE__, #array_);            \
    for ( size_t i = 0; i < size_; i++ ) {                              \
      std::printf( "%d", array_[i] );                                   \
      if ( i != size_-1 )                                               \
        printf( ", " );                                                 \
    }                                                                   \
    std::printf(" }\n");                                                \
  }

#define ENTO_DEBUG_NEWLINE \
  if ( __n > 0 ) { std::printf("\n"); }

#else

#define ENTO_DPRINTF(...) // no-op

#define ENTO_DEBUG(...) // no-op

#define ENTO_DEBUG_ARRAY_INT ( ... )

#define ENTO_DEBUG_NEWLINE;

#define ENTO_DEBUG_MATRIX ( ... )

#define ENTO_DEBUG_ARRAY_FLOAT ( ... )


#endif


