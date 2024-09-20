#include <cstdio>

#ifdef DEBUG
  #define DPRINTF(...) std::printf(__VA_ARGS__)
#else
  #define DPRINTF(...) // no-op
#endif
