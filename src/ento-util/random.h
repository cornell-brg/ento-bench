#ifndef ENTO_RANDOM_H
#define ENTO_RANDOM_H

#include <stdint.h>

namespace EntoUtil
{
  void     prng_seed(uint32_t seed);
  uint32_t prng_next_int(void);
  uint32_t prng_next_int(uint32_t &state);
  uint32_t prng_next_float(void);
  uint32_t prng_next_float(uint32_t &state);
} // namespace EntoUtil

#endif // ENTO_RANDOM_H
