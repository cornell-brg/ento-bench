#include <ento-util/random.h>

namespace EntoUtil
{
// Internal state for Ento PRNG
static uint32_t __ento_rand_state = 0xDEADBEEF;

void prng_seed(uint32_t seed)
{
  if (seed == 0)
  {
    seed = 0xDEADBEEF;
  }
  __ento_rand_state = seed;
}

// Simple XORShift for random sampling. We might need to consider using a better implementation
// depending on profiling/performance.
uint32_t prng_next_int(void)
{
  __ento_rand_state += 0x6D2B79F5;
  uint32_t t = __ento_rand_state;
  t = static_cast<uint32_t>( t ^ ( t >> 15 ) ) * ( t | 1 );
  t = t + static_cast<uint32_t>( ( t ^ ( t >> 7 ) ) & ( t | 61 ));
  t = t ^ (t >> 14);
  return t;
}

uint32_t prng_next_int(uint32_t &state)
{
  state += 0x6D2B79F5;
  uint32_t t = state;
  t = static_cast<uint32_t>( t ^ ( t >> 15 ) ) * ( t | 1 );
  t = t + static_cast<uint32_t>( ( t ^ ( t >> 7 ) ) & ( t | 61 ));
  t = t ^ (t >> 14);
  return t;
}

uint32_t prng_next_float(void)
{
  __ento_rand_state += 0x6D2B79F5;
  uint32_t t = __ento_rand_state;
  t = static_cast<uint32_t>( t ^ ( t >> 15 ) ) * ( t | 1 );
  t = t + static_cast<uint32_t>( ( t ^ ( t >> 7 ) ) & ( t | 61 ));
  t = t ^ (t >> 14);
  return static_cast<float>(t & 0xFFFFFFFF) / 4294967296.0f;
}

uint32_t prng_next_float(uint32_t &state)
{
  state += 0x6D2B79F5;
  uint32_t t = state;
  t = static_cast<uint32_t>( t ^ ( t >> 15 ) ) * ( t | 1 );
  t = t + static_cast<uint32_t>( ( t ^ ( t >> 7 ) ) & ( t | 61 ));
  t = t ^ (t >> 14);
  return static_cast<float>(t & 0xFFFFFFFF) / 4294967296.0f;
}

} // namespace EntoUtil
