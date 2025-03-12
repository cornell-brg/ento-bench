#ifndef SAMPLING_H
#define SAMPLING_H

#include <ento-util/containers.h>
#include <ento-util/random.h>

namespace EntoPose
{

using namespace EntoUtil;

template <typename RNG_t, size_t N = 0>
void draw_sample(size_t sample_size,
                 EntoUtil::EntoContainer<size_t, N> *sample,
                 RNG_t &rng)
{
  int n;
  if constexpr(N == 0) n = sample->size();
  else n = N;
  for (size_t i = 0; i < sample_size; ++i)
  {
    bool done = false;
    while (!done)
    {
      (*sample)[i] = random_int(rng) % n;
      done = true;
      for (size_t j = 0; j < i; ++j)
      {
        if ((*sample)[i] == (*sample)[j])
        {
          done = false;
          break;
        }
      }
    }
  }
}

template <typename Scalar, size_t K, size_t MaxIters, bool Enable>
struct ProsacState {};

template <typename Scalar, size_t K, size_t MaxIters>
struct ProsacState<Scalar, K, MaxIters, true>
{
  // Only needed if PROSAC is enabled...
  EntoUtil::EntoContainer<size_t, 0> growth;
  size_t prosac_k_;
  size_t prosac_subset_sz_;

  void initialize(size_t num_data)
  {
    growth.resize(std::max(num_data, K), 0);

    Scalar t_n = static_cast<Scalar>(MaxIters);
    for (size_t i = 0; i < K; ++i)
      t_n *= static_cast<Scalar>(K - i) / (K - i);

    for (size_t n = 0; n < K; ++n)
      growth[n] = 1;

    size_t t_np = 1;
    for (size_t n = K; n < num_data; ++n)
    {
      Scalar t_n_next = t_n * (n + static_cast<Scalar>(1.0)) /
                              (n + static_cast<Scalar>(1.0) - K);
      t_np += std::ceil(t_n_next - t_n);
      growth[n] = t_np;
      t_n = t_n_next;
    }
    
    prosac_k_ = 1;
    prosac_subset_sz_ = K;

  }
};

template <typename Scalar, size_t K, typename RNG_t = uint32_t, bool UsePROSAC = false, size_t MaxPROSACIters = 100000>
class RandomSampler
{
public:
  RandomSampler(size_t N,
                RNG_t seed = 0)
  : num_data_(N), state_(seed) 
  {
    if constexpr(UsePROSAC)
    {
      prosac_state_.initialize(num_data_);
    }
  }

  void generate_sample(EntoUtil::EntoContainer<size_t> *sample)
  {
    if constexpr (UsePROSAC)
    {
      if (prosac_state_.prosac_k_ < MaxPROSACIters)
      {
        draw_sample<K, RNG_t>(sample_size_ - 1,
                              prosac_state_.prosac_subset_sz_ - 1,
                              sample,
                              state_);
        (*sample)[sample_size_ - 1] = prng_next_int(state_);
      }
    }
    else
    {
      draw_sample<K, RNG_t>(sample_size_, num_data_, sample, state_);
    }
  }

public:
  static constexpr size_t sample_size_ = K;
  size_t num_data_;
  RNG_t  state_;

  ProsacState<Scalar, K, MaxPROSACIters, UsePROSAC> prosac_state_;


};



} // namespace EntoPose

#endif // SAMPLING_H
