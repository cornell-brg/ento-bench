#ifndef SAMPLING_H
#define SAMPLING_H

#include <ento-util/containers.h>
#include <ento-util/random.h>

namespace EntoPose
{

using namespace EntoUtil;

template <typename RNG_t, size_t N = 0>
void draw_sample(size_t sample_size,
                 size_t num_data_points,
                 EntoUtil::EntoContainer<size_t, N> *sample,
                 RNG_t &rng)
{
  for (size_t i = 0; i < sample_size; ++i)
  {
    bool done = false;
    while (!done)
    {
      (*sample)[i] = prng_next_int(rng) % num_data_points;
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

  void generate_sample(EntoUtil::EntoArray<size_t, K> *sample)
  {
    if constexpr (UsePROSAC)
    {
      if (prosac_state_.prosac_k_ < MaxPROSACIters)
      {
        draw_sample<RNG_t, K>(sample_size_ - 1, num_data_, sample, state_);
        (*sample)[sample_size_ - 1] = prng_next_int(state_) % num_data_;
      }
    }
    else
    {
      draw_sample<RNG_t, K>(sample_size_, num_data_, sample, state_);
    }
  }

  void generate_sample(std::vector<size_t> *sample)
  {
    if (UsePROSAC && prosac_state_.prosac_k_ < MaxPROSACIters) {
        draw_sample<RNG_t, K>(sample_size_ - 1, num_data_, sample, state_);
        (*sample)[sample_size_ - 1] = prosac_state_.prosac_subset_sz_ - 1;

        // update prosac state
        prosac_state_.prosac_k_++;
        if (prosac_state_.prosac_k_ < MaxPROSACIters) {
            if (prosac_state_.prosac_k_ > prosac_state_.growth[prosac_state_.prosac_subset_sz_ - 1]) {
                if (++prosac_state_.prosac_subset_sz_ > num_data_) {
                    prosac_state_.prosac_subset_sz_ = num_data_;
                }
            }
        }
    } else {
        // uniform ransac sampling
        draw_sample<RNG_t, K>(sample_size_, num_data_, sample, state_);
    }
  }

public:
  static constexpr size_t sample_size_ = K;
  size_t num_data_;
  RNG_t  state_;

  ProsacState<Scalar, K, MaxPROSACIters, UsePROSAC> prosac_state_;


};

namespace SampleValidation {

// ✅ Check for duplicate indices (already handled in draw_sample)
template <typename Container>
bool has_duplicate_indices(const Container& sample) {
    for (size_t i = 0; i < sample.size(); ++i) {
        for (size_t j = i + 1; j < sample.size(); ++j) {
            if (sample[i] == sample[j]) {
                return true;
            }
        }
    }
    return false;
}

// ✅ Check for collinear points (critical for 8pt algorithm)
template <typename Scalar>
bool are_collinear_3points(const Vec2<Scalar>& p1, const Vec2<Scalar>& p2, const Vec2<Scalar>& p3) {
    // Cross product approach: (p2-p1) × (p3-p1) ≈ 0
    Vec2<Scalar> v1 = p2 - p1;
    Vec2<Scalar> v2 = p3 - p1;
    Scalar cross = v1[0] * v2[1] - v1[1] * v2[0];
    
    // Use adaptive threshold based on point magnitudes
    Scalar magnitude = std::max({std::abs(v1[0]), std::abs(v1[1]), std::abs(v2[0]), std::abs(v2[1])});
    Scalar threshold = magnitude * 1e-6;  // Relative tolerance
    
    return std::abs(cross) < threshold;
}

// ✅ Check if too many points are collinear (generalized for any sample size)
template <typename Scalar, typename Container, typename SampleContainer>
bool has_too_many_collinear(const Container& points2D_a, const Container& points2D_b, 
                            const SampleContainer& sample) {
    // Adaptive max_collinear based on sample size
    size_t max_collinear = sample.size() / 2;  // K=8 -> 4, K=12 -> 6
    size_t collinear_count = 0;
    
    for (size_t i = 0; i < sample.size(); ++i) {
        for (size_t j = i + 1; j < sample.size(); ++j) {
            for (size_t k = j + 1; k < sample.size(); ++k) {
                if (are_collinear_3points<Scalar>(points2D_a[sample[i]], points2D_a[sample[j]], points2D_a[sample[k]]) ||
                    are_collinear_3points<Scalar>(points2D_b[sample[i]], points2D_b[sample[j]], points2D_b[sample[k]])) {
                    collinear_count++;
                    if (collinear_count > max_collinear) {
                        return true;  // Too many collinear points
                    }
                }
            }
        }
    }
    return false;
}

// ✅ Check for degenerate camera motion (baseline too small)
template <typename Scalar, typename Container, typename SampleContainer>
bool has_insufficient_baseline(const Container& points2D_a, const Container& points2D_b,
                              const SampleContainer& sample) {
    // Calculate average disparity between corresponding points
    Scalar total_disparity = 0;
    for (size_t i = 0; i < sample.size(); ++i) {
        auto diff = points2D_a[sample[i]] - points2D_b[sample[i]];
        total_disparity += std::sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
    }
    
    Scalar avg_disparity = total_disparity / sample.size();
    constexpr Scalar min_disparity = 2.0;  // Minimum 2 pixels average motion
    
    return avg_disparity < min_disparity;
}

// ✅ All-in-one validation for 8pt algorithm (and generalized for other sample sizes)
template <typename Scalar, typename Container, typename SampleContainer>
bool is_valid_8pt_sample(const Container& points2D_a, const Container& points2D_b,
                        const SampleContainer& sample) {
    // Check 1: No duplicate indices (already handled by draw_sample)
    // Check 2: Not too many collinear points  
    if (has_too_many_collinear<Scalar>(points2D_a, points2D_b, sample)) {
        return false;
    }
    
    // Check 3: Sufficient baseline/disparity
    if (has_insufficient_baseline<Scalar>(points2D_a, points2D_b, sample)) {
        return false;
    }
    
    return true;  // Sample passed all checks
}

} // namespace SampleValidation

} // namespace EntoPose

#endif // SAMPLING_H
