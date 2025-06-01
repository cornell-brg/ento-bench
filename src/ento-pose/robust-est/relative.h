#ifndef ROBUST_RELATIVE_H
#define ROBUST_RELATIVE_H

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>

namespace EntoPose
{

template <typename Solver, size_t N = 0, typename RNG_t = uint32_t, bool UsePROSAC=false, int PROSACIters = 100000>
class RelativePoseRobustEstimator
{
public:
  using Scalar = typename Solver::scalar_type;
  static constexpr size_t MaxSolns = Solver::MaxSolns;
  static constexpr size_t sample_size_ = Solver::MinSampleSize;

  RelativePoseRobustEstimator(const RansacOptions<Scalar, UsePROSAC, PROSACIters> &opt,
                              const EntoContainer<Vec2<Scalar>> &points2d_1,
                              const EntoContainer<Vec2<Scalar>> &points2d_2)
    : opt_(opt), x1(points2d_1), x2(points2d_2)
  {
    if constexpr (N == 0) num_data_ = points2d_1.size();
    else num_data_ = N;

    sampler(num_data_, opt.seed);
  }

  void generate_models(EntoContainer<CameraPose<Scalar>, MaxSolns> *models)
  {
    sampler.generate_sample(&sample);
    for (size_t k = 0; k < sample_size_; ++k)
    {
      x1_sample_[k] = x1[sample[k]].homogeneous().normalized();
      x2_sample_[k] = x2[sample[k]].homogeneous().normalized();
    }
    
    // Use the solver's solve method
    if constexpr (N == 0) {
      std::vector<CameraPose<Scalar>> solutions;
      int num_sols = Solver::solve(x1_sample_, x2_sample_, &solutions);
      models->clear();
      for (int i = 0; i < num_sols && i < MaxSolns; ++i) {
        models->push_back(solutions[i]);
      }
    } else {
      Solver::solve(x1_sample_, x2_sample_, models);
    }
  }

  Scalar score_model(const CameraPose<Scalar> &pose, size_t *inlier_count) const
  {
    return compute_sampson_msac_score(pose,
                                      x1,
                                      x2,
                                      opt_.max_epipolar_error * opt_.max_epipolar_error,
                                      inlier_count);
  }

  //void refine_model(CameraPose<Scalar> *pose) const
  //{
  //  BundleOptions<Scalar> bundle_opt;
  //  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  //  bundle_opt.loss_scale = opt_.max_epipolar_error;
  //  bundle_opt.max_iterations = 25;

  //  // @TODO: Need to figure out vector support for STM32
  //  std::vector<uint8_t> inliers;
  //  int num_inl = get_inliers(*pose, x1, x2, 5*(opt_.max_epipolar_error * opt_.max_epipolar_error), &inliers);
  //  std::vector<Vec2<Scalar>> x1_inlier, x2_inlier;
  //  x1_inlier.reserve(num_inl);
  //  x2_inlier.reserve(num_inl);

  //  if (num_inl <= 5) return;

  //  for (size_t pt_k = 0; pt_k < x1.size(); ++pt_k)
  //  {
  //    if (inliers[pt_k])
  //    {
  //      x1_inlier.push_back(x1[pt_k]);
  //      x2_inlier.push_back(x2[pt_k]);
  //    }
  //  }

  //  refine_relpose(x1_inlier, x2_inlier, pose, bundle_opt);
  //}

  size_t num_data_;

private:
  RansacOptions<Scalar> opt_;
  const EntoContainer<Vec2<Scalar>, N> &x1;
  const EntoContainer<Vec2<Scalar>, N> &x2;

  RandomSampler<Scalar, sample_size_, RNG_t, UsePROSAC, PROSACIters> sampler;
  EntoContainer<Vec3<Scalar>, sample_size_> x1_sample_, x2_sample_;
  EntoContainer<size_t, sample_size_> sample;
};

// Middle-layer RANSAC function for relative pose estimation
template <typename Solver, size_t N = 0>
RansacStats<typename Solver::scalar_type> ransac_relpose(const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &x1,
                                                         const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &x2,
                                                         const RansacOptions<typename Solver::scalar_type> &opt,
                                                         CameraPose<typename Solver::scalar_type> *best,
                                                         EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
  using Scalar = typename Solver::scalar_type;
  
  best->q << 1.0, 0.0, 0.0, 0.0;
  best->t.setZero();
  
  RansacStats<Scalar> stats;
  
  if constexpr (opt.progressive_sampling)
  {
    constexpr bool use_prosac = opt.progressive_sampling;
    constexpr int prosac_iters = opt.max_prosac_iterations;
    using RNG_t = uint32_t;
    using Estimator = RelativePoseRobustEstimator<Solver, N, RNG_t, use_prosac, prosac_iters>;

    Estimator estimator(opt, x1, x2);
    stats = ransac<Scalar, Estimator>(estimator, opt, best);
  }
  else
  {
    using Estimator = RelativePoseRobustEstimator<Solver, N>;
    Estimator estimator(opt, x1, x2);
    stats = ransac<Scalar, Estimator>(estimator, opt, best);
  }
  
  get_inliers(*best, x1, x2, opt.max_epipolar_error * opt.max_epipolar_error, inliers);
  return stats;
}

} // namespace EntoPose

#endif // ROBUST_RELATIVE_H
