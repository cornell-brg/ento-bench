#ifndef ROBUST_HOMOGRAPHY_H
#define ROBUST_HOMOGRAPHY_H

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>

namespace EntoPose
{

template <typename Solver, size_t N = 0, typename RNG_t = uint32_t, bool UsePROSAC=false, int PROSACIters = 100000>
class HomographyRobustEstimator
{
public:
  using Scalar = typename Solver::scalar_type;
  static constexpr size_t MaxSolns = Solver::MaxSolns;
  static constexpr size_t sample_size_ = Solver::MinSampleSize;

  HomographyRobustEstimator(const RansacOptions<Scalar, UsePROSAC, PROSACIters> &opt,
                            const EntoContainer<Vec2<Scalar>> &points2d_1,
                            const EntoContainer<Vec2<Scalar>> &points2d_2)
    : opt_(opt), x1(points2d_1), x2(points2d_2)
  {
    if constexpr (N == 0) num_data_ = points2d_1.size();
    else num_data_ = N;

    sampler(num_data_, opt.seed);
  }

  void generate_models(EntoContainer<Matrix3x3<Scalar>, MaxSolns> *models)
  {
    sampler.generate_sample(&sample);
    for (size_t k = 0; k < sample_size_; ++k)
    {
      x1_sample_[k] = x1[sample[k]].homogeneous().normalized();
      x2_sample_[k] = x2[sample[k]].homogeneous().normalized();
    }
    
    // Use the solver's solve method
    if constexpr (N == 0) {
      std::vector<Matrix3x3<Scalar>> solutions;
      int num_sols = Solver::solve(x1_sample_, x2_sample_, &solutions);
      models->clear();
      for (int i = 0; i < num_sols && i < MaxSolns; ++i) {
        models->push_back(solutions[i]);
      }
    } else {
      Solver::solve(x1_sample_, x2_sample_, models);
    }
  }

  Scalar score_model(const Matrix3x3<Scalar> &H, size_t *inlier_count) const
  {
    return compute_homography_msac_score(H,
                                         x1,
                                         x2,
                                         opt_.max_reproj_error * opt_.max_reproj_error,
                                         inlier_count);
  }

  //void refine_model(Matrix3x3<Scalar> *H) const
  //{
  //  BundleOptions<Scalar> bundle_opt;
  //  bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
  //  bundle_opt.loss_scale = opt_.max_reproj_error;
  //  bundle_opt.max_iterations = 25;

  //  refine_homography(x1, x2, H, bundle_opt);
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

// Middle-layer RANSAC function for homography estimation
template <typename Solver, size_t N = 0>
RansacStats<typename Solver::scalar_type> ransac_homography(const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &x1,
                                                           const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &x2,
                                                           const RansacOptions<typename Solver::scalar_type> &opt,
                                                           Matrix3x3<typename Solver::scalar_type> *best,
                                                           EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
  using Scalar = typename Solver::scalar_type;
  
  best->setIdentity();
  
  RansacStats<Scalar> stats;
  
  if constexpr (opt.progressive_sampling)
  {
    constexpr bool use_prosac = opt.progressive_sampling;
    constexpr int prosac_iters = opt.max_prosac_iterations;
    using RNG_t = uint32_t;
    using Estimator = HomographyRobustEstimator<Solver, N, RNG_t, use_prosac, prosac_iters>;
    Estimator estimator(opt, x1, x2);
    stats = ransac<Scalar, Estimator, Matrix3x3<Scalar>>(estimator, opt, best);
  }
  else
  {
    using Estimator = HomographyRobustEstimator<Solver, N>;
    Estimator estimator(opt, x1, x2);
    stats = ransac<Scalar, Estimator, Matrix3x3<Scalar>>(estimator, opt, best);
  }
  
  get_homography_inliers(*best, x1, x2, opt.max_reproj_error * opt.max_reproj_error, inliers);
  return stats;
}

} // namespace EntoPose

#endif // ROBUST_HOMOGRAPHY_H
