#ifndef ROBUST_RELATIVE_H
#define ROBUST_RELATIVE_H

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/robust-est/loss.h>
#include <ento-pose/bundle_adjustment.h>

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
                              const EntoContainer<Vec2<Scalar>, N> &points2d_1,
                              const EntoContainer<Vec2<Scalar>, N> &points2d_2)
    : num_data_(N == 0 ? points2d_1.size() : N),
      opt_(opt), x1(points2d_1), x2(points2d_2),
      sampler(num_data_, opt.seed)
  {
    for (size_t k = 0; k < x1.size(); ++k)
    {
      ENTO_DEBUG("x1_[%zu]: %f, %f", k, x1[k](0), x1[k](1));
      ENTO_DEBUG("x2_[%zu]: %f, %f, %f", k, x2[k](0), x2[k](1), x2[k](2));
    }
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
    Solver::template solve<sample_size_>(x1_sample_, x2_sample_, models);
    for ( const auto& model : *models )
    {
      //ENTO_DEBUG("Model: %f, %f, %f, %f, %f, %f, %f, %f", model.q(0), model.q(1), model.q(2), model.q(3), model.t(0), model.t(1), model.t(2));
    }
  }

  Scalar score_model(const CameraPose<Scalar> &pose, size_t *inlier_count) const
  {
    return compute_sampson_msac_score<Scalar, N>
      (pose, x1, x2, opt_.max_epipolar_error * opt_.max_epipolar_error, inlier_count);
  }

  void refine_model(CameraPose<Scalar> *pose) const
  {
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = opt_.max_epipolar_error;
    bundle_opt.max_iterations = 25;

    // @TODO: Need to figure out vector support for STM32
    EntoContainer<uint8_t, N> inliers;
    EntoContainer<Vec2<Scalar>, N> x1_inlier, x2_inlier;
    int num_inl = get_inliers<Scalar, N>(*pose, x1, x2, 5*(opt_.max_epipolar_error * opt_.max_epipolar_error), &inliers);
    if constexpr (N == 0)
    {
      x1_inlier.reserve(num_inl);
      x2_inlier.reserve(num_inl);
    }

    if (num_inl <= sample_size_) return;

    for (size_t pt_k = 0; pt_k < x1.size(); ++pt_k)
    {
      if (inliers[pt_k])
      {
        x1_inlier.push_back(x1[pt_k]);
        x2_inlier.push_back(x2[pt_k]);
      }
    }

    using WeightT = UniformWeightVector<Scalar>;
    using LossFn = TruncatedLoss<Scalar>;
    refine_relpose<Scalar, WeightT, LossFn, N>
      (x1_inlier, x2_inlier, pose, bundle_opt);
  }

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
  
  if (!opt.score_initial_model)
  {
    best->q << 1.0, 0.0, 0.0, 0.0;
    best->t.setZero();
  }
  
  ENTO_DEBUG("In ransac_relpose! x1.size: %i, x2.size: %i", x1.size(), x2.size());
  using Estimator = RelativePoseRobustEstimator<Solver, N>;
  Estimator estimator(opt, x1, x2);
  RansacStats<Scalar> stats = ransac<Scalar, Estimator>(estimator, opt, best);
  
  get_inliers<Scalar, N>(*best, x1, x2, opt.max_epipolar_error * opt.max_epipolar_error, inliers);
  return stats;
}

} // namespace EntoPose

#endif // ROBUST_RELATIVE_H
