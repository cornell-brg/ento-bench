#ifndef ROBUST_ABSOLUTE_H
#define ROBUST_ABSOLUTE_H

#include <ento-pose/abs-pose/p3p.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>
#include <ento-pose/jacobian_accumulator.h>
#include <ento-pose/bundle_adjustment.h>
#include <ento-pose/robust-est/loss.h>

namespace EntoPose
{

template <typename Solver, size_t N = 0, typename RNG_t = uint32_t, bool UsePROSAC=false, int PROSACIters = 100000>
class AbsolutePoseRobustEstimator
{
public:
  using Scalar = typename Solver::scalar_type;
  static constexpr size_t MaxSolns = Solver::MaxSolns;
  static constexpr size_t sample_size_ = Solver::MinSampleSize;
  static constexpr size_t N_ = N;
  using SolverType = Solver;

  AbsolutePoseRobustEstimator(const RansacOptions<Scalar, UsePROSAC, PROSACIters> &opt,
                              const EntoContainer<Vec2<Scalar>, N> &points2D,
                              const EntoContainer<Vec3<Scalar>, N> &points3D)
    : opt_(opt), points2D_(points2D), points3D_(points3D), 
      num_data_(N == 0 ? points2D.size() : N),
      sampler(num_data_, opt.seed)
  {
    for (size_t k = 0; k < points2D.size(); ++k)
    {
      ENTO_DEBUG("points2D_[%zu]: %f, %f", k, points2D[k](0), points2D[k](1));
      ENTO_DEBUG("points3D_[%zu]: %f, %f, %f", k, points3D[k](0), points3D[k](1), points3D[k](2));
    }
  }

  void generate_models(EntoContainer<CameraPose<Scalar>, MaxSolns> *models)
  {
    sampler.generate_sample(&sample);
    for (size_t k = 0; k < sample_size_; ++k)
    {
      x_sample_[k] = points2D_[sample[k]].homogeneous().normalized();
      X_sample_[k] = points3D_[sample[k]];
      //ENTO_DEBUG("x_sample_[%zu]: %f, %f, %f", k, x_sample_[k](0), x_sample_[k](1), x_sample_[k](2));
      //ENTO_DEBUG("X_sample_[%zu]: %f, %f, %f", k, X_sample_[k](0), X_sample_[k](1), X_sample_[k](2));
    }
    
    // Use the solver's solve method - it expects separate x and X arguments
    Solver::template solve<sample_size_>(x_sample_, X_sample_, models);
    for ( const auto& model : *models )
    {
      //ENTO_DEBUG("Model: %f, %f, %f, %f, %f, %f, %f, %f", model.q(0), model.q(1), model.q(2), model.q(3), model.t(0), model.t(1), model.t(2));
    }
  }

  Scalar score_model(const CameraPose<Scalar> &pose, size_t *inlier_count) const
  {
    return compute_msac_score<Scalar, N>(pose,
                              points2D_, 
                              points3D_,
                              opt_.max_reproj_error * opt_.max_reproj_error,
                              inlier_count);
  }

  // Refine model using bundle adjustment with truncated loss (following PoseLib pattern)
  void refine_model(CameraPose<Scalar> *pose) const
  {
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = opt_.max_reproj_error;
    bundle_opt.max_iterations = 25;

    // Use all data with truncated loss (no inlier pre-filtering for absolute pose)
    bundle_adjust<Scalar, UniformWeightVector<Scalar>, TruncatedLoss<Scalar>, N>(points2D_, points3D_, pose, bundle_opt);
  }

  size_t num_data_;

public:
  RansacOptions<Scalar> opt_;
  const EntoUtil::EntoContainer<Vec2<Scalar>, N> &points2D_;
  const EntoUtil::EntoContainer<Vec3<Scalar>, N> &points3D_;

  RandomSampler<Scalar, sample_size_, RNG_t, UsePROSAC, PROSACIters> sampler;
  EntoContainer<Vec3<Scalar>, sample_size_> x_sample_, X_sample_;
  EntoContainer<size_t, sample_size_> sample;
};

// Middle-layer RANSAC function for absolute pose estimation
template <typename Solver, size_t N = 0>
RansacStats<typename Solver::scalar_type> ransac_pnp(const EntoUtil::EntoContainer<Vec2<typename Solver::scalar_type>, N> &x,
                                                     const EntoUtil::EntoContainer<Vec3<typename Solver::scalar_type>, N> &X,
                                                     const RansacOptions<typename Solver::scalar_type> &opt,
                                                     CameraPose<typename Solver::scalar_type> *best,
                                                     EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
  using Scalar = typename Solver::scalar_type;
  
  if (!opt.score_initial_model) {
    best->q << 1.0, 0.0, 0.0, 0.0;
    best->t.setZero();
  }
  
  ENTO_DEBUG("In ransac_pnp! x.size: %i, X.size: %i", x.size(), x.size());
  using RansacOptT = std::remove_reference_t<decltype(opt)>;
  using Estimator = AbsolutePoseRobustEstimator<Solver, N>;
  Estimator estimator(opt, x, X);
  RansacStats<Scalar> stats = ransac<Scalar, Estimator>(estimator, opt, best);
    
  // Get inliers using same threshold as MSAC scoring
  get_inliers<Scalar, N>(*best, x, X, opt.max_reproj_error * opt.max_reproj_error, inliers);
  return stats;
}

} // namespace EntoPose

#endif // ROBUST_ABSOLUTE_H

