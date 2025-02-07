#ifndef ROBUST_ABSOLUTE_H
#define ROBUST_ABSOLUTE_H

#include <ento-pose/abs-pose/p3p.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>

namespace EntoPose
{

template <typename Scalar, size_t K, size_t N = 0, typename RNG_t = uint32_t, bool UsePROSAC=false, int PROSACIters = 100000>
class AbsolutePoseRobustEstimator
{
public:
  AbsolutePoseRobustEstimator(const RansacOptions<Scalar, UsePROSAC, PROSACIters> &opt,
                              const EntoContainer<Vec2<Scalar>, N> &points2D,
                              const EntoContainer<Vec3<Scalar>, N> &points3D)
    : opt_(opt), points2D_(points2D), points3D_(points3D)
  {
    if constexpr (N == 0) num_data_ = points2D.size();
    else num_data_ = N;

    sampler(num_data_, opt.seed);
  }

  void generate_models(EntoContainer<CameraPose<Scalar>, N> *models)
  {
    sampler.generate_sample(&sample);
    for (size_t k = 0; k < sample_size_; ++k)
    {
      x_sample_[k] = points2D_[sample[k]].homogeneous().normalized();
      X_sample_[k] = points3D_[sample[k]].homogeneous().normalized();
    }
    p3p(x_sample_, X_sample_, models);
  }

  Scalar score_model(const CameraPose<Scalar> &pose, size_t *inlier_count) const
  {
    return compute_msac_score(pose,
                              points2D_, 
                              points3D_,
                              opt_.max_reproj_error * opt_.max_reproj_error,
                              inlier_count);
  }

  void refine_model(CameraPose<Scalar> *pose)
  {
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = opt_.max_reproj_error;
    bundle_opt.max_iterations = 25;

    bundle_adjust(points2D_, points3D_, pose, bundle_opt);
  }

  size_t num_data_;
  static constexpr size_t sample_size_ = K;
private:
  RansacOptions<Scalar> opt_;
  const EntoUtil::EntoContainer<Vec2<Scalar>, N> &points2D_;
  const EntoUtil::EntoContainer<Vec3<Scalar>, N> &points3D_;

  RandomSampler<Scalar, K, RNG_t, UsePROSAC, PROSACIters> sampler;
  EntoContainer<Vec3<Scalar>, K> x_sample_, X_sample_;
  EntoContainer<size_t, K> sample;
};



} // namespace EntoPose

#endif // ROBUST_ABSOLUTE_H

