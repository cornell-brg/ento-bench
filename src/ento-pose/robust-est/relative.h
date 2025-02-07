#ifndef ROBUST_RELATIVE_H
#define ROBUST_RELATIVE_H

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>

namespace EntoPose
{

template <typename Scalar, size_t K, size_t N = 0, typename RNG_t = uint32_t, bool UsePROSAC=false, int PROSACIters = 100000>
class RelativePoseRobustEstimator
{
public:
  RelativePoseRobustEstimator(const RansacOptions<Scalar, UsePROSAC, PROSACIters> &opt,
                              const EntoContainer<Vec2<Scalar>> &points2d_1,
                              const EntoContainer<Vec2<Scalar>> &points2d_2)
    : opt_(opt), x1(points2d_1), x2(points2d_2)
  {
    if constexpr (N == 0) num_data_ = points2d_1.size();
    else num_data_ = N;

    sampler(num_data_, opt.seed);
  }

  void generate_models(EntoContainer<CameraPose<Scalar>, N> *models)
  {
    sampler.generate_sample(&sample);
    for (size_t k = 0; k < sample_size_; ++k)
    {
      x1_sample_[k] = x1[sample[k]].homogeneous().normalized();
      x2_sample_[k] = x2[sample[k]].homogeneous().normalized();
    }
    // @TODO: If constexpr func? 8pt or 5pt...
    if constexpr ( sample_size_ == 5 )
    {
      relpose_5pt<Scalar, N>(x1_sample_, x2_sample_, models);
    }
    else if ( sample_size_ == 8 )
    {
      relpose_8pt<Scalar, N>(x1_sample_, x2_sample_, models);
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

  void refine_model(CameraPose<Scalar> *pose) const
  {
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = opt_.max_epipolar_error;
    bundle_opt.max_iterations = 25;

    // @TODO: Need to figure out vector support for STM32
    std::vector<uint8_t> inliers;
    int num_inl = get_inliers(*pose, x1, x2, 5*(opt_.max_epipolar_error * opt_.max_epipolar_error), &inliers);
    std::vector<Vec2<Scalar>> x1_inlier, x2_inlier;
    x1_inlier.reserve(num_inl);
    x2_inlier.reserve(num_inl);

    if (num_inl <= 5) return;

    for (size_t pt_k = 0; pt_k < x1.size(); ++pt_k)
    {
      if (inliers[pt_k])
      {
        x1_inlier.push_back(x1[pt_k]);
        x2_inlier.push_back(x2[pt_k]);
      }
    }

    refine_relpose(x1_inlier, x2_inlier, pose, bundle_opt);
  }

  static constexpr size_t sample_size_ = K;
  size_t num_data_;

private:
  RansacOptions<Scalar> opt_;
  const EntoContainer<Vec2<Scalar>, N> &x1;
  const EntoContainer<Vec2<Scalar>, N> &x2;

  RandomSampler<Scalar, K, RNG_t, UsePROSAC, PROSACIters> sampler;
  EntoContainer<Vec3<Scalar>, K> x1_sample_, x2_sample_;
  EntoContainer<size_t, K> sample;
};

// @TODO: Rename this to lo_ransac_relpose. It will call RelativePoseRobustEstimator with
//   the correct templates to implement LO-RANSAC with local optimization
template <typename Scalar, size_t N = 0, size_t K = 5>
RansacStats<Scalar> ransac_relpose(const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x1,
                                   const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x2,
                                   const RansacOptions<Scalar> &opt,
                                   CameraPose<Scalar> *best,
                                   EntoUtil::EntoContainer<uint8_t, 0> *inliers)
{
  best->q << 1.0, 0.0, 0.0, 0.0;
  best->t.setZero();

  RelativePoseRobustEstimator<Scalar, K, N> estimator(opt, x1, x2);
  RansacStats<Scalar> stats = ransac(estimator, opt, best);
  get_inliers(*best, x1, x2, opt.max_epipolar_error * opt.max_epipolar_error, inliers);
}

} // namespace EntoPose

#endif // ROBUST_RELATIVE_H
