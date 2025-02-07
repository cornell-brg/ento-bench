#ifndef ROBUST_HOMOGRAPHY_H
#define ROBUST_HOMOGRAPHY_H

#include <ento-pose/pose_util.h>
#include <ento-pose/robust-est/sampling.h>
#include <ento-pose/robust-est/ransac_util.h>

namespace EntoPose
{

template <typename Scalar, size_t K, size_t N = 0, typename RNG_t = uint32_t, bool UsePROSAC=false, int PROSACIters = 100000>
class HomographyRobustEstimator
{
public:
  HomographyRobustEstimator(const RansacOptions<Scalar, UsePROSAC, PROSACIters> &opt,
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
    Matrix3x3<Scalar> H;
    int sols = homography_4pt<Scalar, N>(x1_sample_, x2_sample_);
    if (sols > 0) models->push_back(H);
  }

  Scalar score_model(const CameraPose<Scalar> &pose, size_t *inlier_count) const
  {
    return compute_sampson_msac_score(pose,
                                      x1,
                                      x2,
                                      opt_.max_epipolar_error * opt_.max_epipolar_error,
                                      inlier_count);
  }

  void refine_model(Matrix3x3<Scalar> *H) const
  {
    BundleOptions<Scalar> bundle_opt;
    bundle_opt.loss_type = BundleOptions<Scalar>::LossType::TRUNCATED;
    bundle_opt.loss_scale = opt_.max_reproj_error;
    bundle_opt.max_iterations = 25;

    refine_homography(x1, x2, H, bundle_opt);
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


} // namespace EntoPose

#endif // ROBUST_HOMOGRAPHY_H
