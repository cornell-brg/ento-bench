#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include <cstdio>
#include <type_traits>

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-math/quaternion.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/levenberg_marquadt.h>
#include <ento-pose/jacobian_accumulator.h>
#include <ento-pose/robust-est/loss.h>

namespace EntoPose
{

using namespace EntoUtil;

template <typename Scalar>
void print_iteration(const BundleStats<Scalar> &stats)
{
  if (stats.iterations == 0)
  {
    std::printf("initial_cost=%f\n", static_cast<double>(stats.initial_cost));
  }
  std::printf("iter=%d, cost=%f, step=%f, grad=%f, lambda=%f\n",
              stats.iterations,
              static_cast<Scalar>(stats.cost),
              static_cast<Scalar>(stats.step_norm),
              static_cast<Scalar>(stats.grad_norm),
              static_cast<Scalar>(stats.lambda));
}

template <typename Scalar, typename LossFunction>
IterationCallback<Scalar> setup_callback(const BundleOptions<Scalar> &opt, LossFunction &loss_fn)
{
  if constexpr(std::is_same_v<LossFunction, TruncatedLossLeZach<Scalar>>)
  {
    if (opt.verbose)
    {
      return [&loss_fn](const BundleStats<Scalar> &stats)
      {
        print_iteration(stats);
        loss_fn.mu *= TruncatedLossLeZach<Scalar>::alpha;
      };
    }
    else
    {
      return [&loss_fn](const BundleStats<Scalar> &stats)
      {
        loss_fn.mu *= TruncatedLossLeZach<Scalar>::alpha;
      };
    }
  }
  else
  {
    if (opt.verbose)
    {
      return print_iteration;
    }
    else
    {
      return nullptr;
    }
  }
}

// Minimizes reprojection error. Assumes identity intrinsics (calibrated camera)
template <typename Scalar, typename WeightType, typename LossFunction, size_t N = 0>
BundleStats<Scalar> bundle_adjust(const EntoContainer<Point2D<Scalar>, N>    &x,
                                  const EntoContainer<Point3D<Scalar>, N>    &X,
                                  CameraPose<Scalar>                   *pose,
                                  const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                  const EntoContainer<Scalar, N> &weights = EntoContainer<Scalar, N>())
{
  Camera<Scalar> camera;
  bundle_adjust(x, X, camera, pose, opt);
}


// Uses intrinsic calibration from Camera (see colmap_models.h)
// Slightly slower than bundle_adjust above
template <typename Scalar, typename WeightType, typename CameraModel, typename LossFunction, size_t N = 0>
BundleStats<Scalar> bundle_adjust(const std::vector<Point2D<Scalar>> &x,
                                  const std::vector<Point3D<Scalar>> &X,
                                  const Camera<Scalar, CameraModel> &camera,
                                  CameraPose<Scalar> *pose,
                                  const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                  const std::vector<Scalar> &weights = std::vector<Scalar>())
{
  LossFunction loss_fn(opt.loss_scale);
  IterationCallback<Scalar> callback = setup_callback(opt, loss_fn);
  CameraJacobianAccumulator<Scalar, CameraModel, LossFunction, WeightType, N> accum(x, X, camera, loss_fn, weights);
  return lm_impl<decltype(accum)>(accum, pose, opt, callback);
}



// Relative pose refinement. Minimizes Sampson error error. Assumes identity intrinsics (calibrated camera)
template<typename Scalar, typename WeightType, typename LossFunction, size_t N = 0>
BundleStats<Scalar>
refine_relpose(const EntoContainer<Point2D<Scalar>, N> &x1,
               const EntoContainer<Point2D<Scalar>, N> &x2,
               CameraPose<Scalar> *pose,
               const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
               const EntoContainer<Scalar, N> &weights = std::vector<Scalar>())
{
  LossFunction loss_fn(opt.loss_scale);
  IterationCallback<Scalar> callback = setup_callback(opt, loss_fn);
  RelativePoseJacobianAccumulator<Scalar, LossFunction, WeightType, N> accum(x1, x2, loss_fn, weights);
  return lm_impl<decltype(accum)>(accum, pose, opt, callback);
}

// Homography matrix refinement.
template <typename Scalar, typename WeightType, typename LossFunction, size_t N = 0>
BundleStats<Scalar>
refine_homography(const EntoContainer<Point2D<Scalar>, N> &x1,
                  const std::vector<Point2D<Scalar>> &x2,
                  Eigen::Matrix3d *H,
                  const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                  const std::vector<Scalar> &weights = std::vector<Scalar>())
{
  LossFunction loss_fn(opt.loss_scale);
  IterationCallback<Scalar> callback = setup_callback(opt, loss_fn);
  HomographyJacobianAccumulator<Scalar, LossFunction, WeightType, N> accum(x1, x2, loss_fn, weights);
  return lm_impl<decltype(accum)>(accum, H, opt, callback);
}

} // namespace EntoPose

#endif // BUNDLE_ADJUSTMENT_H
