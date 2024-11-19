#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-math/quaternion.h>
#include <ento-pose/pose_util.h>

namespace EntoPose
{

// Minimizes reprojection error. Assumes identity intrinsics (calibrated camera)
template <typename Scalar>
BundleStats<Scalar> bundle_adjust(const std::vector<Point2D<Scalar>>    &x,
                                  const std::vector<Point3D<Scalar>>    &X,
                                  CameraPose<Scalar>         *pose,
                                  const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                  const std::vector<Scalar> &weights = std::vector<Scalar>());

// Uses intrinsic calibration from Camera (see colmap_models.h)
// Slightly slower than bundle_adjust above
template <typename Scalar>
BundleStats<Scalar> bundle_adjust(const std::vector<Point2D<Scalar>> &x,
                                  const std::vector<Point3D<Scalar>> &X,
                                  const Camera<Scalar> &camera,
                                  CameraPose<Scalar> *pose,
                                  const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                  const std::vector<Scalar> &weights = std::vector<Scalar>());

// opt_line is used to define the robust loss used for the line correspondences
template <typename Scalar>
BundleStats<Scalar> bundle_adjust(const std::vector<Point2D<Scalar>> &points2D,
                                  const std::vector<Point3D<Scalar>> &points3D,
                                  const std::vector<Line2D<Scalar>>  &lines2D,
                                  const std::vector<Line3D<Scalar>>  &lines3D,
                                  CameraPose<Scalar>                    *pose,
                                  const BundleOptions<Scalar>            &opt = BundleOptions<Scalar>(),
                                  const BundleOptions<Scalar>       &opt_line = BundleOptions<Scalar>(),
                                  const std::vector<Scalar>      &weights_pts = std::vector<Scalar>(),
                                  const std::vector<Scalar>      &weights_line = std::vector<Scalar>());

/*
// Camera models for lines are currently not supported...
int bundle_adjust(const std::vector<Point2D> &points2D,
                  const std::vector<Point3D> &points3D,
                  const std::vector<Line2D> &lines2D,
                  const std::vector<Line3D> &lines3D,
                  const Camera &camera,
                  CameraPose *pose,
                  const BundleOptions &opt = BundleOptions(),
                  const std::vector<Scalar> &weights = std::vector<Scalar>());
*/

// Minimizes reprojection error. Assumes identity intrinsics (calibrated camera)
template <typename Scalar>
BundleStats<Scalar>
generalized_bundle_adjust(const std::vector<std::vector<Point2D<Scalar>>> &x,
                          const std::vector<std::vector<Point3D<Scalar>>> &X,
                          const std::vector<CameraPose<Scalar>>  &camera_ext,
                          CameraPose<Scalar>                           *pose,
                          const BundleOptions<Scalar>                   &opt = BundleOptions<Scalar>(),
                          const std::vector<std::vector<Scalar>>    &weights = std::vector<std::vector<Scalar>>());

// Uses intrinsic calibration from Camera (see colmap_models.h)
// Slightly slower than bundle_adjust above
template<typename Scalar>
BundleStats<Scalar>
generalized_bundle_adjust(const std::vector<std::vector<Point2D<Scalar>>> &x,
                          const std::vector<std::vector<Point3D<Scalar>>> &X,
                          const std::vector<CameraPose<Scalar>> &camera_ext,
                          const std::vector<Camera<Scalar>> &cameras,
                          CameraPose<Scalar> *pose,
                          const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                          const std::vector<std::vector<Scalar>> &weights = std::vector<std::vector<Scalar>>());

// Relative pose refinement. Minimizes Sampson error error. Assumes identity intrinsics (calibrated camera)
template<typename Scalar>
BundleStats<Scalar>refine_relpose(const std::vector<Point2D<Scalar>> &x1,
                                  const std::vector<Point2D<Scalar>> &x2,
                                  CameraPose<Scalar> *pose,
                                  const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                  const std::vector<Scalar> &weights = std::vector<Scalar>());

// Fundamental matrix refinement. Minimizes Sampson error error.
template <typename Scalar>
BundleStats<Scalar> refine_fundamental(const std::vector<Point2D<Scalar>> &x1, const std::vector<Point2D<Scalar>> &x2, Eigen::Matrix3d *F,
                                       const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                       const std::vector<Scalar> &weights = std::vector<Scalar>());

// Homography matrix refinement.
template <typename Scalar>
BundleStats<Scalar> refine_homography(const std::vector<Point2D<Scalar>> &x1, const std::vector<Point2D<Scalar>> &x2, Eigen::Matrix3d *H,
                                      const BundleOptions<Scalar> &opt = BundleOptions<Scalar>(),
                                      const std::vector<Scalar> &weights = std::vector<Scalar>());



}

#endif // BUNDLE_ADJUSTMENT_H
