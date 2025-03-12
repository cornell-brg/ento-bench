
#include <ento-util/containers.h>
#include <ento-math/core.h>

#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/bundle_adjustment.h>
#include <ento-pose/robust-est/ransac.h>
#include <ento-pose/prob_gen.h>

using namespace Eigen;
using namespace EntoMath;
using namespace EntoUtil;
using namespace EntoPose;

void test_robust_abs_pose()
{
  using Scalar = float;
  using Problem = AbsolutePoseProblemInstance<Scalar>;
  typedef CalibPoseValidator<Scalar> validator;
  constexpr Scalar tol = 1e-4;

  // Setup dataset paths.
  const char* base_path = DATASET_PATH;
  const char* rel_path = "robust-est";

  // Camera Model
  using CameraModel = IdentityCameraModel<Scalar>;
  using Params = std::array<Scalar, 0>;
  CameraModel camera_model;

  // Inputs for abs pose robust estimator:

  // 1. points2D, read from file.

  // 2. points3D, read from file.

  // 3. camera
  Params params;
  Camera<Scalar, IdentityCameraModel<Scalar>> cam(10, 10, params);

  // 4. ransac_opt
  RansacOptions<Scalar> ransac_opt;
  ransac_opt.max_iters = 100000;
  ransac_opt.min_ters = 1000;
  ransac_opt.max_reproj_error /= cam.focal();

  // 5. bundle_opt, initialized with defaults
  BundleOptions<Scalar> bundle_opt;
  // 6. CameraPose*
  // 7. vector<char> inliers


  
  
  std::vector<Vec2<Scalar>> points2D_calib(points2D.size());
  for (size_t k = 0; k < points2D.size(); ++k) {
      camera.unproject(points2D[k], &points2D_calib[k]);
  }



  RansacStats stats = ransac_pnp(points2D_calib, points3D, ransac_opt, pose, inliers);

  if (stats.num_inliers > 3) {
      // Collect inlier for additional bundle adjustment
      std::vector<Vec2<Scalar>> points2D_inliers;
      std::vector<Point3D> points3D_inliers;
      points2D_inliers.reserve(points2D.size());
      points3D_inliers.reserve(points3D.size());

      // We re-scale with focal length to improve numerics in the opt.
      const double scale = 1.0 / camera.focal();
      Camera norm_camera = camera;
      norm_camera.rescale(scale);
      BundleOptions bundle_opt_scaled = bundle_opt;
      bundle_opt_scaled.loss_scale *= scale;
      for (size_t k = 0; k < points2D.size(); ++k) {
          if (!(*inliers)[k])
              continue;
          points2D_inliers.push_back(points2D[k] * scale);
          points3D_inliers.push_back(points3D[k]);
      }

      bundle_adjust(points2D_inliers, points3D_inliers, norm_camera, pose, bundle_opt_scaled);
  }

  return stats;

}

int main ( int argc, char ** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_pose_est_reader_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_robust_abs_pose();
}
