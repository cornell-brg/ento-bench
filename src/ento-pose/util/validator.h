#ifndef ENTO_POSE_VALIDATOR_H
#define ENTO_POSE_VALIDATOR_H

//#include <ento-pose/problem-types/absolute_pose_problem.h>
//#include <ento-pose/problem-types/relative_pose_problem.h>
//#include <ento-pose/problem-types/homography_problem.h>

#include <ento-pose/pose_util.h>

namespace EntoPose
{

template <typename Scalar>
struct CalibratedRelativePoseValidator
{

  static Scalar compute_pose_error(const CameraPose<Scalar> &pose,
                                   const CameraPose<Scalar> &pose_gt)
  {
    return (pose_gt.R() - pose.R()).norm() + (pose_gt.t - pose.t).norm();
  }

  template <typename CameraModel>
  static Scalar compute_pose_error(const CameraPose<Scalar> &pose_gt,
                                   Scalar focal_gt,
                                   const ImagePair<Scalar, CameraModel> &image_pair)
  {
    return (pose_gt.R() - image_pair.pose.R()).norm() + (pose_gt.t - image_pair.pose.t).norm() +
           std::abs(focal_gt - image_pair.camera1.focal()) / focal_gt +
           std::abs(focal_gt - image_pair.camera2.focal()) / focal_gt;
  }
  
  static bool is_valid(const CameraPose<Scalar> &pose,
                       Scalar tol)
  {
    if ((pose.R().transpose() * pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
      return false;
  }

  template <size_t N = 0>
  static bool is_valid(const EntoContainer<Vec3<Scalar>, N>& x1,
                       const EntoContainer<Vec3<Scalar>, N>& x2,
                       const CameraPose<Scalar> &pose,
                       Scalar tol)
  {
    if ((pose.R().transpose() * pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
      return false;

    // Point to point correspondences
    // R * (alpha * p1 + lambda1 * x1) + t = alpha * p2 + lambda2 * x2
    //
    // cross(R*x1, x2)' * (alpha * p2 - t - alpha * R*p1) = 0
    for (size_t i = 0; i < x1.size(); ++i)
    {
      Scalar err = std::abs(x2[i]
                              .cross(pose.R() * x1[i])
                              .normalized()
                              .dot(pose.t));
      if (err > tol)
        return false;
    }
    return true;
  }

  template <typename CameraModel, size_t N = 0>
  static bool is_valid(const EntoContainer<Vec3<Scalar>, N>& x1,
                       const EntoContainer<Vec3<Scalar>, N>& x2,
                       const ImagePair<Scalar, CameraModel> &image_pair,
                       Scalar tol)
  {
    if ((image_pair.pose.R().transpose() * image_pair.pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
      return false;

    Matrix3x3<Scalar> K_1_inv, K_2_inv;
    K_1_inv << 1.0 / image_pair.camera1.focal(), 0.0, 0.0, 0.0, 1.0 / image_pair.camera1.focal(), 0.0, 0.0, 0.0, 1.0;
    K_2_inv << 1.0 / image_pair.camera2.focal(), 0.0, 0.0, 0.0, 1.0 / image_pair.camera2.focal(), 0.0, 0.0, 0.0, 1.0;

    // Point to point correspondences
    // cross(R*x1, x2)' * - t = 0
    // This currently works only for focal information from calib
    for (size_t i = 0; i < x1.size(); ++i)
    {
      Vec3<Scalar> x1_u = K_1_inv * x1[i];
      Vec3<Scalar> x2_u = K_2_inv * x2[i];
      Scalar err = std::abs((x2_u.cross(image_pair.pose.R() * x1_u).dot(-image_pair.pose.t)));
      if (err > tol)
        return false;
    }

    // return is_valid(instance, image_pair.pose, tol) && (std::fabs(image_pair.camera.focal() - instance.focal_gt) <
    // tol);
    return true;
  }
};
  //{
  //  if ((pose.R().transpose() * pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
  //    return false;

  //  // Point to point correspondences
  //  // R * (alpha * p1 + lambda1 * x1) + t = alpha * p2 + lambda2 * x2
  //  //
  //  // cross(R*x1, x2)' * (alpha * p2 - t - alpha * R*p1) = 0
  //  for (size_t i = 0; i < instance.x1_.size(); ++i)
  //  {
  //    //Scalar err = std::abs(instance.x2_[i]
  //    //                        .cross(pose.R() * instance.x1_[i])
  //    //                        .normalized()
  //    //                        .dot(pose.R() * instance.p1_[i] + pose.t - instance.p2_[i]));
  //    Scalar err = std::abs(instance.x2_[i]
  //                            .cross(pose.R() * instance.x1_[i])
  //                            .normalized()
  //                            .dot(pose.t));
  //    if (err > tol)
  //      return false;
  //  }
  //  return true;
  //}

  //{
  //  if ((image_pair.pose.R().transpose() * image_pair.pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
  //    return false;

  //  Matrix3x3<Scalar> K_1_inv, K_2_inv;
  //  K_1_inv << 1.0 / image_pair.camera1.focal(), 0.0, 0.0, 0.0, 1.0 / image_pair.camera1.focal(), 0.0, 0.0, 0.0, 1.0;
  //  K_2_inv << 1.0 / image_pair.camera2.focal(), 0.0, 0.0, 0.0, 1.0 / image_pair.camera2.focal(), 0.0, 0.0, 0.0, 1.0;

  //  // Point to point correspondences
  //  // cross(R*x1, x2)' * - t = 0
  //  // This currently works only for focal information from calib
  //  for (size_t i = 0; i < instance.x1_.size(); ++i)
  //  {
  //    Vec3<Scalar> x1_u = K_1_inv * instance.x1_[i];
  //    Vec3<Scalar> x2_u = K_2_inv * instance.x2_[i];
  //    Scalar err = std::abs((x2_u.cross(image_pair.pose.R() * x1_u).dot(-image_pair.pose.t)));
  //    if (err > tol)
  //      return false;
  //  }

  //  // return is_valid(instance, image_pair.pose, tol) && (std::fabs(image_pair.camera.focal() - instance.focal_gt) <
  //  // tol);
  //  return true;
  //}


template <typename Scalar>
struct CalibratedAbsolutePoseValidator
{
  // Compute Pose error for absolute pose. 
  static Scalar compute_pose_error(const CameraPose<Scalar> &pose,
                                   const CameraPose<Scalar> &pose_gt,
                                   Scalar scale, Scalar scale_gt)
  {
    return (pose_gt.R() - pose.R()).norm() + (pose_gt.t - pose.t).norm() +
           std::abs(scale_gt - scale);
  }

  template <size_t N = 0>
  static bool is_valid(const EntoContainer<Vec3<Scalar>, N>& x,
                       const EntoContainer<Vec3<Scalar>, N>& X,
                       const CameraPose<Scalar>& pose,
                       Scalar tol)
  {
    // Point to point correspondences
    // alpha * p + lambda*x = R*X + t
    for (int i = 0; i < x.size(); ++i)
    {
      Scalar err = 1.0 - std::abs(x[i].dot(
                               (pose.R() * X[i] + pose.t).normalized()));
      if (err > tol)
      {
        return false;
      }
    }
    return true;
  }
};

template <typename Scalar>
struct HomographyValidator 
{
  // Computes the distance to the ground truth pose
  static Scalar compute_pose_error(const Matrix3x3<Scalar> &H,
                                   const Matrix3x3<Scalar> &H_gt)
  {
    Scalar err1 = (H.normalized() - H_gt.normalized()).norm();
    Scalar err2 = (H.normalized() + H_gt.normalized()).norm();
    return std::min(err1, err2);
  }

  // Checks if the solution is valid (i.e. is rotation matrix and satisfies projection constraints)
  template <size_t N = 0>
  static bool is_valid(const Matrix3x3<Scalar>& H,
                       const EntoContainer<Vec3<Scalar>, N>& x1,
                       const EntoContainer<Vec3<Scalar>, N>& x2,
                       Scalar tol)
  {
    for (size_t i = 0; i < x1.size(); ++i)
    {
      Vec3<Scalar> z = H * x1[i];
      Scalar err = 1.0 - std::abs(z.normalized().dot(x2[i].normalized()));
      if (err > tol)
        return false;
    }
    return true;
  }
};

} // namespace EntoPose


#endif // ENTO_POSE_VALIDATOR_H
