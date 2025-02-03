#ifndef PROB_GEN_H
#define PROB_GEN_H

#include "ento-util/debug.h"
#if defined(NATIVE)

#include <cassert>
#include <random>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>

#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>

using namespace EntoUtil;

namespace EntoPose
{

template <typename Scalar, std::size_t NumPts=0>
struct AbsolutePoseProblemInstance {
public:
  // Ground truth camera pose
  CameraPose<Scalar> pose_gt;
  Scalar scale_gt = 1.0;
  Scalar focal_gt = 1.0;

  std::size_t n_point_point_ = NumPts;

  // Point-to-point correspondences
  EntoContainer<Vec3<Scalar>, NumPts> x_point_;
  EntoContainer<Vec3<Scalar>, NumPts> X_point_;

  // For generalized cameras we have an offset in the camera coordinate system
  // EntoContainer<Vec3<Scalar>, NumPts> p_point_;
  void clear()
  {
    x_point_.clear();
    X_point_.clear();
    n_point_point_ = 0;
  }

  void set_num_pts(std::size_t num_points)
  {
#ifdef NATIVE
    n_point_point_ = num_points;
#endif
  }

#ifdef NATIVE
  static std::string header()
  {
    return "problem_type,num_points,pose_gt,scale_gt,focal_gt,x_point,X_point";
  }
#else
  static constexpr const char *header()
  {
    return "problem_type,pose_gt,scale_gt,focal_gt,x_point,X_point";

  }
#endif

  std::string serialize() const
  {
#ifdef NATIVE
    std::ostringstream oss;
    constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

    // Problem type ID for Absolute Pose
    oss << 1 << "," << n_point_point_ << ",";

    // Serialize quaternion (q) and translation (t)
    for (int i = 0; i < 4; ++i) {
        oss << std::fixed << std::setprecision(precision) << pose_gt.q[i] << ",";
    }
    for (int i = 0; i < 3; ++i) {
        oss << std::fixed << std::setprecision(precision) << pose_gt.t[i] << ",";
    }

    // Serialize scale and focal length
    oss << scale_gt << "," << focal_gt;

    // Serialize x_point and X_point correspondences
    for (const auto &x_point : x_point_) {
        oss << "," << x_point.x() << "," << x_point.y() << "," << x_point.z(); 
    }
    for (const auto &X_point : X_point_) {
        oss << "," << X_point.x() << "," << X_point.y() << "," << X_point.z();
    }

    // Remove the trailing comma, if present
    std::string serialized = oss.str();
    if (!serialized.empty() && serialized.back() == ',')
    {
      serialized.pop_back();
    }

    return serialized;
#else
    // Serialization is not supported on microcontrollers

    ENTO_DEBUG("WARNING: SERIALIZATION CALLED FROM A MCU. Not supported")
    return "";
#endif
  }

  static bool deserialize(const char* line, AbsolutePoseProblemInstance* instance)
  {
#ifdef NATIVE
      // Native build: Use std::istringstream for parsing
      std::istringstream iss(line);
      char comma;

      // Parse problem type
      int problem_type;
      if (!(iss >> problem_type >> comma) || problem_type != 1 || comma != ',') {
          return false; // Parsing failed
      }
      //ENTO_DEBUG("Problem type: %i", problem_type);

      int num_points;
      if (!(iss >> num_points >> comma) || problem_type != 1 || comma != ',') {
          return false; // Parsing failed
      }
      if (instance->n_point_point_ == 0)
      {
        instance->n_point_point_ = num_points;
      }
      else
      {
        if (instance->n_point_point_ != num_points)
        {
          ENTO_DEBUG("Capacity does not match num points. Error!");
          return false;
        }
      }
      //ENTO_DEBUG("Num points: %i", num_points);

      // Parse quaternion (q)
      for (int i = 0; i < 4; ++i)
      {
        //ENTO_DEBUG("i: %i", i);
        if (!(iss >> instance->pose_gt.q[i] >> comma) || (comma != ','))
        {
            return false; // Parsing failed
        }
      }
      //ENTO_DEBUG_EIGEN_MATRIX(instance->pose_gt.q, 4, 1, float)

      // Parse translation (t)
      for (int i = 0; i < 3; ++i) {
          if (!(iss >> instance->pose_gt.t[i] >> comma) || (comma != ',')) {
              return false; // Parsing failed
          }
      }
      //ENTO_DEBUG_EIGEN_MATRIX(instance->pose_gt.t, 3, 1, float)

      // Parse scale_gt and focal_gt
      if (!(iss >> instance->scale_gt >> comma) || comma != ',') {
          return false; // Parsing failed
      }

      if (!(iss >> instance->focal_gt >> comma) || comma != ',') {
          return false; // Parsing failed
      }
      //ENTO_DEBUG("Scale gt, focal gt: %f, %f", instance->scale_gt, instance->focal_gt);

      // Parse x_point correspondences
      Scalar x, y, z;
      for (std::size_t i = 0; i < num_points; ++i) {
          if (!(iss >> x >> comma) || comma != ',') {
              return false; // Parsing failed
          }
          if (!(iss >> y >> comma) || comma != ',') {
              return false; // Parsing failed
          }
          if (!(iss >> z >> comma) || comma != ',') {
              return false; // Parsing failed
          }
          instance->x_point_.push_back(Vec3<Scalar>(x, y, z));
          //ENTO_DEBUG_EIGEN_MATRIX(instance->x_point_[i], 3, 1, float)

      }

      // Parse X_point correspondences
      for (std::size_t i = 0; i < num_points; ++i) {
          if (!(iss >> x >> comma) || comma != ',') {
              return false; // Parsing failed
          }
          if (!(iss >> y >> comma) || comma != ',') {
              return false; // Parsing failed
          }
          if (i != (num_points-1))
          {
            if (!(iss >> z >> comma) || comma != ',') {
                return false; // Parsing failed
            }
          }
          else
          {
            if (!(iss >> z)) {
                return false; // Parsing failed
            }
          }
          instance->X_point_.push_back(Vec3<Scalar>(x, y, z));
      }

      return true; // Successfully parsed
#else
      // Microcontroller build: Use sscanf for parsing
      char* pos = const_cast<char*>(line);
      int problem_type;
      if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 1) {
          return false; // Parsing failed
      }
      pos = strchr(pos, ',') + 1;

      // Parse quaternion (q)
      for (int i = 0; i < 4; ++i) {
          if (sscanf(pos, "%lf,", &instance->pose_gt.q[i]) != 1) {
              return false; // Parsing failed
          }
          pos = strchr(pos, ',') + 1;
      }

      // Parse translation (t)
      for (int i = 0; i < 3; ++i) {
          if (sscanf(pos, "%lf,", &instance->pose_gt.t[i]) != 1) {
              return false; // Parsing failed
          }
          pos = strchr(pos, ',') + 1;
      }

      // Parse scale_gt and focal_gt
      if (sscanf(pos, "%lf,%lf,", &instance->scale_gt, &instance->focal_gt) != 2) {
          return false; // Parsing failed
      }
      pos = strchr(pos, ',') + 1;

      // Parse x_point correspondences
      Scalar x, y, z;
      for (std::size_t i = 0; i < instance->x_point_.capacity(); ++i) {
          if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
              return false; // Parsing failed
          }
          instance->x_point_.push_back(Vec3<Scalar>(x, y, z));
          pos = strchr(pos, ',') + 1;
      }

      // Parse X_point correspondences
      for (std::size_t i = 0; i < instance->X_point_.capacity(); ++i) {
          if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
              return false; // Parsing failed
          }
          instance->X_point_.push_back(Vec3<Scalar>(x, y, z));
          pos = strchr(pos, ',') + 1;
      }

      return true; // Successfully parsed
#endif
  }

};

template <typename Scalar, std::size_t NumPts=0>
struct RelativePoseProblemInstance {
public:

  // Ground truth camera pose
  CameraPose<Scalar> pose_gt;
  Matrix3x3<Scalar> H_gt; // for homography problems
  Scalar scale_gt = 1.0;
  Scalar focal_gt = 1.0;

  size_t n_point_point_ = NumPts;

  // Point-to-point correspondences
  //EntoContainer<Vec3<Scalar>, NumPts> p1_;
  EntoContainer<Vec3<Scalar>, NumPts> x1_;

  //EntoContainer<Vec3<Scalar>, NumPts> p2_;
  EntoContainer<Vec3<Scalar>, NumPts> x2_;

  void clear()
  {
    x1_.clear();
    x2_.clear();
    //p1_.clear();
    //p2_.clear();
    n_point_point_ = 0;
  }

  void set_num_pts(std::size_t num_points)
  {
#ifdef NATIVE
    n_point_point_ = num_points;
#endif
  }

#ifdef NATIVE
  static std::string header()
  {
    return "problem_type,num_points,quat_gt,tvec_gt,scale_gt,focal_gt,x1,x2";
  }
#else
  static constexpr const char *header()
  {
    return "problem_type,num_points,quat_gt,tvec_gt,scale_gt,focal_gt,x1,x2";

  }
#endif

  std::string serialize() const
  {
#ifdef NATIVE
    std::ostringstream oss;
    constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

    // Problem type ID for Absolute Pose
    oss << 3 << "," << n_point_point_ << ",";

    // Serialize quaternion (q) and translation (t)
    for (int i = 0; i < 4; ++i) {
        oss << std::fixed << std::setprecision(precision) << pose_gt.q[i] << ",";
    }
    for (int i = 0; i < 3; ++i) {
        oss << std::fixed << std::setprecision(precision) << pose_gt.t[i] << ",";
    }

    // Serialize scale and focal length
    oss << scale_gt << "," << focal_gt;

    // Serialize x1_ and x2_ point correspondences
    for (const auto &x1 : x1_) {
        oss << "," << x1.x() << "," << x1.y() << "," << x1.z(); 
    }
    for (const auto &x2 : x2_) {
        oss << "," << x2.x() << "," << x2.y() << "," << x2.z();
    }

    // Remove the trailing comma, if present
    std::string serialized = oss.str();
    if (!serialized.empty() && serialized.back() == ',')
    {
      serialized.pop_back();
    }

    return serialized;
#else
    // Serialization is not supported on microcontrollers

    ENTO_DEBUG("WARNING: SERIALIZATION CALLED FROM A MCU. Not supported")
    return "";
#endif
  }

  static bool deserialize(const char* line, RelativePoseProblemInstance* instance)
  {
#ifdef NATIVE
    // Native build: Use std::istringstream for parsing
    std::istringstream iss(line);
    char comma;

    // Parse problem type
    int problem_type;
    if (!(iss >> problem_type >> comma) || problem_type != 3 || comma != ',')
    {
      return false; // Parsing failed
    }
    ENTO_DEBUG("Problem type: %i", problem_type);

    int num_points;
    if (!(iss >> num_points >> comma) || problem_type != 3 || comma != ',')
    {
      return false; // Parsing failed
    }
    if (instance->n_point_point_ == 0)
    {
      instance->n_point_point_ = num_points;
    }
    else
    {
      if (instance->n_point_point_ != num_points)
      {
        ENTO_DEBUG("Capacity does not match num points. Error!");
        return false;
      }
    }
    ENTO_DEBUG("Num points: %i", num_points);

    // Parse quaternion (q)
    for (int i = 0; i < 4; ++i)
    {
      //ENTO_DEBUG("i: %i", i);
      if (!(iss >> instance->pose_gt.q[i] >> comma) || (comma != ','))
      {
        return false; // Parsing failed
      }
    }
    //ENTO_DEBUG_EIGEN_MATRIX(instance->pose_gt.q, 4, 1, float)

    // Parse translation (t)
    for (int i = 0; i < 3; ++i)
    {
      if (!(iss >> instance->pose_gt.t[i] >> comma) || (comma != ','))
      {
        return false; // Parsing failed
      }
    }
    //ENTO_DEBUG_EIGEN_MATRIX(instance->pose_gt.t, 3, 1, float)


    // Parse scale_gt and focal_gt
    if (!(iss >> instance->scale_gt >> comma) || comma != ',') 
    {
      return false; // Parsing failed
    }

    if (!(iss >> instance->focal_gt >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    //ENTO_DEBUG("Scale gt, focal gt: %f, %f", instance->scale_gt, instance->focal_gt);

    // Parse x1 point correspondences
    Scalar x, y, z;
    for (std::size_t i = 0; i < num_points; ++i) {
      if (!(iss >> x >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (!(iss >> y >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (!(iss >> z >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      instance->x1_.push_back(Vec3<Scalar>(x, y, z));
      //ENTO_DEBUG_EIGEN_MATRIX(instance->x_point_[i], 3, 1, float)

    }

    // Parse x2 point correspondences
    for (std::size_t i = 0; i < num_points; ++i)
    {
      if (!(iss >> x >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (!(iss >> y >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (i != (num_points-1))
      {
        if (!(iss >> z >> comma) || comma != ',')
        {
        return false; // Parsing failed
        }
      }
      else
      {
        if (!(iss >> z)) {
          return false; // Parsing failed
        }
      }
      instance->x2_.push_back(Vec3<Scalar>(x, y, z));
    }

    return true; // Successfully parsed
#else
    // Microcontroller build: Use sscanf for parsing
    char* pos = const_cast<char*>(line);
    int problem_type;
    if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 1) {
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;

    // Parse quaternion (q)
    for (int i = 0; i < 4; ++i) {
        if (sscanf(pos, "%lf,", &instance->pose_gt.q[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse translation (t)
    for (int i = 0; i < 3; ++i) {
        if (sscanf(pos, "%lf,", &instance->pose_gt.t[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse scale_gt and focal_gt
    if (sscanf(pos, "%lf,%lf,", &instance->scale_gt, &instance->focal_gt) != 2) {
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;

    // Parse x_point correspondences
    Scalar x, y, z;
    for (std::size_t i = 0; i < instance->x_point_.capacity(); ++i) {
        if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
            return false; // Parsing failed
        }
        instance->x_point_.push_back(Vec3<Scalar>(x, y, z));
        pos = strchr(pos, ',') + 1;
    }

    // Parse X_point correspondences
    for (std::size_t i = 0; i < instance->X_point_.capacity(); ++i) {
        if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
            return false; // Parsing failed
        }
        instance->X_point_.push_back(Vec3<Scalar>(x, y, z));
        pos = strchr(pos, ',') + 1;
    }

    return true; // Successfully parsed
#endif
  }

};

template <typename Scalar, std::size_t NumPts=0>
struct HomographyProblemInstance
{
public:
  Matrix3x3<Scalar> H_gt; // for homography problems
  Scalar scale_gt = 1.0;
  Scalar focal_gt = 1.0;

  // Point-to-point correspondences
  EntoContainer<Vec3<Scalar>, NumPts> x1_;
  EntoContainer<Vec3<Scalar>, NumPts> x2_;

  size_t n_point_point_;

  HomographyProblemInstance()
  {}

  HomographyProblemInstance(const RelativePoseProblemInstance<Scalar, NumPts>& rel)
    : H_gt(rel.H_gt),
      scale_gt(rel.scale_gt),
      focal_gt(rel.focal_gt),
      x1_(rel.x1_),
      x2_(rel.x2_),
      n_point_point_(rel.n_point_point_)
  {
  }

  void clear()
  {
    x1_.clear();
    x2_.clear();
    n_point_point_ = 0;
  }

  void set_num_pts(std::size_t num_points)
  {
#ifdef NATIVE
    n_point_point_ = num_points;
#endif
  }

#ifdef NATIVE
  static std::string header()
  {
    return "problem_type,num_points,H_gt,scale_gt,focal_gt,x1,x2";
  }
#else
  static constexpr const char *header()
  {
    return "problem_type,num_points,H_gt,scale_gt,focal_gt,x1,x2";

  }
#endif

  std::string serialize() const
  {
#ifdef NATIVE
    std::ostringstream oss;
    constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

    // Problem type ID for Absolute Pose
    oss << 3 << "," << n_point_point_ << ",";

    // Serialize Homography (H)
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        oss << std::fixed << std::setprecision(precision)
            << H_gt(i, j) << ",";
      }
    }

    // Serialize scale and focal length
    oss << scale_gt << "," << focal_gt;

    // Serialize x1_ and x2_ point correspondences
    for (const auto &x1 : x1_) {
        oss << "," << x1.x() << "," << x1.y() << "," << x1.z(); 
    }
    for (const auto &x2 : x2_) {
        oss << "," << x2.x() << "," << x2.y() << "," << x2.z();
    }

    // Remove the trailing comma, if present
    std::string serialized = oss.str();
    if (!serialized.empty() && serialized.back() == ',')
    {
      serialized.pop_back();
    }

    return serialized;
#else
    // Serialization is not supported on microcontrollers

    ENTO_DEBUG("WARNING: SERIALIZATION CALLED FROM A MCU. Not supported")
    return "";
#endif
  }

  static bool deserialize(const char* line, HomographyProblemInstance* instance)
  {
#ifdef NATIVE
    // Native build: Use std::istringstream for parsing
    std::istringstream iss(line);
    char comma;

    // Parse problem type
    int problem_type;
    if (!(iss >> problem_type >> comma) || problem_type != 3 || comma != ',')
    {
      return false; // Parsing failed
    }
    ENTO_DEBUG("Problem type: %i", problem_type);

    int num_points;
    if (!(iss >> num_points >> comma) || problem_type != 3 || comma != ',')
    {
      return false; // Parsing failed
    }
    if (instance->n_point_point_ == 0)
    {
      instance->n_point_point_ = num_points;
    }
    else
    {
      if (instance->n_point_point_ != num_points)
      {
        ENTO_DEBUG("Capacity does not match num points. Error!");
        return false;
      }
    }
    ENTO_DEBUG("Num points: %i", num_points);

    // Parse Homography matrix (H)
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        //ENTO_DEBUG("i: %i", i);
        if (!(iss >> instance->H_gt(i, j) >> comma) || (comma != ','))
        {
          return false; // Parsing failed
        }
      }
    }
    //ENTO_DEBUG_EIGEN_MATRIX(instance->pose_gt.q, 4, 1, float)


    // Parse scale_gt and focal_gt
    if (!(iss >> instance->scale_gt >> comma) || comma != ',') 
    {
      return false; // Parsing failed
    }

    if (!(iss >> instance->focal_gt >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    //ENTO_DEBUG("Scale gt, focal gt: %f, %f", instance->scale_gt, instance->focal_gt);

    // Parse x_point correspondences
    Scalar x, y, z;
    for (std::size_t i = 0; i < num_points; ++i) {
      if (!(iss >> x >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (!(iss >> y >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (!(iss >> z >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      instance->x1_.push_back(Vec3<Scalar>(x, y, z));
      //ENTO_DEBUG_EIGEN_MATRIX(instance->x_point_[i], 3, 1, float)

    }

    // Parse X_point correspondences
    for (std::size_t i = 0; i < num_points; ++i)
    {
      if (!(iss >> x >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (!(iss >> y >> comma) || comma != ',')
      {
        return false; // Parsing failed
      }
      if (i != (num_points-1))
      {
        if (!(iss >> z >> comma) || comma != ',')
        {
        return false; // Parsing failed
        }
      }
      else
      {
        if (!(iss >> z)) {
          return false; // Parsing failed
        }
      }
      instance->x2_.push_back(Vec3<Scalar>(x, y, z));
    }

    return true; // Successfully parsed
#else
    // Microcontroller build: Use sscanf for parsing
    char* pos = const_cast<char*>(line);
    int problem_type;
    if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 1) {
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;

    // Parse quaternion (q)
    for (int i = 0; i < 4; ++i) {
        if (sscanf(pos, "%lf,", &instance->pose_gt.q[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse translation (t)
    for (int i = 0; i < 3; ++i) {
        if (sscanf(pos, "%lf,", &instance->pose_gt.t[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse scale_gt and focal_gt
    if (sscanf(pos, "%lf,%lf,", &instance->scale_gt, &instance->focal_gt) != 2) {
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;

    // Parse x_point correspondences
    Scalar x, y, z;
    for (std::size_t i = 0; i < instance->x_point_.capacity(); ++i) {
        if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
            return false; // Parsing failed
        }
        instance->x_point_.push_back(Vec3<Scalar>(x, y, z));
        pos = strchr(pos, ',') + 1;
    }

    // Parse X_point correspondences
    for (std::size_t i = 0; i < instance->X_point_.capacity(); ++i) {
        if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
            return false; // Parsing failed
        }
        instance->X_point_.push_back(Vec3<Scalar>(x, y, z));
        pos = strchr(pos, ',') + 1;
    }

    return true; // Successfully parsed
#endif
  }
};


template <typename Scalar, size_t NumPts = 0>
struct CalibPoseValidator
{
  // Computes the distance to the ground truth pose
  static Scalar compute_pose_error(const AbsolutePoseProblemInstance<Scalar, NumPts> &instance,
                                   const CameraPose<Scalar> &pose,
                                   Scalar scale)
  {
    return (instance.pose_gt.R() - pose.R()).norm() + (instance.pose_gt.t - pose.t).norm() +
           std::abs(instance.scale_gt - scale);
  }

  static Scalar compute_pose_error(const RelativePoseProblemInstance<Scalar, NumPts> &instance,
                                   const CameraPose<Scalar> &pose)
  {
    return (instance.pose_gt.R() - pose.R()).norm() + (instance.pose_gt.t - pose.t).norm();
  }

  static Scalar compute_pose_error(const RelativePoseProblemInstance<Scalar> &instance,
                                   const ImagePair<Scalar> &image_pair)
  {
    return (instance.pose_gt.R() - image_pair.pose.R()).norm() + (instance.pose_gt.t - image_pair.pose.t).norm() +
           std::abs(instance.focal_gt - image_pair.camera1.focal()) / instance.focal_gt +
           std::abs(instance.focal_gt - image_pair.camera2.focal()) / instance.focal_gt;
  }

  // Checks if the solution is valid (i.e. is rotation matrix and satisfies projection constraints)
  static bool is_valid(const AbsolutePoseProblemInstance<Scalar, NumPts> &instance,
                       const CameraPose<Scalar> &pose, Scalar scale, Scalar tol)
  {
    // Point to point correspondences
    // alpha * p + lambda*x = R*X + t
    for (size_t i = 0; i < instance.x_point_.size(); ++i) {
      Scalar err = 1.0 - std::abs(instance.x_point_[i].dot(
                             (pose.R() * instance.X_point_[i] + pose.t).normalized()));
      if (err > tol)
        return false;
    }

    return true;
  }

  static bool is_valid(const RelativePoseProblemInstance<Scalar> &instance,
                       const CameraPose<Scalar> &pose, Scalar tol)
  {
    if ((pose.R().transpose() * pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
      return false;

    // Point to point correspondences
    // R * (alpha * p1 + lambda1 * x1) + t = alpha * p2 + lambda2 * x2
    //
    // cross(R*x1, x2)' * (alpha * p2 - t - alpha * R*p1) = 0
    for (size_t i = 0; i < instance.x1_.size(); ++i)
    {
      //Scalar err = std::abs(instance.x2_[i]
      //                        .cross(pose.R() * instance.x1_[i])
      //                        .normalized()
      //                        .dot(pose.R() * instance.p1_[i] + pose.t - instance.p2_[i]));
      Scalar err = std::abs(instance.x2_[i]
                              .cross(pose.R() * instance.x1_[i])
                              .normalized()
                              .dot(pose.t));
      if (err > tol)
        return false;
    }
    return true;
  }

  static bool is_valid(const RelativePoseProblemInstance<Scalar> &instance,
                       const ImagePair<Scalar> &image_pair, Scalar tol)
  {
    if ((image_pair.pose.R().transpose() * image_pair.pose.R() - Matrix3x3<Scalar>::Identity()).norm() > tol)
      return false;

    Matrix3x3<Scalar> K_1_inv, K_2_inv;
    K_1_inv << 1.0 / image_pair.camera1.focal(), 0.0, 0.0, 0.0, 1.0 / image_pair.camera1.focal(), 0.0, 0.0, 0.0, 1.0;
    K_2_inv << 1.0 / image_pair.camera2.focal(), 0.0, 0.0, 0.0, 1.0 / image_pair.camera2.focal(), 0.0, 0.0, 0.0, 1.0;

    // Point to point correspondences
    // cross(R*x1, x2)' * - t = 0
    // This currently works only for focal information from calib
    for (size_t i = 0; i < instance.x1_.size(); ++i)
    {
      Vec3<Scalar> x1_u = K_1_inv * instance.x1_[i];
      Vec3<Scalar> x2_u = K_2_inv * instance.x2_[i];
      Scalar err = std::abs((x2_u.cross(image_pair.pose.R() * x1_u).dot(-image_pair.pose.t)));
      if (err > tol)
        return false;
    }

    // return is_valid(instance, image_pair.pose, tol) && (std::fabs(image_pair.camera.focal() - instance.focal_gt) <
    // tol);
    return true;
  }
};

template <typename Scalar>
struct HomographyValidator {
  // Computes the distance to the ground truth pose
  static Scalar compute_pose_error(const RelativePoseProblemInstance<Scalar> &instance, const Matrix3x3<Scalar> &H)
  {
    Scalar err1 = (H.normalized() - instance.H_gt.normalized()).norm();
    Scalar err2 = (H.normalized() + instance.H_gt.normalized()).norm();
    return std::min(err1, err2);
  }

  // Checks if the solution is valid (i.e. is rotation matrix and satisfies projection constraints)
  static bool is_valid(const RelativePoseProblemInstance<Scalar> &instance, const Matrix3x3<Scalar> &H, Scalar tol)
  {
    for (size_t i = 0; i < instance.x1_.size(); ++i) {
      Vec3<Scalar> z = H * instance.x1_[i];
      Scalar err = 1.0 - std::abs(z.normalized().dot(instance.x2_[i].normalized()));
      if (err > tol)
        return false;
    }
    return true;
  }

  static Scalar compute_pose_error(const HomographyProblemInstance<Scalar> &instance,
                                   const Matrix3x3<Scalar> &H)
  {
    Scalar err1 = (H.normalized() - instance.H_gt.normalized()).norm();
    Scalar err2 = (H.normalized() + instance.H_gt.normalized()).norm();
    return std::min(err1, err2);
  }

  // Checks if the solution is valid (i.e. is rotation matrix and satisfies projection constraints)
  static bool is_valid(const HomographyProblemInstance<Scalar> &instance,
                       const Matrix3x3<Scalar> &H,
                       Scalar tol)
  {
    for (size_t i = 0; i < instance.x1_.size(); ++i) {
      Vec3<Scalar> z = H * instance.x1_[i];
      Scalar err = 1.0 - std::abs(z.normalized().dot(instance.x2_[i].normalized()));
      if (err > tol)
        return false;
    }
    return true;
  }
};

template <typename Scalar>
struct ProblemOptions {
    Scalar min_depth_ = 0.1;
    Scalar max_depth_ = 10.0;
    Scalar camera_fov_ = 70.0;
    int n_point_point_ = 0;
    bool upright_ = false;
    bool planar_ = false;
    bool generalized_ = false;
    bool generalized_duplicate_obs_ = false;
    int generalized_first_cam_obs_ = 0; // how many of the points should from the first camera (relpose only)
    bool unknown_scale_ = false;
    bool unknown_focal_ = false;
    bool radial_lines_ = false;
    Scalar min_scale_ = 0.1;
    Scalar max_scale_ = 10.0;
    Scalar min_focal_ = 100.0;
    Scalar max_focal_ = 1000.0;
    std::string additional_name_ = "";
};

template <typename Scalar>
void set_random_pose(CameraPose<Scalar> &pose,
                     bool upright,
                     bool planar)
{
  if (upright)
  {
    Vec2<Scalar> r;
    r.setRandom().normalize();
    Matrix3x3<Scalar> R;
    R << r(0), Scalar(0.0), r(1),
          Scalar(0.0), Scalar(1.0), Scalar(0.0),
          -r(1), Scalar(0.0), r(0); // y-gravity
    // pose.R << r(0), r(1), 0.0, -r(1), r(0), 0.0, 0.0, 0.0, 1.0; // z-gravity
    pose.q = rotmat_to_quat<Scalar>(R);
  }
  else
  {
    pose.q = Eigen::Quaternion<Scalar>::UnitRandom().coeffs();
  }

  pose.t.setRandom();
  if (planar)
      pose.t.y() = 0;
}

#ifdef NATIVE
template <typename Scalar, size_t NumPts = 0>
void generate_abspose_problems(int n_problems,
                               std::vector<AbsolutePoseProblemInstance<Scalar, NumPts>> *problem_instances,
                               const ProblemOptions<Scalar> &options)
{
  problem_instances->clear();
  problem_instances->reserve(n_problems);

  constexpr Scalar kPI = Scalar(3.14159265358979323846);
  Scalar fov_scale = std::tan(options.camera_fov_ / 2.0 * kPI / 180.0);

  // Random generators
  std::default_random_engine random_engine;
  std::uniform_real_distribution<Scalar> depth_gen(options.min_depth_, options.max_depth_);
  std::uniform_real_distribution<Scalar> coord_gen(-fov_scale, fov_scale);
  std::uniform_real_distribution<Scalar> scale_gen(options.min_scale_, options.max_scale_);
  std::uniform_real_distribution<Scalar> focal_gen(options.min_focal_, options.max_focal_);
  std::normal_distribution<Scalar> direction_gen(0.0, 1.0);
  std::normal_distribution<Scalar> offset_gen(0.0, 1.0);

  for (int i = 0; i < n_problems; ++i)
  {
    AbsolutePoseProblemInstance<Scalar, NumPts> instance;
    set_random_pose<Scalar>(instance.pose_gt, options.upright_, options.planar_);

    if (options.unknown_scale_)
    {
      instance.scale_gt = scale_gen(random_engine);
    }
    if (options.unknown_focal_)
    {
      instance.focal_gt = focal_gen(random_engine);
    }

    // Point to point correspondences
    instance.x_point_.reserve(options.n_point_point_);
    instance.X_point_.reserve(options.n_point_point_);
    //instance.p_point_.reserve(options.n_point_point_);
    for (int j = 0; j < options.n_point_point_; ++j)
    {
      Vec3<Scalar> p{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
      Vec3<Scalar> x{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x.normalize();
      Vec3<Scalar> X;

      if (options.generalized_)
      {
        p << offset_gen(random_engine), offset_gen(random_engine), offset_gen(random_engine);
      }

      X = instance.scale_gt * p + x * depth_gen(random_engine);

      X = instance.pose_gt.R().transpose() * (X - instance.pose_gt.t);

      if (options.unknown_focal_)
      {
        x.template block<2, 1>(0, 0) *= instance.focal_gt;
        x.normalize();
      }

      instance.x_point_.push_back(x);
      instance.X_point_.push_back(X);
      //instance.p_point_.push_back(p);
    }

    // This generates instances where the same 3D point is observed twice in a generalized camera
    // This is degenerate case for the 3Q3 based gp3p/gp4ps solver unless specifically handled.
    //if (options.generalized_ && options.generalized_duplicate_obs_)
    //{
    //  std::vector<int> ind = {0, 1, 2, 3};
    //  assert(options.n_point_point_ >= 4);

    //  std::random_device rd;
    //  std::mt19937 g(rd());
    //  std::shuffle(ind.begin(), ind.end(), g);
    //  instance.X_point_[ind[1]] = instance.X_point_[ind[0]];
    //  instance.x_point_[ind[1]] = (instance.pose_gt.R() * instance.X_point_[ind[0]] + instance.pose_gt.t -
    //                               instance.scale_gt * instance.p_point_[ind[1]])
    //                                  .normalized();
    //}


    problem_instances->push_back(instance);
  }
}

template <typename Scalar>
void generate_relpose_problems(int n_problems,
                               std::vector<RelativePoseProblemInstance<Scalar>> *problem_instances,
                               const ProblemOptions<Scalar> &options)
{
  problem_instances->clear();
  problem_instances->reserve(n_problems);

  constexpr Scalar kPI = Scalar(3.14159265358979323846);
  Scalar fov_scale = std::tan(options.camera_fov_ / 2.0 * kPI / 180.0);

  // Random generators
  std::default_random_engine random_engine;
  std::uniform_real_distribution<Scalar> depth_gen(options.min_depth_, options.max_depth_);
  std::uniform_real_distribution<Scalar> coord_gen(-fov_scale, fov_scale);
  std::uniform_real_distribution<Scalar> scale_gen(options.min_scale_, options.max_scale_);
  std::uniform_real_distribution<Scalar> focal_gen(options.min_focal_, options.max_focal_);
  std::normal_distribution<Scalar> direction_gen(0.0, 1.0);
  std::normal_distribution<Scalar> offset_gen(0.0, 1.0);

  for (int i = 0; problem_instances->size() < n_problems; ++i) {
      RelativePoseProblemInstance<Scalar> instance;
      set_random_pose<Scalar>(instance.pose_gt, options.upright_, options.planar_);

      if (options.unknown_scale_) {
          instance.scale_gt = scale_gen(random_engine);
      }
      if (options.unknown_focal_) {
          instance.focal_gt = focal_gen(random_engine);
      }

      if (!options.generalized_) {
          instance.pose_gt.t.normalize();
      }

      // Point to point correspondences
      //instance.p1_.reserve(options.n_point_point_);
      instance.x1_.reserve(options.n_point_point_);
      //instance.p2_.reserve(options.n_point_point_);
      instance.x2_.reserve(options.n_point_point_);

      for (int j = 0; j < options.n_point_point_; ++j) {

          Vec3<Scalar> p1{0.0, 0.0, 0.0};
          Vec3<Scalar> p2{0.0, 0.0, 0.0};
          Vec3<Scalar> x1{coord_gen(random_engine), coord_gen(random_engine), 1.0};
          x1.normalize();
          Vec3<Scalar> X;

          /*if (options.generalized_) {
              p1 << offset_gen(random_engine), offset_gen(random_engine), offset_gen(random_engine);
              p2 << offset_gen(random_engine), offset_gen(random_engine), offset_gen(random_engine);

              if (j > 0 && j < options.generalized_first_cam_obs_) {
                  p1 = instance.p1_[0];
                  p2 = instance.p2_[0];
              }
          }*/

          //X = instance.scale_gt * p1 + x1 * depth_gen(random_engine);
          X = x1 * depth_gen(random_engine);
          // Map into second image
          X = instance.pose_gt.R() * X + instance.pose_gt.t;

          Vec3<Scalar> x2 = (X - instance.scale_gt * p2).normalized();

          //@TODO: ensure FoV of second cameras as well...

          //instance.p1_.push_back(p1);
          instance.x1_.push_back(x1);
          //instance.p2_.push_back(p2);
          instance.x2_.push_back(x2);
      }

      // we do not add instance if not all points were valid
      if (instance.x1_.size() < options.n_point_point_)
          continue;

      problem_instances->push_back(instance);
  }
}

template <typename Scalar>
void generate_homography_problems(int n_problems,
                                  std::vector<RelativePoseProblemInstance<Scalar>> *problem_instances,
                                  const ProblemOptions<Scalar> &options)
{
  problem_instances->clear();
  problem_instances->reserve(n_problems);

  constexpr Scalar kPI = Scalar(3.14159265358979323846);
  Scalar fov_scale = std::tan(options.camera_fov_ / 2.0 * kPI / 180.0);

  // Random generators
  std::default_random_engine random_engine;
  std::uniform_real_distribution<Scalar> depth_gen(options.min_depth_, options.max_depth_);
  std::uniform_real_distribution<Scalar> coord_gen(-fov_scale, fov_scale);
  std::uniform_real_distribution<Scalar> scale_gen(options.min_scale_, options.max_scale_);
  std::uniform_real_distribution<Scalar> focal_gen(options.min_focal_, options.max_focal_);
  std::normal_distribution<Scalar> direction_gen(0.0, 1.0);
  std::normal_distribution<Scalar> offset_gen(0.0, 1.0);

  while (problem_instances->size() < static_cast<size_t>(n_problems)) {
      RelativePoseProblemInstance<Scalar> instance;
      set_random_pose<Scalar>(instance.pose_gt, options.upright_, options.planar_);

      if (options.unknown_scale_) {
          instance.scale_gt = scale_gen(random_engine);
      }
      if (options.unknown_focal_) {
          instance.focal_gt = focal_gen(random_engine);
      }

      if (!options.generalized_) {
          instance.pose_gt.t.normalize();
      }

      // Point to point correspondences
      instance.x1_.reserve(options.n_point_point_);
      instance.x2_.reserve(options.n_point_point_);

      // Generate plane
      Vec3<Scalar> n;
      n << direction_gen(random_engine), direction_gen(random_engine), direction_gen(random_engine);
      n.normalize();

      // Choose depth of plane such that center point of image 1 is at depth d
      Scalar d_center = depth_gen(random_engine);
      Scalar alpha = d_center / n(2);
      // plane is n'*X = alpha

      // ground truth homography
      instance.H_gt = alpha * instance.pose_gt.R() + instance.pose_gt.t * n.transpose();

      bool failed_instance = false;
      for (int j = 0; j < options.n_point_point_; ++j) {
          bool point_okay = false;
          for (int trials = 0; trials < 10; ++trials) {
              Vec3<Scalar> x1{coord_gen(random_engine), coord_gen(random_engine), 1.0};
              x1.normalize();
              Vec3<Scalar> X;

              // compute depth
              Scalar lambda = alpha / n.dot(x1);
              X = x1 * lambda;
              // Map into second image
              X = instance.pose_gt.R() * X + instance.pose_gt.t;

              Vec3<Scalar> x2 = X.normalized();

              // Check cheirality
              if (x2(2) < 0 || lambda < 0) {
                  // try to generate another point
                  continue;
              }

              // Check FoV of second camera
              Vec2<Scalar> x2h = x2.hnormalized();
              if (x2h(0) < -fov_scale || x2h(0) > fov_scale || x2h(1) < -fov_scale || x2h(1) > fov_scale) {
                  // try to generate another point
                  continue;
              }

              if (options.generalized_) {
                  // NYI
                  assert(false);
              }
              if (options.unknown_focal_) {
                  // NYI
                  assert(false);
              }

              instance.x1_.push_back(x1);
              instance.x2_.push_back(x2);
              point_okay = true;
              break;
          }
          if (!point_okay) {
              failed_instance = true;
              break;
          }
      }
      if (failed_instance) {
          continue;
      }

      problem_instances->push_back(instance);
  }
}
#endif

}; // namespace poselib

#endif

#endif // PROB_GEN_H
