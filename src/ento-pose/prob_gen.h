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
#include <algorithm>

#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>
#include <ento-pose/problem-types/absolute_pose_problem.h>
#include <ento-pose/problem-types/relative_pose_problem.h>

using namespace EntoUtil;

namespace EntoPose
{

template <typename Scalar, size_t NumPts=0>
class RobustPoseProblemInstance
{
private:
  static constexpr size_t calculate_container_size(size_t num_points)
  {
    return (num_points + 7) / 8;
  }
  static constexpr size_t StaticInlierFlagSize = (NumPts > 0) ? calculate_container_size(NumPts) : 0;
  size_t dyn_inlier_flag_size_ = 0;

protected:
  Scalar inlier_ratio_;

  using InlierFlagsContainer = EntoContainer<uint8_t, StaticInlierFlagSize>;
  InlierFlagsContainer inlier_flags_;

public:
  RobustPoseProblemInstance() = default;


  std::string serialize() const
  {
#ifdef NATIVE
    std::string result;
    for (size_t i = 0; i < inlier_flags_.size(); ++i)
    {
      result += std::to_string(inlier_flags_[i]); // Convert each byte to a string
      if (i != inlier_flags_.size() - 1)
      {
        result += ','; // Add a comma between bytes
      }
    }
    return result;
#endif
  }

  static bool deserialize(const char* line, RobustPoseProblemInstance* instance)
  {
#ifdef NATIVE
    std::istringstream iss(line);
    std::string token;

    // Clear and resize the inlier_flags_ container
    instance->inlier_flags_.clear();

    while (std::getline(iss, token, ',')) // Parse each byte as a token
    {
      try
      {
        // Convert the token to a byte (uint8_t) and add it to the container
        uint8_t byte_value = static_cast<uint8_t>(std::stoi(token));
        instance->inlier_flags_.push_back(byte_value);
      }
      catch (const std::exception& e)
      {
        ENTO_DEBUG("Error: Failed to parse token '%s' as a byte.", token.c_str());
        return false; // Parsing failed
      }
    }

    return true; // Successfully parsed
#else
    char* pos = const_cast<char*>(line);

    // Clear and resize the inlier_flags_ container
    instance->inlier_flags_.clear();

    uint8_t byte_value;
    while (sscanf(pos, "%hhu,", &byte_value) == 1) // Parse each byte as an unsigned 8-bit integer
    {
      instance->inlier_flags_.push_back(byte_value);
      pos = strchr(pos, ','); // Move to the next comma
      if (!pos) break; // Stop if no more commas are found
      pos += 1; // Skip the comma
    }

    return true; // Successfully parsed
#endif
  }
  // Inititalize inlier flags with given size
  void initialize_inliers()
  {
    //if constexpr (NumPts == 0)
    //{
    //  ENTO_DEBUG("Resizing inlier flags to: %i", InlierFlagSize);
    //  inlier_flags_.resize(InlierFlagSize);
    //}
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  void initialize_inliers(size_t num_pts)
  {
    static_assert(NumPts == 0);
    dyn_inlier_flag_size_ = calculate_container_size(num_pts);
    ENTO_DEBUG("Resizing inlier flags to: %i", dyn_inlier_flag_size_);
    inlier_flags_.resize(dyn_inlier_flag_size_);
  }

  void is_inlier(size_t index)
  {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    return (inlier_flags_[byte_index] & bit_mask) != 0;
  }

  // Set a specific point as inlier or outlier
  void set_inlier(size_t index, bool flag)
  {
    ENTO_DEBUG("Setting inlier at %i with %i", index, flag);
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    ENTO_DEBUG("Byte index: %i", byte_index);
    ENTO_DEBUG("Bit mask: %i", bit_mask);
    ENTO_DEBUG("Index at: %i", inlier_flags_[byte_index]);
    if (flag) inlier_flags_[byte_index] |= bit_mask;
    else inlier_flags_[byte_index] &= ~bit_mask;
    ENTO_DEBUG("Inlier Flags post set: %i", inlier_flags_[byte_index]);
  }

  Scalar get_inlier_ratio() const { return inlier_ratio_; }
};

template <typename Scalar, size_t NumPts=0>
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

      // Parse translation (t)
      for (int i = 0; i < 3; ++i) {
          if (!(iss >> instance->pose_gt.t[i] >> comma) || (comma != ',')) {
              return false; // Parsing failed
          }
      }

      // Parse scale_gt and focal_gt
      if (!(iss >> instance->scale_gt >> comma) || comma != ',') {
          return false; // Parsing failed
      }

      if (!(iss >> instance->focal_gt >> comma) || comma != ',') {
          return false; // Parsing failed
      }

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
struct RelativePoseProblemInstance
{
public:

  // Ground truth camera pose
  CameraPose<Scalar> pose_gt;
  Matrix3x3<Scalar> H_gt; // for homography problems
  Scalar scale_gt = 1.0;
  Scalar focal_gt = 1.0;

  size_t n_point_point_ = NumPts;

  // Point-to-point correspondences
  EntoContainer<Vec3<Scalar>, NumPts> x1_;
  EntoContainer<Vec3<Scalar>, NumPts> x2_;

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
      if (!(iss >> instance->pose_gt.q[i] >> comma) || (comma != ','))
      {
        return false; // Parsing failed
      }
    }

    // Parse translation (t)
    for (int i = 0; i < 3; ++i)
    {
      if (!(iss >> instance->pose_gt.t[i] >> comma) || (comma != ','))
      {
        return false; // Parsing failed
      }
    }


    // Parse scale_gt and focal_gt
    if (!(iss >> instance->scale_gt >> comma) || comma != ',') 
    {
      return false; // Parsing failed
    }

    if (!(iss >> instance->focal_gt >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }

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
        if (!(iss >> instance->H_gt(i, j) >> comma) || (comma != ','))
        {
          return false; // Parsing failed
        }
      }
    }


    // Parse scale_gt and focal_gt
    if (!(iss >> instance->scale_gt >> comma) || comma != ',') 
    {
      return false; // Parsing failed
    }

    if (!(iss >> instance->focal_gt >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }

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

// NumPts differs from the non Robust problem instances. Here it is the total number of
// points and not the number to satisfy the model used. 
template <typename Scalar, size_t NumPts = 0>
struct RobustAbsolutePoseProblemInstance : AbsolutePoseProblemInstance<Scalar, NumPts>,
                                           RobustPoseProblemInstance<Scalar, NumPts>
{
  std::string serialize() const
  {
    std::string exp_data    = AbsolutePoseProblemInstance<Scalar, NumPts>::serialize();
    std::string ransac_data = RobustPoseProblemInstance<Scalar, NumPts>::serialize();
    return exp_data + "," + ransac_data;
  }

};

// NumPts differs from the non Robust problem instances. Here it is the total number of
// points and not the number to satisfy the model used. 
template <typename Scalar, size_t NumPts = 0>
struct RobustRelativePoseProblemInstance : RelativePoseProblemInstance<Scalar, NumPts>,
                                           RobustPoseProblemInstance<Scalar, NumPts>
{
  std::string serialize() const
  {
    std::string exp_data    = RelativePoseProblemInstance<Scalar, NumPts>::serialize();
    std::string ransac_data = RobustPoseProblemInstance<Scalar, NumPts>::serialize();
    return exp_data + "," + ransac_data;
  }
};

template <typename Scalar, size_t NumPts = 0>
struct RobustHomographyPoseProblemInstance : RelativePoseProblemInstance<Scalar, NumPts>,
                                             RobustPoseProblemInstance<Scalar, NumPts>
{
  std::string serialize() const
  {
    std::string exp_data    = HomographyProblemInstance<Scalar, NumPts>::serialize();
    std::string ransac_data = RobustPoseProblemInstance<Scalar, NumPts>::serialize();
    return exp_data + "," + ransac_data;
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

  template <typename CameraModel>
  static Scalar compute_pose_error(const RelativePoseProblemInstance<Scalar> &instance,
                                   const ImagePair<Scalar, CameraModel> &image_pair)
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

  template <typename CameraModel>
  static bool is_valid(const RelativePoseProblemInstance<Scalar> &instance,
                       const ImagePair<Scalar, CameraModel> &image_pair, Scalar tol)
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

/*template <typename Scalar, std::size_t NumPts = 0>
struct HomographyValidator {
  // Computes the distance to the ground truth pose
  static Scalar compute_pose_error(const RelativePoseProblemInstance<Scalar, NumPts> &instance, const Matrix3x3<Scalar> &H)
  {
    Scalar err1 = (H.normalized() - instance.H_gt.normalized()).norm();
    Scalar err2 = (H.normalized() + instance.H_gt.normalized()).norm();
    return std::min(err1, err2);
  }

  // Checks if the solution is valid (i.e. is rotation matrix and satisfies projection constraints)
  static bool is_valid(const RelativePoseProblemInstance<Scalar, NumPts> &instance, const Matrix3x3<Scalar> &H, Scalar tol)
  {
    for (size_t i = 0; i < instance.x1_.size(); ++i) {
      Vec3<Scalar> z = H * instance.x1_[i];
      Scalar err = 1.0 - std::abs(z.normalized().dot(instance.x2_[i].normalized()));
      if (err > tol)
        return false;
    }
    return true;
  }

  static Scalar compute_pose_error(const HomographyProblemInstance<Scalar, NumPts> &instance,
                                   const Matrix3x3<Scalar> &H)
  {
    Scalar err1 = (H.normalized() - instance.H_gt.normalized()).norm();
    Scalar err2 = (H.normalized() + instance.H_gt.normalized()).norm();
    return std::min(err1, err2);
  }

  // Checks if the solution is valid (i.e. is rotation matrix and satisfies projection constraints)
  static bool is_valid(const HomographyProblemInstance<Scalar, NumPts> &instance,
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
};*/

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
template <AbsolutePoseProblemConcept Problem>
void generate_abspose_problems(int n_problems,
                               std::vector<Problem> *problem_instances,
                               const ProblemOptions<typename Problem::Scalar_> &options)
{
  using Scalar = typename Problem::Scalar_;
  using Solver = typename Problem::Solver_;

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
    //AbsolutePoseProblemInstance<Scalar, NumPts> instance;
    Problem instance(Solver{});

    set_random_pose<Scalar>(instance.pose_gt_, options.upright_, options.planar_);

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

      X = instance.scale_gt_ * p + x * depth_gen(random_engine);

      X = instance.pose_gt_.R().transpose() * (X - instance.pose_gt_.t);

      instance.x_point_.push_back(x);
      instance.X_point_.push_back(X);
    }

    problem_instances->push_back(instance);
  }
}

template <RelativePoseProblemConcept Problem>
void generate_relpose_problems(int n_problems,
                               std::vector<Problem> *problem_instances,
                               const ProblemOptions<typename Problem::Scalar_> &options)
{
  using Scalar = typename Problem::Scalar_;
  using Solver = typename Problem::Solver_;

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
      //RelativePoseProblemInstance<Scalar> instance;
      Problem instance(Solver{}) ;
      set_random_pose<Scalar>(instance.pose_gt_, options.upright_, options.planar_);

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

          //X = instance.scale_gt * p1 + x1 * depth_gen(random_engine);
          X = x1 * depth_gen(random_engine);
          // Map into second image
          X = instance.pose_gt_.R() * X + instance.pose_gt_.t;

          Vec3<Scalar> x2 = (X - instance.scale_gt_ * p2).normalized();

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

template <RelativePoseProblemConcept Problem>
void generate_homography_problems(int n_problems,
                                  std::vector<Problem> *problem_instances,
                                  const ProblemOptions<typename Problem::Scalar_> &options)
{
  using Scalar = typename Problem::Scalar_;
  using Solver = typename Problem::Solver_;
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
      Problem instance(Solver{});
      set_random_pose<Scalar>(instance.pose_gt_, options.upright_, options.planar_);

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
      instance.H_gt_ = alpha * instance.pose_gt_.R() + instance.pose_gt_.t * n.transpose();

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
              X = instance.pose_gt_.R() * X + instance.pose_gt_.t;

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


template <typename Scalar>
void generate_robust_abspose_problems(size_t n_problems,
                                      Scalar inlier_ratio,
                                      std::vector<RobustAbsolutePoseProblemInstance<Scalar>> *problem_instances,
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

  ENTO_DEBUG("Starting experiment loop");
  for (size_t i = 0; i < n_problems; ++i)
  {
    RobustAbsolutePoseProblemInstance<Scalar> instance;
    instance.set_num_pts(options.n_point_point_);
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
    
    size_t num_inliers = static_cast<int>(inlier_ratio * options.n_point_point_);
    size_t num_outliers = options.n_point_point_ - num_inliers;
    
    
    ENTO_DEBUG("Generating inliers");
    for (size_t j = 0; j < num_inliers; ++j)
    {
      Vec3<Scalar> p{Scalar(0.0), Scalar(0.0), Scalar(0.0)};
      Vec3<Scalar> x{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x.normalize();
      Vec3<Scalar> X;

      X = instance.scale_gt * p + x * depth_gen(random_engine);

      X = instance.pose_gt.R().transpose() * (X - instance.pose_gt.t);

      instance.x_point_.push_back(x);
      instance.X_point_.push_back(X);
    }

    // Generate outliers
    ENTO_DEBUG("Generating outliers");
    for (size_t j = 0; j < num_outliers; ++j)
    {
      Vec3<Scalar> x{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x.normalize();

      Vec3<Scalar> X{depth_gen(random_engine), depth_gen(random_engine), depth_gen(random_engine)};

      instance.x_point_.push_back(x);
      instance.X_point_.push_back(X);
      
    }

    ENTO_DEBUG("Starting to shuffle correspondences...");
    // Generate indices and shuffle them
    std::vector<size_t> indices(instance.x_point_.size());
    std::iota(indices.begin(), indices.end(), 0); // Fill indices with 0, 1, ..., n-1
    std::shuffle(indices.begin(), indices.end(), random_engine);

    // Create temporary vectors for shuffled points
    std::vector<Vec3<Scalar>> shuffled_x_points;
    std::vector<Vec3<Scalar>> shuffled_X_points;
    std::vector<bool>         shuffled_inlier_flags;
    shuffled_x_points.reserve(indices.size());
    shuffled_X_points.reserve(indices.size());

    // Reorder the points based on the shuffled indices
    ENTO_DEBUG("Reordering correspondences...");
    for (size_t idx : indices) {
        shuffled_x_points.push_back(instance.x_point_[idx]);
        shuffled_X_points.push_back(instance.X_point_[idx]);
        shuffled_inlier_flags.push_back(idx < num_inliers);
    }

    // Replace the original points with the shuffled ones
    ENTO_DEBUG("Moving correspondences into instance...");
    instance.x_point_ = std::move(shuffled_x_points);
    instance.X_point_ = std::move(shuffled_X_points);

    // Set inlier flags.
    ENTO_DEBUG("Initializing inlier flags...");
    instance.initialize_inliers(options.n_point_point_);
    ENTO_DEBUG("Setting inlier flags");
    for (size_t idx = 0; idx < shuffled_inlier_flags.size(); ++idx)
    {
      ENTO_DEBUG("Setting inlier at idx: %i", idx);
      instance.set_inlier(idx, shuffled_inlier_flags[idx]);
    }

    ENTO_DEBUG("Pushing back instance!");
    problem_instances->push_back(instance);
  }
}

template <typename Scalar>
void generate_robust_relpose_problems(size_t n_problems,
                                      Scalar inlier_ratio,
                                      std::vector<RobustRelativePoseProblemInstance<Scalar>> *problem_instances,
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

  for (size_t i = 0; problem_instances->size() < n_problems; ++i)
  {
    RobustRelativePoseProblemInstance<Scalar> instance;
    instance.set_num_pts(options.n_point_point_);
    set_random_pose<Scalar>(instance.pose_gt, options.upright_, options.planar_);

    // Point to point correspondences
    instance.x1_.reserve(options.n_point_point_);
    instance.x2_.reserve(options.n_point_point_);

    size_t num_inliers = static_cast<int>(inlier_ratio * options.n_point_point_);
    size_t num_outliers = options.n_point_point_ - num_inliers;

    // Generate Inliers
    for (size_t j = 0; j < num_inliers; ++j)
    {
      Vec3<Scalar> p1{0.0, 0.0, 0.0};
      Vec3<Scalar> p2{0.0, 0.0, 0.0};
      Vec3<Scalar> x1{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x1.normalize();
      Vec3<Scalar> X;


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

    // Generate Outliers
    for (size_t j = 0; j < num_outliers; ++j)
    {
      Vec3<Scalar> x1{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x1.normalize();

      Vec3<Scalar> x2{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x2.normalize();

      instance.x1_.push_back(x1);
      instance.x2_.push_back(x2);
    }

    // we do not add instance if not all points were valid
    if (instance.x1_.size() < options.n_point_point_)
      continue;

    // Generate indices and shuffle them
    std::vector<size_t> indices(instance.x1_.size());
    std::iota(indices.begin(), indices.end(), 0); // Fill indices with 0, 1, ..., n-1
    std::shuffle(indices.begin(), indices.end(), random_engine);

    // Create temporary vectors for shuffled points
    std::vector<Vec3<Scalar>> shuffled_x1_points;
    std::vector<Vec3<Scalar>> shuffled_x2_points;
    std::vector<bool>         shuffled_inlier_flags;
    shuffled_x1_points.reserve(indices.size());
    shuffled_x2_points.reserve(indices.size());

    // Reorder the points based on the shuffled indices
    for (size_t idx : indices)
    {
      shuffled_x1_points.push_back(instance.x1_[idx]);
      shuffled_x2_points.push_back(instance.x2_[idx]);
      shuffled_inlier_flags.push_back(idx < num_inliers);
    }

    // Replace the original points with the shuffled ones
    instance.x1_ = std::move(shuffled_x1_points);
    instance.x2_ = std::move(shuffled_x2_points);

    instance.initialize_inliers(options.n_point_point_);
    for (size_t idx = 0; idx < shuffled_inlier_flags.size(); ++idx)
    {
      instance.set_inlier(idx, shuffled_inlier_flags[idx]);
    }

    problem_instances->push_back(instance);
  }
}


template <typename Scalar>
void generate_robust_homography_problems(size_t n_problems,
                                         Scalar inlier_ratio,
                                         std::vector<RobustRelativePoseProblemInstance<Scalar>> *problem_instances,
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

  while (problem_instances->size() < static_cast<size_t>(n_problems))
  {
    RobustRelativePoseProblemInstance<Scalar> instance;
    instance.set_num_pts(options.n_point_point_);
    set_random_pose<Scalar>(instance.pose_gt, options.upright_, options.planar_);

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
    instance.H_gt_ = alpha * instance.pose_gt.R() + instance.pose_gt.t * n.transpose();

    size_t num_inliers = static_cast<int>(inlier_ratio * options.n_point_point_);
    size_t num_outliers = options.n_point_point_ - num_inliers;

    bool failed_instance = false;
    for (size_t j = 0; j < num_inliers; ++j)
    {
      bool point_okay = false;
      for (int trials = 0; trials < 10; ++trials)
      {
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
        if (x2(2) < 0 || lambda < 0)
        {
            // try to generate another point
            continue;
        }

        // Check FoV of second camera
        Vec2<Scalar> x2h = x2.hnormalized();
        if (x2h(0) < -fov_scale || x2h(0) > fov_scale || x2h(1) < -fov_scale || x2h(1) > fov_scale)
        {
            // try to generate another point
            continue;
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

    // Generate outliers
    for (size_t j = 0; j < num_outliers; ++j)
    {
      Vec3<Scalar> x1{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x1.normalize();
      Vec3<Scalar> x2{coord_gen(random_engine), coord_gen(random_engine), 1.0};
      x2.normalize();

      instance.x1_.push_back(x1);
      instance.x2_.push_back(x2);
    }

    // Generate indices and shuffle them
    std::vector<size_t> indices(instance.x1_.size());
    std::iota(indices.begin(), indices.end(), 0); // Fill indices with 0, 1, ..., n-1
    std::shuffle(indices.begin(), indices.end(), random_engine);

    // Create temporary vectors for shuffled points
    std::vector<Vec3<Scalar>> shuffled_x1_points;
    std::vector<Vec3<Scalar>> shuffled_x2_points;
    std::vector<bool>         shuffled_inlier_flags;
    shuffled_x1_points.reserve(indices.size());
    shuffled_x2_points.reserve(indices.size());

    // Reorder the points based on the shuffled indices
    for (size_t idx : indices)
    {
      shuffled_x1_points.push_back(instance.x1_[idx]);
      shuffled_x2_points.push_back(instance.x2_[idx]);
      shuffled_inlier_flags.push_back(idx < num_inliers);
    }

    // Replace the original points with the shuffled ones
    instance.x1_ = std::move(shuffled_x1_points);
    instance.x2_ = std::move(shuffled_x2_points);

    instance.initialize_inliers(options.n_point_point_);
    for (size_t idx = 0; idx < shuffled_inlier_flags.size(); ++idx)
    {
      instance.set_inlier(idx, shuffled_inlier_flags[idx]);
    }

    problem_instances->push_back(instance);
  }
}

#endif

}; // namespace poselib

#endif

#endif // PROB_GEN_H;
