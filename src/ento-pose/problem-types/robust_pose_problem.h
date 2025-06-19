#ifndef ROBUST_POSE_PROBLEM_H
#define ROBUST_POSE_PROBLEM_H

#include <cstdint>
#include <limits>
#include <concepts>
#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-pose/problem-types/pose_problem.h>
#include <ento-pose/problem-types/absolute_pose_problem.h>
#include <ento-pose/problem-types/relative_pose_problem.h>
#include <ento-pose/problem-types/homography_problem.h>
#include <ento-pose/robust-est/ransac_util.h>

#ifdef NATIVE
#include <iostream>
#include <iomanip>
#include <sstream>
#endif

namespace EntoPose
{
template <typename Scalar, size_t NumPts=0>
class RobustPoseProblem : 
  public EntoBench::EntoProblem<RobustPoseProblem<Scalar, NumPts>>
{
protected:
  static constexpr size_t calculate_inlier_container_size(size_t num_points)
  {
    return (num_points + 7) / 8;
  }
  static constexpr size_t StaticInlierFlagSize = (NumPts > 0) ? calculate_inlier_container_size(NumPts) : 0;
  size_t dyn_inlier_flag_size_ = 0;
  Scalar inlier_ratio_;
  using InlierFlagsContainer = EntoUtil::EntoContainer<uint8_t, StaticInlierFlagSize>;
  InlierFlagsContainer inlier_flags_;

public:
  CameraPose<Scalar> best_model_;
  RansacStats<Scalar> ransac_stats_;
  EntoUtil::EntoContainer<uint8_t, NumPts> inliers_;

  RobustPoseProblem() = default;
  RobustPoseProblem(Scalar inlier_ratio) : inlier_ratio_(inlier_ratio), best_model_(), ransac_stats_() {};

  void initialize_inliers()
  {
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  InlierFlagsContainer& inlier_flags() { return inlier_flags_; }

  void initialize_inliers(size_t num_pts)
  {
    if constexpr (NumPts == 0) {
      dyn_inlier_flag_size_ = calculate_inlier_container_size(num_pts);
      inlier_flags_.resize(dyn_inlier_flag_size_);
    } else {
      static_assert(NumPts == 0, "initialize_inliers(size_t) only valid for dynamic-size problems");
    }
  }

  bool is_inlier(size_t index) const
  {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    return (inlier_flags_[byte_index] & bit_mask) != 0;
  }

  void set_inlier(size_t index, bool flag)
  {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    if (flag) inlier_flags_[byte_index] |= bit_mask;
    else inlier_flags_[byte_index] &= ~bit_mask;
  }

  Scalar get_inlier_ratio() const { return inlier_ratio_; }

#ifdef NATIVE
  bool deserialize_inlier_mask(const std::string& line)
  {
    inlier_flags_.clear();
    std::istringstream iss(line);
    std::string token;
    while (std::getline(iss, token, ',')) {
      try {
        uint8_t byte_value = static_cast<uint8_t>(std::stoi(token));
        inlier_flags_.push_back(byte_value);
      } catch (const std::exception& e) {
        ENTO_DEBUG("Error: Failed to parse token '%s' as a byte.", token.c_str());
        return false;
      }
    }
    return true;
  }
#endif
  // MCU version: takes a const char*
  bool deserialize_inlier_mask(const char* line)
  {
    inlier_flags_.clear();
    uint8_t byte_value;
    while (sscanf(line, "%hhu,", &byte_value) == 1) {
      inlier_flags_.push_back(byte_value);
      line = strchr(line, ',');
      if (!line) break;
      line += 1;
    }
    return true;
  }
#ifdef NATIVE
  // Pointer-advancing deserializer for inlier mask (native)
  bool deserialize_inlier_mask(char*& pos, size_t num_inlier_bytes) {
    inlier_flags_.clear();
    for (size_t i = 0; i < num_inlier_bytes; ++i) {
      int byte_value = 0;
      int n = 0;
      if (sscanf(pos, "%d,%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(static_cast<uint8_t>(byte_value));
        pos += n;
      } else if (sscanf(pos, "%d%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(static_cast<uint8_t>(byte_value));
        pos += n;
      } else {
        // Not enough bytes in input
        return false;
      }
    }
    return true;
  }
#else
  // Pointer-advancing deserializer for inlier mask (MCU)
  bool deserialize_inlier_mask(char*& pos, size_t num_inlier_bytes) {
    inlier_flags_.clear();
    for (size_t i = 0; i < num_inlier_bytes; ++i) {
      uint8_t byte_value = 0;
      int n = 0;
      if (sscanf(pos, "%hhu,%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(byte_value);
        pos += n;
      } else if (sscanf(pos, "%hhu%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(byte_value);
        pos += n;
      } else {
        // Not enough bytes in input
        return false;
      }
    }
    return true;
  }
#endif
}; // End RobustPoseProblem

// --- Robust Problem Specializations (namespace scope, no shadowing) ---
template <typename Scalar, typename Solver, size_t NumPts = 0>
struct RobustAbsolutePoseProblem : 
  public EntoBench::EntoProblem<RobustAbsolutePoseProblem<Scalar, Solver, NumPts>>
{
  // Type aliases
  using Scalar_ = Scalar;
  using Solver_ = Solver;
  static constexpr size_t NumPts_ = NumPts;
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = false;
  static constexpr bool RequiresSetup_ = false;
  static constexpr size_t MaxSolns_ = Solver::MaxSolns;

  // From AbsolutePoseProblem
  Solver solver_;
  CameraPose<Scalar> pose_gt_;
  Scalar scale_gt_;
  Scalar focal_gt_;
  std::size_t n_point_point_ = NumPts;
  size_t num_solns_;

#ifdef NATIVE
  EntoUtil::EntoContainer<EntoPose::CameraPose<Scalar>, 0> solns_; 
#else
  EntoUtil::EntoContainer<EntoPose::CameraPose<Scalar>, MaxSolns_> solns_; 
#endif

  EntoUtil::EntoContainer<Vec3<Scalar>, NumPts> x_point_;
  EntoUtil::EntoContainer<Vec3<Scalar>, NumPts> X_point_;
  const Scalar tol_ = 1e-6;

  // From RobustPoseProblem
  static constexpr size_t calculate_inlier_container_size(size_t num_points) {
    return (num_points + 7) / 8;
  }
  static constexpr size_t StaticInlierFlagSize = (NumPts > 0) ? calculate_inlier_container_size(NumPts) : 0;
  size_t dyn_inlier_flag_size_ = 0;
  Scalar inlier_ratio_;
  using InlierFlagsContainer = EntoUtil::EntoContainer<uint8_t, StaticInlierFlagSize>;
  InlierFlagsContainer inlier_flags_;
  CameraPose<Scalar> best_model_;
  RansacStats<Scalar> ransac_stats_;
  EntoUtil::EntoContainer<uint8_t, NumPts> inliers_;

  // Current additional members
  EntoUtil::EntoContainer<Vec2<Scalar>, NumPts> x_img_;
  EntoUtil::EntoContainer<Vec3<Scalar>, NumPts> X_world_;

  // Constructors
  explicit RobustAbsolutePoseProblem(Solver solver)
    : solver_(std::move(solver)), 
      pose_gt_(), scale_gt_(), focal_gt_(), 
      inlier_ratio_(), best_model_(), ransac_stats_(),
      x_img_(), X_world_() {}

  RobustAbsolutePoseProblem(Solver solver, Scalar inlier_ratio)
    : solver_(std::move(solver)), 
      pose_gt_(), scale_gt_(), focal_gt_(), 
      inlier_ratio_(inlier_ratio), best_model_(), ransac_stats_(),
      x_img_(), X_world_() {}

  // Required EntoProblem methods
  static constexpr const char* header_impl() {
    return "problem_type,num_points,pose_gt,scale_gt,focal_gt,x_point,X_point,inlier_mask";
  }

  void solve_impl() {
    this->ransac_stats_ = this->solver_.solve(x_img_, X_world_, &this->best_model_, &this->inliers_);
  }

  bool validate_impl() {
    // TODO: Implement validation if needed
    return true;
  }

  void clear_impl() {
    x_point_.clear();
    X_point_.clear();
    x_img_.clear();
    X_world_.clear();
    solns_.clear();
    inlier_flags_.clear();
    inliers_.clear();
    n_point_point_ = 0;
    num_solns_ = 0;
  }

#ifdef NATIVE
  std::string serialize_impl() const {
    std::ostringstream oss;
    constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

    // Problem type ID for Robust Absolute Pose
    oss << 1 << "," << n_point_point_ << ",";

    // Serialize quaternion and translation
    for (int i = 0; i < 4; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.q[i] << ",";
    }
    for (int i = 0; i < 3; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.t[i] << ",";
    }

    // Serialize scale and focal length
    oss << scale_gt_ << "," << focal_gt_;

    // Serialize x_img (2D points)
    for (const auto &x : x_img_) {
      oss << "," << x.x() << "," << x.y(); 
    }
    
    // Serialize X_world (3D points)
    for (const auto &X : X_world_) {
      oss << "," << X.x() << "," << X.y() << "," << X.z();
    }

    // Serialize inlier mask as bytes
    for (size_t i = 0; i < inlier_flags_.size(); ++i) {
      oss << "," << static_cast<uint32_t>(inlier_flags_[i]);
    }

    return oss.str();
  }

  bool deserialize_impl(const std::string& line) {
    return deserialize_impl(line.c_str());
  }
#endif

  // Robust helper methods
  void initialize_inliers() {
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  void initialize_inliers(size_t num_pts) {
    if constexpr (NumPts == 0) {
      dyn_inlier_flag_size_ = calculate_inlier_container_size(num_pts);
      inlier_flags_.resize(dyn_inlier_flag_size_);
    }
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  bool is_inlier(size_t index) const {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    return (inlier_flags_[byte_index] & bit_mask) != 0;
  }

  void set_inlier(size_t index, bool flag) {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    if (flag) inlier_flags_[byte_index] |= bit_mask;
    else inlier_flags_[byte_index] &= ~bit_mask;
  }

  Scalar get_inlier_ratio() const { return inlier_ratio_; }

  // Absolute pose helper methods
  void set_num_pts(size_t num_pts) {
    if constexpr (NumPts == 0)
      n_point_point_ = num_pts;
  }

  void set_tolerance(Scalar tol) {
    const_cast<Scalar&>(tol_) = tol;
  }

#ifdef NATIVE
  // Deserialize inlier mask methods (copied from RobustPoseProblem)
  bool deserialize_inlier_mask(const std::string& line) {
    inlier_flags_.clear();
    std::istringstream iss(line);
    std::string token;
    while (std::getline(iss, token, ',')) {
      try {
        uint8_t byte_value = static_cast<uint8_t>(std::stoi(token));
        inlier_flags_.push_back(byte_value);
      } catch (const std::exception& e) {
        ENTO_DEBUG("Error: Failed to parse token '%s' as a byte.", token.c_str());
        return false;
      }
    }
    return true;
  }
#endif

  // MCU version: takes a const char*
  bool deserialize_inlier_mask(const char* line) {
    inlier_flags_.clear();
    uint8_t byte_value;
    while (sscanf(line, "%hhu,", &byte_value) == 1) {
      inlier_flags_.push_back(byte_value);
      line = strchr(line, ',');
      if (!line) break;
      line++; // Skip the comma
    }
    return true;
  }

#ifdef NATIVE
  // Pointer-advancing deserializer for inlier mask (native)
  bool deserialize_inlier_mask(char*& pos, size_t num_inlier_bytes) {
    inlier_flags_.clear();
    for (size_t i = 0; i < num_inlier_bytes; ++i) {
      int byte_value = 0;
      int n = 0;
      if (sscanf(pos, "%d,%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(static_cast<uint8_t>(byte_value));
        pos += n;
      } else if (sscanf(pos, "%d%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(static_cast<uint8_t>(byte_value));
        pos += n;
      } else {
        // Not enough bytes in input
        return false;
      }
    }
    return true;
  }
#else
  // Pointer-advancing deserializer for inlier mask (MCU)
  bool deserialize_inlier_mask(char*& pos, size_t num_inlier_bytes) {
    inlier_flags_.clear();
    for (size_t i = 0; i < num_inlier_bytes; ++i) {
      uint8_t byte_value = 0;
      int n = 0;
      if (sscanf(pos, "%hhu,%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(byte_value);
        pos += n;
      } else if (sscanf(pos, "%hhu%n", &byte_value, &n) == 1) {
        inlier_flags_.push_back(byte_value);
        pos += n;
      } else {
        // Not enough bytes in input
        return false;
      }
    }
    return true;
  }
#endif

  bool deserialize_impl(const char* line)
  {
    ENTO_DEBUG("[deserialize_impl] Starting deserialization");
    char* pos = const_cast<char*>(line);
    
    // Parse problem type
    int problem_type;
    if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 1) {
      ENTO_DEBUG("[deserialize_impl] Invalid problem type: %d", problem_type);
      return false;
    }
    pos = strchr(pos, ',') + 1;

    // Parse num_pts
    int num_pts;
    if (sscanf(pos, "%d,", &num_pts) != 1) {
      ENTO_DEBUG("[deserialize_impl] Failed to parse num_pts");
      if (NumPts == 0) this->n_point_point_ = num_pts;
      else if (NumPts != this->n_point_point_) return false;
      return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;
    
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }

    // Parse quaternion (4 floats)
    Scalar qi = 0;
    for (int i = 0; i < 4; ++i) {
      if (sscanf(pos, "%f,", &qi) != 1) {
        ENTO_DEBUG("[deserialize_impl] Failed to parse quaternion component %d", i);
        return false;
      }
      this->pose_gt_.q[i] = qi;
      pos = strchr(pos, ',') + 1;
    }

    // Parse translation (3 floats)
    Scalar ti = 0;
    for (int i = 0; i < 3; ++i) {
      if (sscanf(pos, "%f,", &ti) != 1) {
        ENTO_DEBUG("[deserialize_impl] Failed to parse translation component %d", i);
        return false;
      }
      this->pose_gt_.t[i] = ti;
      pos = strchr(pos, ',') + 1;
    }

    // Parse scale_gt and focal_gt
    if (sscanf(pos, "%f,%f,", &this->scale_gt_, &this->focal_gt_) != 2) {
      ENTO_DEBUG("[deserialize_impl] Failed to parse scale_gt and focal_gt");
      return false;
    }
    pos = strchr(pos, ',') + 1;
    pos = strchr(pos, ',') + 1;

    // Parse x_point correspondences
    Scalar x, y;
    for (std::size_t i = 0; i < x_img_.capacity(); ++i)
    {
      if (sscanf(pos, "%f,%f,", &x, &y) != 2)
      {
        ENTO_DEBUG("[abspose] failed to parse x[%zu]", i);
        return false; // Parsing failed
      }
      x_img_.push_back(Vec2<Scalar>(x, y));
      ENTO_DEBUG("[abspose] x_[%zu]=(%f,%f), size=%zu", i, x, y, x_img_.size());
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;
      ENTO_DEBUG("[abspose] pos after x_[%zu]: %s", i, pos);
    }

    // Parse X_point correspondences
    Scalar z;
    for (std::size_t i = 0; i < X_world_.capacity(); ++i)
    {
      if ( i != X_world_.capacity() - 1)
      {
        if (sscanf(pos, "%f,%f,%f", &x, &y, &z) != 3)
        {
          ENTO_DEBUG("[abspose] failed to parse X[%zu]", i);
          return false; // Parsing failed
        }
      }
      else
      {
        if (sscanf(pos, "%f,%f,%f", &x, &y, &z) != 3)
        {
          ENTO_DEBUG("[abspose] failed to parse X[%zu] (last)", i);
          return false; // Parsing failed
        }
      }
      X_world_.push_back(Vec3<Scalar>(x, y, z));
      ENTO_DEBUG("[abspose] X_[%zu]=(%f,%f,%f), size=%zu", i, x, y, z, X_world_.size());
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;  // Add missing comma advancement for Z coordinate
      ENTO_DEBUG("[abspose] pos after X_[%zu]: %s", i, pos);
    }

    ENTO_DEBUG("[deserialize_impl] pos after parsing: %s", pos);

    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
    bool mask_ok = deserialize_inlier_mask(pos, num_inlier_bytes);
    return mask_ok;
  } 
};

template <typename Scalar, typename Solver, size_t NumPts = 0>
struct RobustRelativePoseProblem : 
  public EntoBench::EntoProblem<RobustRelativePoseProblem<Scalar, Solver, NumPts>>
{
  // Type aliases
  using Scalar_ = Scalar;
  using Solver_ = Solver;
  static constexpr size_t NumPts_ = NumPts;
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = false;
  static constexpr bool RequiresSetup_ = false;
  static constexpr size_t MaxSolns_ = Solver::MaxSolns;

  // From RelativePoseProblem
  Solver solver_;
  CameraPose<Scalar> pose_gt_;
  Scalar scale_gt_;
  Scalar focal_gt_;
  std::size_t n_point_point_ = NumPts;
  size_t num_solns_;

#ifdef NATIVE
  EntoUtil::EntoContainer<EntoPose::CameraPose<Scalar>, 0> solns_; 
#else
  EntoUtil::EntoContainer<EntoPose::CameraPose<Scalar>, MaxSolns_> solns_; 
#endif

  EntoUtil::EntoContainer<Vec3<Scalar>, NumPts> x1_point_;
  EntoUtil::EntoContainer<Vec3<Scalar>, NumPts> x2_point_;
  const Scalar tol_ = 1e-6;

  // From RobustPoseProblem
  static constexpr size_t calculate_inlier_container_size(size_t num_points) {
    return (num_points + 7) / 8;
  }
  static constexpr size_t StaticInlierFlagSize = (NumPts > 0) ? calculate_inlier_container_size(NumPts) : 0;
  size_t dyn_inlier_flag_size_ = 0;
  Scalar inlier_ratio_;
  using InlierFlagsContainer = EntoUtil::EntoContainer<uint8_t, StaticInlierFlagSize>;
  InlierFlagsContainer inlier_flags_;
  CameraPose<Scalar> best_model_;
  RansacStats<Scalar> ransac_stats_;
  EntoUtil::EntoContainer<uint8_t, NumPts> inliers_;

  // Current additional members
  EntoUtil::EntoContainer<Vec2<Scalar>, NumPts> x1_img_;
  EntoUtil::EntoContainer<Vec2<Scalar>, NumPts> x2_img_;

  // Constructors
  explicit RobustRelativePoseProblem(Solver solver)
    : solver_(std::move(solver)), 
      pose_gt_(), scale_gt_(), focal_gt_(), 
      inlier_ratio_(), best_model_(), ransac_stats_(),
      x1_img_(), x2_img_() {}

  RobustRelativePoseProblem(Solver solver, Scalar inlier_ratio)
    : solver_(std::move(solver)), 
      pose_gt_(), scale_gt_(), focal_gt_(), 
      inlier_ratio_(inlier_ratio), best_model_(), ransac_stats_(),
      x1_img_(), x2_img_() {}

  // Required EntoProblem methods
  static constexpr const char* header_impl() {
    return "problem_type,num_points,pose_gt,scale_gt,focal_gt,x1_point,x2_point,inlier_mask";
  }

  bool validate_impl() {
    // TODO: Implement validation if needed
    return true;
  }

  void clear_impl() {
    x1_point_.clear();
    x2_point_.clear();
    x1_img_.clear();
    x2_img_.clear();
    solns_.clear();
    inlier_flags_.clear();
    inliers_.clear();
    n_point_point_ = 0;
    num_solns_ = 0;
  }

  // Add missing inlier_flags() method for test compatibility
  InlierFlagsContainer& inlier_flags() { return inlier_flags_; }

  void solve_impl()
  {
    ENTO_DEBUG("Calling robust relative pose problem solver_.solve!!!");
    for (int i = 0; i < NumPts; ++i) {
      ENTO_DEBUG("x1_img_[%d] = (%f, %f)", i, x1_img_[i][0], x1_img_[i][1]);
      ENTO_DEBUG("x2_img_[%d] = (%f, %f)", i, x2_img_[i][0], x2_img_[i][1]);
    }
    for (const auto& x : x1_img_) {
      ENTO_DEBUG("x: (%f, %f)", x[0], x[1]);
    }
    for (const auto& x : x2_img_) {
      ENTO_DEBUG("x: (%f, %f)", x[0], x[1]);
    }
    this->ransac_stats_ = this->solver_.solve(x1_img_, x2_img_, &this->best_model_, &this->inliers_);
  }

#ifdef NATIVE
  std::string serialize_impl() const {
    std::ostringstream oss;
    constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

    // Problem type ID for Robust Relative Pose
    oss << 3 << "," << n_point_point_ << ",";

    // Serialize quaternion and translation
    for (int i = 0; i < 4; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.q[i] << ",";
    }
    for (int i = 0; i < 3; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.t[i] << ",";
    }

    // Serialize scale and focal length
    oss << scale_gt_ << "," << focal_gt_;

    // Serialize x1_img (2D points from image 1)
    for (const auto &x : x1_img_) {
      oss << "," << x.x() << "," << x.y(); 
    }
    
    // Serialize x2_img (2D points from image 2)
    for (const auto &x : x2_img_) {
      oss << "," << x.x() << "," << x.y();
    }

    // Serialize inlier mask as bytes
    for (size_t i = 0; i < inlier_flags_.size(); ++i) {
      oss << "," << static_cast<uint32_t>(inlier_flags_[i]);
    }

    return oss.str();
  }
#endif

  // Robust helper methods
  void initialize_inliers() {
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  void initialize_inliers(size_t num_pts) {
    if constexpr (NumPts == 0) {
      dyn_inlier_flag_size_ = calculate_inlier_container_size(num_pts);
      inlier_flags_.resize(dyn_inlier_flag_size_);
    }
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  bool is_inlier(size_t index) const {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    return (inlier_flags_[byte_index] & bit_mask) != 0;
  }

  void set_inlier(size_t index, bool flag) {
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    if (flag) inlier_flags_[byte_index] |= bit_mask;
    else inlier_flags_[byte_index] &= ~bit_mask;
  }

  Scalar get_inlier_ratio() const { return inlier_ratio_; }

  void set_num_pts(size_t num_pts) {
    if constexpr (NumPts == 0)
      n_point_point_ = num_pts;
  }

  void set_tolerance(Scalar tol) {
    const_cast<Scalar&>(tol_) = tol;
  }

#ifdef NATIVE
  // Deserialize inlier mask methods (copied from RobustPoseProblem)
  bool deserialize_inlier_mask(const std::string& line) {
    inlier_flags_.clear();
    std::istringstream iss(line);
    std::string token;
    while (std::getline(iss, token, ',')) {
      try {
        uint8_t byte_value = static_cast<uint8_t>(std::stoi(token));
        inlier_flags_.push_back(byte_value);
      } catch (const std::exception& e) {
        ENTO_DEBUG("Error: Failed to parse token '%s' as a byte.", token.c_str());
        return false;
      }
    }
    return true;
  }
#endif

  // MCU version: takes a const char*
  bool deserialize_inlier_mask(const char* line) {
    inlier_flags_.clear();
    uint8_t byte_value;
    while (sscanf(line, "%hhu,", &byte_value) == 1) {
      inlier_flags_.push_back(byte_value);
      line = strchr(line, ',');
      if (!line) break;
      line++; // Skip the comma
    }
    return true;
  }

  // Pointer-advancing version for deserialize_impl
  bool deserialize_inlier_mask(char*& pos, size_t num_inlier_bytes) {
    inlier_flags_.clear();
    for (size_t i = 0; i < num_inlier_bytes; ++i) {
      uint8_t byte_value;
      if (sscanf(pos, "%hhu,", &byte_value) != 1) {
        ENTO_DEBUG("Error: Failed to parse inlier byte %zu", i);
        return false;
      }
      inlier_flags_.push_back(byte_value);
      pos = strchr(pos, ',');
      if (!pos) {
        if (i == num_inlier_bytes - 1) break; // Last byte might not have comma
        ENTO_DEBUG("Error: Failed to find comma after inlier byte %zu", i);
        return false;
      }
      pos++; // Skip the comma
    }
    return true;
  }

  bool deserialize_impl(const char* line)
  {
    ENTO_DEBUG("[deserialize_impl] Starting deserialization");
    char* pos = const_cast<char*>(line);
    
    // Parse problem type
    int problem_type;
    if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 3) {
      ENTO_DEBUG("[deserialize_impl] Invalid problem type: %d", problem_type);
      return false;
    }
    pos = strchr(pos, ',') + 1;

    // Parse num_pts
    int num_pts;
    if (sscanf(pos, "%d,", &num_pts) != 1) {
      ENTO_DEBUG("[deserialize_impl] Failed to parse num_pts");
      return false;
    }
    pos = strchr(pos, ',') + 1;
    
    if constexpr (NumPts == 0) {
      n_point_point_ = num_pts;
      initialize_inliers(n_point_point_);
    } else {
      if (NumPts != num_pts) {
        ENTO_DEBUG("[deserialize_impl] NumPts mismatch: expected %zu, got %d", NumPts, num_pts);
        return false;
      }
      initialize_inliers();
    }

    // Parse quaternion (4 floats)
    for (int i = 0; i < 4; ++i) {
      Scalar qi;
      if (sscanf(pos, "%f,", &qi) != 1) {
        ENTO_DEBUG("[deserialize_impl] Failed to parse quaternion component %d", i);
        return false;
      }
      pose_gt_.q[i] = qi;
      pos = strchr(pos, ',') + 1;
    }

    // Parse translation (3 floats)
    for (int i = 0; i < 3; ++i) {
      Scalar ti;
      if (sscanf(pos, "%f,", &ti) != 1) {
        ENTO_DEBUG("[deserialize_impl] Failed to parse translation component %d", i);
        return false;
      }
      pose_gt_.t[i] = ti;
      pos = strchr(pos, ',') + 1;
    }

    // Parse scale_gt and focal_gt
    if (sscanf(pos, "%f,%f,", &scale_gt_, &focal_gt_) != 2) {
      ENTO_DEBUG("[deserialize_impl] Failed to parse scale_gt and focal_gt");
      return false;
    }
    pos = strchr(pos, ',') + 1;
    pos = strchr(pos, ',') + 1;

    // Parse x1_img (2D points from image 1)
    for (std::size_t i = 0; i < x1_img_.capacity(); ++i) {
      Scalar x, y;
      if (sscanf(pos, "%f,%f,", &x, &y) != 2) {
        ENTO_DEBUG("[deserialize_impl] Failed to parse x1_img point %zu", i);
        return false;
      }
      x1_img_.push_back(Vec2<Scalar>(x, y));
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;
    }

    // Parse x2_img (2D points from image 2)
    for (std::size_t i = 0; i < x2_img_.capacity(); ++i) {
      Scalar x, y;
      if (sscanf(pos, "%f,%f,", &x, &y) != 2) {
        ENTO_DEBUG("[deserialize_impl] Failed to parse x2_img point %zu", i);
        return false;
      }
      x2_img_.push_back(Vec2<Scalar>(x, y));
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;
    }

    // Parse inlier mask
    ENTO_DEBUG("[deserialize_impl] inlier_flags_ size after initialize_inliers: %zu", inlier_flags_.size());
    size_t num_inlier_bytes = calculate_inlier_container_size(n_point_point_);
    bool mask_ok = deserialize_inlier_mask(pos, num_inlier_bytes);
    ENTO_DEBUG("[deserialize_impl] inlier_flags_ size after mask: %zu", inlier_flags_.size());
    for (size_t i = 0; i < inlier_flags_.size(); ++i) {
      ENTO_DEBUG("[deserialize_impl] inlier_flags_[%zu] = %u", i, inlier_flags_[i]);
    }

    return mask_ok;
  }

#ifdef NATIVE
  bool deserialize_impl(const std::string& line) {
    return deserialize_impl(line.c_str());
  }
#endif
};

template <typename Scalar, typename Solver, size_t NumPts = 0>
struct RobustHomographyProblem : 
  public HomographyProblem<Scalar, Solver, NumPts>,
  public RobustPoseProblem<Scalar, NumPts>
{
  using HomogProblem = HomographyProblem<Scalar, Solver, NumPts>;
  using RobustProblem = RobustPoseProblem<Scalar, NumPts>;
  std::string serialize() const
  {
    // TODO: Implement serialization
    return "";
  }

#ifdef NATIVE
  bool deserialize_impl(const std::string& line)
  {
    char* pos = const_cast<char*>(line.c_str());
    if (!HomogProblem::deserialize_impl(pos)) return false;
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
    if (!RobustPoseProblem<Scalar, NumPts>::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
  }
#endif
  bool deserialize_impl(const char* line)
  {
    char* pos = const_cast<char*>(line);
    if (!HomogProblem::deserialize_impl(pos)) return false;
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
    if (!RobustPoseProblem<Scalar, NumPts>::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
  }
};

template <typename T>
concept RobustPoseProblemConcept =
  PoseProblemConcept<T> &&
  requires(T obj, std::size_t idx, bool flag)
  {
    { obj.is_inlier(idx) }            -> std::same_as<bool>;
    { obj.set_inlier(idx, flag) }     -> std::same_as<void>;
    { obj.initialize_inliers() }      -> std::same_as<void>;
    { obj.initialize_inliers(idx) }   -> std::same_as<void>;
    { obj.get_inlier_ratio() }        -> std::convertible_to<typename T::Scalar_>;
  };

template <typename T>
concept RobustAbsolutePoseProblemConcept =
  RobustPoseProblemConcept<T> && AbsolutePoseProblemConcept<T>;

template <typename T>
concept RobustRelativePoseProblemConcept =
  RobustPoseProblemConcept<T> && RelativePoseProblemConcept<T>;

template <typename T>
concept RobustHomographyProblemConcept =
  RobustPoseProblemConcept<T> && HomographyProblemConcept<T>;

} // namespace EntoPose

#endif // ROBUST_POSE_PROBLEM_H
