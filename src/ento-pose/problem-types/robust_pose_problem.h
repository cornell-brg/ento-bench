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
  public AbsolutePoseProblem<Scalar, Solver, NumPts>,
  public RobustPoseProblem<Scalar, NumPts>
{
  using AbsBase    = AbsolutePoseProblem<Scalar, Solver, NumPts>;
  using RobustBase = RobustPoseProblem <Scalar,             NumPts>;

  explicit RobustAbsolutePoseProblem(Solver solver)
    : AbsBase(std::move(solver)), RobustBase{} {}
  
#ifdef NATIVE
  std::string serialize() const
  {
    return AbsBase::serialize() + "," + RobustBase::serialize();
  }
  bool deserialize_impl(const std::string& line)
  {
    char* pos = const_cast<char*>(line.c_str());
    if (!AbsBase::deserialize_impl(pos)) return false;
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = RobustPoseProblem<Scalar, NumPts>::calculate_inlier_container_size(this->n_point_point_);
    if (!RobustBase::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
  }
#endif

  bool deserialize_impl(const char* line)
  {
    char* pos = const_cast<char*>(line);
    if (!AbsBase::deserialize_impl(pos)) return false;
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = RobustPoseProblem<Scalar, NumPts>::calculate_inlier_container_size(this->n_point_point_);
    if (!RobustBase::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
  } 

  void solve_impl();
  bool validate_impl();
  void clear_impl();
};

template <typename Scalar, typename Solver, size_t NumPts = 0>
struct RobustRelativePoseProblem : 
  public RelativePoseProblem<Scalar, Solver, NumPts>,
  public RobustPoseProblem<Scalar, NumPts>
{
  using RelBase = RelativePoseProblem<Scalar, Solver, NumPts>;
  using RobustBase = RobustPoseProblem<Scalar, NumPts>;

  EntoUtil::EntoContainer<Vec2<Scalar>, NumPts> x1_img_;
  EntoUtil::EntoContainer<Vec2<Scalar>, NumPts> x2_img_;

  RobustRelativePoseProblem(Solver solver)
    : RelBase(solver) , RobustBase{}, x1_img_(), x2_img_() {}

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

  std::string serialize() const
  {
    // TODO: Implement serialization
    return "";
  }

  // Helper to count commas in base relative pose data
  static size_t count_base_commas(size_t num_points) {
    // Problem type + num_points
    size_t commas = 2;
    // Quaternion (4 floats)
    commas += 4;
    // Translation (3 floats)
    commas += 3;
    // Scale and focal length
    commas += 2;
    // x1 points (3 floats per point)
    commas += num_points * 3;
    // x2 points (3 floats per point)
    commas += num_points * 3;
    return commas;
  }

  bool deserialize_impl(const char* line)
  {
    char* pos = const_cast<char*>(line);
    ENTO_DEBUG("[deserialize_impl] pos before base deserialization: %s", pos);
    
    ENTO_DEBUG("[relpose] pos at start: %s", pos);
    int problem_type;

    if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 3)
    {
        ENTO_DEBUG("[relpose] failed to parse problem_type");
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;
    ENTO_DEBUG("[relpose] pos after problem_type: %s", pos);

    int num_pts;
    if (sscanf(pos, "%d,", &num_pts) != 1 )
    {
      ENTO_DEBUG("[relpose] failed to parse num_pts");
      if (NumPts == 0) this->n_point_point_ = num_pts;
      else if (NumPts != this->n_point_point_) return false;
      return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;
    ENTO_DEBUG("[relpose] num_pts: %d", num_pts);
    ENTO_DEBUG("[relpose] pos after num_pts: %s", pos);

    // Parse quaternion (q)
    Scalar qi = 0;
    for (int i = 0; i < 4; ++i)
    {
      if (sscanf(pos, "%f,", &qi) != 1)
      {
        ENTO_DEBUG("[relpose] failed to parse q[%d]", i);
        return false; // Parsing failed
      }
      this->pose_gt_.q[i] = qi;
      pos = strchr(pos, ',') + 1;
      ENTO_DEBUG("[relpose] q[%d]=%f, pos=%s", i, qi, pos);
    }

    // Parse translation (t)
    Scalar ti = 0;
    for (int i = 0; i < 3; ++i)
    {
      if (sscanf(pos, "%f,", &ti) != 1)
      {
          ENTO_DEBUG("[relpose] failed to parse t[%d]", i);
          return false; // Parsing failed
      }
      this->pose_gt_.t[i] = ti;
      pos = strchr(pos, ',') + 1;
      ENTO_DEBUG("[relpose] t[%d]=%f, pos=%s", i, ti, pos);
    }

    // Parse scale_gt and focal_gt
    if (sscanf(pos, "%f,%f,", &this->scale_gt_, &this->focal_gt_) != 2)
    {
      ENTO_DEBUG("[relpose] failed to parse scale/focal");
      return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;
    pos = strchr(pos, ',') + 1;
    ENTO_DEBUG("[relpose] scale_gt=%f, focal_gt=%f, pos=%s", this->scale_gt_, this->focal_gt_, pos);

    // Parse x_point correspondences
    Scalar x, y;
    for (std::size_t i = 0; i < x1_img_.capacity(); ++i)
    {
      if (sscanf(pos, "%f,%f,", &x, &y) != 2)
      {
        ENTO_DEBUG("[relpose] failed to parse x1[%zu]", i);
        return false; // Parsing failed
      }
      x1_img_.push_back(Vec2<Scalar>(x, y));
      ENTO_DEBUG("[relpose] x1_[%zu]=(%f,%f), size=%zu", i, x, y, x1_img_.size());
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;
      ENTO_DEBUG("[relpose] pos after x1_[%zu]: %s", i, pos);
    }

    // Parse X_point correspondences
    for (std::size_t i = 0; i < x2_img_.capacity(); ++i)
    {
      if ( i != x2_img_.capacity() - 1)
      {
        if (sscanf(pos, "%f,%f,", &x, &y) != 2)
        {
          ENTO_DEBUG("[relpose] failed to parse x2[%zu]", i);
          return false; // Parsing failed
        }
      }
      else
      {
        if (sscanf(pos, "%f,%f,", &x, &y) != 2)
        {
          ENTO_DEBUG("[relpose] failed to parse x2[%zu] (last)", i);
          return false; // Parsing failed
        }
      }
      x2_img_.push_back(Vec2<Scalar>(x, y));
      ENTO_DEBUG("[relpose] x2_[%zu]=(%f,%f), size=%zu", i, x, y, x2_img_.size());
      pos = strchr(pos, ',') + 1;
      pos = strchr(pos, ',') + 1;
      ENTO_DEBUG("[relpose] pos after x2_[%zu]: %s", i, pos);
    }

    // Count how many commas we need to skip to get to inlier flags
    //size_t num_commas = count_base_commas(this->n_point_point_);
    //ENTO_DEBUG("[deserialize_impl] Need to skip %zu commas", num_commas);
    
    //// Skip through the commas
    //for (size_t i = 0; i < num_commas; ++i) {
    //  pos = strchr(pos, ',');
    //  if (!pos) {
    //    ENTO_DEBUG("[deserialize_impl] Failed to find comma %zu", i);
    //    return false;
    //  }
    //  pos++; // Skip the comma
    //}
    ENTO_DEBUG("[deserialize_impl] pos after skipping commas: %s", pos);

    ENTO_DEBUG("[deserialize_impl] inlier_flags_ size after initialize_inliers: %zu", this->inlier_flags().size());
    size_t num_inlier_bytes = RobustPoseProblem<Scalar, NumPts>::calculate_inlier_container_size(this->n_point_point_);
    bool mask_ok = RobustBase::deserialize_inlier_mask(pos, num_inlier_bytes);
    ENTO_DEBUG("[deserialize_impl] inlier_flags_ size after mask: %zu", this->inlier_flags().size());
    for (size_t i = 0; i < this->inlier_flags().size(); ++i) {
      ENTO_DEBUG("[deserialize_impl] inlier_flags_[%zu] = %d", i, static_cast<int>(this->inlier_flags()[i]));
    }
    return mask_ok;
  }
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
    size_t num_inlier_bytes = RobustPoseProblem<Scalar, NumPts>::calculate_inlier_container_size(this->n_point_point_);
    if (!RobustProblem::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
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
    size_t num_inlier_bytes = RobustPoseProblem<Scalar, NumPts>::calculate_inlier_container_size(this->n_point_point_);
    if (!RobustProblem::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
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
