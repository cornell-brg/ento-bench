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
private:
  static constexpr size_t calculate_inlier_container_size(size_t num_points)
  {
    return (num_points + 7) / 8;
  }
  static constexpr size_t StaticInlierFlagSize = (NumPts > 0) ? calculate_inlier_container_size(NumPts) : 0;
  size_t dyn_inlier_flag_size_ = 0;

protected:
  Scalar inlier_ratio_;
  using InlierFlagsContainer = EntoUtil::EntoContainer<uint8_t, StaticInlierFlagSize>;
  InlierFlagsContainer inlier_flags_;

public:
  RobustPoseProblem() = default;
  RobustPoseProblem(Scalar inlier_ratio) : inlier_ratio_(inlier_ratio) {};

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

  bool is_inlier(size_t index)
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
    : AbsBase(solver), RobustBase{} {}
  
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
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
    if (!RobustBase::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
  }
#endif

  bool deserialize_impl(const char* line)
  {
    char* pos = const_cast<char*>(line);
    if (!AbsBase::deserialize_impl(pos)) return false;
    // Initialize inliers after base deserialization
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
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

  RobustRelativePoseProblem(Solver solver)
    : RelBase(solver) , RobustBase{} {}

  std::string serialize() const
  {
    // TODO: Implement serialization
    return "";
  }

  bool deserialize_impl(const char* line)
  {
    char* pos = const_cast<char*>(line);
    if (!RelBase::deserialize_impl(pos)) return false;
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
    if (!RobustBase::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
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
    // Initialize inliers after base deserialization
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
    if (!RobustProblem::deserialize_inlier_mask(pos, num_inlier_bytes)) return false;
    return true;
  }
#endif
  bool deserialize_impl(const char* line)
  {
    char* pos = const_cast<char*>(line);
    if (!HomogProblem::deserialize_impl(pos)) return false;
    // Initialize inliers after base deserialization
    if constexpr (NumPts == 0) {
      this->initialize_inliers(this->n_point_point_);
    } else {
      this->initialize_inliers();
    }
    size_t num_inlier_bytes = calculate_inlier_container_size(this->n_point_point_);
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
