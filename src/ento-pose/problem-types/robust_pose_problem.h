#ifndef ROBUST_POSE_PROBLEM_H
#define ROBUST_POSE_PROBLEM_H

#include <cstdint>
#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-bench/problem-types/absolute_pose_problem.h>
#include <ento-bench/problem-types/relative_pose_problem.h>
#include <ento-bench/problem-types/homography_problem.h>

#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif

namespace EntoBench
{
template <typename Scalar, size_t NumPts=0>
class RobustPoseProblem : 
  EntoProblem<RobustPoseProblem<Scalar, NumPts>>
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

  using InlierFlagsContainer = EntoUtil::EntoContainer<uint8_t, StaticInlierFlagSize>;
  InlierFlagsContainer inlier_flags_;

public:
  RobustPoseProblem() = default;

  // Inititalize inlier flags with given size
  void initialize_inliers()
  {
    std::fill(inlier_flags_.begin(), inlier_flags_.end(), 0);
  }

  void initialize_inliers(size_t num_pts)
  {
    static_assert(NumPts == 0);
    dyn_inlier_flag_size_ = calculate_container_size(num_pts);
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
    size_t byte_index = index / 8;
    uint8_t bit_mask = (1 << (index % 8));
    if (flag) inlier_flags_[byte_index] |= bit_mask;
    else inlier_flags_[byte_index] &= ~bit_mask;
  }

  Scalar get_inlier_ratio() const { return inlier_ratio_; }


  static bool deserialize(const char* line,
                          RobustPoseProblem<Scalar, NumPts>* instance)
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
};


template <typename Scalar, size_t NumPts = 0>
struct RobustAbsolutePoseProblem : AbsolutePoseProblem<Scalar, NumPts>,
                                   RobustPoseProblem<Scalar, NumPts>
{
  std::string serialize() const
  {
    std::string exp_data    = AbsolutePoseProblem<Scalar, NumPts>::serialize();
    std::string ransac_data = RobustPoseProblem<Scalar, NumPts>::serialize();
    return exp_data + "," + ransac_data;
  }
};


template <typename Scalar, size_t NumPts = 0>
struct RobustRelativePoseProblem : RelativePoseProblem<Scalar, NumPts>,
                                   RobustPoseProblem<Scalar, NumPts>
{
  std::string serialize() const
  {
    std::string exp_data    = RelativePoseProblem<Scalar, NumPts>::serialize();
    std::string ransac_data = RobustPoseProblem<Scalar, NumPts>::serialize();
    return exp_data + "," + ransac_data;
  }
};

template <typename Scalar, size_t NumPts = 0>
struct RobustHomographyProblem : HomographyProblem<Scalar, NumPts>,
                                     RobustPoseProblem<Scalar, NumPts>
{
  std::string serialize() const
  {
    std::string exp_data    = HomographyProblem<Scalar, NumPts>::serialize();
    std::string ransac_data = RobustPoseProblem<Scalar, NumPts>::serialize();
    return exp_data + "," + ransac_data;
  }

#ifdef NATIVE
  static void deserialize(std::istream& stream,
                          RobustHomographyProblem<Scalar, NumPts>* instance)
  {
  }
#endif
  static void deserialize(const char* line,
                          RobustHomographyProblem<Scalar, NumPts>)
  {

  }
};

} // namespace EntoBench

#endif // ROBUST_POSE_PRBLEM_H
