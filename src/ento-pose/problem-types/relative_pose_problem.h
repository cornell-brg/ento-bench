#ifndef RELATIVE_POSE_PROBLEM_H
#define RELATIVE_POSE_PROBLEM_H

#include <ento-pose/problem-types/pose_problem.h>
#include <ento-math/core.h>
#include <ento-util/containers.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/util/validator.h>

#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif // ifdef NATIVE

namespace EntoPose
{

template <typename Scalar, typename Solver, size_t NumPts=0>
struct RelativePoseProblem :
  public PoseProblem<RelativePoseProblem<Scalar, Solver, NumPts>>
{
public:
  using Scalar_ = Scalar;
  using Solver_ = Solver;
  static constexpr size_t NumPts_ = NumPts;
  static constexpr Scalar tol = 1e-6;
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = false;
  static constexpr size_t MaxSolns_ = Solver::MaxSolns;
  //////// Class Members /////////
  // Solver Callable
  Solver solver_;

  // Ground truth 
  EntoPose::CameraPose<Scalar> pose_gt_;
  //EntoMath::Matrix3x3<Scalar> H_gt; // for homography problems
  static constexpr Scalar scale_gt_ = 1.0;
  static constexpr Scalar focal_gt_ = 1.0;
  size_t n_point_point_ = NumPts;

  // RelativePose Algorithm Outputs
  size_t num_solns_;

  // @TODO: The size of the solns container should be determined at compile time
  //  if possible. In this sense we always allocate enough space to store all
  //  solutions even if we don't need the space depending on the current input
  //  data. Alternatively, we could always use std::vector and perform a solns.reserve().
  //  As of now it is defaulting to a std::vector.
#ifdef NATIVE
  EntoContainer<EntoPose::CameraPose<Scalar>, 0> solns_; 
#else
  EntoContainer<EntoPose::CameraPose<Scalar>, MaxSolns_> solns_;

#endif

  // Point-to-point (2D-2D, homogeneous) correspondence containers
  EntoContainer<Vec3<Scalar>, NumPts> x1_;
  EntoContainer<Vec3<Scalar>, NumPts> x2_;

  // Validator Typedef
  typedef CalibratedRelativePoseValidator<Scalar> validator_;


  //////// Class Functions /////////

  // Constructor
  RelativePoseProblem(Solver solver) : solver_(std::move(solver)) {}

  // Problem Interface Functions

  // File I/O
#ifdef NATIVE
  std::string serialize_impl() const;
  bool        deserialize_impl(std::string& line);
#endif
  bool        deserialize_impl(const char* line);

  void solve_impl();
  void validate_impl();
  void clear_impl();

  static constexpr const char* header_impl()
  {
    return "problem_type,pose_gt,scale_gt,focal_gt,x1,x2";
  }

  // Class Specific Functions
  void set_num_pts(size_t num_pts);

};

///////////////////// CONCEPT /////////////////////

template <typename T>
concept RelativePoseProblemConcept = PoseProblemConcept<T> && requires(T t)
{
  { t.x1_ };
  { t.x2_ };
};

///////////////////// FACTORY /////////////////////

template <typename Scalar, size_t NumPts, typename Solver>
auto makeRelativePoseProblem(Solver solver)
{
  return RelativePoseProblem<Scalar, Solver, NumPts>(std::move(solver));
}

///////////////////// IMPLEMENTATIONS /////////////////////

template <typename Scalar, typename Solver, size_t NumPts>
void RelativePoseProblem<Scalar, Solver, NumPts>::solve_impl()
{
  num_solns_ = solver_.solve(*this, &solns_);
}


template <typename Scalar, typename Solver, size_t NumPts>
void RelativePoseProblem<Scalar, Solver, NumPts>::clear_impl()
{
  x1_.clear();
  x2_.clear();
  n_point_point_ = 0;
}

template <typename Scalar, typename Solver, size_t NumPts>
void RelativePoseProblem<Scalar, Solver, NumPts>::set_num_pts(std::size_t num_points)
{
#ifdef NATIVE
  n_point_point_ = num_points;
#endif
}

#ifdef NATIVE
template <typename Scalar, typename Solver, size_t NumPts>
std::string RelativePoseProblem<Scalar, Solver, NumPts>::serialize_impl() const
{
  std::ostringstream oss;
  constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

  // Problem type ID for Absolute Pose
  oss << 3 << "," << n_point_point_ << ",";

  // Serialize quaternion (q) and translation (t)
  for (int i = 0; i < 4; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.q[i] << ",";
  }
  for (int i = 0; i < 3; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.t[i] << ",";
  }

  // Serialize scale and focal length
  oss << scale_gt_ << "," << focal_gt_;

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
}


template <typename Scalar, typename Solver, size_t NumPts>
bool RelativePoseProblem<Scalar, Solver, NumPts>::deserialize_impl(std::string& line)
{
  // Native build: Use std::istringstream for parsing
  std::istringstream is(line);
  char comma;
  
  // Parse problem type
  int problem_type;
  if (!(is >> problem_type >> comma) || problem_type != 3 || comma != ',')
  {
    return false; // Parsing failed
  }

  size_t num_points;
  if (!(is >> num_points >> comma) || problem_type != 3 || comma != ',')
  {
    return false; // Parsing failed
  }
  if (n_point_point_ == 0)
  {
    n_point_point_ = num_points;
  }
  else
  {
    if (n_point_point_ != num_points)
    {
      ENTO_DEBUG("Capacity (NumPts) does not match num points. Error!");
      return false;
    }
  }

  // Parse quaternion (q)
  for (int i = 0; i < 4; ++i)
  {
    if (!(is >> pose_gt_.q[i] >> comma) || (comma != ','))
    {
      return false; // Parsing failed
    }
  }

  // Parse translation (t)
  for (int i = 0; i < 3; ++i)
  {
    if (!(is >> pose_gt_.t[i] >> comma) || (comma != ','))
    {
      return false; // Parsing failed
    }
  }


  // Parse scale_gt and focal_gt
  if (!(is >> scale_gt_ >> comma) || comma != ',') 
  {
    return false; // Parsing failed
  }

  if (!(is >> focal_gt_ >> comma) || comma != ',')
  {
    return false; // Parsing failed
  }

  // Parse x1 point correspondences
  Scalar x, y, z;
  for (std::size_t i = 0; i < num_points; ++i) {
    if (!(is >> x >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    if (!(is >> y >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    if (!(is >> z >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    x1_.push_back(Vec3<Scalar>(x, y, z));

  }

  // Parse x2 point correspondences
  for (std::size_t i = 0; i < num_points; ++i)
  {
    if (!(is >> x >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    if (!(is >> y >> comma) || comma != ',')
    {
      return false; // Parsing failed
    }
    if (i != (num_points-1))
    {
      if (!(is >> z >> comma) || comma != ',')
      {
      return false; // Parsing failed
      }
    }
    else
    {
      if (!(is >> z))
      {
        return false; // Parsing failed
      }
    }
    x2_.push_back(Vec3<Scalar>(x, y, z));
  }

  return true; // Successfully parsed
}
#endif // NATIVE

template <typename Scalar, typename Solver, size_t NumPts>
bool RelativePoseProblem<Scalar, Solver, NumPts>::deserialize_impl(const char* line)
{
  char* pos = const_cast<char*>(line);
  int problem_type;


  if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 3)
  {
      return false; // Parsing failed
  }
  pos = strchr(pos, ',') + 1;

  
  int num_pts;
  if (sscanf(pos, "%d,", &num_pts) != 1 )
  {
    if (NumPts == 0) n_point_point_ = num_pts;
    else if (NumPts != n_point_point_) return false;
    return false; // Parsing failed
  }
  pos = strchr(pos, ',') + 1;


  // Parse quaternion (q)
  Scalar qi = 0;
  for (int i = 0; i < 4; ++i)
  {
    if (sscanf(pos, "%f,", &qi) != 1)
    {
      return false; // Parsing failed
    }
    pose_gt_.q[i] = qi;
    pos = strchr(pos, ',') + 1;
  }

  // Parse translation (t)
  Scalar ti = 0;
  for (int i = 0; i < 3; ++i)
  {
    if (sscanf(pos, "%f,", &ti) != 1)
    {
        return false; // Parsing failed
    }
    pose_gt_.t[i] = ti;
    pos = strchr(pos, ',') + 1;
  }

  // Parse scale_gt and focal_gt
  if (sscanf(pos, "%f,%f,", &scale_gt_, &focal_gt_) != 2)
  {
    return false; // Parsing failed
  }
  pos = strchr(pos, ',') + 1;
  pos = strchr(pos, ',') + 1;

  // Parse x_point correspondences
  Scalar x, y, z;
  for (std::size_t i = 0; i < x1_.capacity(); ++i)
  {
    if (sscanf(pos, "%f,%f,%f,", &x, &y, &z) != 3)
    {
      return false; // Parsing failed
    }
    x1_.push_back(Vec3<Scalar>(x, y, z));
    pos = strchr(pos, ',') + 1;
    pos = strchr(pos, ',') + 1;
    pos = strchr(pos, ',') + 1;
  }

  // Parse X_point correspondences
  for (std::size_t i = 0; i < x2_.capacity(); ++i)
  {
    if ( i != x2_.capacity() - 1)
    {
      if (sscanf(pos, "%f,%f,%f,", &x, &y, &z) != 3)
      {
      return false; // Parsing failed
      }
    }
    else
    {
      if (sscanf(pos, "%f,%f,%f", &x, &y, &z) != 3)
      {
        return false; // Parsing failed
      }
    }
    x2_.push_back(Vec3<Scalar>(x, y, z));
    pos = strchr(pos, ',') + 1;
    pos = strchr(pos, ',') + 1;
    if ( i != x2_.capacity() - 1)
      pos = strchr(pos, ',') + 1;
  }

  return true; // Successfully parsed
}

} // namespace EntoPose

#endif // RELATIVE_POSE_PROBLEM_H
