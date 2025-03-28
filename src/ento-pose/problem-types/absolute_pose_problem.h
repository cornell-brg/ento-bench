#ifndef ABSOLUTE_POSE_PROBLEM_H
#define ABSOLUTE_POSE_PROBLEM_H

#include <Eigen/Dense>

#include <ento-pose/problem-types/pose_problem.h>

#include <ento-math/core.h>
#include <ento-pose/camera_models.h>
#include <ento-pose/pose_util.h>
#include <ento-pose/util/validator.h>
//#include <ento-pose/prob_gen.h>

#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif

namespace EntoPose 
{

template <typename Scalar, typename Solver, size_t NumPts=0>
class AbsolutePoseProblem : public
  PoseProblem<AbsolutePoseProblem<Scalar, Solver, NumPts>>
{
public:
  using Scalar_ = Scalar;
  using Solver_ = Solver;
  static constexpr size_t NumPts_ = NumPts;
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = false;
  static constexpr size_t MaxSolns_ = Solver::MaxSolns;

  //////// Class Members /////////
  // Solver Callable
  Solver solver_;

  // Ground truth 
  CameraPose<Scalar> pose_gt_;
  static constexpr Scalar scale_gt_ = 1.0;
  static constexpr Scalar focal_gt_ = 1.0;
  std::size_t n_point_point_ = NumPts;

  // AbsolutePose Algorithm Outputs
  size_t num_solns_;

  // @TODO: The size of the solns container should be determined at compile time
  //  if possible. In this sense we always allocate enough space to store all
  //  solutions even if we don't need the space depending on the current input
  //  data. Alternatively, we could always use std::vector and perform a solns.reserve().
  //  As of now it is defaulting to a std::vector.
  //  Current idea for a solution is to store a static constexpr in Pose Estimation 
  //  "Solvers"
#ifdef NATIVE
  EntoContainer<EntoPose::CameraPose<Scalar>, 0> solns_; 
#else
  EntoContainer<EntoPose::CameraPose<Scalar>, MaxSolns_> solns_; 
#endif

  // Point-to-point correspondences containers
  EntoContainer<Vec3<Scalar>, NumPts> x_point_;
  EntoContainer<Vec3<Scalar>, NumPts> X_point_;

  // Validator Alias
  typedef CalibratedAbsolutePoseValidator<Scalar> validator_;

  // Tolerance
  // @TODO: Make this a parameter
  const Scalar tol_ = 1e-6;

  //////// Class Functions /////////
  
  // Constructor
  AbsolutePoseProblem(Solver solver) : solver_(std::move(solver)) {}

  // Problem Interface Functions

  // File I/O
#ifdef NATIVE
  std::string serialize_impl() const;
  bool        deserialize_impl(const std::string& line);
#else
  const char* serialize_impl() const;
#endif

  bool        deserialize_impl(const char* line);

  void solve_impl();
  bool validate_impl();
  void clear_impl();

  static constexpr const char* header_impl()
  {
    return "problem_type,num_points,pose_gt,scale_gt,focal_gt,x_point,X_point";
  }

  // Class Specific Functions
  void set_num_pts(size_t num_pts);
  void is_valid(Scalar tol);
  void set_tolerance(Scalar tol);
};

///////////////////// CONCEPT /////////////////////

template <typename T>
concept AbsolutePoseProblemConcept = PoseProblemConcept<T> && requires(T t)
{
  { t.x_point_ };
  { t.X_point_ };
};

///////////////////// FACTORY /////////////////////

template <typename Scalar, size_t NumPts, typename Solver>
auto make_absolute_pose_problem(Solver solver)
{
  return AbsolutePoseProblem<Scalar, Solver, NumPts>(std::move(solver));
}

///////////////////// IMPLEMENTATIONS /////////////////////

template <typename Scalar, typename Solver, size_t NumPts>
void AbsolutePoseProblem<Scalar, Solver, NumPts>::solve_impl()
{
  num_solns_ = solver_.solve(*this, &solns_);
}

template <typename Scalar, typename Solver, size_t NumPts>
bool AbsolutePoseProblem<Scalar, Solver, NumPts>::validate_impl()
{
  Scalar pose_error = std::numeric_limits<Scalar>::max();
  for (size_t i = 0; i < num_solns_; i++)
  {
    const CameraPose<Scalar> &pose = solns_[i];
    Scalar err = validator_::compute_pose_error(pose, pose_gt_, 1.0, scale_gt_);
    pose_error = std::min(pose_error, err);
    ENTO_DEBUG("Pose error for soln %li/%i: %f", i+1, num_solns_, pose_error);
  }

  return pose_error < tol_;
}

// For generalized cameras we have an offset in the camera coordinate system
// EntoContainer<Vec3<Scalar>, NumPts> p_point_;
template <typename Scalar, typename Solver, size_t NumPts>
void AbsolutePoseProblem<Scalar, Solver, NumPts>::clear_impl()
{
  x_point_.clear();
  X_point_.clear();
  solns_.clear();
  n_point_point_ = 0;
}

template <typename Scalar, typename Solver, size_t NumPts>
void AbsolutePoseProblem<Scalar, Solver, NumPts>::set_num_pts(size_t num_points)
{
  if constexpr (NumPts == 0)
    n_point_point_ = num_points;
  else
    static_assert(true, "Setting Absolute Pose Problem NumPts when already templated on nonzero NumPts.");
}

#ifdef NATIVE
template <typename Scalar, typename Solver, size_t NumPts>
std::string AbsolutePoseProblem<Scalar, Solver, NumPts>::serialize_impl() const
{
  std::ostringstream oss;
  constexpr int precision = std::numeric_limits<Scalar>::max_digits10;

  // Problem type ID for Absolute Pose
  oss << 1 << "," << n_point_point_ << ",";

  // Serialize quaternion (q) and translation (t)
  for (int i = 0; i < 4; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.q[i] << ",";
  }
  for (int i = 0; i < 3; ++i) {
      oss << std::fixed << std::setprecision(precision) << pose_gt_.t[i] << ",";
  }

  // Serialize scale and focal length
  oss << scale_gt_ << "," << focal_gt_;

  // Serialize x_point and X_point correspondences
  for (const auto &x_point : x_point_)
  {
    oss << "," << x_point.x() << "," << x_point.y() << "," << x_point.z(); 
  }
  for (const auto &X_point : X_point_)
  {
    oss << "," << X_point.x() << "," << X_point.y() << "," << X_point.z();
  }

  // Remove the trailing comma, if present
  std::string serialized = oss.str();
  if (!serialized.empty() && serialized.back() == ',')
  {
    serialized.pop_back();
  }

  return serialized;
//#else
//  // Serialization is not supported on microcontrollers
//
//  ENTO_DEBUG("WARNING: SERIALIZATION CALLED FROM A MCU. Not supported");
//  //return "";
}

template <typename Scalar, typename Solver, size_t NumPts>
bool AbsolutePoseProblem<Scalar, Solver, NumPts>::deserialize_impl(const std::string& line)
{
  // Native build: Use std::istringstream for parsing
  std::istringstream iss(line);
  char comma;

  // Parse problem type
  int problem_type;
  if (!(iss >> problem_type >> comma) || problem_type != 1 || comma != ',')
  {
      return false; // Parsing failed
  }

  size_t num_points;
  if (!(iss >> num_points >> comma) || problem_type != 1 || comma != ',')
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
      ENTO_DEBUG("Capacity does not match num points. Error!");
      return false;
    }
  }

  // Parse quaternion (q)
  for (int i = 0; i < 4; ++i)
  {
    if (!(iss >> pose_gt_.q[i] >> comma) || (comma != ','))
    {
      return false; // Parsing failed
    }
  }

  // Parse translation (t)
  for (int i = 0; i < 3; ++i) {
    if (!(iss >> pose_gt_.t[i] >> comma) || (comma != ',')) {
      return false; // Parsing failed
    }
  }

  // Parse scale_gt and focal_gt
  if (!(iss >> scale_gt_ >> comma) || comma != ',') {
    return false; // Parsing failed
  }

  if (!(iss >> focal_gt_ >> comma) || comma != ',') {
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
    x_point_.push_back(Vec3<Scalar>(x, y, z));

  }

  // Parse X_point correspondences
  for (size_t i = 0; i < num_points; ++i) {
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
    X_point_.push_back(Vec3<Scalar>(x, y, z));
  }

  return true; // Successfully parsed
}
#endif

template <typename Scalar, typename Solver, size_t NumPts>
bool AbsolutePoseProblem<Scalar, Solver, NumPts>::deserialize_impl(const char* line)
{
    // Microcontroller build: Use sscanf for parsing
    char* pos = const_cast<char*>(line);
    int problem_type;
    if (sscanf(pos, "%d,", &problem_type) != 1 || problem_type != 1) {
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;

    // Parse quaternion (q)
    for (int i = 0; i < 4; ++i) {
        if (sscanf(pos, "%lf,", &pose_gt_.q[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse translation (t)
    for (int i = 0; i < 3; ++i) {
        if (sscanf(pos, "%lf,", &pose_gt_.t[i]) != 1) {
            return false; // Parsing failed
        }
        pos = strchr(pos, ',') + 1;
    }

    // Parse scale_gt and focal_gt
    if (sscanf(pos, "%lf,%lf,", &scale_gt_, &focal_gt_) != 2) {
        return false; // Parsing failed
    }
    pos = strchr(pos, ',') + 1;

    // Parse x_point correspondences
    Scalar x, y, z;
    for (std::size_t i = 0; i < x_point_.capacity(); ++i) {
        if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
            return false; // Parsing failed
        }
        x_point_.push_back(Vec3<Scalar>(x, y, z));
        pos = strchr(pos, ',') + 1;
    }

    // Parse X_point correspondences
    for (std::size_t i = 0; i < X_point_.capacity(); ++i) {
        if (sscanf(pos, "%lf,%lf,%lf,", &x, &y, &z) != 3) {
            return false; // Parsing failed
        }
        X_point_.push_back(Vec3<Scalar>(x, y, z));
        pos = strchr(pos, ',') + 1;
    }

    return true; // Successfully parsed
}


} // namespace EntoBench

#endif

