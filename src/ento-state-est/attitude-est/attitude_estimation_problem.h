#ifndef ATTITUDE_PROBLEM_H
#define ATTITUDE_PROBLEM_H

#include "attitude_measurement.h"
#include <ento-bench/problem.h>

namespace EntoAttitude
{

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter = true>
class AttitudeProblem : 
  public EntoBench::EntoProblem<AttitudeProblem<Scalar, Kernel, UseMag>>
{ 
public:
  // Expose Template Typenames to Others
  using Scalar_ = Scalar;
  using Kernel_ = Kernel;
 
  // Required by Problem Interface for Experiment I/O
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SavesResults_ = true;

  // @TODO: Add other member fields for input data, ground truth data
  //   and output data.
  Kernel kernel_;

  // Inputs from dataset file
  AttitudeMeasurement<Scalar, UseMag> measurement_;
  Eigen::Quaternion<Scalar> q_gt_;
  Scalar dt_;

  // Outputs from Kernel
  Eigen::Quaternion<Scalar> q_prev_;
  Eigen::Quaternion<Scalar> q_;

  // Validation threshold in degrees (set in constructor)
  Scalar angle_threshold_deg_;

#ifdef NATIVE
  std::string serialize_impl() const;
  bool deserialize_impl(const std::string &line);
#else

  // @TODO: Add serialize implementation for embedded builds.
  const char* serialize_impl() const;
#endif

  // @TODO: Add deserialize implementation for embedded builds
  bool deserialize_impl(const char* line);

  // Compute quaternion angle distance between estimated and ground truth
  Scalar computeQuaternionAngleDistance(const Eigen::Quaternion<Scalar>& q1, 
                                        const Eigen::Quaternion<Scalar>& q2) const;
  
  // Validate implementation that accepts a threshold
  bool validate_impl();
  
  // Helper function to validate with a specified threshold
  bool validate(Scalar angle_threshold_deg);

  // @TODO: Complete solve implementation.
  void solve_impl();

  // @TODO: Complete clear implementation.
  bool clear_impl();

  static constexpr const char* header_impl()
  {
    return ""; //@TODO: Add string for header (input)
  }

  // Constructor with kernel and angle threshold parameter
  AttitudeProblem(Kernel kernel, Scalar angle_threshold_deg = static_cast<Scalar>(1.0)) 
    : kernel_(std::move(kernel)),
      angle_threshold_deg_(angle_threshold_deg) {};
};


template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::deserialize_impl(const char* line)
{
  std::istringstream iss(line);
  Scalar ax, ay, az, gx, gy, gz, mx, my, mz;
  Scalar qw, qx, qy, qz;
  Scalar dt;
  
  if constexpr (UseMag) {
      if (!(iss >> ax >> ay >> az >> gx >> gy >> gz >> mx >> my >> mz >> qw >> qx >> qy >> qz >> dt)) {
          return false; // Failed to parse line
      }
      // measurement_ = AttitudeMeasurement<Scalar, UseMag>(ax, ay, az, gx, gy, gz, mx, my, mz);
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
      EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
      EntoMath::Vec3<Scalar>(ax, ay, az), // Accelerometer
      EntoMath::Vec3<Scalar>(mx, my, mz)  // Magnetometer
    );
  } else {
      if (!(iss >> ax >> ay >> az >> gx >> gy >> gz >> qw >> qx >> qy >> qz >> dt)) {
          return false; // Failed to parse line
      }
      // measurement_ = AttitudeMeasurement<Scalar, UseMag>(ax, ay, az, gx, gy, gz);
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
        EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
        EntoMath::Vec3<Scalar>(ax, ay, az)  // Accelerometer
    );
  }
  
  q_gt_ = Eigen::Quaternion<Scalar>(qw, qx, qy, qz);
  dt_ = dt;
  return true;
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
std::string AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::serialize_impl() const
{
    std::ostringstream oss;
    oss << q_.w() << "," << q_.x() << "," << q_.y() << "," << q_.z();
    return oss.str();
}

#ifndef NATIVE
template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
const char* AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::serialize_impl() const
{
    static char buffer[64];  // Ensure sufficient space
    snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f", q_.w(), q_.x(), q_.y(), q_.z());
    return buffer;
}
#endif


template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
void AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::solve_impl() 
{
    // If it's the first time solving, initialize q_prev_ using the ground truth q_gt_
    if (q_prev_.coeffs().isZero()) {
        q_prev_ = q_gt_;
    }

    // Call the kernel function to update q_ based on the measurement and time step
    kernel_(q_prev_, measurement_, dt_, &q_);

    // Update q_prev_ for the next iteration
    q_prev_ = q_;
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
Scalar AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::computeQuaternionAngleDistance(
    const Eigen::Quaternion<Scalar>& q1, const Eigen::Quaternion<Scalar>& q2) const
{
    // Compute the absolute value of the dot product of the quaternions
    // This gives the cosine of half the rotation angle between them
    Scalar dot_product = std::abs(q1.dot(q2));
    
    // Clamp the dot product to [-1, 1] to handle numerical errors
    dot_product = std::min(static_cast<Scalar>(1.0), std::max(static_cast<Scalar>(-1.0), dot_product));
    
    // Calculate the quaternion angle distance in radians
    // QAD = 2 * arccos(|q1·q2|)
    Scalar angle_distance = static_cast<Scalar>(2.0) * std::acos(dot_product);
    
    // Convert to degrees for better interpretation
    Scalar angle_distance_deg = angle_distance * static_cast<Scalar>(180.0 / M_PI);
    
    return angle_distance_deg;
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::validate_impl()
{
    // Use the threshold that was set in the constructor, but pass it to the validate function
    // This way the threshold is not hardcoded here
    return validate(angle_threshold_deg_);
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::validate(Scalar angle_threshold_deg)
{
    // Compute angle distance using the dedicated function
    Scalar angle_distance_deg = computeQuaternionAngleDistance(q_, q_gt_);
    
    // Return true if the angle distance is below the threshold
    return angle_distance_deg < angle_threshold_deg;
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::clear_impl()
{
    // Reset quaternions to identity (or zero)
    q_ = Eigen::Quaternion<Scalar>::Identity();
    q_prev_ = Eigen::Quaternion<Scalar>(0, 0, 0, 0); // Directly initialize to zero
    
    // Reset measurement data
    measurement_ = AttitudeMeasurement<Scalar, UseMag>();
    
    // Reset ground truth quaternion
    q_gt_ = Eigen::Quaternion<Scalar>::Identity();
    
    // Reset time step
    dt_ = static_cast<Scalar>(0.0);
    
    return true;
}

} // namespace EntoAttitude

#endif // ATTITUDE_PROBLEM_H