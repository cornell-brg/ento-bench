#ifndef ATTITUDE_PROBLEM_H
#define ATTITUDE_PROBLEM_H

#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <ento-bench/problem.h>
#include <cmath>

// Include filter headers for SFINAE type checking
#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/madgwick.h>
#include <ento-state-est/attitude-est/mahoney_fixed.h>
#include <ento-state-est/attitude-est/madgwick_fixed.h>

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
  static constexpr bool SaveResults_ = true;
  static constexpr bool RequiresSetup_ = false;

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

  // Filter parameters - will be set based on filter type
  Scalar kp_, ki_, gain_;
  EntoMath::Vec3<Scalar> bias_;

#ifdef NATIVE
  std::string serialize_impl() const;
  bool deserialize_impl(const std::string &line);
#else
  // @TODO: Add serialize implementation for embedded builds.
  const char* serialize_impl() const;
  // @TODO: Add deserialize implementation for embedded builds
  bool deserialize_impl(const char* line);
#endif

  // @TODO: Complete validate implementation. 
  bool validate_impl();

  // @TODO: Complete solve implementation.
  void solve_impl();


  // @TODO: Complete clear implementation.
  void clear_impl();

  static constexpr const char* header_impl()
  {
    return ""; //@TODO: Add string for header (input)
  }

  // Constructor for Mahoney filters (kp, ki parameters)
  template<typename T = Kernel>
 AttitudeProblem(Kernel kernel, Scalar kp, Scalar ki, 
                 typename std::enable_if_t<std::is_same_v<T, FilterMahoney<Scalar, UseMag>> ||
                                          std::is_same_v<T, FilterMahonyFixed<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(kp), ki_(ki), gain_(Scalar(0.0f)), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for Madgwick filters (gain parameter)  
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar gain,
                 typename std::enable_if_t<std::is_same_v<T, FilterMadgwick<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(gain), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for Madgwick fixed-point filters (gain parameter)  
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar gain,
                 typename std::enable_if_t<std::is_same_v<T, FilterMadgwickFixed<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(gain), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for fixed-point filters (already have internal state) - excludes FilterMadgwickFixed, FilterMahoney, FilterMahonyFixed
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel,
                 typename std::enable_if_t<!std::is_same_v<T, FilterMahoney<Scalar, UseMag>> && 
                                          !std::is_same_v<T, FilterMadgwick<Scalar, UseMag>> &&
                                          !std::is_same_v<T, FilterMadgwickFixed<Scalar, UseMag>> &&
                                          !std::is_same_v<T, FilterMahonyFixed<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(Scalar(0.0f)), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}
};

#ifdef NATIVE
template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::deserialize_impl(const std::string &line)
{
  std::istringstream iss(line);
  Scalar ax, ay, az, gx, gy, gz, mx, my, mz;
  Scalar qw, qx, qy, qz;
  Scalar dt;
  
  if constexpr (UseMag) {
      if (!(iss >> ax >> ay >> az >> gx >> gy >> gz >> mx >> my >> mz >> qw >> qx >> qy >> qz >> dt)) {
          return false; // Failed to parse line
      }
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
      EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
      EntoMath::Vec3<Scalar>(ax, ay, az), // Accelerometer
      EntoMath::Vec3<Scalar>(mx, my, mz)  // Magnetometer
    );
  } else {
      if (!(iss >> ax >> ay >> az >> gx >> gy >> gz >> qw >> qx >> qy >> qz >> dt)) {
          return false; // Failed to parse line
      }
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
        EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
        EntoMath::Vec3<Scalar>(ax, ay, az)  // Accelerometer
    );
  }
  
  q_gt_ = Eigen::Quaternion<Scalar>(qw, qx, qy, qz);
  dt_ = dt;
  return true;
}
#else
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
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
      EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
      EntoMath::Vec3<Scalar>(ax, ay, az), // Accelerometer
      EntoMath::Vec3<Scalar>(mx, my, mz)  // Magnetometer
    );
  } else {
      if (!(iss >> ax >> ay >> az >> gx >> gy >> gz >> qw >> qx >> qy >> qz >> dt)) {
          return false; // Failed to parse line
      }
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
        EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
        EntoMath::Vec3<Scalar>(ax, ay, az)  // Accelerometer
    );
  }
  
  q_gt_ = Eigen::Quaternion<Scalar>(qw, qx, qy, qz);
  dt_ = dt;
  return true;
}
#endif

#ifdef NATIVE
template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
std::string AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::serialize_impl() const
{
    std::ostringstream oss;
    oss << q_.w() << "," << q_.x() << "," << q_.y() << "," << q_.z();
    return oss.str();
}
#else
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

    // Call the kernel function with appropriate parameters based on filter type
    if constexpr (std::is_same_v<Kernel, FilterMahoney<Scalar, UseMag>>) {
        // Mahoney filter: needs kp, ki, bias
        q_ = kernel_(q_prev_, measurement_, dt_, kp_, ki_, bias_);
    } else if constexpr (std::is_same_v<Kernel, FilterMahonyFixed<Scalar, UseMag>>) {
        // Mahoney fixed-point filter: needs kp, ki, bias (same interface as FilterMahoney)
        q_ = kernel_(q_prev_, measurement_, dt_, kp_, ki_, bias_);
    } else if constexpr (std::is_same_v<Kernel, FilterMadgwick<Scalar, UseMag>>) {
        // Madgwick filter: needs gain
        q_ = kernel_(q_prev_, measurement_, dt_, gain_);
    } else if constexpr (std::is_same_v<Kernel, FilterMadgwickFixed<Scalar, UseMag>>) {
        // Madgwick fixed-point filter: needs gain (same interface as FilterMadgwick)
        q_ = kernel_(q_prev_, measurement_, dt_, gain_);
    } else {
        // Other fixed-point filters: use 4-parameter interface with internal state
        kernel_(q_prev_, measurement_, dt_, &q_);
    }

    // Update q_prev_ for the next iteration
    q_prev_ = q_;
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::validate_impl()
{
    // Check if the quaternion is unchanged (algorithm didn't run)
    static const Eigen::Quaternion<Scalar> zero_quat(Scalar(0.0), Scalar(0.0), Scalar(0.0), Scalar(0.0));
    
    // Check if q_ is zero (algorithm completely failed)
    float q_norm = std::sqrt(static_cast<float>(q_.w() * q_.w() + q_.x() * q_.x() + q_.y() * q_.y() + q_.z() * q_.z()));
    if (q_norm < 1e-6f) {
        // Quaternion is essentially zero - algorithm failed
        return false;
    }
    
    // Compute the quaternion angle distance (QAD) between estimated quaternion q_ and ground truth q_gt_
    
    // Compute the absolute value of the dot product of the quaternions
    // This gives the cosine of half the rotation angle between them
    float dot_product_float = std::abs(static_cast<float>(q_.w() * q_gt_.w() + q_.x() * q_gt_.x() + q_.y() * q_gt_.y() + q_.z() * q_gt_.z()));
    
    // Clamp the dot product to [-1, 1] to handle numerical errors
    dot_product_float = std::min(1.0f, std::max(-1.0f, dot_product_float));
    
    // Calculate the quaternion angle distance in radians
    // QAD = 2 * arccos(|q1·q2|)
    float angle_distance = 2.0f * std::acos(dot_product_float);
    
    // Convert to degrees for better interpretation
    float angle_distance_deg = angle_distance * 180.0f / 3.14159265359f;
    
    // Validation threshold - algorithms should be accurate to within 1 degree
    const float ANGLE_THRESHOLD_DEG = 1.0f;
    
    // Return true if the angle distance is below the threshold
    return angle_distance_deg < ANGLE_THRESHOLD_DEG;
}

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
void AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::clear_impl()
{
    // Reset quaternions to identity (or zero)
    q_ = Eigen::Quaternion<Scalar>::Identity();
    
    // Reset q_prev_ using float literals to avoid constructor ambiguity
    q_prev_ = Eigen::Quaternion<Scalar>(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f), Scalar(0.0f));
    
    // Reset measurement data
    measurement_ = AttitudeMeasurement<Scalar, UseMag>();
    
    // Reset ground truth quaternion
    q_gt_ = Eigen::Quaternion<Scalar>::Identity();
    
    // Reset time step
    dt_ = static_cast<Scalar>(0.0f);
    
    // Reset bias (important for filter state)
    bias_ = EntoMath::Vec3<Scalar>(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f));
}





} // namespace EntoAttitude

#endif // ATTITUDE_PROBLEM_H
