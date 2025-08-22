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
#include <ento-state-est/attitude-est/fourati_nonlinear.h>
#include <ento-state-est/attitude-est/fourati_fixed.h>
#include <ento-state-est/attitude-est-exp/kernels/mahoney_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/mahoney_fixed_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/madgwick_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/madgwick_fixed_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/fourati_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/fourati_fixed_checked.h>

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
  static constexpr bool SaveResults_ = false;
  static constexpr bool RequiresSetup_ = false;

  // @TODO: Add other member fields for input data, ground truth data
  //   and output data.
  Kernel kernel_;

  // Inputs from dataset file
  AttitudeMeasurement<Scalar, UseMag> measurement_;
  
  // Ground truth quaternion - use appropriate precision type based on Scalar
  // For double filters, we need double precision ground truth to avoid precision loss
  using GroundTruthType = typename std::conditional_t<std::is_same_v<Scalar, double>, double, float>;
  Eigen::Quaternion<GroundTruthType> q_gt_;
  
  Scalar dt_;

  // Outputs from Kernel
  Eigen::Quaternion<Scalar> q_prev_;
  Eigen::Quaternion<Scalar> q_;

  // Filter parameters - will be set based on filter type
  Scalar kp_ = Scalar(1.0f);
  Scalar ki_ = Scalar(0.1f);
  Scalar gain_ = Scalar(0.1f);
  EntoMath::Vec3<Scalar> bias_;
  
  // Fourati-specific reference quaternions
  Eigen::Quaternion<Scalar> g_q_ = Eigen::Quaternion<Scalar>(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f), Scalar(1.0f));  // Gravity reference
  Eigen::Quaternion<Scalar> m_q_ = Eigen::Quaternion<Scalar>(Scalar(0.0f), Scalar(0.707f), Scalar(0.0f), Scalar(0.707f)); // Magnetic field reference (45° dip)
  
  // Flag to track if filter has been initialized
  bool filter_initialized_ = false;

#ifdef NATIVE
  std::string serialize_impl() const;
  bool deserialize_impl(const std::string &line);
#endif
  // @TODO: Add serialize implementation for embedded builds.
#ifndef NATIVE
  const char* serialize_impl() const;
#endif
  bool deserialize_impl(const char* line);

  bool validate_impl();

  void solve_impl();

  void clear_impl();

  static constexpr const char* header_impl()
  {
    return "Attitude Estimation Problem";
  }

  // Constructor for Mahoney filters (kp, ki parameters)
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar kp, Scalar ki, 
                 typename std::enable_if_t<std::is_same_v<T, FilterMahoney<Scalar, UseMag>> ||
                                          std::is_same_v<T, FilterMahonyFixed<Scalar, UseMag>> ||
                                          std::is_same_v<T, EntoAttitudeExp::FilterMahoneyChecked<Scalar, UseMag>> ||
                                          std::is_same_v<T, EntoAttitudeExp::FilterMahoneyFixedChecked<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(kp), ki_(ki), gain_(Scalar(0.0f)), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {
        // Set parameters for checked Mahoney filters
        if constexpr (std::is_same_v<T, EntoAttitudeExp::FilterMahoneyChecked<Scalar, UseMag>>) {
            kernel_.setParameters(kp, ki);
        } else if constexpr (std::is_same_v<T, EntoAttitudeExp::FilterMahoneyFixedChecked<Scalar, UseMag>>) {
            kernel_.setParameters(kp, ki);
        }
    }

  // Constructor for Madgwick filters (gain parameter)  
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar gain,
                 typename std::enable_if_t<std::is_same_v<T, FilterMadgwick<Scalar, UseMag>> ||
                                          std::is_same_v<T, EntoAttitudeExp::FilterMadgwickChecked<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(gain), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for Madgwick fixed-point filters (gain parameter)  
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar gain,
                 typename std::enable_if_t<std::is_same_v<T, FilterMadgwickFixed<Scalar, UseMag>> ||
                                          std::is_same_v<T, EntoAttitudeExp::FilterMadgwickFixedChecked<Scalar, UseMag>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(gain), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for Fourati filters (gain parameter)  
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar gain,
                 typename std::enable_if_t<std::is_same_v<T, FilterFourati<Scalar>> ||
                                          std::is_same_v<T, EntoAttitudeExp::FilterFouratiChecked<Scalar>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(gain), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for Fourati fixed-point filters (gain parameter)  
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel, Scalar gain,
                 typename std::enable_if_t<std::is_same_v<T, FilterFouratiFixed<Scalar>> ||
                                          std::is_same_v<T, EntoAttitudeExp::FilterFouratiFixedChecked<Scalar>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(gain), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}

  // Constructor for fixed-point filters (already have internal state) - excludes FilterMadgwickFixed, FilterMahoney, FilterMahonyFixed, FilterFourati, FilterFouratiFixed and their checked variants
  template<typename T = Kernel>
  AttitudeProblem(Kernel kernel,
                 typename std::enable_if_t<!std::is_same_v<T, FilterMahoney<Scalar, UseMag>> && 
                                          !std::is_same_v<T, FilterMadgwick<Scalar, UseMag>> &&
                                          !std::is_same_v<T, FilterMadgwickFixed<Scalar, UseMag>> &&
                                          !std::is_same_v<T, FilterMahonyFixed<Scalar, UseMag>> &&
                                          !std::is_same_v<T, FilterFourati<Scalar>> &&
                                          !std::is_same_v<T, FilterFouratiFixed<Scalar>> &&
                                          !std::is_same_v<T, EntoAttitudeExp::FilterMadgwickChecked<Scalar, UseMag>> &&
                                          !std::is_same_v<T, EntoAttitudeExp::FilterMadgwickFixedChecked<Scalar, UseMag>> &&
                                          !std::is_same_v<T, EntoAttitudeExp::FilterMahoneyChecked<Scalar, UseMag>> &&
                                          !std::is_same_v<T, EntoAttitudeExp::FilterMahoneyFixedChecked<Scalar, UseMag>> &&
                                          !std::is_same_v<T, EntoAttitudeExp::FilterFouratiChecked<Scalar>> &&
                                          !std::is_same_v<T, EntoAttitudeExp::FilterFouratiFixedChecked<Scalar>>>* = nullptr)
    : kernel_(std::move(kernel)), kp_(Scalar(0.0f)), ki_(Scalar(0.0f)), gain_(Scalar(0.0f)), bias_(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f)) {}
};

#ifdef NATIVE
template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::deserialize_impl(const std::string &line)
{
  std::istringstream iss(line);
  Scalar ax, ay, az, gx, gy, gz, mx, my, mz;
  float qw, qx, qy, qz;  // Parse quaternion as float for ground truth
  float dt;
  
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
  
  q_gt_ = Eigen::Quaternion<float>(qw, qx, qy, qz);
  dt_ = Scalar(dt);  // Convert parsed float dt to Scalar type
  if (!filter_initialized_) {
    // Cast float ground truth to Scalar for filter initialization
    q_prev_ = Eigen::Quaternion<Scalar>(
      static_cast<Scalar>(q_gt_.w()),
      static_cast<Scalar>(q_gt_.x()),
      static_cast<Scalar>(q_gt_.y()),
      static_cast<Scalar>(q_gt_.z())
    );
    filter_initialized_ = true;
  }
  return true;
}
#endif
template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::deserialize_impl(const char* line)
{
  // Scale factors for different precision types to improve Q3.12 compatibility
  constexpr float GYR_SCALE_FACTOR = []() {
    // Never scale gyroscope - we have 4700x headroom in Q7.24 and excellent precision
    return 1.0f;
  }();
  
  constexpr float ACC_SCALE_FACTOR = []() {
    // Never scale accelerometer - we have 15x headroom in Q7.24, adequate for 8.4 m/s²
    return 1.0f;
  }();
  
  constexpr float MAG_SCALE_FACTOR = []() {
    if constexpr (std::is_same_v<Scalar, Q7_24>) {
      return 0.33f;    // Scale down ~3x for Q7.24: 35µT * 0.33 = 11.7µT, squared = 137 < 127 ✅
    } else {
      return 1.0f;     // No scaling for other types
    }
  }();
  
  // Use appropriate precision for quaternion and dt parsing based on Scalar type
  using ParseType = typename std::conditional_t<std::is_same_v<Scalar, double>, double, float>;
  
  Scalar ax, ay, az, gx, gy, gz, mx, my, mz;
  ParseType qw, qx, qy, qz, dt;
  
  int parsed_count = 0;
  
  if constexpr (UseMag) {
      // Parse 14 values for MARG: ax ay az gx gy gz mx my mz qw qx qy qz dt
      if constexpr (std::is_same_v<Scalar, float>) {
          parsed_count = sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
                                &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, 
                                &qw, &qx, &qy, &qz, &dt);
      } else if constexpr (std::is_same_v<Scalar, double>) {
          parsed_count = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                                &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, 
                                &qw, &qx, &qy, &qz, &dt);
      } else {
          // For fixed-point types, parse as float then convert
          float fax, fay, faz, fgx, fgy, fgz, fmx, fmy, fmz;
          float fqw, fqx, fqy, fqz, fdt;
          parsed_count = sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f",
                                &fax, &fay, &faz, &fgx, &fgy, &fgz, &fmx, &fmy, &fmz, 
                                &fqw, &fqx, &fqy, &fqz, &fdt);
          ax = Scalar(fax * ACC_SCALE_FACTOR); ay = Scalar(fay * ACC_SCALE_FACTOR); az = Scalar(faz * ACC_SCALE_FACTOR);
          gx = Scalar(fgx * GYR_SCALE_FACTOR); gy = Scalar(fgy * GYR_SCALE_FACTOR); gz = Scalar(fgz * GYR_SCALE_FACTOR);
          mx = Scalar(fmx * MAG_SCALE_FACTOR); my = Scalar(fmy * MAG_SCALE_FACTOR); mz = Scalar(fmz * MAG_SCALE_FACTOR);
          qw = fqw; qx = fqx; qy = fqy; qz = fqz;  // Cast to ParseType
          dt = fdt;  // Cast to ParseType
      }
      
      if (parsed_count != 14) {
          return false; // Failed to parse all 14 values
      }
      
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
      EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
      EntoMath::Vec3<Scalar>(ax, ay, az), // Accelerometer
      EntoMath::Vec3<Scalar>(mx, my, mz)  // Magnetometer
    );
  } else {
      // Parse 11 values for IMU: ax ay az gx gy gz qw qx qy qz dt
      if constexpr (std::is_same_v<Scalar, float>) {
          parsed_count = sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f",
                                &ax, &ay, &az, &gx, &gy, &gz, 
                                &qw, &qx, &qy, &qz, &dt);
      } else if constexpr (std::is_same_v<Scalar, double>) {
          parsed_count = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                                &ax, &ay, &az, &gx, &gy, &gz, 
                                &qw, &qx, &qy, &qz, &dt);
      } else {
          // For fixed-point types, parse as float then convert
          float fax, fay, faz, fgx, fgy, fgz;
          float fqw, fqx, fqy, fqz, fdt;
          parsed_count = sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f",
                                &fax, &fay, &faz, &fgx, &fgy, &fgz, 
                                &fqw, &fqx, &fqy, &fqz, &fdt);
          ax = Scalar(fax * ACC_SCALE_FACTOR); ay = Scalar(fay * ACC_SCALE_FACTOR); az = Scalar(faz * ACC_SCALE_FACTOR);
          gx = Scalar(fgx * GYR_SCALE_FACTOR); gy = Scalar(fgy * GYR_SCALE_FACTOR); gz = Scalar(fgz * GYR_SCALE_FACTOR);
          qw = fqw; qx = fqx; qy = fqy; qz = fqz;  // Cast to ParseType
          dt = fdt;  // Cast to ParseType
      }
      
      if (parsed_count != 11) {
          return false; // Failed to parse all 11 values
      }
      
      measurement_ = AttitudeMeasurement<Scalar, UseMag>(
        EntoMath::Vec3<Scalar>(gx, gy, gz), // Gyroscope
        EntoMath::Vec3<Scalar>(ax, ay, az)  // Accelerometer
    );
  }
  
  // Store ground truth with appropriate precision
  q_gt_ = Eigen::Quaternion<GroundTruthType>(qw, qx, qy, qz);
  dt_ = Scalar(dt);  // Convert parsed dt to Scalar type
  
  if (!filter_initialized_) {
    // Cast ground truth to Scalar for filter initialization
    q_prev_ = Eigen::Quaternion<Scalar>(
      static_cast<Scalar>(q_gt_.w()),
      static_cast<Scalar>(q_gt_.x()),
      static_cast<Scalar>(q_gt_.y()),
      static_cast<Scalar>(q_gt_.z())
    );
    filter_initialized_ = true;
  }
  return true;
}

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
    // Initialize filter with the first ground truth orientation (only once)
    // This represents knowing the robot's starting orientation
    //if (!filter_initialized_) {
    //    q_prev_ = q_gt_;
    //    filter_initialized_ = true;
    //}

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
    } else if constexpr (std::is_same_v<Kernel, FilterFourati<Scalar>>) {
        // Fourati filter: needs gain, g_q, m_q (MARG only)
        static_assert(UseMag, "Fourati filter requires magnetometer data (MARG)");
        // Convert AttitudeMeasurement to MARGMeasurement for Fourati
        MARGMeasurement<Scalar> marg_meas;
        marg_meas.gyr = measurement_.gyr;
        marg_meas.acc = measurement_.acc;
        marg_meas.mag = measurement_.mag;
        q_ = kernel_(q_prev_, marg_meas, dt_, gain_, g_q_, m_q_);
    } else if constexpr (std::is_same_v<Kernel, FilterFouratiFixed<Scalar>>) {
        // Fourati fixed-point filter: needs gain, g_q, m_q (MARG only, same interface as FilterFourati)
        static_assert(UseMag, "Fourati fixed-point filter requires magnetometer data (MARG)");
        // Convert AttitudeMeasurement to MARGMeasurement for Fourati
        MARGMeasurement<Scalar> marg_meas;
        marg_meas.gyr = measurement_.gyr;
        marg_meas.acc = measurement_.acc;
        marg_meas.mag = measurement_.mag;
        q_ = kernel_(q_prev_, marg_meas, dt_, gain_, g_q_, m_q_);
    } else if constexpr (std::is_same_v<Kernel, EntoAttitudeExp::FilterFouratiChecked<Scalar>>) {
        // Fourati checked filter: needs gain, g_q, m_q (MARG only, same interface as FilterFourati)
        static_assert(UseMag, "Fourati checked filter requires magnetometer data (MARG)");
        // Convert AttitudeMeasurement to MARGMeasurement for Fourati
        MARGMeasurement<Scalar> marg_meas;
        marg_meas.gyr = measurement_.gyr;
        marg_meas.acc = measurement_.acc;
        marg_meas.mag = measurement_.mag;
        q_ = kernel_(q_prev_, marg_meas, dt_, gain_, g_q_, m_q_);
    } else if constexpr (std::is_same_v<Kernel, EntoAttitudeExp::FilterFouratiFixedChecked<Scalar>>) {
        // Fourati fixed-point checked filter: needs gain, g_q, m_q (MARG only, same interface as FilterFourati)
        static_assert(UseMag, "Fourati fixed-point checked filter requires magnetometer data (MARG)");
        // Convert AttitudeMeasurement to MARGMeasurement for Fourati
        MARGMeasurement<Scalar> marg_meas;
        marg_meas.gyr = measurement_.gyr;
        marg_meas.acc = measurement_.acc;
        marg_meas.mag = measurement_.mag;
        q_ = kernel_(q_prev_, marg_meas, dt_, gain_, g_q_, m_q_);
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
    static const Eigen::Quaternion<GroundTruthType> zero_quat(GroundTruthType(0.0), GroundTruthType(0.0), GroundTruthType(0.0), GroundTruthType(0.0));

    // Convert q_ to appropriate computation type for comparison
    using ComputeType = typename std::conditional_t<std::is_same_v<GroundTruthType, double>, double, float>;
    Eigen::Quaternion<ComputeType> q_compute(
        static_cast<ComputeType>(q_.w()), 
        static_cast<ComputeType>(q_.x()), 
        static_cast<ComputeType>(q_.y()), 
        static_cast<ComputeType>(q_.z())
    );
    
    // Check if q_ is zero (algorithm completely failed)
    ComputeType q_norm = std::sqrt(q_compute.w() * q_compute.w() + q_compute.x() * q_compute.x() + 
                                  q_compute.y() * q_compute.y() + q_compute.z() * q_compute.z());
    if (q_norm < static_cast<ComputeType>(1e-6)) {
        // Quaternion is essentially zero - algorithm failed
        return false;
    }
    
    // Compute the quaternion angle distance (QAD) between estimated quaternion q_ and ground truth q_gt_
    
    // Compute the absolute value of the dot product of the quaternions
    // This gives the cosine of half the rotation angle between them
    ComputeType gt_w = static_cast<ComputeType>(q_gt_.w());
    ComputeType gt_x = static_cast<ComputeType>(q_gt_.x());
    ComputeType gt_y = static_cast<ComputeType>(q_gt_.y());
    ComputeType gt_z = static_cast<ComputeType>(q_gt_.z());
    
    ComputeType dot_product = std::abs(q_compute.w() * gt_w + q_compute.x() * gt_x + 
                                      q_compute.y() * gt_y + q_compute.z() * gt_z);
    
    // Clamp the dot product to [-1, 1] to handle numerical errors
    dot_product = std::min(static_cast<ComputeType>(1.0), std::max(static_cast<ComputeType>(-1.0), dot_product));
    
    // Calculate the quaternion angle distance in radians
    // QAD = 2 * arccos(|q1·q2|)
    ComputeType angle_distance = static_cast<ComputeType>(2.0) * std::acos(dot_product);
    
    // Convert to degrees for better interpretation
    ComputeType angle_distance_deg = angle_distance * static_cast<ComputeType>(180.0) / static_cast<ComputeType>(3.14159265359);
    
    // Validation threshold - algorithms should be accurate to within 1 degree
    const ComputeType ANGLE_THRESHOLD_DEG = static_cast<ComputeType>(1.0);
    
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
    
    // Reset ground truth quaternion with appropriate type
    q_gt_ = Eigen::Quaternion<GroundTruthType>::Identity();
    
    // Reset time step
    dt_ = Scalar(0.0f);
    
    // Reset bias (important for filter state)
    bias_ = EntoMath::Vec3<Scalar>(Scalar(0.0f), Scalar(0.0f), Scalar(0.0f));
    
    // Reset initialization flag
    // filter_initialized_ = false;
}





} // namespace EntoAttitude

#endif // ATTITUDE_PROBLEM_H
