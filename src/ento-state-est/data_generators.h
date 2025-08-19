#ifndef EKF_DATA_GENERATORS_H
#define EKF_DATA_GENERATORS_H

#include <vector>
#include <random>
#include <cmath>
#include <ento-state-est/EKFProblem.h>

namespace EntoStateEst
{

/// @brief Synthetic data generator for RoboFly EKF
template<typename Scalar>
class RoboFlyDataGenerator
{
public:
  using DataPoint = EKFDataPoint<Scalar, 4, 1>; // 4 measurements, 1 control
  
private:
  std::mt19937 rng_;
  std::normal_distribution<Scalar> noise_dist_;
  
public:
  RoboFlyDataGenerator(uint32_t seed = 42) 
    : rng_(seed), noise_dist_(0.0, 1.0) {}
  
  /// @brief Generate a sequence of RoboFly sensor data
  /// @param duration Total simulation time (seconds)
  /// @param dt Time step (seconds) 
  /// @param noise_levels Noise standard deviations for each sensor
  /// @return Vector of data points in proper format
  std::vector<DataPoint> generateTrajectory(Scalar duration, 
                                           Scalar dt,
                                           const std::array<Scalar, 4>& noise_levels) {
    std::vector<DataPoint> data;
    size_t num_steps = static_cast<size_t>(duration / dt);
    data.reserve(num_steps);
    
    // Simulate simple hover + pitch maneuver
    Scalar time = 0.0;
    Scalar true_theta = 0.0;      // True pitch angle
    Scalar true_vx = 0.0;         // True x-velocity
    Scalar true_z = 1.0;          // True altitude (1m hover)
    Scalar true_vz = 0.0;         // True z-velocity
    Scalar omega_cmd = 0.0;       // Angular velocity command
    
    for (size_t step = 0; step < num_steps; ++step) {
      DataPoint point;
      point.timestamp = time;
      point.dt = dt;
      
      // Simple sinusoidal pitch maneuver
      Scalar pitch_amplitude = 0.1; // 0.1 radian amplitude
      Scalar pitch_freq = 0.5;      // 0.5 Hz
      true_theta = pitch_amplitude * std::sin(2 * M_PI * pitch_freq * time);
      omega_cmd = pitch_amplitude * 2 * M_PI * pitch_freq * std::cos(2 * M_PI * pitch_freq * time);
      
      // Generate synthetic measurements with noise:
      // z_meas[0] = range (ToF sensor) = z/cos(theta) + noise
      // z_meas[1] = optical flow x-velocity + noise  
      // z_meas[2] = accelerometer x = -g*sin(theta) + noise
      // z_meas[3] = accelerometer z = g*cos(theta) + noise
      
      Scalar g = 9.81;
      point.measurements(0) = true_z / std::cos(true_theta) + noise_levels[0] * noise_dist_(rng_);
      point.measurements(1) = true_vx + noise_levels[1] * noise_dist_(rng_); // Simplified flow
      point.measurements(2) = -g * std::sin(true_theta) + noise_levels[2] * noise_dist_(rng_);
      point.measurements(3) = g * std::cos(true_theta) + noise_levels[3] * noise_dist_(rng_);
      
      // Control input (angular velocity command)
      point.controls(0) = omega_cmd;
      
      // All sensors available by default
      point.sensor_mask.fill(true);
      
      // Randomly drop some sensor measurements (10% dropout rate)
      for (size_t i = 0; i < 4; ++i) {
        if (std::uniform_real_distribution<Scalar>(0, 1)(rng_) < 0.1) {
          point.sensor_mask[i] = false;
        }
      }
      
      data.push_back(point);
      time += dt;
    }
    
    return data;
  }
  
  /// @brief Generate data formatted for Crazyflie optical flow deck
  /// @param cf_log_data Raw Crazyflie log data (timestamp, flow.deltaX, flow.deltaY, range.zrange, acc.x, acc.z, gyro.z)
  /// @return Converted data points
  std::vector<DataPoint> convertCrazyflieData(const std::vector<std::vector<Scalar>>& cf_log_data) {
    std::vector<DataPoint> data;
    data.reserve(cf_log_data.size());
    
    for (size_t i = 0; i < cf_log_data.size(); ++i) {
      if (cf_log_data[i].size() < 7) continue; // Need at least 7 fields
      
      DataPoint point;
      point.timestamp = cf_log_data[i][0];
      point.dt = (i > 0) ? (cf_log_data[i][0] - cf_log_data[i-1][0]) : 0.01; // Default 10ms
      
      // Map Crazyflie data to RoboFly measurements:
      // point.measurements(0) = range.zrange (ToF distance)
      // point.measurements(1) = flow.deltaX (optical flow x-velocity) 
      // point.measurements(2) = acc.x (accelerometer x-component)
      // point.measurements(3) = acc.z (accelerometer z-component)
      point.measurements(0) = cf_log_data[i][3]; // range.zrange
      point.measurements(1) = cf_log_data[i][1]; // flow.deltaX
      point.measurements(2) = cf_log_data[i][4]; // acc.x
      point.measurements(3) = cf_log_data[i][5]; // acc.z
      
      // Control input from gyro command
      point.controls(0) = cf_log_data[i][6]; // gyro.z
      
      // Check sensor validity (Crazyflie sets invalid readings to specific values)
      point.sensor_mask[0] = (cf_log_data[i][3] > 0.02 && cf_log_data[i][3] < 2.0); // Valid range
      point.sensor_mask[1] = true; // Flow usually always valid
      point.sensor_mask[2] = true; // Accelerometer usually always valid  
      point.sensor_mask[3] = true; // Accelerometer usually always valid
      
      data.push_back(point);
    }
    
    return data;
  }
};

/// @brief Synthetic data generator for RoboBee EKF  
template<typename Scalar>
class RoboBeeDataGenerator
{
public:
  using DataPoint = EKFDataPoint<Scalar, 4, 4>; // 4 measurements, 4 controls
  
private:
  std::mt19937 rng_;
  std::normal_distribution<Scalar> noise_dist_;
  
public:
  RoboBeeDataGenerator(uint32_t seed = 42)
    : rng_(seed), noise_dist_(0.0, 1.0) {}
  
  /// @brief Generate a sequence of RoboBee sensor data
  /// @param duration Total simulation time (seconds)
  /// @param dt Time step (seconds)
  /// @param noise_levels Noise standard deviations for each sensor  
  /// @return Vector of data points in proper format
  std::vector<DataPoint> generateTrajectory(Scalar duration,
                                           Scalar dt, 
                                           const std::array<Scalar, 4>& noise_levels) {
    std::vector<DataPoint> data;
    size_t num_steps = static_cast<size_t>(duration / dt);
    data.reserve(num_steps);
    
    // Simulate simple attitude control + altitude hold
    Scalar time = 0.0;
    Scalar true_phi = 0.0;    // True roll angle
    Scalar true_theta = 0.0;  // True pitch angle  
    Scalar true_psi = 0.0;    // True yaw angle
    Scalar true_z = 0.5;      // True altitude (0.5m hover)
    
    for (size_t step = 0; step < num_steps; ++step) {
      DataPoint point;
      point.timestamp = time;
      point.dt = dt;
      
      // Simple sinusoidal attitude motion
      Scalar attitude_amp = 0.05; // 0.05 radian amplitude
      Scalar attitude_freq = 1.0;  // 1 Hz
      true_phi = attitude_amp * std::sin(2 * M_PI * attitude_freq * time);
      true_theta = attitude_amp * std::cos(2 * M_PI * attitude_freq * time);
      true_psi += 0.01 * dt; // Slow yaw drift
      
      // Generate synthetic measurements with noise:
      // z_meas[0] = roll angle (phi) + noise
      // z_meas[1] = pitch angle (theta) + noise
      // z_meas[2] = yaw angle (psi) + noise  
      // z_meas[3] = altitude (z) + noise
      point.measurements(0) = true_phi + noise_levels[0] * noise_dist_(rng_);
      point.measurements(1) = true_theta + noise_levels[1] * noise_dist_(rng_);
      point.measurements(2) = true_psi + noise_levels[2] * noise_dist_(rng_);
      point.measurements(3) = true_z + noise_levels[3] * noise_dist_(rng_);
      
      // Control inputs (torques + thrust)
      // Simple PD control to maintain attitude and altitude
      point.controls(0) = -10.0 * true_phi;  // Roll torque (Tx)
      point.controls(1) = -10.0 * true_theta; // Pitch torque (Ty)  
      point.controls(2) = -5.0 * (true_psi - 0.0); // Yaw torque (Tz)
      point.controls(3) = 8.6e-5 * 9.8; // Thrust to counteract gravity (F)
      
      // All sensors available by default
      point.sensor_mask.fill(true);
      
      // Randomly drop some sensor measurements (5% dropout rate)
      for (size_t i = 0; i < 4; ++i) {
        if (std::uniform_real_distribution<Scalar>(0, 1)(rng_) < 0.05) {
          point.sensor_mask[i] = false;
        }
      }
      
      data.push_back(point);
      time += dt;
    }
    
    return data;
  }
};

/// @brief Example data format strings for reference
namespace DataFormats 
{
  /// RoboFly CSV format:
  /// timestamp,dt,range_tof,flow_x,accel_x,accel_z,omega_cmd,sensor_mask
  constexpr const char* ROBOFLY_FORMAT = 
    "# RoboFly EKF Data Format\n"
    "# StateDim=4, MeasurementDim=4, ControlDim=1\n" 
    "# Format: timestamp,dt,range_tof,flow_x,accel_x,accel_z,omega_cmd,sensor_mask\n"
    "# State: [theta, vx, z, vz]\n"
    "# Measurements: [range, flow_x, accel_x, accel_z]\n"
    "# Controls: [omega_cmd]\n";
    
  /// RoboBee CSV format:
  /// timestamp,dt,roll,pitch,yaw,altitude,Tx,Ty,Tz,F,sensor_mask  
  constexpr const char* ROBOBEE_FORMAT =
    "# RoboBee EKF Data Format\n"
    "# StateDim=10, MeasurementDim=4, ControlDim=4\n"
    "# Format: timestamp,dt,roll,pitch,yaw,altitude,Tx,Ty,Tz,F,sensor_mask\n"
    "# State: [phi, theta, psi, p, q, r, z, vx, vy, vz]\n"
    "# Measurements: [phi, theta, psi, z]\n" 
    "# Controls: [Tx, Ty, Tz, F]\n";
}

} // namespace EntoStateEst

#endif // EKF_DATA_GENERATORS_H 