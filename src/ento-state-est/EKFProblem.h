#ifndef EKF_PROBLEM_H
#define EKF_PROBLEM_H

#include <Eigen/Dense>
#include <ento-bench/problem.h>
#include <ento-state-est/ekf.h>
#include <ento-util/containers.h>
#include <array>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdlib>

#ifdef NATIVE
#include <iostream>
#include <fstream>
#endif

namespace EntoStateEst
{
  
/// @brief Data structure representing a single timestep of EKF data
template <typename Scalar, size_t MeasurementDim, size_t ControlDim>
struct EKFDataPoint
{
  Scalar timestamp;
  Scalar dt;
  Eigen::Matrix<Scalar, MeasurementDim, 1> measurements;
  Eigen::Matrix<Scalar, ControlDim, 1> controls;
  std::array<bool, MeasurementDim> sensor_mask;
  
  EKFDataPoint() : timestamp(0), dt(0), sensor_mask{} {
    measurements.setZero();
    controls.setZero();
    sensor_mask.fill(true); // Default: all sensors available
  }
};

/// @brief EKF Problem class templated on the EKF kernel type
/// @tparam EKFKernel The EKF implementation (e.g., RoboFly, RoboBee) 
/// @tparam DynamicsModel The dynamics model for the EKF
/// @tparam MeasurementModel The measurement model for the EKF
/// @tparam MaxTrajectoryPoints Maximum trajectory points to store (0 = unlimited on native)
template <typename EKFKernel, typename DynamicsModel, typename MeasurementModel, size_t MaxTrajectoryPoints = 0>
class EKFProblem : public EntoBench::EntoProblem<EKFProblem<EKFKernel, DynamicsModel, MeasurementModel, MaxTrajectoryPoints>>
{
public:
  // Type aliases from the EKF kernel
  using Scalar = typename EKFKernel::Scalar_;
  static constexpr size_t StateDim = EKFKernel::StateDim_;
  static constexpr size_t MeasurementDim = EKFKernel::MeasurementDim_;
  static constexpr size_t ControlDim = EKFKernel::ControlDim_;
  
  using DataPoint = EKFDataPoint<Scalar, MeasurementDim, ControlDim>;
  using StateVector = Eigen::Matrix<Scalar, StateDim, 1>;
  using MeasurementVector = Eigen::Matrix<Scalar, MeasurementDim, 1>;
  using CovarianceMatrix = Eigen::Matrix<Scalar, StateDim, StateDim>;
  using ControlVector = Eigen::Matrix<Scalar, ControlDim, 1>;

  // EntoBench interface requirements
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = false;
  static constexpr bool RequiresSetup_ = false;

  // Configuration
  enum class UpdateMethod {
    SYNCHRONOUS,
    SEQUENTIAL, 
    TRUNCATED
  };

private:
  // EKF components
  EKFKernel ekf_;
  DynamicsModel dynamics_model_;
  MeasurementModel measurement_model_;
  
  // Data storage
  EntoUtil::EntoContainer<StateVector, MaxTrajectoryPoints> state_trajectory_;
  EntoUtil::EntoContainer<CovarianceMatrix, MaxTrajectoryPoints> covariance_trajectory_;
  EntoUtil::EntoContainer<Scalar, MaxTrajectoryPoints> timestamps_;
  
  // Real-time processing state
  bool has_current_data_;
  DataPoint current_data_point_;
  
  // Initial conditions
  StateVector initial_state_;
  CovarianceMatrix initial_covariance_;
  
  // Configuration
  UpdateMethod update_method_;

public:
  /// @brief Constructor
  /// @param ekf The EKF instance with proper Q and R matrices
  /// @param dynamics The dynamics model
  /// @param measurement The measurement model
  /// @param update_method Which update method to use
  EKFProblem(EKFKernel ekf, 
             DynamicsModel dynamics, 
             MeasurementModel measurement,
             UpdateMethod method = UpdateMethod::SYNCHRONOUS)
    : ekf_(std::move(ekf))
    , dynamics_model_(std::move(dynamics))
    , measurement_model_(std::move(measurement))
    , has_current_data_(false)
    , update_method_(method)
  {
    initial_state_.setZero();
    initial_covariance_.setIdentity();
  }

  /// @brief Set initial state and covariance
  void setInitialConditions(const StateVector& x0, const CovarianceMatrix& P0) {
    initial_state_ = x0;
    initial_covariance_ = P0;
  }

  /// @brief Get the current state estimate
  const StateVector& getCurrentState() const { 
    return ekf_.getState(); 
  }
  
  /// @brief Get the current covariance matrix
  const CovarianceMatrix& getCurrentCovariance() const { 
    return ekf_.getCovariance(); 
  }

  /// @brief Get the full state trajectory after solving
  const EntoUtil::EntoContainer<StateVector, MaxTrajectoryPoints>& getStateTrajectory() const { 
    return state_trajectory_; 
  }

  /// @brief Get the full covariance trajectory after solving  
  const EntoUtil::EntoContainer<CovarianceMatrix, MaxTrajectoryPoints>& getCovarianceTrajectory() const { 
    return covariance_trajectory_; 
  }

  /// @brief Get timestamps for each step
  const EntoUtil::EntoContainer<Scalar, MaxTrajectoryPoints>& getTimestamps() const { 
    return timestamps_; 
  }

  /// @brief Get initial state (for testing/debugging)
  const StateVector& getInitialState() const { return initial_state_; }
  
  /// @brief Get initial covariance (for testing/debugging)  
  const CovarianceMatrix& getInitialCovariance() const { return initial_covariance_; }
  
  /// @brief Get reference to EKF kernel (for debugging/testing)
  EKFKernel& getKernel() { return ekf_; }
  const EKFKernel& getKernel() const { return ekf_; }
  
  /// @brief Get reference to dynamics model (for debugging/testing)
  DynamicsModel& getDynamicsModel() { return dynamics_model_; }
  const DynamicsModel& getDynamicsModel() const { return dynamics_model_; }
  
  /// @brief Get reference to measurement model (for debugging/testing)
  MeasurementModel& getMeasurementModel() { return measurement_model_; }
  const MeasurementModel& getMeasurementModel() const { return measurement_model_; }

  //////// EntoBench Problem Interface ////////

#ifdef NATIVE
  std::string serialize_impl() const {
    std::ostringstream oss;
    
    // Header with metadata
    oss << "# EKF Problem Data\n";
    oss << "# StateDim=" << StateDim << ", MeasurementDim=" << MeasurementDim 
        << ", ControlDim=" << ControlDim << "\n";
    oss << "# Format: timestamp,dt,measurements...,controls...,sensor_mask\n";
    oss << "# Initial State: ";
    for (size_t i = 0; i < StateDim; ++i) {
      oss << initial_state_(i);
      if (i < StateDim - 1) oss << ",";
    }
    oss << "\n";
    oss << "# Initial Covariance: ";
    for (size_t i = 0; i < StateDim; ++i) {
      for (size_t j = 0; j < StateDim; ++j) {
        oss << initial_covariance_(i, j);
        if (i < StateDim - 1 || j < StateDim - 1) oss << ",";
      }
    }
    oss << "\n";
    
    // For trajectory data, serialize the results
    for (size_t i = 0; i < state_trajectory_.size(); ++i) {
      oss << timestamps_[i];
      for (size_t j = 0; j < StateDim; ++j) {
        oss << "," << state_trajectory_[i](j);
      }
      oss << "\n";
    }
    
    return oss.str();
  }

  bool deserialize_impl(const std::string& data) {
    std::istringstream iss(data);
    std::string line;
    
    // Clear previous results
    state_trajectory_.clear();
    covariance_trajectory_.clear(); 
    timestamps_.clear();
    has_current_data_ = false;
    
    // Initialize EKF with initial conditions
    ekf_.setState(initial_state_);
    ekf_.setCovariance(initial_covariance_);
    
    while (std::getline(iss, line)) {
      // Skip comments and empty lines
      if (line.empty() || line[0] == '#') {
        // Parse initial conditions from comments
        if (line.find("# Initial State:") == 0) {
          parseInitialState(line.substr(16));
        } else if (line.find("# Initial Covariance:") == 0) {
          parseInitialCovariance(line.substr(21));
        }
        continue;
      }
      
      // Process each data line immediately
      DataPoint point;
      if (parseDataLine(line, point)) {
        processDataPoint(point);
      }
    }
    
    return state_trajectory_.size() > 0;
  }
#else
  const char* serialize_impl() const {
    // For MCU: simplified format or pre-allocated buffer
    static char buffer[4096]; // Adjust size as needed
    // Implement compact serialization for MCU
    return buffer;
}
#endif

  bool deserialize_impl(const char* line) {
    ENTO_DEBUG("EKF deserialize_impl called with line: %.50s", line);
    
    // Initialize EKF on first call
    if (!has_current_data_ && state_trajectory_.empty()) {
      ekf_.setState(initial_state_);
      ekf_.setCovariance(initial_covariance_);
      ENTO_DEBUG("EKF initialized with initial conditions");
    }
    
    // Parse single CSV line and process immediately (MCU path)
    DataPoint point;
    
    if (!parseDataPointFromCString(line, point)) {
      ENTO_DEBUG("EKF parsing failed for line: %.50s", line);
      return false;
    }
    
    ENTO_DEBUG("EKF parsed successfully: t=%.3f, dt=%.3f", point.timestamp, point.dt);
    
    // Process the data point immediately
    processDataPoint(point);
    has_current_data_ = true;
    current_data_point_ = point;
    
    ENTO_DEBUG("EKF data point processed successfully");
    return true;
  }

  bool validate_impl() const {
    // For real-time processing: validate we have current data
    // For batch processing: validate trajectory is non-empty
    return has_current_data_ || state_trajectory_.size() > 0;
  }

  void solve_impl() {
    // For single-point processing, data is already processed in deserialize_impl
    // This method is mainly for batch processing or when explicitly called
    if (has_current_data_) {
      // Process current data point if not already processed
      processDataPoint(current_data_point_);
    }
  }

  void clear_impl() {
    state_trajectory_.clear();
    covariance_trajectory_.clear();
    timestamps_.clear();
    has_current_data_ = false;
  }

  static constexpr const char* header_impl() { 
    return "EKF Problem"; 
  }
  
  static constexpr const char* output_header_impl() { 
    return "timestamp,x0,x1,x2,x3,cov_trace"; // Adjust based on StateDim
  }

private:
  /// @brief Process a single data point through the EKF
  void processDataPoint(const DataPoint& point) {
    // Prediction step
    ekf_.predict(dynamics_model_, point.controls);
    
    // Update step (depends on selected method)
    switch (update_method_) {
      case UpdateMethod::SYNCHRONOUS:
        ekf_.update(measurement_model_, point.measurements);
        break;
        
      case UpdateMethod::SEQUENTIAL: {
        std::array<bool, MeasurementDim> mask = point.sensor_mask;
        ekf_.update_sequential(measurement_model_, point.measurements, point.controls, mask);
        break;
      }
      
      case UpdateMethod::TRUNCATED: {
        std::array<bool, MeasurementDim> mask = point.sensor_mask;
        ekf_.update_truncated(measurement_model_, point.measurements, point.controls, mask);
        break;
      }
    }
    
    // Store results (if there's space)
    if constexpr (MaxTrajectoryPoints == 0) {
      // Unlimited storage on native
      state_trajectory_.push_back(ekf_.getState());
      covariance_trajectory_.push_back(ekf_.getCovariance());
      timestamps_.push_back(point.timestamp);
    } else {
      // Limited storage on MCU
      if (!state_trajectory_.full()) {
        state_trajectory_.push_back(ekf_.getState());
        covariance_trajectory_.push_back(ekf_.getCovariance());
        timestamps_.push_back(point.timestamp);
      }
    }
  }

  /// @brief Parse data point from C string (MCU-compatible)
  bool parseDataPointFromCString(const char* line, DataPoint& point) {
    const char* pos = line;
    char* end_pos;
    
    // Skip empty lines
    if (!line || strlen(line) == 0) {
      ENTO_DEBUG("EKF parse: empty line");
      return false;
    }
    
    ENTO_DEBUG("EKF parse: starting with line length %zu", strlen(line));
    
    // Parse timestamp
    point.timestamp = strtod(pos, &end_pos);
    if (end_pos == pos || *end_pos != ',') {
      ENTO_DEBUG("EKF parse: timestamp failed at pos %zu", pos - line);
      return false;
    }
    pos = end_pos + 1;
    ENTO_DEBUG("EKF parse: timestamp=%.3f", point.timestamp);
    
    // Parse dt
    point.dt = strtod(pos, &end_pos);
    if (end_pos == pos || *end_pos != ',') {
      ENTO_DEBUG("EKF parse: dt failed at pos %zu", pos - line);
      return false;
    }
    pos = end_pos + 1;
    ENTO_DEBUG("EKF parse: dt=%.3f", point.dt);
    
    // Parse measurements
    for (size_t i = 0; i < MeasurementDim; ++i) {
      double val = strtod(pos, &end_pos);
      if (end_pos == pos || (i < MeasurementDim - 1 && *end_pos != ',')) {
        ENTO_DEBUG("EKF parse: measurement[%zu] failed at pos %zu", i, pos - line);
        return false;
      }
      point.measurements(i) = static_cast<Scalar>(val);
      if (i < MeasurementDim - 1) pos = end_pos + 1;
      else {
        pos = end_pos;
        if (*pos == ',') pos++; // Move past comma if present
      }
    }
    ENTO_DEBUG("EKF parse: measurements done");
    
    // Parse controls
    for (size_t i = 0; i < ControlDim; ++i) {
      double val = strtod(pos, &end_pos);
      if (end_pos == pos || (i < ControlDim - 1 && *end_pos != ',')) {
        ENTO_DEBUG("EKF parse: control[%zu] failed at pos %zu", i, pos - line);
        return false;
      }
      point.controls(i) = static_cast<Scalar>(val);
      if (i < ControlDim - 1) pos = end_pos + 1;
      else {
        pos = end_pos;
        if (*pos == ',') pos++; // Move past comma if present  
      }
    }
    ENTO_DEBUG("EKF parse: controls done");
    
    // Parse sensor mask
    for (size_t i = 0; i < MeasurementDim; ++i) {
      long val = strtol(pos, &end_pos, 10);
      if (end_pos == pos || (val != 0 && val != 1)) {
        ENTO_DEBUG("EKF parse: sensor_mask[%zu] failed at pos %zu, val=%ld", i, pos - line, val);
        return false;
      }
      point.sensor_mask[i] = (val == 1);
      
      if (i < MeasurementDim - 1) {
        if (*end_pos != ',') {
          ENTO_DEBUG("EKF parse: sensor_mask[%zu] missing comma at pos %zu", i, end_pos - line);
          return false;
        }
        pos = end_pos + 1;
      } else {
        // Last element - should be end of line or end of string
        pos = end_pos;
      }
    }
    
    ENTO_DEBUG("EKF parse: all fields parsed successfully");
    return true;
  }

  /// @brief Parse initial state from comment line
  void parseInitialState(const std::string& data) {
    std::istringstream iss(data);
    std::string token;
    size_t idx = 0;
    
    while (std::getline(iss, token, ',') && idx < StateDim) {
      initial_state_(idx++) = std::stod(token);
    }
  }
  
  /// @brief Parse initial covariance from comment line
  void parseInitialCovariance(const std::string& data) {
    std::istringstream iss(data);
    std::string token;
    size_t idx = 0;
    
    while (std::getline(iss, token, ',') && idx < StateDim * StateDim) {
      size_t i = idx / StateDim;
      size_t j = idx % StateDim;
      initial_covariance_(i, j) = std::stod(token);
      idx++;
    }
  }

  /// @brief Parse a single data line into a DataPoint (native version)
  bool parseDataLine(const std::string& line, DataPoint& point) {
    std::istringstream iss(line);
    std::string token;
    size_t field = 0;
    
    while (std::getline(iss, token, ',')) {
      if (field == 0) {
        point.timestamp = std::stod(token);
      } else if (field == 1) {
        point.dt = std::stod(token);
      } else if (field >= 2 && field < 2 + MeasurementDim) {
        point.measurements(field - 2) = std::stod(token);
      } else if (field >= 2 + MeasurementDim && field < 2 + MeasurementDim + ControlDim) {
        point.controls(field - 2 - MeasurementDim) = std::stod(token);
      } else if (field >= 2 + MeasurementDim + ControlDim && field < 2 + MeasurementDim + ControlDim + MeasurementDim) {
        // Parse individual sensor mask fields
        size_t mask_idx = field - 2 - MeasurementDim - ControlDim;
        point.sensor_mask[mask_idx] = (std::stoi(token) == 1);
      }
      field++;
    }
    
    return field >= 2 + MeasurementDim + ControlDim + MeasurementDim; // All required fields
  }
};

/// @brief Convenience type aliases for common EKF problems
// Note: These would need the actual EKF kernel types once they're defined
// template<typename Scalar>
// using RoboFlyEKFProblem = EKFProblem<RoboFlyEKF<Scalar>, RoboFlyDynamics<Scalar>, RoboFlyMeasurement<Scalar>>;

// template<typename Scalar>  
// using RoboBeeEKFProblem = EKFProblem<RoboBeeEKF<Scalar>, RoboBeeDynamics<Scalar>, RoboBeeMeasurement<Scalar>>;

} // namespace EntoStateEst

#endif // EKF_PROBLEM_H
