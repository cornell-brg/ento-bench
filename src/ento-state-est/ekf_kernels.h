#ifndef EKF_KERNELS_H
#define EKF_KERNELS_H

#include <ento-state-est/ekf.h>
#include <ento-state-est/robofly_ekf_v1.h>
#include <ento-state-est/robobee_ekf.h>

namespace EntoStateEst
{

/// @brief Complete EKF kernel for RoboFly combining EKF + dynamics + measurement models
template<typename Scalar>
class RoboFlyEKFKernel
{
public:
  using Scalar_ = Scalar;
  static constexpr size_t StateDim_ = 4;
  static constexpr size_t MeasurementDim_ = 4;
  static constexpr size_t ControlDim_ = 1;
  
  using DynamicsModel = RoboFlyV1DynamicsModel<Scalar>;
  using MeasurementModel = RoboFlyV1MeasurementModel<Scalar>;
  using EKFType = EKF<Scalar, StateDim_, MeasurementDim_, ControlDim_>;
  
private:
  EKFType ekf_;
  DynamicsModel dynamics_;
  MeasurementModel measurement_;
  
public:
  /// @brief Constructor with noise matrices
  /// @param Q Process noise covariance (4x4)
  /// @param R Measurement noise covariance (4x4)
  RoboFlyEKFKernel(const Eigen::Matrix<Scalar, StateDim_, StateDim_>& Q,
                   const Eigen::Matrix<Scalar, MeasurementDim_, MeasurementDim_>& R)
    : ekf_(Q, R), dynamics_(), measurement_()
  {}
  
  // Forward EKF interface methods
  void predict(const DynamicsModel& dynamics, const Eigen::Matrix<Scalar, ControlDim_, 1>& u) {
    ekf_.predict(dynamics, u);
  }
  
  void update(const MeasurementModel& measurement, const Eigen::Matrix<Scalar, MeasurementDim_, 1>& z) {
    ekf_.update(measurement, z);
  }
  
  void update_sequential(const MeasurementModel& measurement,
                        const Eigen::Matrix<Scalar, MeasurementDim_, 1>& z,
                        const Eigen::Matrix<Scalar, ControlDim_, 1>& u,
                        std::array<bool, MeasurementDim_>& sensor_mask) {
    ekf_.update_sequential(measurement, z, u, sensor_mask);
  }
  
  void update_truncated(const MeasurementModel& measurement,
                       const Eigen::Matrix<Scalar, MeasurementDim_, 1>& z,
                       const Eigen::Matrix<Scalar, ControlDim_, 1>& u,
                       std::array<bool, MeasurementDim_>& sensor_mask) {
    ekf_.update_truncated(measurement, z, u, sensor_mask);
  }
  
  Eigen::Matrix<Scalar, StateDim_, 1> getState() const {
    return ekf_.getState();
  }
  
  Eigen::Matrix<Scalar, StateDim_, StateDim_> getCovariance() const {
    return ekf_.getCovariance();
  }
  
  void setState(const Eigen::Matrix<Scalar, StateDim_, 1>& x) {
    ekf_.set_state(x);
  }
  
  void setCovariance(const Eigen::Matrix<Scalar, StateDim_, StateDim_>& P) {
    ekf_.set_covariance(P);
  }
  
  // Access to noise matrices (for debugging)
  const Eigen::Matrix<Scalar, StateDim_, StateDim_>& getQ() const {
    return ekf_.getQ();
  }
  
  const Eigen::Matrix<Scalar, MeasurementDim_, MeasurementDim_>& getR() const {
    return ekf_.getR();
  }
  
  // Access to underlying models (for EKFProblem)
  const DynamicsModel& getDynamicsModel() const { return dynamics_; }
  const MeasurementModel& getMeasurementModel() const { return measurement_; }
  DynamicsModel& getDynamicsModel() { return dynamics_; }
  MeasurementModel& getMeasurementModel() { return measurement_; }
};

/// @brief Complete EKF kernel for RoboBee
template<typename Scalar>
class RoboBeeEKFKernel
{
public:
  using Scalar_ = Scalar;
  static constexpr size_t StateDim_ = 10;
  static constexpr size_t MeasurementDim_ = 4;  
  static constexpr size_t ControlDim_ = 4;
  
  using DynamicsModel = RobobeeDynamicsModel<Scalar>;
  using MeasurementModel = RobobeeMeasurementModel<Scalar>;
  using EKFType = EKF<Scalar, StateDim_, MeasurementDim_, ControlDim_>;
  
private:
  EKFType ekf_;
  DynamicsModel dynamics_;
  MeasurementModel measurement_;
  
public:
  /// @brief Constructor with noise matrices
  /// @param Q Process noise covariance (10x10)
  /// @param R Measurement noise covariance (4x4)
  RoboBeeEKFKernel(const Eigen::Matrix<Scalar, StateDim_, StateDim_>& Q,
                   const Eigen::Matrix<Scalar, MeasurementDim_, MeasurementDim_>& R)
    : ekf_(Q, R), dynamics_(), measurement_()
  {}
  
  // Forward EKF interface methods
  void predict(const DynamicsModel& dynamics, const Eigen::Matrix<Scalar, ControlDim_, 1>& u) {
    ekf_.predict(dynamics, u);
  }
  
  void update(const MeasurementModel& measurement, const Eigen::Matrix<Scalar, MeasurementDim_, 1>& z) {
    ekf_.update(measurement, z);
  }
  
  void update_sequential(const MeasurementModel& measurement,
                        const Eigen::Matrix<Scalar, MeasurementDim_, 1>& z,
                        const Eigen::Matrix<Scalar, ControlDim_, 1>& u,
                        std::array<bool, MeasurementDim_>& sensor_mask) {
    ekf_.update_sequential(measurement, z, u, sensor_mask);
  }
  
  void update_truncated(const MeasurementModel& measurement,
                       const Eigen::Matrix<Scalar, MeasurementDim_, 1>& z,
                       const Eigen::Matrix<Scalar, ControlDim_, 1>& u,
                       std::array<bool, MeasurementDim_>& sensor_mask) {
    ekf_.update_truncated(measurement, z, u, sensor_mask);
  }
  
  Eigen::Matrix<Scalar, StateDim_, 1> getState() const {
    return ekf_.getState();
  }
  
  Eigen::Matrix<Scalar, StateDim_, StateDim_> getCovariance() const {
    return ekf_.getCovariance();
  }
  
  void setState(const Eigen::Matrix<Scalar, StateDim_, 1>& x) {
    ekf_.set_state(x);
  }
  
  void setCovariance(const Eigen::Matrix<Scalar, StateDim_, StateDim_>& P) {
    ekf_.set_covariance(P);
  }
  
  // Access to noise matrices (for debugging)
  const Eigen::Matrix<Scalar, StateDim_, StateDim_>& getQ() const {
    return ekf_.getQ();
  }
  
  const Eigen::Matrix<Scalar, MeasurementDim_, MeasurementDim_>& getR() const {
    return ekf_.getR();
  }
  
  // Access to underlying models (for EKFProblem)
  const DynamicsModel& getDynamicsModel() const { return dynamics_; }
  const MeasurementModel& getMeasurementModel() const { return measurement_; }
  DynamicsModel& getDynamicsModel() { return dynamics_; }
  MeasurementModel& getMeasurementModel() { return measurement_; }
};

/// @brief Convenience type aliases for the dynamics and measurement models
template<typename Scalar>
using RoboflyDynamicsModel = RoboFlyV1DynamicsModel<Scalar>;

template<typename Scalar>
using RoboflyMeasurementModel = RoboFlyV1MeasurementModel<Scalar>;

/// @brief Convenience type aliases for complete EKF problems
template<typename Scalar>
using RoboFlyEKFProblem = EKFProblem<RoboFlyEKFKernel<Scalar>, 
                                     RoboFlyV1DynamicsModel<Scalar>, 
                                     RoboFlyV1MeasurementModel<Scalar>>;

template<typename Scalar>
using RoboBeeEKFProblem = EKFProblem<RoboBeeEKFKernel<Scalar>,
                                     RobobeeDynamicsModel<Scalar>, 
                                     RobobeeMeasurementModel<Scalar>>;

} // namespace EntoStateEst

#endif // EKF_KERNELS_H 