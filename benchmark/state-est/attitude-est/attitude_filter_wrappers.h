#ifndef ATTITUDE_FILTER_WRAPPERS_H
#define ATTITUDE_FILTER_WRAPPERS_H

#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/mahoney_fixed.h>
#include <ento-state-est/attitude-est/madgwick.h>
#include <ento-state-est/attitude-est/madgwick_fixed.h>

namespace EntoAttitude {

// =============================================================================
// Wrapper Functors for AttitudeProblem Interface (4-parameter)
// =============================================================================

// Mahony Float Wrapper
template<typename Scalar, bool UseMag>
class FilterMahonyWrapper {
public:
  FilterMahonyWrapper(Scalar kp = Scalar(1.0), Scalar ki = Scalar(0.1)) 
    : kp_(kp), ki_(ki), bias_(Scalar(0), Scalar(0), Scalar(0)) {}

  Eigen::Quaternion<Scalar> operator()(const Eigen::Quaternion<Scalar>& q_prev,
                                      const AttitudeMeasurement<Scalar, UseMag>& meas,
                                      Scalar dt,
                                      Eigen::Quaternion<Scalar>* q_out) {
    if constexpr (UseMag) {
      *q_out = EntoAttitude::mahony_update_marg(q_prev, meas.gyr, meas.acc, meas.mag, dt, kp_, ki_, bias_);
    } else {
      *q_out = EntoAttitude::mahony_update_imu(q_prev, meas.gyr, meas.acc, dt, kp_, ki_, bias_);
    }
    return *q_out;
  }

  static constexpr const char *name() {
    if constexpr (UseMag)
      return "Mahony MARG Float";
    else
      return "Mahony IMU Float";
  }

private:
  Scalar kp_, ki_;
  Eigen::Matrix<Scalar, 3, 1> bias_;
};

// Madgwick Float Wrapper
template<typename Scalar, bool UseMag>
class FilterMadgwickWrapper {
public:
  FilterMadgwickWrapper(Scalar gain = Scalar(0.1)) : gain_(gain) {}

  Eigen::Quaternion<Scalar> operator()(const Eigen::Quaternion<Scalar>& q_prev,
                                      const AttitudeMeasurement<Scalar, UseMag>& meas,
                                      Scalar dt,
                                      Eigen::Quaternion<Scalar>* q_out) {
    if constexpr (UseMag) {
      *q_out = EntoAttitude::madgwick_update_marg(q_prev, meas.gyr, meas.acc, meas.mag, dt, gain_);
    } else {
      *q_out = EntoAttitude::madgwick_update_imu(q_prev, meas.gyr, meas.acc, dt, gain_);
    }
    return *q_out;
  }

  static constexpr const char *name() {
    if constexpr (UseMag)
      return "Madgwick MARG Float";
    else
      return "Madgwick IMU Float";
  }

private:
  Scalar gain_;
};

// Madgwick Fixed-Point Wrapper
template<typename Scalar, bool UseMag>
class FilterMadgwickFixedWrapper {
public:
  FilterMadgwickFixedWrapper(Scalar gain = Scalar(0.1f)) : gain_(gain) {}

  Eigen::Quaternion<Scalar> operator()(const Eigen::Quaternion<Scalar>& q_prev,
                                      const AttitudeMeasurement<Scalar, UseMag>& meas,
                                      Scalar dt,
                                      Eigen::Quaternion<Scalar>* q_out) {
    if constexpr (UseMag) {
      *q_out = EntoAttitude::madgwick_update_marg_fixed(q_prev, meas.gyr, meas.acc, meas.mag, dt, gain_);
    } else {
      *q_out = EntoAttitude::madgwick_update_imu_fixed(q_prev, meas.gyr, meas.acc, dt, gain_);
    }
    return *q_out;
  }

  static constexpr const char *name() {
    if constexpr (UseMag)
      return "Madgwick MARG Fixed";
    else
      return "Madgwick IMU Fixed";
  }

private:
  Scalar gain_;
};

} // namespace EntoAttitude

#endif // ATTITUDE_FILTER_WRAPPERS_H 