#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <ento-util/debug.h>

template <typename Scalar>
struct RoboFlyV1DynamicsModel
{
  void operator()(Eigen::Matrix<Scalar, 4, 1>& x,
                  Eigen::Matrix<Scalar, 1, 1>& u,
                  Scalar dt)
  {
    ENTO_DEBUG("=== DYNAMICS MODEL CALL ===");
    ENTO_DEBUG("Input state x: [%.6f, %.6f, %.6f, %.6f]", x(0), x(1), x(2), x(3));
    ENTO_DEBUG("Input control u: [%.6f]", u(0));
    ENTO_DEBUG("Input dt: %.6f", dt);
    
    Scalar theta = x(0);
    Scalar vx    = x(1);
    Scalar z     = x(2);
    Scalar vz    = x(3);

    Scalar omega = u(0);

    Eigen::Matrix<Scalar, 4, 1> f_c;
    f_c(0) = omega;
    f_c(1) = Scalar(0);
    f_c(2) = vz;
    f_c(3) = Scalar(0);

    ENTO_DEBUG("Dynamics vector f_c: [%.6f, %.6f, %.6f, %.6f]", f_c(0), f_c(1), f_c(2), f_c(3));
    ENTO_DEBUG("dt * f_c: [%.6f, %.6f, %.6f, %.6f]", dt*f_c(0), dt*f_c(1), dt*f_c(2), dt*f_c(3));

    x += dt * f_c;
    
    ENTO_DEBUG("Output state x: [%.6f, %.6f, %.6f, %.6f]", x(0), x(1), x(2), x(3));
    ENTO_DEBUG("=== DYNAMICS MODEL END ===");
  }
  constexpr void jacobian(Eigen::Matrix<Scalar, 4, 4>& A,
                          [[maybe_unused]] const Eigen::Matrix<Scalar, 4, 1>& x,
                          [[maybe_unused]] const Eigen::Matrix<Scalar, 1, 1>& u) const
  {
    A.setZero();
    A(2, 3) = Scalar(1);
  }
};

template <typename Scalar>
struct RoboFlyV1MeasurementModel
{
  static constexpr Scalar g_ = 9.81;

  inline void operator()(const Eigen::Matrix<Scalar, 4, 1>& x,
                         const Eigen::Matrix<Scalar, 1, 1>& u,
                               Eigen::Matrix<Scalar, 4, 1>& pred) const
  {
    Scalar theta = x(0);
    Scalar vx    = x(1);
    Scalar z     = x(2);
    Scalar vz    = x(3);

    Scalar omega = u(0);

    // Pre-compute cos/sin theta
    Scalar cos_theta = std::cos(theta);
    Scalar sin_theta = std::sin(theta);

    pred(0) = z / cos_theta;
    pred(1) = (cos_theta / z) * (vx * cos_theta + vz * sin_theta) - omega;
    pred(2) = -g_ * sin_theta;
    pred(3) = g_ * cos_theta;
  }

  void jacobian(Eigen::Matrix<Scalar, 4, 4>& H,
                const Eigen::Matrix<Scalar, 4, 1>& x) const
  {
    Scalar theta = x(0);
    Scalar vx    = x(1);
    Scalar z     = x(2);
    Scalar vz    = x(3);
    
    // Pre-compute cos/sin theta
    Scalar cos_theta = std::cos(theta);
    Scalar sin_theta = std::sin(theta);
    
    H.setZero();
    
    // ∂(range)/∂theta = z * sin(theta) / cos²(theta)
    H(0, 0) = z * sin_theta / (cos_theta * cos_theta);
    // ∂(range)/∂z = 1 / cos(theta)
    H(0, 2) = Scalar(1) / cos_theta;
    
    // ∂(optical_flow)/∂theta: complex partial derivative
    H(1, 0) = (Scalar(1) / z) * (vz * cos_theta - vx * sin_theta - (vx * cos_theta + vz * sin_theta) * sin_theta / cos_theta);
    // ∂(optical_flow)/∂vx = cos(theta) / z
    H(1, 1) = cos_theta / z;
    // ∂(optical_flow)/∂z = -(cos_theta / z²) * (vx * cos_theta + vz * sin_theta)
    H(1, 2) = -(cos_theta / (z * z)) * (vx * cos_theta + vz * sin_theta);
    // ∂(optical_flow)/∂vz = sin(theta) / z
    H(1, 3) = sin_theta / z;
    
    // ∂(accel_x)/∂theta = -g * cos(theta)
    H(2, 0) = -g_ * cos_theta;
    
    // ∂(accel_z)/∂theta = -g * sin(theta)  
    H(3, 0) = -g_ * sin_theta;
  }
};
