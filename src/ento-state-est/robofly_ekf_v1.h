#include <Eigen/Dense>
#include <cmath>
#include <iostream>

template <typename Scalar>
struct RoboFlyV1DynamicsModel
{
  void operator()(Eigen::Matrix<Scalar, 4, 1>& x,
                  Eigen::Matrix<Scalar, 1, 1>& u,
                  Scalar dt)
  {
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

    x += dt * f_c;
  }
  constexpr void jacobian(Eigen::Matrix<Scalar, 4, 4>& A,
                          [[maybe_unused]] Eigen::Matrix<Scalar, 4, 1>& x,
                          [[maybe_unused]] Eigen::Matrix<Scalar, 1, 1>& u)
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

    pred(0) = z / cos(theta);
    pred(1) = (cos_theta / z) * (vx * cos_theta + vz * sin_theta) - omega;
    pred(2) = -g_ * sin_theta;
    pred(3) = g_ * cos_theta;
  }

  void jacobian(Eigen::Matrix<Scalar, 4, 4>& H,
                Eigen::Matrix<Scalar, 4, 1>& x)
  {
    H.setZero();
    H(0, 2) = 1;
    H(1, 1) = Scalar(1) / x[2];
    H(2, 0) = -g_;
  }
};
