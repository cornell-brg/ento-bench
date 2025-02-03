#include <Eigen/Dense>
#include <iostream>
#include <cmath>

// Define the Scalar type (float or double)
template<typename Scalar>
class RobobeeCEKF {
public:
  // Typedefs for convenience
  using Vector10 = Eigen::Matrix<Scalar, 10, 1>;
  using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
  using Matrix10 = Eigen::Matrix<Scalar, 10, 10>;
  using Matrix4x10 = Eigen::Matrix<Scalar, 4, 10>;
  using Matrix10x4 = Eigen::Matrix<Scalar, 10, 4>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

  // Member variables
  Matrix4x10 H;
  Matrix10 Q;
  Matrix4 R;
  Vector10 x;
  Matrix10 P;

  Scalar dt;
  Scalar bw;
  Scalar rw;

  Eigen::Matrix<Scalar, 3, 1> I; // Inertia vector
  Scalar m;

  // Constructor
  RobobeeCEKF()
  {
      // Initialize H
      H.setZero();
      H(0, 0) = Scalar(1);
      H(1, 1) = Scalar(1);
      H(2, 2) = Scalar(1);
      H(3, 6) = Scalar(1);

      // Initialize Q
      Q = Matrix10::Identity();

      // Initialize R
      R.setZero();
      R(0, 0) = Scalar(0.07);
      R(1, 1) = Scalar(0.07);
      R(2, 2) = Scalar(0.07);
      R(3, 3) = Scalar(0.002);

      // Initialize x
      x.setZero();

      // Initialize P
      P = (Scalar(M_PI) / 2) * Matrix10::Identity();

      dt = Scalar(4e-3);
      bw = Scalar(2e-4);
      rw = Scalar(9e-3);

      I(0) = Scalar(1.42e-9);
      I(1) = Scalar(1.34e-9);
      I(2) = Scalar(4.5e-10);

      m = Scalar(8.6e-5);
  }

  // Update function
  Vector4 update(const Vector4 &z, const Vector4 &u, Scalar delta_t = Scalar(0)) {
      if (delta_t != Scalar(0)) {
          dt = delta_t;
      }

      Matrix10 A = jacobian(x, u);
      Vector10 xp = fx(x, u);
      Matrix10 Pp = A * P * A.transpose() + Q;

      Matrix4 G = H * Pp * H.transpose() + R;

      Matrix10x4 K = Pp * H.transpose() * G.inverse();

      Vector4 e = z - H * xp;

      x = xp + K * e;
      P = Pp - K * H * Pp;

      return H * x;
  }

  // State transition function
  Vector10 fx(const Vector10 &xhat, const Vector4 &u) {
      // Unpack xhat
      Scalar phi = xhat(0);
      Scalar theta = xhat(1);
      Scalar psi = xhat(2);

      Scalar p = xhat(3);
      Scalar q = xhat(4);
      Scalar r = xhat(5);

      Scalar z = xhat(6);

      Scalar vx = xhat(7);
      Scalar vy = xhat(8);
      Scalar vz = xhat(9);

      Scalar Ixx = I(0);
      Scalar Iyy = I(1);
      Scalar Izz = I(2);

      Scalar F = u(3);
      Scalar Tx = u(0);
      Scalar Ty = u(1);
      Scalar Tz = u(2);

      Vector10 xdot;
      xdot.setZero();

      Scalar g = Scalar(9.8);

      Scalar v = vx * cos(psi) * cos(theta) + vy * cos(theta) * sin(psi) - vz * sin(theta);
      Scalar w = p * (cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)) +
                 q * (sin(psi) * sin(theta) * sin(phi) - cos(psi) * cos(phi)) +
                 r * cos(theta) * sin(phi);

      Scalar fd = -bw * (rw * w + v);
      Scalar td = -rw * fd;

      // Forces in world coordinates
      Scalar F_total_world_x = cos(psi) * cos(theta) * fd +
                               (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * F;
      Scalar F_total_world_y = sin(psi) * cos(theta) * fd +
                               (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) * F;
      Scalar F_total_world_z = -sin(theta) * fd + cos(theta) * cos(phi) * F - m * g;

      // Torques in world coordinates
      Scalar tau_total_world_x = cos(psi) * cos(theta) * Tx +
                                 (cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)) * (Ty + td) +
                                 (cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)) * Tz;

      Scalar tau_total_world_y = sin(psi) * cos(theta) * Tx +
                                 (sin(psi) * sin(theta) * sin(phi) - cos(psi) * cos(phi)) * (Ty + td) +
                                 (sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) * Tz;

      Scalar tau_total_world_z = -sin(theta) * Tx +
                                 (cos(theta) * sin(phi)) * (Ty + td) +
                                 cos(theta) * cos(phi) * Tz;

      xdot(0) = p;
      xdot(1) = q;
      xdot(2) = r;
      xdot(3) = (1 / Ixx) * tau_total_world_x;
      xdot(4) = (1 / Iyy) * tau_total_world_y;
      xdot(5) = (1 / Izz) * tau_total_world_z;
      xdot(6) = vz;

      xdot(7) = F_total_world_x / m;
      xdot(8) = F_total_world_y / m;
      xdot(9) = F_total_world_z / m;

      return xhat + xdot * dt;
  }

  // Jacobian function
  Matrix10 jacobian(const Vector10 &xhat, const Vector4 &u) {
      Matrix10 A;
      A.setZero();

      // Unpack variables (similar to fx function)
      Scalar phi = xhat(0);
      Scalar theta = xhat(1);
      Scalar psi = xhat(2);

      Scalar p = xhat(3);
      Scalar q = xhat(4);
      Scalar r = xhat(5);

      Scalar vx = xhat(7);
      Scalar vy = xhat(8);
      Scalar vz = xhat(9);

      Scalar Ixx = I(0);
      Scalar Iyy = I(1);
      Scalar Izz = I(2);

      Scalar Tx = u(0);
      Scalar Ty = u(1);
      Scalar Tz = u(2);
      Scalar F = u(3);

  }
};
