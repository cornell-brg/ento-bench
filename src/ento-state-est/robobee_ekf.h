#include <Eigen/Dense>
#include <cmath>


template<typename Scalar>
struct RobobeeDynamicsModel
{
  // Constructor Inputs
  Scalar dt_;
  Scalar bw_;
  Scalar rw_;
  
  // Inertia Constants
  static constexpr Eigen::Matrix<Scalar, 3, 1> I_ =
    (Eigen::Matrix<Scalar, 3, 1>() << Scalar(1.42e-9), Scalar(1.34e-9), Scalar(4.5e-10)).finished();
  static constexpr Scalar Ixx_ = I_(0);
  static constexpr Scalar Iyy_ = I_(1);
  static constexpr Scalar Izz_ = I_(2);
  static constexpr Scalar Ixx_inv_ = Scalar(1) / Ixx_;
  static constexpr Scalar Iyy_inv_ = Scalar(1) / Iyy_;
  static constexpr Scalar Izz_inv_ = Scalar(1) / Izz_;

  Scalar m_;



  RobobeeDynamicsModel(Scalar dt = Scalar(4e-3),
                       Scalar bw = Scalar(2e-4),
                       Scalar rw = Scalar(9e-3),
                       Scalar m = Scalar(8.6e-5))
    : dt_(dt), bw_(bw), rw_(rw), m_(m) {}

  void operator()(Eigen::Matrix<Scalar, 10, 1>& x,
                  const Eigen::Matrix<Scalar, 4, 1>& u) const
  {
    Scalar phi   = x(0);
    Scalar theta = x(1);
    Scalar psi   = x(2);
    Scalar p     = x(3);
    Scalar q     = x(4);
    Scalar r     = x(5);
    Scalar z     = x(6);
    Scalar vx    = x(7);
    Scalar vy    = x(8);
    Scalar vz    = x(9);

    // Unpack control inputs
    Scalar Tx = u(0);
    Scalar Ty = u(1);
    Scalar Tz = u(2);
    Scalar F  = u(3);

    Scalar g = Scalar(9.8);

    // Compute intermediate values
    Scalar cos_psi   = std::cos(psi);
    Scalar sin_psi   = std::sin(psi);
    Scalar cos_theta = std::cos(theta);
    Scalar sin_theta = std::sin(theta);
    Scalar cos_phi   = std::cos(phi);
    Scalar sin_phi   = std::sin(phi);

    // Compute v and w (as in Python)
    Scalar v_val = vx * cos_psi * cos_theta +
                   vy * cos_theta * sin_psi -
                   vz * sin_theta;
    Scalar w_val = p * (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi)
                 + q * (sin_psi * sin_theta * sin_phi - cos_psi * cos_phi)
                 + r * cos_theta * sin_phi;

    // Compute fd and td
    Scalar fd = -bw_ * (rw_ * w_val + v_val);
    Scalar td = -rw_ * fd;

    // Compute F_total_world (3x1)
    Eigen::Matrix<Scalar, 3, 1> F_world;
    F_world(0) = cos_psi * cos_theta * fd +
                 (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi) * F;
    F_world(1) = sin_psi * cos_theta * fd +
                 (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * F;
    F_world(2) = -sin_theta * fd + cos_theta * cos_phi * F - m_ * g;

    // Compute tau_total_world (3x1)
    Eigen::Matrix<Scalar, 3, 1> tau_total;
    tau_total(0) = cos_psi * cos_theta * Tx +
                   (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi) * (Ty + td) +
                   (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi) * Tz;
    tau_total(1) = sin_psi * cos_theta * Tx +
                   (sin_psi * sin_theta * sin_phi - cos_psi * cos_phi) * (Ty + td) +
                   (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * Tz;
    tau_total(2) = -sin_theta * Tx +
                   cos_theta * sin_phi * (Ty + td) +
                   cos_theta * cos_phi * Tz;

    // Compute xdot (10x1) following Python code:
    Eigen::Matrix<Scalar, 10, 1> xdot;
    xdot(0) = p;
    xdot(1) = q;
    xdot(2) = r;
    xdot(3) = tau_total(0) / I_(0);
    xdot(4) = tau_total(1) / I_(1);
    xdot(5) = tau_total(2) / I_(2);
    xdot(6) = vz;
    xdot(7) = F_world(0) / m_;
    xdot(8) = F_world(1) / m_;
    xdot(9) = F_world(2) / m_;

    // Update state: x = x + dt * xdot
    x = x + dt_ * xdot;
  }
  
  void jacobian(Eigen::Matrix<Scalar, 10, 10>& A,
                const Eigen::Matrix<Scalar, 10, 1>& x,
                const Eigen::Matrix<Scalar, 4, 1>& u) const
  {
    Scalar phi   = x(0);
    Scalar theta = x(1);
    Scalar psi   = x(2);
    Scalar p     = x(3);
    Scalar q     = x(4);
    Scalar r     = x(5);
    Scalar z     = x(6);
    Scalar vx    = x(7);
    Scalar vy    = x(8);
    Scalar vz    = x(9);

    // Unpack control inputs
    Scalar Tx = u(0);
    Scalar Ty = u(1);
    Scalar Tz = u(2);
    Scalar F  = u(3);

    Scalar g = Scalar(9.8);

    // Compute intermediate values
    Scalar cos_psi   = std::cos(psi);
    Scalar sin_psi   = std::sin(psi);
    Scalar cos_theta = std::cos(theta);
    Scalar sin_theta = std::sin(theta);
    Scalar cos_phi   = std::cos(phi);
    Scalar sin_phi   = std::sin(phi);

    Scalar m_inv    = Scalar(1) / m_;

    A.setIdentity();

    Scalar v = vx*cos_psi*cos_theta + vy*cos_theta*cos_psi - vz*sin_theta;
    Scalar w = p*cos_psi*sin_theta*sin_phi - sin_psi*cos_phi + 
                   q*(sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) + r*cos_theta*sin_phi;

    Scalar fd      = -bw_(rw_*v+v);
    Scalar td      = -rw_*fd;
    Scalar dtd_dfd = -rw_;
    Scalar dfd_dw  = -rw_;
    Scalar dfd_dv  = -bw_;
    
    Scalar dv_dvx    = cos_psi * cos_theta;
    Scalar dv_dvy    = -sin_theta;
    Scalar dv_dvz    = cos_theta * sin_psi;
    Scalar dv_dtheta = -vx*cos_psi*sin_theta - vy*sin_theta*sin_psi - vz*cos_theta;
    Scalar dv_dpsi   = p*(-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) +
                          q*(cos_psi*sin_theta*sin_phi + sin_psi*cos_phi);

    Scalar dw_dp = cos_psi*sin_theta*sin_phi - sin_psi*cos_phi;
    Scalar dw_dq = sin_psi*sin_theta*sin_phi - cos_psi*cos_phi;
    Scalar dw_dr = cos_theta*sin_phi;
    Scalar dw_dtheta = p*(cos_psi*cos_theta*sin_phi) + q*(sin_psi*cos_theta*sin_phi) * r*sin_theta*sin_phi;
    Scalar dw_dpsi = p*(sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) + q*(cos_psi*sin_theta*sin_phi + sin_psi*cos_phi);

    A(0, 3) = 1;
    A(1, 4) = 1;
    A(2, 5) = 1;

    A(3, 0) = Ixx_inv_ * ((Ty + td) * (cos_psi*sin_theta*cos_phi + sin_psi*sin_phi)
                          + Tz*(cos_psi*sin_theta*cos_phi + sin_psi*sin_phi));
    A(3, 1) = Ixx_inv_ * (-Tx*cos_psi*sin_theta)
                + (cos_psi*sin_theta*sin_phi - sin_psi*cos_phi) * (dtd_dfd*(dfd_dw*dw_dtheta + dfd_dv*dv_dtheta))
                + (Ty + td)*(cos_psi*cos_theta*sin_phi) + (cos_psi*cos_theta*cos_phi*Tz);

    A(3, 2) = Ixx_inv_ * (-Tx*cos_psi*sin_theta
                + (cos_psi*sin_theta*sin_phi
                    - sin_psi*cos_phi)*(dtd_dfd*(dfd_dw*dw_dtheta + dfd_dv*dv_dtheta)) 
                + (Ty + td)*(cos_psi*cos_theta*sin_phi)
                + (cos_psi*cos_theta*cos_phi)*Tz);

    A(3, 2) = Ixx_inv_*(-sin_psi*cos_theta*Tx
                + (-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*(Ty + td)
                + (cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*(dtd_dfd*(dfd_dw*dw_dpsi + dfd_dv*dv_dpsi))
                + (-sin_psi*sin_theta*cos_phi + cos_psi*sin_phi)*Tz);
    A(3, 3) = Ixx_inv_*((cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*dtd_dfd*dfd_dw*dw_dp);
    A(3, 4) = Ixx_inv_*((cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*dtd_dfd*dfd_dw*dw_dq);
    A(3, 5) = Ixx_inv_*((cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*dtd_dfd*dfd_dw*dw_dr);

    A(3, 7) = Ixx_inv_*((cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*dtd_dfd*dfd_dv*dv_dvx);
    A(3, 8) = Ixx_inv_*((cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*dtd_dfd*dfd_dv*dv_dvy);
    A(3, 9) = Ixx_inv_*((cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*dtd_dfd*dfd_dv*dv_dvz);



    A(4, 0) = Iyy_inv_*((Ty + td)*(-sin_psi*sin_theta*cos_phi - cos_psi*sin_phi) 
                      + Tz*(-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi));
    A(4, 1) = Iyy_inv_*(sin_psi*-sin_theta*Tx 
                       + (sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*(dtd_dfd*(dfd_dw*dw_dtheta + dfd_dv*dv_dtheta)) 
                       + (Ty + td)*(sin_psi*cos_theta*sin_phi)
                       + (sin_psi*cos_theta*cos_phi)*Tz);
    A(4, 2) = Iyy_inv_*(cos_psi*cos_theta*Tx
                       + (cos_psi*sin_theta*sin_phi + sin_psi*cos_phi)*(Ty + td)
                       + (sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*(dtd_dfd*(dfd_dw*dw_dpsi + dfd_dv*dv_dpsi))
                       + (-cos_psi*sin_theta*cos_phi - sin_psi*sin_phi)*Tz);
    A(4, 3) = Iyy_inv_*((sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*dtd_dfd*dfd_dw*dw_dp);
    A(4, 4) = Iyy_inv_*((sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*dtd_dfd*dfd_dw*dw_dq);
    A(4, 5) = Iyy_inv_*((sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*dtd_dfd*dfd_dw*dw_dr);

    A(4, 7) = Iyy_inv_*((sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*dtd_dfd*dfd_dv*dv_dvx);
    A(4, 8) = Iyy_inv_*((sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*dtd_dfd*dfd_dv*dv_dvy);
    A(4, 9) = Iyy_inv_*((sin_psi*sin_theta*sin_phi - cos_psi*cos_phi)*dtd_dfd*dfd_dv*dv_dvz);


    A(5, 0) = Izz_inv_*((cos_theta*cos_phi)*(Ty + td)
                        - cos_theta*sin_phi*Tz);
    A(5, 1) = Izz_inv_*(-cos_theta*Tx 
                      + (sin_theta*sin_phi)*(Ty + td)
                      + (cos_theta*sin_phi)*(dtd_dfd*(dfd_dw*dw_dtheta + dfd_dv*dv_dtheta)) 
                      - sin_theta*cos_phi*Tz);
    A(5, 3) = Izz_inv_*((cos_theta*sin_phi)*dtd_dfd*dfd_dw*dw_dp);
    A(5, 4) = Izz_inv_*((cos_theta*sin_phi)*dtd_dfd*dfd_dw*dw_dq);
    A(5, 5) = Izz_inv_*((cos_theta*sin_phi)*dtd_dfd*dfd_dw*dw_dr);

    A(5, 7) = Izz_inv_*((cos_theta*sin_phi)*dtd_dfd*dfd_dv*dv_dvx);
    A(5, 8) = Izz_inv_*((cos_theta*sin_phi)*dtd_dfd*dfd_dv*dv_dvy);
    A(5, 9) = Izz_inv_*((cos_theta*sin_phi)*dtd_dfd*dfd_dv*dv_dvz);

    A(6, 9) = 1;


    A(7, 0) = m_inv*((cos_psi*sin_theta*-sin_phi 
                + sin_psi*cos_phi)*F);
    A(7, 1) = m_inv*(cos_psi*-sin_theta*fd
                + cos_psi*cos_theta*(dfd_dv*dv_dtheta + dfd_dw*dw_dtheta)
                + (cos_psi*cos_theta*cos_phi)*F);
    A(7, 2) = m_inv*(-sin_psi*cos_theta*fd 
                + cos_psi*cos_theta*(dfd_dv*dv_dpsi + dfd_dw*dw_dpsi)
                + (-sin_psi*sin_theta*cos_phi + cos_psi*sin_phi)*F);
    A(7, 3) = m_inv*cos_psi*cos_theta*dfd_dw*dw_dp;
    A(7, 4) = m_inv*cos_psi*cos_theta*dfd_dw*dw_dq;
    A(7, 5) = m_inv*cos_psi*cos_theta*dfd_dw*dw_dr;

    A(7, 7) = m_inv*cos_psi*cos_theta*dfd_dv*dv_dvx;
    A(7, 8) = m_inv*cos_psi*cos_theta*dfd_dv*dv_dvy;
    A(7, 9) = m_inv*cos_psi*cos_theta*dfd_dv*dv_dvz;

    A(8, 0) = m_inv*((cos_psi*sin_theta*-sin_phi 
                + sin_psi*cos_phi)*F);
    A(8, 1) = m_inv*(sin_psi*-sin_theta*fd
                + sin_psi*cos_theta*(dfd_dv*dv_dtheta + dfd_dw*dw_dtheta)
                + (sin_psi*cos_theta*cos_phi - cos_psi*sin_phi)*F);
    A(8, 2) = m_inv*(cos_psi*cos_theta*fd
                + sin_psi*cos_theta*(dfd_dv*dv_dpsi + dfd_dw*dw_dpsi)
                + (cos_psi*sin_theta*cos_phi + sin_psi*sin_phi)*F);
    A(8, 3) = m_inv*sin_psi*cos_theta*dfd_dw*dw_dp;
    A(8, 4) = m_inv*sin_psi*cos_theta*dfd_dw*dw_dq;
    A(8, 5) = m_inv*sin_psi*cos_theta*dfd_dw*dw_dr;

    A(8, 7) = m_inv*sin_psi*cos_theta*dfd_dv*dv_dvx;
    A(8, 8) = m_inv*sin_psi*cos_theta*dfd_dv*dv_dvy;
    A(8, 9) = m_inv*sin_psi*cos_theta*dfd_dv*dv_dvz;

    A(9, 0) = m_inv * (cos_theta*-sin_phi*F);
    A(9, 1) = m_inv * (-cos_theta*fd 
                + -sin_theta*(dfd_dv*dv_dtheta + dfd_dw*dw_dtheta)
                - sin_theta*cos_phi*F);
    A(9, 2) = m_inv * (-sin_theta*(dfd_dv*dv_dpsi + dfd_dw*dw_dpsi));
    A(9, 3) = m_inv*-sin_theta*dfd_dw*dw_dp;
    A(9, 4) = m_inv*-sin_theta*dfd_dw*dw_dq;
    A(9, 5) = m_inv*-sin_theta*dfd_dw*dw_dr;

    A(9, 7) = m_inv*-sin_theta*dfd_dv*dv_dvx;
    A(9, 8) = m_inv*-sin_theta*dfd_dv*dv_dvy;
    A(9, 9) = m_inv*-sin_theta*dfd_dv*dv_dvz;
  }
};

template <typename Scalar>
struct RobobeeMeasurementModel
{
  void operator()(const Eigen::Matrix<Scalar, 10, 1>& x,
                  [[maybe_unused]] const Eigen::Matrix<Scalar, 4, 1>& /* u */,
                  Eigen::Matrix<Scalar, 4, 1>& pred) const
  {
    pred(0) = x(0);
    pred(1) = x(1);
    pred(2) = x(2);
    pred(3) = x(6);
  }

  constexpr void jacobian(Eigen::Matrix<Scalar, 4, 10>& H,
                          [[maybe_unused]] const Eigen::Matrix<Scalar, 10, 1>& /* x */)
  {
    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;
    H(3, 6) = 1;
  }
};

