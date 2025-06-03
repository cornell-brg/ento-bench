#ifndef POSE_EST_UTIL_HH
#define POSE_EST_UTIL_HH

#include <Eigen/Dense>
#include <ento-math/core.h>
#include <ento-math/quaternion.h>
#include <ento-util/containers.h>
#include <ento-pose/camera_models.h>
#include <ento-util/debug.h>

using namespace EntoMath;
using namespace EntoUtil;

namespace EntoPose
{

template <typename Scalar>
struct BundleStats {
    size_t iterations = 0;
    Scalar initial_cost;
    Scalar cost;
    Scalar lambda;
    size_t invalid_steps;
    Scalar step_norm;
    Scalar grad_norm;
};

template <typename Scalar>
struct BundleOptions {
    size_t max_iterations = 100;
    enum LossType {
        TRIVIAL,
        TRUNCATED,
        HUBER,
        CAUCHY,
        // This is the TR-IRLS scheme from Le and Zach, 3DV 2021
        TRUNCATED_LE_ZACH
    } loss_type = LossType::CAUCHY;
    Scalar loss_scale = 1.0;
    Scalar gradient_tol = 1e-10;
    Scalar step_tol = 1e-8;
    Scalar initial_lambda = 1e-3;
    Scalar min_lambda = 1e-10;
    Scalar max_lambda = 1e10;
    bool verbose = false;
};


template <typename Scalar>
using Point2D = Vec2<Scalar>;

template <typename Scalar>
using Point3D = Vec3<Scalar>;

template <typename Scalar>
struct Line2D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Line2D() {}
    Line2D(const Vec2<Scalar> &e1,
           const Vec2<Scalar> &e2) : x1(e1), x2(e2) {}
    Vec2<Scalar> x1, x2;
};

template <typename Scalar>
struct Line3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Line3D() {}
    Line3D(const Vec3<Scalar> &e1,
           const Vec3<Scalar> &e2) : X1(e1), X2(e2) {}
    Vec3<Scalar> X1, X2;
};

template <typename Scalar>
struct DistortionCoeffs
{
  Scalar k1;
  Scalar k2;
  Scalar p1;
  Scalar p2;
  Scalar k3;
};

template <typename Scalar>
struct CameraPose
{
  // Rotation is represented as a unit quaternion
  // with real part first, i.e. QW, QX, QY, QZ
  Vec4<Scalar> q;
  Vec3<Scalar> t;

  // Constructors (Defaults to identity camera)
  CameraPose() : q(1.0, 0.0, 0.0, 0.0), t(0.0, 0.0, 0.0) {}
  CameraPose(const Eigen::Vector4d &qq, const Vec3<Scalar> &tt) : q(qq), t(tt) {}
  CameraPose(const Matrix3x3<Scalar> &R, const Vec3<Scalar> &tt) : q(rotmat_to_quat<Scalar>(R)), t(tt) {}

  // Helper functions
  inline Matrix3x3<Scalar> R() const { return quat_to_rotmat<Scalar>(q); }
  inline Eigen::Matrix<Scalar, 3, 4> Rt() const {
      Eigen::Matrix<Scalar, 3, 4> tmp;
      tmp.block<3, 3>(0, 0) = quat_to_rotmat<Scalar>(q);
      tmp.col(3) = t;
      return tmp;
  }
  inline Vec3<Scalar> rotate(const Vec3<Scalar> &p) const { return quat_rotate<Scalar>(q, p); }
  inline Vec3<Scalar> derotate(const Vec3<Scalar> &p) const { return quat_rotate<Scalar>(quat_conj(q), p); }
  inline Vec3<Scalar> apply(const Vec3<Scalar> &p) const { return rotate(p) + t; }

  inline Vec3<Scalar> center() const { return -derotate(t); }

  // Extract rotation and translation from a projection matrix
  inline void from_projection(const Eigen::Matrix<Scalar, 3, 4>& P) {
    // Extract rotation matrix from first 3x3 block
    Matrix3x3<Scalar> R = P.template block<3, 3>(0, 0);
    
    // Normalize the rotation matrix to ensure it's orthogonal
    Eigen::JacobiSVD<Matrix3x3<Scalar>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    
    // Convert rotation matrix to quaternion
    q = rotmat_to_quat<Scalar>(R);
    
    // Extract translation vector
    t = P.col(3);
  }
};

template <typename Scalar, typename CameraModel>
struct alignas(32) ImagePair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Struct simply holds information about two cameras and their relative pose
  CameraPose<Scalar> pose;
  Camera<Scalar, CameraModel> camera1;
  Camera<Scalar, CameraModel> camera2;

  // Constructors (Defaults to identity camera and poses)
  ImagePair() : pose(CameraPose<Scalar>()),
                camera1(Camera<Scalar, CameraModel>()),
                camera2(Camera<Scalar, CameraModel>()) {}
  ImagePair(CameraPose<Scalar> pose, Camera<Scalar, CameraModel> camera1, Camera<Scalar, CameraModel> camera2)
      : pose(std::move(pose)), camera1(std::move(camera1)), camera2(std::move(camera2)) {}
};

// =======================================================
// Normalization/Unnormalization
template <typename Derived>
bool normalize_points(const Eigen::DenseBase<Derived>& P,
                      const int& num_samples,
                      Eigen::DenseBase<Derived>* Q,
                      Eigen::DenseBase<Derived>* T);


template <typename Scalar, int Order=0>
void unnormalize_homography(Eigen::Matrix<Scalar, 3, 3, Order>* H ,
                            const Eigen::Matrix<Scalar, 3, 3, Order>& T1,
                            const Eigen::Matrix<Scalar, 3, 3, Order>& T2);
// =======================================================

// =======================================================
// Undistortion Points
template <typename Scalar, int MaxM, int Order=0>
void undistort_points(const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxM, 3>& P,
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxM, 3>* Q,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const DistortionCoeffs<Scalar>& dist);

template <typename Derived, typename Scalar>
void undistort_points(const Eigen::DenseBase<Derived>& P,
                      Eigen::DenseBase<Derived>* Q,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const DistortionCoeffs<Scalar>& dist);
// =======================================================


// =======================================================
// Homography Decomposition
template <typename Scalar, int Order=0>
void decompose_homography(Scalar* h,
                          Eigen::Matrix<Scalar, 3, 3, Order>* R,
                          Eigen::Matrix<Scalar, 3, 1, Order>* t);

template <typename Scalar, int Order=0>
void decompose_homography(Eigen::Matrix<Scalar, 3, 3, Order>* H,
                          Eigen::Matrix<Scalar, 3, 3, Order>* R,
                          Eigen::Matrix<Scalar, 3, 1, Order>* t);
// =======================================================

template <typename Scalar>
void solve_quadratic(Scalar a,
                     Scalar b,
                     Scalar c,
                     std::complex<Scalar> roots[2]) {
  std::complex<Scalar> b2m4ac = b * b - 4 * a * c;
  std::complex<Scalar> sq = std::sqrt(b2m4ac);

  roots[0] = (b > Scalar(0)) ? (2 * c) / (-b - sq) : (2 * c) / (-b + sq);
  roots[1] = c / (a * roots[0]);
}

template <typename Scalar>
int solve_quadratic_real(Scalar a,
                         Scalar b,
                         Scalar c,
                         Scalar roots[2])
{
  Scalar b2m4ac = b * b - 4 * a * c;
  if (b2m4ac < Scalar(0))
  {
    return 0;
  }

  Scalar sq = std::sqrt(b2m4ac);
  roots[0] = (b > Scalar(0)) ? (2 * c) / (-b - sq) : (2 * c) / (-b + sq);
  roots[1] = c / (a * roots[0]);

  return 2;
}

template <typename Scalar>
bool solve_cubic_single_real(Scalar c2,
                             Scalar c1,
                             Scalar c0,
                             Scalar &root)
{
  Scalar a = c1 - c2 * c2 / Scalar(3);
  Scalar b = (2 * c2 * c2 * c2 - Scalar(9) * c2 * c1) / Scalar(27) + c0;
  Scalar c = b * b / Scalar(4) + a * a * a / Scalar(27);

  if (c > Scalar(0))
  {
    c = std::sqrt(c);
    b *= Scalar(-0.5);
    root = std::cbrt(b + c) + std::cbrt(b - c) - c2 / Scalar(3);
    return true;
  }
  else
  {
    c = Scalar(3) * b / (Scalar(2) * a) * std::sqrt(-Scalar(3) / a);
    root = Scalar(2) * std::sqrt(-a / Scalar(3)) * std::cos(std::acos(c) / Scalar(3)) - c2 / Scalar(3);
  }

  return false;
}

template <typename Scalar>
int solve_cubic_real(Scalar c2,
                     Scalar c1,
                     Scalar c0,
                     Scalar roots[3]) {
  Scalar a = c1 - c2 * c2 / Scalar(3);
  Scalar b = (Scalar(2) * c2 * c2 * c2 - Scalar(9) * c2 * c1) / Scalar(27) + c0;
  Scalar c = b * b / Scalar(4) + a * a * a / Scalar(27);

  int n_roots;
  if (c > Scalar(0))
  {
    c = std::sqrt(c);
    b *= Scalar(-0.5);
    roots[0] = std::cbrt(b + c) + std::cbrt(b - c) - c2 / Scalar(3);
    n_roots = 1;
  }
  else
  {
    c = Scalar(3) * b / (Scalar(2) * a) * std::sqrt(-Scalar(3) / a);
    Scalar d = Scalar(2) * std::sqrt(-a / Scalar(3));
    roots[0] = d * std::cos(std::acos(c) / Scalar(3)) - c2 / Scalar(3);
    roots[1] = d * std::cos(std::acos(c) / Scalar(3) - Scalar(2.094395102393195)) - c2 / Scalar(3); // 2*pi/3
    roots[2] = d * std::cos(std::acos(c) / Scalar(3) - Scalar(4.188790204786390)) - c2 / Scalar(3); // 4*pi/3
    n_roots = 3;
  }

  // Single Newton iteration
  for (int i = 0; i < n_roots; ++i)
  {
    Scalar x = roots[i];
    Scalar x2 = x * x;
    Scalar x3 = x * x2;
    Scalar dx = -(x3 + c2 * x2 + c1 * x + c0) / (Scalar(3) * x2 + Scalar(2) * c2 * x + c1);
    roots[i] += dx;
  }

  return n_roots;
}


template <typename Scalar> 
inline bool root2real(Scalar b,
                      Scalar c,
                      Scalar &r1,
                      Scalar &r2)
{
  Scalar THRESHOLD = Scalar(-1.0e-12);
  Scalar v = b * b - 4.0 * c;
  if (v < THRESHOLD)
  {
    r1 = r2 = Scalar(-0.5) * b;
    return v >= 0;
  }
  if (v > THRESHOLD && v < Scalar(0.0))
  {
    r1 = Scalar(-0.5) * b;
    r2 = Scalar(-2);
    return true;
  }

  Scalar y = std::sqrt(v);
  if (b < 0)
  {
    r1 = Scalar(0.5) * (-b + y);
    r2 = Scalar(0.5) * (-b - y);
  }
  else
  {
    r1 = Scalar(2.0) * c / (-b + y);
    r2 = Scalar(2.0) * c / (-b - y);
  }
  return true;
}

template <typename Scalar>
void essential_from_motion(const CameraPose<Scalar> &pose, Matrix3x3<Scalar> *E)
{
  *E << 0.0, -pose.t(2), pose.t(1), pose.t(2), 0.0, -pose.t(0), -pose.t(1), pose.t(0), 0.0;
  *E = (*E) * pose.R();
}

template <typename Scalar>
bool check_cheirality(const CameraPose<Scalar> &pose,
                      const Vec3<Scalar> &x1,
                      const Vec3<Scalar> &x2,
                      Scalar min_depth = 0) {
  // This code assumes that x1 and x2 are unit vectors
  const Vec3<Scalar> Rx1 = pose.rotate(x1);

  // [1 a; a 1] * [lambda1; lambda2] = [b1; b2]
  // [lambda1; lambda2] = [1 -a; -a 1] * [b1; b2] / (1 - a*a)

  const Scalar a = -Rx1.dot(x2);
  const Scalar b1 = -Rx1.dot(pose.t);
  const Scalar b2 = x2.dot(pose.t);

  // Note that we drop the factor 1.0/(1-a*a) since it is always positive.
  const Scalar lambda1 = b1 - a * b2;
  const Scalar lambda2 = -a * b1 + b2;

  min_depth = min_depth * (1 - a * a);
  
  //ENTO_DEBUG("Pose - R: [%f %f %f; %f %f %f; %f %f %f], t: [%f %f %f]",
  //           pose.R()(0,0), pose.R()(0,1), pose.R()(0,2),
  //           pose.R()(1,0), pose.R()(1,1), pose.R()(1,2),
  //           pose.R()(2,0), pose.R()(2,1), pose.R()(2,2),
  //           pose.t(0), pose.t(1), pose.t(2));
  
  return lambda1 > min_depth && lambda2 > min_depth;
}

template <typename Scalar>
bool check_cheirality(const CameraPose<Scalar> &pose,
                      const Vec3<Scalar> &p1,
                      const Vec3<Scalar> &x1,
                      const Vec3<Scalar> &p2,
                      const Vec3<Scalar> &x2,
                      Scalar min_depth = 0)
{

  // This code assumes that x1 and x2 are unit vectors
  const Vec3<Scalar> Rx1 = pose.rotate(x1);

  // [1 a; a 1] * [lambda1; lambda2] = [b1; b2]
  // [lambda1; lambda2] = [1 -a; -a 1] * [b1; b2] / (1 - a*a)
  const Vec3<Scalar> rhs = pose.t + pose.rotate(p1) - p2;
  const Scalar a = -Rx1.dot(x2);
  const Scalar b1 = -Rx1.dot(rhs);
  const Scalar b2 = x2.dot(rhs);

  // Note that we drop the factor 1.0/(1-a*a) since it is always positive.
  const Scalar lambda1 = b1 - a * b2;
  const Scalar lambda2 = -a * b1 + b2;

  min_depth = min_depth * (1 - a * a);
  return lambda1 > min_depth && lambda2 > min_depth;
}

// wrappers for vectors
template <typename Scalar>
bool check_cheirality(const CameraPose<Scalar> &pose,
                      const std::vector<Vec3<Scalar>> &x1,
                      const std::vector<Vec3<Scalar>> &x2,
                      Scalar min_depth = 0)
{
  for (size_t i = 0; i < x1.size(); ++i) {
    if (!check_cheirality(pose, x1[i], x2[i], min_depth))
    {
      return false;
    }
  }
  return true;
}

template <typename Scalar, size_t N>
bool check_cheirality(const CameraPose<Scalar> &pose,
                      const EntoArray<Vec3<Scalar>, N> &x1,
                      const EntoArray<Vec3<Scalar>, N> &x2,
                      Scalar min_depth = 0)
{
  for (size_t i = 0; i < x1.size(); ++i) {
    if (!check_cheirality(pose, x1[i], x2[i], min_depth))
    {
      return false;
    }
  }
  return true;
}

// Corresponding generalized version
template <typename Scalar>
bool check_cheirality(const CameraPose<Scalar> &pose,
                      const std::vector<Vec3<Scalar>> &p1,
                      const std::vector<Vec3<Scalar>> &x1, const std::vector<Vec3<Scalar>> &p2,
                      const std::vector<Vec3<Scalar>> &x2, Scalar min_depth = 0)
{
  for (size_t i = 0; i < x1.size(); ++i)
  {
    if (!check_cheirality(pose, p1[i], x1[i], p2[i], x2[i], min_depth))
    {
      return false;
    }
  }
  return true;
}

template <typename Scalar, std::size_t N=0, std::size_t M=4>
void motion_from_essential(const Matrix3x3<Scalar> &E,
                           const EntoContainer<Vec3<Scalar>, N> &x1,
                           const EntoContainer<Vec3<Scalar>, N> &x2,
                                 EntoContainer<CameraPose<Scalar>, M> *relative_poses)
{

  // Compute the necessary cross products
  Vec3<Scalar> u12 = E.col(0).cross(E.col(1));
  Vec3<Scalar> u13 = E.col(0).cross(E.col(2));
  Vec3<Scalar> u23 = E.col(1).cross(E.col(2));
  const Scalar n12 = u12.squaredNorm();
  const Scalar n13 = u13.squaredNorm();
  const Scalar n23 = u23.squaredNorm();
  Matrix3x3<Scalar> UW;
  Matrix3x3<Scalar> Vt;

  // Compute the U*W factor
  if (n12 > n13)
  {
    if (n12 > n23)
    {
        UW.col(1) = E.col(0).normalized();
        UW.col(2) = u12 / std::sqrt(n12);
    }
    else
    {
      UW.col(1) = E.col(1).normalized();
      UW.col(2) = u23 / std::sqrt(n23);
    }
  }
  else
  {
    if (n13 > n23)
    {
      UW.col(1) = E.col(0).normalized();
      UW.col(2) = u13 / std::sqrt(n13);
    }
    else
    {
      UW.col(1) = E.col(1).normalized();
      UW.col(2) = u23 / std::sqrt(n23);
    }
  }
  UW.col(0) = -UW.col(2).cross(UW.col(1));

  // Compute the V factor
  Vt.row(0) = UW.col(1).transpose() * E;
  Vt.row(1) = -UW.col(0).transpose() * E;
  Vt.row(0).normalize();

  // Here v1 and v2 should be orthogonal. However, if E is not exactly an essential matrix they might not be
  // To ensure we end up with a rotation matrix we orthogonalize them again here, this should be a nop for good data
  Vt.row(1) -= Vt.row(0).dot(Vt.row(1)) * Vt.row(0);

  Vt.row(1).normalize();
  Vt.row(2) = Vt.row(0).cross(Vt.row(1));

  CameraPose<Scalar> pose;
  pose.q = rotmat_to_quat<Scalar>(UW * Vt);
  pose.t = UW.col(2);
  //ENTO_DEBUG("Performing 4 cheirality checks ...");
  if (check_cheirality(pose, x1, x2))
  {
    relative_poses->emplace_back(pose);
    //ENTO_DEBUG("Cheirality check #1 passed.");
    //ENTO_DEBUG("Pose: %f %f %f %f %f %f %f %f %f %f %f %f", pose.q(0), pose.q(1), pose.q(2), pose.q(3), pose.t(0), pose.t(1), pose.t(2));
  }
  pose.t = -pose.t;
  if (check_cheirality(pose, x1, x2))
  {
    relative_poses->emplace_back(pose);
    //ENTO_DEBUG("Cheirality check #2 passed.");
    //ENTO_DEBUG("Pose: %f %f %f %f %f %f %f %f %f %f %f %f", pose.q(0), pose.q(1), pose.q(2), pose.q(3), pose.t(0), pose.t(1), pose.t(2));
  }

  // U * W.transpose()
  UW.template block<3, 2>(0, 0) = -UW.template block<3, 2>(0, 0);
  pose.q = rotmat_to_quat<Scalar>(UW * Vt);
  if (check_cheirality(pose, x1, x2)) 
  {
    relative_poses->emplace_back(pose);
    //ENTO_DEBUG("Cheirality check #3 passed.");
    //ENTO_DEBUG("Pose: %f %f %f %f %f %f %f %f %f %f %f %f", pose.q(0), pose.q(1), pose.q(2), pose.q(3), pose.t(0), pose.t(1), pose.t(2));
  }
  pose.t = -pose.t;
  if (check_cheirality(pose, x1, x2))
  {
    relative_poses->emplace_back(pose);
    //ENTO_DEBUG("Cheirality check #4 passed.");
    //ENTO_DEBUG("Pose: %f %f %f %f %f %f %f %f %f %f %f %f", pose.q(0), pose.q(1), pose.q(2), pose.q(3), pose.t(0), pose.t(1), pose.t(2));
  }
}

template <typename Scalar, std::size_t N=0, std::size_t M=2>
void motion_from_essential_planar(Scalar e01,
                                  Scalar e21,
                                  Scalar e10,
                                  Scalar e12,
                                  const EntoContainer<Vec3<Scalar>, N> &x1,
                                  const EntoContainer<Vec3<Scalar>, N> &x2,
                                  EntoContainer<CameraPose<Scalar>, M> *relative_poses)
{

  Vec2<Scalar> z;
  z << -e01 * e10 - e21 * e12, -e21 * e10 + e01 * e12;
  z.normalize();

  CameraPose<Scalar> pose;
  Matrix3x3<Scalar> R;
  R << z(0), 0.0, -z(1), 0.0, 1.0, 0.0, z(1), 0.0, z(0);
  pose.q = rotmat_to_quat<Scalar>(R);
  pose.t << e21, 0.0, -e01;
  pose.t.normalize();

  if (check_cheirality(pose, x1, x2))
  {
    relative_poses->push_back(pose);
  }
  pose.t = -pose.t;
  if (check_cheirality(pose, x1, x2))
  {
    relative_poses->push_back(pose);
  }

    // There are two more flipped solutions where
    //    R = [a 0 b; 0 -1 0; b 0 -a]
    // These are probably not interesting in the planar case

    /*
            z << e01 * e10 - e21 * e12, e21* e10 + e01 * e12;
            z.normalize();
            pose.R << z(0), 0.0, z(1), 0.0, -1.0, 0.0, z(1), 0.0, -z(0);
            relative_poses->push_back(pose);
            pose.t = -pose.t;
            relative_poses->push_back(pose);
    */
}

template <typename Scalar, std::size_t N=0>
void motion_from_essential_svd(const Matrix3x3<Scalar> &E,
                               const EntoContainer<Vec3<Scalar>, N> &x1,
                               const EntoContainer<Vec3<Scalar>, N> &x2,
                               EntoArray<CameraPose<Scalar>, 4> *relative_poses)
{
  Eigen::JacobiSVD<Matrix3x3<Scalar>> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3x3<Scalar> U = USV.matrixU();
  Matrix3x3<Scalar> Vt = USV.matrixV().transpose();

  // Last column of U is undetermined since d = (a a 0).
  if (U.determinant() < 0) {
      U.col(2) *= -1;
  }
  // Last row of Vt is undetermined since d = (a a 0).
  if (Vt.determinant() < 0) {
      Vt.row(2) *= -1;
  }

  Matrix3x3<Scalar> W;
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  const Matrix3x3<Scalar> U_W_Vt = U * W * Vt;
  const Matrix3x3<Scalar> U_Wt_Vt = U * W.transpose() * Vt;

  const std::array<Matrix3x3<Scalar>, 2> R{{U_W_Vt, U_Wt_Vt}};
  const std::array<Vec3<Scalar>, 2> t{{U.col(2), -U.col(2)}};
  if (relative_poses)
  {
    CameraPose<Scalar> pose;
    pose.q = rotmat_to_quat<Scalar>(R[0]);
    pose.t = t[0];
    if (check_cheirality(pose, x1, x2))
    {
      relative_poses->emplace_back(pose);
    }

    pose.t = t[1];
    if (check_cheirality(pose, x1, x2))
    {
      relative_poses->emplace_back(pose);
    }

    pose.q = rotmat_to_quat<Scalar>(R[1]);
    pose.t = t[0];
    if (check_cheirality(pose, x1, x2))
    {
      relative_poses->emplace_back(pose);
    }

    pose.t = t[1];
    if (check_cheirality(pose, x1, x2))
    {
      relative_poses->emplace_back(pose);
    }
  }
}

} // namespace EntoPose

#endif // POSE_EST_UTIL_HH
