#ifndef JACOBIAN_ACCUM_H
#define JACOBIAN_ACCUM_H

#include <ento-pose/pose_util.h>
#include <ento-pose/camera_models.h>
#include <ento-math/core.h>

namespace EntoPose
{

template <typename Scalar>
class UniformWeightVector
{
public:
  UniformWeightVector() {}
  constexpr Scalar operator[](size_t idx) const { return static_cast<Scalar>(1); }
};

template <typename Scalar>
class UniformWeightVectors
{
public:
  UniformWeightVectors() {}
  constexpr const UniformWeightVector<Scalar> &operator[](size_t idx) const { return w; }
  const UniformWeightVector<Scalar> w;
  typedef UniformWeightVector<Scalar> value_type;
};


template <typename Scalar,
          typename CameraModel,
          typename LossFunction,
          typename ResidualWeightVector = UniformWeightVector<Scalar>,
          size_t N = 0>
class CameraJacobianAccumulator
{
private:
  const EntoUtil::EntoContainer<EntoMath::Vec2<Scalar>, N> &x_;
  const EntoUtil::EntoContainer<EntoMath::Vec3<Scalar>, N> &X_;
  const Camera<Scalar> &camera_;
  const LossFunction &loss_fn_;
  const ResidualWeightVector &weights_;
  using CameraParams = typename CameraModel::Params;

public:
  typedef CameraPose<Scalar> param_t;
  static constexpr size_t num_params = 6;
  CameraJacobianAccumulator(const EntoUtil::EntoContainer<EntoMath::Vec2<Scalar>, N>& points2d,
                            const EntoUtil::EntoContainer<EntoMath::Vec3<Scalar>, N>& points3d,
                            const Camera<Scalar>& cam,
                            const LossFunction &loss,
                            const ResidualWeightVector &w = ResidualWeightVector())
    : x_(points2d), X_(points3d), camera_(cam), loss_fn_(loss), weights_(w) {}

  Scalar residual(const CameraPose<Scalar> pose) const
  {
    Scalar cost = 0;
    for (size_t i = 0; i < x_.size(); i++)
    {
      const Vec3<Scalar> Z = pose.apply(X_[i]);
      if (Z(2) < 0) continue;
      const Scalar inv_z = static_cast<Scalar>(1) / Z(2);
      Vec2<Scalar> p(Z(0) * inv_z, Z(1) * inv_z);
      const Scalar r0 = p(0) - x_[i](0);
      const Scalar r1 = p(1) - x_[i](1);
      const Scalar r_squared = r0 * r0 + r1 * r1;
      cost += weights_[i] * loss_fn_.loss(r_squared);
    }
    return cost;
  }

  
  // computes J.transpose() * J and J.transpose() * res
  // Only computes the lower half of JtJ
  size_t accumulate(const CameraPose<Scalar> &pose, Eigen::Matrix<Scalar, 6, 6> &JtJ,
                    Eigen::Matrix<Scalar, 6, 1> &Jtr) const
  {
    // ENTO_DEBUG("accumulate: Starting accumulate function");
    Matrix3x3<Scalar> R = pose.R();
    // ENTO_DEBUG("accumulate: Got rotation matrix");
    Matrix2x2<Scalar> Jcam;
    Jcam.setIdentity(); // we initialize to identity here (this is for the calibrated case)
    // ENTO_DEBUG("accumulate: Initialized Jcam");
    size_t num_residuals = 0;
    // ENTO_DEBUG("accumulate: x_.size() = %zu, X_.size() = %zu", x_.size(), X_.size());
    for (size_t i = 0; i < x_.size(); ++i)
    {
      // ENTO_DEBUG("accumulate: Processing point %zu", i);
      const Vec3<Scalar> Z = R * X_[i] + pose.t;
      // ENTO_DEBUG("accumulate: Computed Z = (%f, %f, %f)", Z.x(), Z.y(), Z.z());
      const Vec2<Scalar> z = Z.hnormalized();
      // ENTO_DEBUG("accumulate: Computed z = (%f, %f)", z.x(), z.y());

      // Note this assumes points that are behind the camera will stay behind the camera
      // during the optimization
      if (Z(2) < 0) {
        // ENTO_DEBUG("accumulate: Point %zu is behind camera, skipping", i);
        continue;
      }

      // Project with intrinsics
      Vec2<Scalar> zp = z;
      // ENTO_DEBUG("accumulate: About to call project_with_jac");
      CameraModel::project_with_jac(camera_.params(), z, &zp, &Jcam);
      // ENTO_DEBUG("accumulate: Called project_with_jac");

      // Setup residual
      Vec2<Scalar> r = zp - x_[i];
      const Scalar r_squared = r.squaredNorm();
      const Scalar weight = weights_[i] * loss_fn_.weight(r_squared);

      if (weight == 0.0) continue;
      num_residuals++;

      // Compute jacobian w.r.t. Z (times R)
      Eigen::Matrix<Scalar, 2, 3> dZ;
      dZ.template block<2, 2>(0, 0) = Jcam;
      dZ.col(2) = -Jcam * z;
      dZ *= 1.0 / Z(2);
      dZ *= R;

      const Scalar X0 = X_[i](0);
      const Scalar X1 = X_[i](1);
      const Scalar X2 = X_[i](2);
      const Scalar dZtdZ_0_0 = weight * dZ.col(0).dot(dZ.col(0));
      const Scalar dZtdZ_1_0 = weight * dZ.col(1).dot(dZ.col(0));
      const Scalar dZtdZ_1_1 = weight * dZ.col(1).dot(dZ.col(1));
      const Scalar dZtdZ_2_0 = weight * dZ.col(2).dot(dZ.col(0));
      const Scalar dZtdZ_2_1 = weight * dZ.col(2).dot(dZ.col(1));
      const Scalar dZtdZ_2_2 = weight * dZ.col(2).dot(dZ.col(2));
      JtJ(0, 0) += X2 * (X2 * dZtdZ_1_1 - X1 * dZtdZ_2_1) + X1 * (X1 * dZtdZ_2_2 - X2 * dZtdZ_2_1);
      JtJ(1, 0) += -X2 * (X2 * dZtdZ_1_0 - X0 * dZtdZ_2_1) - X1 * (X0 * dZtdZ_2_2 - X2 * dZtdZ_2_0);
      JtJ(2, 0) += X1 * (X0 * dZtdZ_2_1 - X1 * dZtdZ_2_0) - X2 * (X0 * dZtdZ_1_1 - X1 * dZtdZ_1_0);
      JtJ(3, 0) += X1 * dZtdZ_2_0 - X2 * dZtdZ_1_0;
      JtJ(4, 0) += X1 * dZtdZ_2_1 - X2 * dZtdZ_1_1;
      JtJ(5, 0) += X1 * dZtdZ_2_2 - X2 * dZtdZ_2_1;
      JtJ(1, 1) += X2 * (X2 * dZtdZ_0_0 - X0 * dZtdZ_2_0) + X0 * (X0 * dZtdZ_2_2 - X2 * dZtdZ_2_0);
      JtJ(2, 1) += -X2 * (X1 * dZtdZ_0_0 - X0 * dZtdZ_1_0) - X0 * (X0 * dZtdZ_2_1 - X1 * dZtdZ_2_0);
      JtJ(3, 1) += X2 * dZtdZ_0_0 - X0 * dZtdZ_2_0;
      JtJ(4, 1) += X2 * dZtdZ_1_0 - X0 * dZtdZ_2_1;
      JtJ(5, 1) += X2 * dZtdZ_2_0 - X0 * dZtdZ_2_2;
      JtJ(2, 2) += X1 * (X1 * dZtdZ_0_0 - X0 * dZtdZ_1_0) + X0 * (X0 * dZtdZ_1_1 - X1 * dZtdZ_1_0);
      JtJ(3, 2) += X0 * dZtdZ_1_0 - X1 * dZtdZ_0_0;
      JtJ(4, 2) += X0 * dZtdZ_1_1 - X1 * dZtdZ_1_0;
      JtJ(5, 2) += X0 * dZtdZ_2_1 - X1 * dZtdZ_2_0;
      JtJ(3, 3) += dZtdZ_0_0;
      JtJ(4, 3) += dZtdZ_1_0;
      JtJ(5, 3) += dZtdZ_2_0;
      JtJ(4, 4) += dZtdZ_1_1;
      JtJ(5, 4) += dZtdZ_2_1;
      JtJ(5, 5) += dZtdZ_2_2;
      r *= weight;
      Jtr(0) += (r(0) * (X1 * dZ(0, 2) - X2 * dZ(0, 1)) + r(1) * (X1 * dZ(1, 2) - X2 * dZ(1, 1)));
      Jtr(1) += (-r(0) * (X0 * dZ(0, 2) - X2 * dZ(0, 0)) - r(1) * (X0 * dZ(1, 2) - X2 * dZ(1, 0)));
      Jtr(2) += (r(0) * (X0 * dZ(0, 1) - X1 * dZ(0, 0)) + r(1) * (X0 * dZ(1, 1) - X1 * dZ(1, 0)));
      Jtr(3) += (dZ(0, 0) * r(0) + dZ(1, 0) * r(1));
      Jtr(4) += (dZ(0, 1) * r(0) + dZ(1, 1) * r(1));
      Jtr(5) += (dZ(0, 2) * r(0) + dZ(1, 2) * r(1));
    }
    return num_residuals;
  }

  CameraPose<Scalar> step(Eigen::Matrix<Scalar, 6, 1> dp,
                          const CameraPose<Scalar> &pose) const
  {
    CameraPose<Scalar> pose_new;
    // Convert block to matrix before passing to quat_step_post
    Eigen::Matrix<Scalar, 3, 1> w_delta = dp.template block<3, 1>(0, 0);
    pose_new.q = quat_step_post(pose.q, w_delta);

    // Translation is parameterized as (negative) shift in position
    //  i.e. t(delta) = t + R*delta
    pose_new.t = pose.t + pose.rotate(dp.template block<3, 1>(3, 0));
    return pose_new;
  }
};

template <typename Scalar,
          typename LossFunction,
          typename ResidualWeightVector = UniformWeightVector<Scalar>,
          size_t N = 0>
class RelativePoseJacobianAccumulator
{
private:
  const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x1_;
  const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x2_;
  const LossFunction &loss_fn_;
  const ResidualWeightVector &weights_;
  Eigen::Matrix<Scalar, 3, 2> tangent_basis_;
public:
  typedef CameraPose<Scalar> param_t;
  static constexpr size_t num_params = 5;

  RelativePoseJacobianAccumulator(const EntoUtil::EntoContainer<Vec2<Scalar>, N> &points2d_1,
                                  const EntoUtil::EntoContainer<Vec2<Scalar>, N> &points2d_2,
                                  const LossFunction &loss, const ResidualWeightVector &w = UniformWeightVector<Scalar>())
    : x1_(points2d_1), x2_(points2d_2), loss_fn_(loss), weights_(w) {}

  Scalar residual(const CameraPose<Scalar> &pose)
  {
    Matrix3x3<Scalar> E;
    essential_from_motion<Scalar>(pose, &E);
    Scalar cost = static_cast<Scalar>(0);
    for (size_t k = 0; k < x1_.size(); ++k)
    {
      Scalar C = x2_[k].homogeneous().dot(E * x1_[k].homogeneous());
      Scalar nJc_sq = (E.template block<2,3>(0, 0) * x1_[k].homogeneous()).squaredNorm() +
                      (E.template block<3,2>(0, 0).transpose() * x2_[k].homogeneous()).squaredNorm();
      Scalar r2 = (C * C) / nJc_sq;
      cost += weights_[k] * loss_fn_.loss(r2);
    }
    return cost;
  }

  size_t accumulate(const CameraPose<Scalar> &pose, Eigen::Matrix<Scalar, 5, 5> &JtJ, Eigen::Matrix<Scalar, 5, 1> &Jtr) {
    // We start by setting up a basis for the updates in the translation (orthogonal to t)
    // We find the minimum element of t and cross product with the corresponding basis vector.
    // (this ensures that the first cross product is not close to the zero vector)
    if (std::abs(pose.t.x()) < std::abs(pose.t.y()))
    {
      // x < y
      if (std::abs(pose.t.x()) < std::abs(pose.t.z())) {
        tangent_basis_.col(0) = pose.t.cross(Vec3<Scalar>::UnitX()).normalized();
      }
      else
      {
        tangent_basis_.col(0) = pose.t.cross(Vec3<Scalar>::UnitZ()).normalized();
      }
    } else {
      // x > y
      if (std::abs(pose.t.y()) < std::abs(pose.t.z()))
      {
        tangent_basis_.col(0) = pose.t.cross(Vec3<Scalar>::UnitY()).normalized();
      }
      else
      {
        tangent_basis_.col(0) = pose.t.cross(Vec3<Scalar>::UnitZ()).normalized();
      }
    }
    tangent_basis_.col(1) = tangent_basis_.col(0).cross(pose.t).normalized();

    Matrix3x3<Scalar> E, R;
    R = pose.R();
    essential_from_motion<Scalar>(pose, &E);

    // Matrices contain the jacobians of E w.r.t. the rotation and translation parameters
    Eigen::Matrix<Scalar, 9, 3> dR;
    Eigen::Matrix<Scalar, 9, 2> dt;

    // Each column is vec(E*skew(e_k)) where e_k is k:th basis vector
    dR.template block<3, 1>(0, 0).setZero();
    dR.template block<3, 1>(0, 1) = -E.col(2);
    dR.template block<3, 1>(0, 2) = E.col(1);
    dR.template block<3, 1>(3, 0) = E.col(2);
    dR.template block<3, 1>(3, 1).setZero();
    dR.template block<3, 1>(3, 2) = -E.col(0);
    dR.template block<3, 1>(6, 0) = -E.col(1);
    dR.template block<3, 1>(6, 1) = E.col(0);
    dR.template block<3, 1>(6, 2).setZero();

    // Each column is vec(skew(tangent_basis_[k])*R)
    dt.template block<3, 1>(0, 0) = tangent_basis_.col(0).cross(R.col(0));
    dt.template block<3, 1>(0, 1) = tangent_basis_.col(1).cross(R.col(0));
    dt.template block<3, 1>(3, 0) = tangent_basis_.col(0).cross(R.col(1));
    dt.template block<3, 1>(3, 1) = tangent_basis_.col(1).cross(R.col(1));
    dt.template block<3, 1>(6, 0) = tangent_basis_.col(0).cross(R.col(2));
    dt.template block<3, 1>(6, 1) = tangent_basis_.col(1).cross(R.col(2));

    size_t num_residuals = 0;
    for (size_t k = 0; k < x1_.size(); ++k)
    {
      Scalar C = x2_[k].homogeneous().dot(E * x1_[k].homogeneous());

      // J_C is the Jacobian of the epipolar constraint w.r.t. the image points
      Vec4<Scalar> J_C;
      J_C << E.template block<3, 2>(0, 0).transpose() * x2_[k].homogeneous(), E.template block<2, 3>(0, 0) * x1_[k].homogeneous();
      const Scalar nJ_C = J_C.norm();
      const Scalar inv_nJ_C = 1.0 / nJ_C;
      const Scalar r = C * inv_nJ_C;

      // Compute weight from robust loss function (used in the IRLS)
      const Scalar weight = weights_[k] * loss_fn_.weight(r * r);
      if (weight == 0.0) continue;
      num_residuals++;

      // Compute Jacobian of Sampson error w.r.t the fundamental/essential matrix (3x3)
      Eigen::Matrix<Scalar, 1, 9> dF;
      dF << x1_[k](0) * x2_[k](0), x1_[k](0) * x2_[k](1), x1_[k](0), x1_[k](1) * x2_[k](0), x1_[k](1) * x2_[k](1),
            x1_[k](1), x2_[k](0), x2_[k](1), 1.0;
      const Scalar s = C * inv_nJ_C * inv_nJ_C;
      dF(0) -= s * (J_C(2) * x1_[k](0) + J_C(0) * x2_[k](0));
      dF(1) -= s * (J_C(3) * x1_[k](0) + J_C(0) * x2_[k](1));
      dF(2) -= s * (J_C(0));
      dF(3) -= s * (J_C(2) * x1_[k](1) + J_C(1) * x2_[k](0));
      dF(4) -= s * (J_C(3) * x1_[k](1) + J_C(1) * x2_[k](1));
      dF(5) -= s * (J_C(1));
      dF(6) -= s * (J_C(2));
      dF(7) -= s * (J_C(3));
      dF *= inv_nJ_C;

      // and then w.r.t. the pose parameters (rotation + tangent basis for translation)
      Eigen::Matrix<Scalar, 1, 5> J;
      J.template block<1, 3>(0, 0) = dF * dR;
      J.template block<1, 2>(0, 3) = dF * dt;

      // Accumulate into JtJ and Jtr
      Jtr += weight * C * inv_nJ_C * J.transpose();
      JtJ(0, 0) += weight * (J(0) * J(0));
      JtJ(1, 0) += weight * (J(1) * J(0));
      JtJ(1, 1) += weight * (J(1) * J(1));
      JtJ(2, 0) += weight * (J(2) * J(0));
      JtJ(2, 1) += weight * (J(2) * J(1));
      JtJ(2, 2) += weight * (J(2) * J(2));
      JtJ(3, 0) += weight * (J(3) * J(0));
      JtJ(3, 1) += weight * (J(3) * J(1));
      JtJ(3, 2) += weight * (J(3) * J(2));
      JtJ(3, 3) += weight * (J(3) * J(3));
      JtJ(4, 0) += weight * (J(4) * J(0));
      JtJ(4, 1) += weight * (J(4) * J(1));
      JtJ(4, 2) += weight * (J(4) * J(2));
      JtJ(4, 3) += weight * (J(4) * J(3));
      JtJ(4, 4) += weight * (J(4) * J(4));
    }
    return num_residuals;
  }

  CameraPose<Scalar> step(Eigen::Matrix<Scalar, 5, 1> dp, const CameraPose<Scalar> &pose) const
  {
    CameraPose<Scalar> pose_new;
    // Convert block to matrix before passing to quat_step_post
    Eigen::Matrix<Scalar, 3, 1> w_delta = dp.template block<3, 1>(0, 0);
    pose_new.q = quat_step_post(pose.q, w_delta);
    pose_new.t = pose.t + tangent_basis_ * dp.template block<2, 1>(3, 0);
    return pose_new;
  }
};

template <typename Scalar, typename LossFunction, typename ResidualWeightVector = UniformWeightVector<Scalar>, size_t N = 0>
class HomographyJacobianAccumulator
{
private:
  const EntoUtil::EntoArray<EntoMath::Vec2<Scalar>, N> &x1_;
  const EntoUtil::EntoArray<EntoMath::Vec3<Scalar>, N> &x2_;
  const LossFunction &loss_fn_;
  const ResidualWeightVector &weights_;
public:
  typedef Matrix3x3<Scalar> param_t;
  static constexpr size_t num_params = 8;

  HomographyJacobianAccumulator(const EntoUtil::EntoArray<EntoMath::Vec2<Scalar>, N> &points2d_1,
                                const EntoUtil::EntoArray<EntoMath::Vec3<Scalar>, N> &points2d_2,
                                const LossFunction &loss, const ResidualWeightVector &w = UniformWeightVector<Scalar>())
    : x1_(points2d_1), x2_(points2d_2), loss_fn_(loss), weights_(w) {}

  Scalar residual(const Matrix3x3<Scalar> &H) const
  {
    Scalar cost = static_cast<Scalar>(0);
    const Scalar H0_0 = H(0, 0), H0_1 = H(0, 1), H0_2 = H(0, 2);
    const Scalar H1_0 = H(1, 0), H1_1 = H(1, 1), H1_2 = H(1, 2);
    const Scalar H2_0 = H(2, 0), H2_1 = H(2, 1), H2_2 = H(2, 2);

    for (size_t k = 0; k < x1_.size(); ++k) 
    {
      const Scalar x1_0 = x1_[k](0), x1_1 = x1_[k](1);
      const Scalar x2_0 = x2_[k](0), x2_1 = x2_[k](1);

      const Scalar Hx1_0 = H0_0 * x1_0 + H0_1 * x1_1 + H0_2;
      const Scalar Hx1_1 = H1_0 * x1_0 + H1_1 * x1_1 + H1_2;
      const Scalar inv_Hx1_2 = 1.0 / (H2_0 * x1_0 + H2_1 * x1_1 + H2_2);

      const Scalar r0 = Hx1_0 * inv_Hx1_2 - x2_0;
      const Scalar r1 = Hx1_1 * inv_Hx1_2 - x2_1;
      const Scalar r2 = r0 * r0 + r1 * r1;
      cost += weights_[k] * loss_fn_.loss(r2);
    }
    return cost;
  }

  size_t accumulate(const Eigen::Matrix3d &H, Eigen::Matrix<Scalar, 8, 8> &JtJ, Eigen::Matrix<Scalar, 8, 1> &Jtr) {
    Eigen::Matrix<Scalar, 2, 8> dH;
    const Scalar H0_0 = H(0, 0), H0_1 = H(0, 1), H0_2 = H(0, 2);
    const Scalar H1_0 = H(1, 0), H1_1 = H(1, 1), H1_2 = H(1, 2);
    const Scalar H2_0 = H(2, 0), H2_1 = H(2, 1), H2_2 = H(2, 2);

    size_t num_residuals = 0;
    for (size_t k = 0; k < x1_.size(); ++k)
    {
      const Scalar x1_0 = x1_[k](0), x1_1 = x1_[k](1);
      const Scalar x2_0 = x2_[k](0), x2_1 = x2_[k](1);

      const Scalar Hx1_0 = H0_0 * x1_0 + H0_1 * x1_1 + H0_2;
      const Scalar Hx1_1 = H1_0 * x1_0 + H1_1 * x1_1 + H1_2;
      const Scalar inv_Hx1_2 = 1.0 / (H2_0 * x1_0 + H2_1 * x1_1 + H2_2);

      const Scalar z0 = Hx1_0 * inv_Hx1_2;
      const Scalar z1 = Hx1_1 * inv_Hx1_2;

      const Scalar r0 = z0 - x2_0;
      const Scalar r1 = z1 - x2_1;
      const Scalar r2 = r0 * r0 + r1 * r1;

      // Compute weight from robust loss function (used in the IRLS)
      const Scalar weight = weights_[k] * loss_fn_.weight(r2);
      if (weight == 0.0) continue;
      num_residuals++;

      dH << x1_0, 0.0, -x1_0 * z0, x1_1, 0.0, -x1_1 * z0, 1.0, 0.0, // -z0,
          0.0, x1_0, -x1_0 * z1, 0.0, x1_1, -x1_1 * z1, 0.0, 1.0;   // -z1,
      dH = dH * inv_Hx1_2;

      // accumulate into JtJ and Jtr
      Jtr += dH.transpose() * (weight * Eigen::Vector2d(r0, r1));
      for (size_t i = 0; i < 8; ++i)
      {
        for (size_t j = 0; j <= i; ++j)
        {
          JtJ(i, j) += weight * dH.col(i).dot(dH.col(j));
        }
      }
    }
    return num_residuals;
  }

  Eigen::Matrix3d step(Eigen::Matrix<Scalar, 8, 1> dp, const Matrix3x3<Scalar> &H) const
  {
    Matrix3x3<Scalar> H_new = H;
    Eigen::Map<Eigen::Matrix<Scalar, 8, 1>>(H_new.data()) += dp;
    return H_new;
  }
};

} // namespace EntoPose

#endif // JACOBIAN_ACCUM_H
