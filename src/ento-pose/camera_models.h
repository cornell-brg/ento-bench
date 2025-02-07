#ifndef ENTO_CAMERA_MODELS_H
#define ENTO_CAMERA_MODELS_H

#include <Eigen/Dense>
#include <ento-util/containers.h>
#include <ento-math/core.h>

namespace EntoPose
{

using namespace EntoMath;

template <typename Derived, typename Scalar, size_t NumParams>
class CameraModel
{
public:
  using Params = std::array<Scalar, NumParams>;
  static void project(const Params& p, const Vec2<Scalar>& x, Vec2<Scalar>* xp)
  {
    Derived::project_impl(p, x, xp);
  }
  static void project_with_jac(const Params& p,
                               const Vec2<Scalar>& x,
                               Vec2<Scalar>* xp,
                               Matrix2x2<Scalar>* jac)
  {
    Derived::project_with_jac(p, x, xp, jac);
  }

  void unproject(const Params& p, const Vec2<Scalar>& xp, Vec2<Scalar>* x) const
  {
    Derived::unproject(p, xp, x);
  }
};

template <typename Scalar>
class NullCameraModel : public CameraModel<NullCameraModel<Scalar>, Scalar, 0>
{
public:
  using Base = CameraModel<NullCameraModel<Scalar>, Scalar, 3>;
  using Params = Base::Params;

  static constexpr size_t focal_size = 0;
  static constexpr size_t principal_size = 0;
  static constexpr std::array<size_t, focal_size> focal_idx;
  static constexpr std::array<size_t, principal_size> principal_point_idx;
  static void project_impl([[maybe_unused]] const Params &params,
                           [[maybe_unused]] const Vec2<Scalar> &x,
                           [[maybe_unused]] Vec2<Scalar> *xp) {}
  static void project_with_jac_impl([[maybe_unused]] const Params &params,
                                    [[maybe_unused]] const Vec2<Scalar> &x,
                                    [[maybe_unused]] Vec2<Scalar> *xp, 
                                    [[maybe_unused]] Matrix2x2<Scalar> *jac) {}
  static void unproject_impl([[maybe_unused]] const Params &params,
                             [[maybe_unused]] const Vec2<Scalar> &xp,
                             [[maybe_unused]] Vec2<Scalar> *x) {}
};

// Camera Model Implementations

template <typename Scalar>
class PinholeCameraModel : public CameraModel<PinholeCameraModel<Scalar>, Scalar, 3>
{
public:
  using Base = CameraModel<PinholeCameraModel<Scalar>, Scalar, 3>;
  using Params = Base::Params;

  static constexpr size_t focal_size = 2;
  static constexpr size_t principal_size = 2;
  static constexpr std::array<size_t, focal_size> focal_idx = {0, 1};
  static constexpr std::array<size_t, principal_size> principal_point_idx = {2, 3};

  static void project_impl(const Params& params, const Vec2<Scalar>& x, Vec2<Scalar>* xp)
  {
    (*xp)(0) = params[0] * x(0) + params[2];
    (*xp)(1) = params[1] * x(1) + params[3];
  }

  static void project_with_jac_impl(const Params& p,
                                    const Vec2<Scalar>&x,
                                    Vec2<Scalar>* xp,
                                    Matrix2x2<Scalar>* jac)
  {
    (*xp)(0) = p[0] * x(0) + p[2];
    (*xp)(1) = p[1] * x(1) + p[3];
    (*jac)(0, 0) = p[0];
    (*jac)(0, 1) = static_cast<Scalar>(0.0);
    (*jac)(1, 0) = static_cast<Scalar>(0.0);
    (*jac)(0, 1) = p[1];
  }

  static void unproject_impl(const Params& p, const Vec2<Scalar>& xp, Vec2<Scalar>* x)
  {
    (*x)(0) = (xp(0) - p[2]) / p[0];
    (*x)(1) = (xp(1) - p[3]) / p[1];
  }
};

template <typename Scalar>
class RadialCameraModel : public CameraModel<RadialCameraModel<Scalar>, Scalar, 5>
{
public:
  using Base = CameraModel<RadialCameraModel<Scalar>, Scalar, 5>;
  using Params = typename Base::Params;

  static constexpr size_t focal_size = 1;
  static constexpr size_t principal_size = 2;
  static constexpr std::array<size_t, focal_size> focal_idx = {0};
  static constexpr std::array<size_t, principal_size> principal_point_idx = {1, 2};

  static void project_impl(const Params& params, const Vec2<Scalar>& x, Vec2<Scalar>* xp)
  {
    Scalar r2 = x.squaredNorm();
    Scalar alpha = 1 + params[3] * r2 + params[4] * r2 * r2; // Radial distortion
    (*xp)(0) = params[0] * alpha * x(0) + params[1];
    (*xp)(1) = params[0] * alpha * x(1) + params[2];
  }

  static void project_with_jac_impl(const Params& params,
                                    const Vec2<Scalar>& x,
                                    Vec2<Scalar>* xp,
                                    Matrix2x2<Scalar>* jac)
  {
    Scalar r2 = x.squaredNorm();
    Scalar alpha = 1 + params[3] * r2 + params[4] * r2 * r2;
    Scalar alpha_derivative = 2 * params[3] + 4 * params[4] * r2;

    (*xp)(0) = params[0] * alpha * x(0) + params[1];
    (*xp)(1) = params[0] * alpha * x(1) + params[2];

    *jac = alpha_derivative * (x * x.transpose()); // Outer product
    (*jac)(0, 0) += alpha;
    (*jac)(1, 1) += alpha;
    *jac *= params[0];
  }

  static void unproject_impl(const Params& params, const Vec2<Scalar>& xp, Vec2<Scalar>* x)
  {
    (*x)(0) = (xp(0) - params[1]) / params[0];
    (*x)(1) = (xp(1) - params[2]) / params[0];

    Scalar r0 = x->norm();
    Scalar r = undistort_poly2(params[3], params[4], r0); // Solve distortion inverse
    *x *= r / r0;
  }

private:
  static Scalar undistort_poly2(Scalar k1, Scalar k2, Scalar rd)
  {
    Scalar r = rd;
    for (size_t i = 0; i < 25; ++i) {
      Scalar r2 = r * r;
      Scalar f = k2 * r2 * r2 * r + k1 * r2 * r + r - rd;
      if (std::abs(f) < 1e-10) break;
      Scalar fp = 5 * k2 * r2 * r2 + 3 * k1 * r2 + 1;
      r -= f / fp;
    }
    return r;
  }
};

template <typename Scalar>
class OpenCVCameraModel : public CameraModel<OpenCVCameraModel<Scalar>, Scalar, 8>
{
public:
  using Base = CameraModel<OpenCVCameraModel<Scalar>, Scalar, 8>;
  using Params = typename Base::Params;

  static constexpr size_t focal_size = 2;
  static constexpr size_t principal_size = 2;
  static constexpr std::array<size_t, focal_size> focal_idx = {0, 1};
  static constexpr std::array<size_t, principal_size> principal_point_idx = {2, 3};

  static void project_impl(const Params& params, const Vec2<Scalar>& x, Vec2<Scalar>* xp)
  {
    Vec2<Scalar> xd;
    compute_distortion(params, x, &xd);
    (*xp)(0) = params[0] * xd(0) + params[2];
    (*xp)(1) = params[1] * xd(1) + params[3];
  }

  static void project_with_jac_impl(const Params& params,
                                    const Vec2<Scalar>& x,
                                    Vec2<Scalar>* xp,
                                    Matrix2x2<Scalar>* jac)
  {
    Vec2<Scalar> xd;
    compute_distortion_jacobian(params, x, &xd, jac);

    (*xp)(0) = params[0] * xd(0) + params[2];
    (*xp)(1) = params[1] * xd(1) + params[3];
    jac->row(0) *= params[0];
    jac->row(1) *= params[1];
  }

  static void unproject_impl(const Params& params, const Vec2<Scalar>& xp, Vec2<Scalar>* x)
  {
    (*x)(0) = (xp(0) - params[2]) / params[0];
    (*x)(1) = (xp(1) - params[3]) / params[1];
    undistort_opencv(params, *x);
  }

private:
  static void compute_distortion(const Params& params, const Vec2<Scalar>& x, Vec2<Scalar>* xd)
  {
    Scalar r2 = x.squaredNorm();
    Scalar radial = 1 + params[4] * r2 + params[5] * r2 * r2;
    (*xd)(0) = radial * x(0) + 2 * params[6] * x(0) * x(1) + params[7] * (r2 + 2 * x(0) * x(0));
    (*xd)(1) = radial * x(1) + 2 * params[7] * x(0) * x(1) + params[6] * (r2 + 2 * x(1) * x(1));
  }

  static void compute_distortion_jacobian(const Params& params,
                                          const Vec2<Scalar>& x,
                                          Vec2<Scalar>* xd,
                                          Matrix2x2<Scalar>* jac)
  {
    Scalar r2 = x.squaredNorm();
    Scalar radial = 1 + params[4] * r2 + params[5] * r2 * r2;
    compute_distortion(params, x, xd);

    Scalar d_radial_dr2 = params[4] + 2 * params[5] * r2;
    (*jac)(0, 0) = radial + d_radial_dr2 * 2 * x(0) * x(0) + 2 * params[6] * x(1) + 6 * params[7] * x(0);
    (*jac)(0, 1) = d_radial_dr2 * 2 * x(0) * x(1) + 2 * params[6] * x(0) + 2 * params[7] * x(1);
    (*jac)(1, 0) = (*jac)(0, 1); // Symmetric
    (*jac)(1, 1) = radial + d_radial_dr2 * 2 * x(1) * x(1) + 6 * params[6] * x(1) + 2 * params[7] * x(0);
  }

  static void undistort_opencv(const Params& params, Vec2<Scalar>& x)
  {
    for (int iter = 0; iter < 25; ++iter) {
      Vec2<Scalar> xd;
      compute_distortion(params, x, &xd);
      Vec2<Scalar> delta = x - xd;
      if (delta.norm() < 1e-10) break;
      x += delta; // Update guess
    }
  }
};

template <typename Scalar>
class IdentityCameraModel : public CameraModel<IdentityCameraModel<Scalar>, Scalar, 0>
{
public:
  using Base = CameraModel<IdentityCameraModel<Scalar>, Scalar, 0>;
  using Params = typename Base::Params;

  static constexpr size_t focal_size = 0;
  static constexpr size_t principal_size = 0;
  static constexpr std::array<size_t, focal_size> focal_idx = {};
  static constexpr std::array<size_t, principal_size> principal_point_idx = {};

  static void project_impl(const Params&, const Vec2<Scalar>& x, Vec2<Scalar>* xp)
  {
    *xp = x; // Identity mapping
  }

  static void project_with_jac_impl(const Params&, const Vec2<Scalar>& x, Vec2<Scalar>* xp, Matrix2x2<Scalar>* jac)
  {
    *xp = x;          // Identity mapping
    *jac = Matrix2x2<Scalar>::Identity(); // Identity Jacobian
  }

  static void unproject_impl(const Params&, const Vec2<Scalar>& xp, Vec2<Scalar>* x)
  {
    *x = xp; // Identity mapping
  }
};

template <typename Scalar>
class SimplePinholeCameraModel : public CameraModel<SimplePinholeCameraModel<Scalar>, Scalar, 3>
{
public:
  using Base = CameraModel<SimplePinholeCameraModel<Scalar>, Scalar, 3>;
  using Params = typename Base::Params;

  static constexpr size_t focal_size = 1;
  static constexpr size_t principal_size = 2;
  static constexpr std::array<size_t, focal_size> focal_idx = {0};
  static constexpr std::array<size_t, principal_size> principal_point_idx = {1, 2};

  static void project_impl(const Params& params, const Vec2<Scalar>& x, Vec2<Scalar>* xp)
  {
    (*xp)(0) = params[0] * x(0) + params[1];
    (*xp)(1) = params[0] * x(1) + params[2];
  }

  static void project_with_jac_impl(const Params& params, const Vec2<Scalar>& x, Vec2<Scalar>* xp,
                                    Matrix2x2<Scalar>* jac)
  {
    (*xp)(0) = params[0] * x(0) + params[1];
    (*xp)(1) = params[0] * x(1) + params[2];
    (*jac)(0, 0) = params[0];
    (*jac)(0, 1) = static_cast<Scalar>(0.0);
    (*jac)(1, 0) = static_cast<Scalar>(0.0);
    (*jac)(1, 1) = params[0];
  }

  static void unproject_impl(const Params& params, const Vec2<Scalar>& xp, Vec2<Scalar>* x)
  {
    (*x)(0) = (xp(0) - params[1]) / params[0];
    (*x)(1) = (xp(1) - params[2]) / params[0];
  }
};

template <typename Scalar, typename CameraModel = IdentityCameraModel<Scalar>>
struct Camera
{
  int model_id_;
  int width_;
  int height_;
  
  // Vector of params?
  using Params = typename CameraModel::Params;
  Params params_;
  
  Camera(int w, int h, const Params& p)
    : width_(w), height_(h), params_(p) {}

  Camera(int w, int h, const std::vector<Scalar>& p)
    : width_(w), height_(h)
  {
    std::copy(p.begin(), p.begin() + Params().size(), params_.begin());
  }

  void project(const Vec2<Scalar>& x, Vec2<Scalar>* xp) const
  {
    CameraModel::project(params_, x, xp);
  }
  
  void project_with_jac(const Vec2<Scalar>& x, Vec2<Scalar>* xp, Matrix2x2<Scalar>* jac) const
  {
    CameraModel::project_with_jac(params_, x, xp, jac);
  }
  
  void unproject(const Vec2<Scalar>& xp, Vec2<Scalar>* x) const
  {
    CameraModel::unproject(params_, xp, x);
  }

  Scalar focal() const
  {
    if constexpr (params_.empty())
    {
      return static_cast<Scalar>(1.0);
    }
    else
    {
      Scalar focal = 0.0;
      for (size_t idx : CameraModel::focal_idx)
        focal += params_.at(idx) / CameraModel::focal_idx.size();
      return focal;
    }
  }
  
};



} // namespace EntoPose


#endif // ENTO_CAMERA_MODELS_H
