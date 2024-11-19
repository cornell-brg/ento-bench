#ifndef P3P_H
#define P3P_H

#include <Eigen/Dense>
#include <ento-pose/pose_util.h>

namespace EntoPose
{

// =========================================================
// Function Prototypes


#if defined(NATIVE)
template <typename Scalar>
int p3p(const std::vector<Vec3<Scalar>> &x,
        const std::vector<Vec3<Scalar>> &X,
              std::vector<CameraPose<Scalar>>* output);
#endif

template <typename Scalar, size_t N>
int p3p(const std::array<Vec3<Scalar>,    N> &x,
        const std::array<Vec3<Scalar>,    N> &X,
              std::array<CameraPose<Scalar>, 4>* output);

template <typename Scalar>
inline std::array<Vec3<Scalar>, 2>
compute_pq(Matrix3x3<Scalar> C);

// Performs a few newton steps on the equations
template <typename Scalar>
inline void refine_lambda(Scalar &lambda1, Scalar &lambda2, Scalar &lambda3, const Scalar a12, const Scalar a13,
                          const Scalar a23, const Scalar b12, const Scalar b13, const Scalar b23) {

    for (int iter = 0; iter < 5; ++iter) {
        Scalar r1 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda2 * b12 + lambda2 * lambda2 - a12);
        Scalar r2 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda3 * b13 + lambda3 * lambda3 - a13);
        Scalar r3 = (lambda2 * lambda2 - 2.0 * lambda2 * lambda3 * b23 + lambda3 * lambda3 - a23);
        if (std::abs(r1) + std::abs(r2) + std::abs(r3) < 1e-10)
            return;
        Scalar x11 = lambda1 - lambda2 * b12;
        Scalar x12 = lambda2 - lambda1 * b12;
        Scalar x21 = lambda1 - lambda3 * b13;
        Scalar x23 = lambda3 - lambda1 * b13;
        Scalar x32 = lambda2 - lambda3 * b23;
        Scalar x33 = lambda3 - lambda2 * b23;
        Scalar detJ = 0.5 / (x11 * x23 * x32 + x12 * x21 * x33); // half minus inverse determinant
        // This uses the closed form of the inverse for the jacobean.
        // Due to the zero elements this actually becomes quite nice.
        lambda1 += (-x23 * x32 * r1 - x12 * x33 * r2 + x12 * x23 * r3) * detJ;
        lambda2 += (-x21 * x33 * r1 + x11 * x33 * r2 - x11 * x23 * r3) * detJ;
        lambda3 += (x21 * x32 * r1 - x11 * x32 * r2 - x12 * x21 * r3) * detJ;
    }
}

// =========================================================
// Function Implementations

template <typename Scalar, size_t N>
int p3p(const std::array<Vec3<Scalar>,    N> &x,
        const std::array<Vec3<Scalar>,    N> &X,
              std::array<CameraPose<Scalar>, 4>* output)
{
  Vec3<Scalar> X01 = X[0] - X[1];
  Vec3<Scalar> X02 = X[0] - X[2];
  Vec3<Scalar> X12 = X[1] - X[2];

  Scalar a01 = X01.squaredNorm();
  Scalar a02 = X02.squaredNorm();
  Scalar a12 = X12.squaredNorm();

  std::array<Vec3<Scalar>, 3> Xw = {X[0], X[1], X[2]};
  std::array<Vec3<Scalar>, 3> xw = {x[0], x[1], x[2]};

  // Switch Xw,xw so that BC is the largest distance among {X01, X02, X12}
  if (a01 > a02) {
      if (a01 > a12) {
          std::swap(xw[0], xw[2]);
          std::swap(Xw[0], Xw[2]);
          std::swap(a01, a12);
          X01 = -X12;
          X02 = -X02;
      }
  } else if (a02 > a12) {
      std::swap(xw[0], xw[1]);
      std::swap(Xw[0], Xw[1]);
      std::swap(a02, a12);
      X01 = -X01;
      X02 = X12;
  }

  const Scalar a12d = 1.0 / a12;
  const Scalar a = a01 * a12d;
  const Scalar b = a02 * a12d;

  const Scalar m01 = xw[0].dot(xw[1]);
  const Scalar m02 = xw[0].dot(xw[2]);
  const Scalar m12 = xw[1].dot(xw[2]);

  // Ugly parameters to simplify the calculation
  const Scalar m12sq = -m12 * m12 + 1.0;
  const Scalar m02sq = -1.0 + m02 * m02;
  const Scalar m01sq = -1.0 + m01 * m01;
  const Scalar ab = a * b;
  const Scalar bsq = b * b;
  const Scalar asq = a * a;
  const Scalar m013 = -2.0 + 2.0 * m01 * m02 * m12;
  const Scalar bsqm12sq = bsq * m12sq;
  const Scalar asqm12sq = asq * m12sq;
  const Scalar abm12sq = 2.0 * ab * m12sq;

  const Scalar k3_inv = 1.0 / (bsqm12sq + b * m02sq);
  const Scalar k2 = k3_inv * ((-1.0 + a) * m02sq + abm12sq + bsqm12sq + b * m013);
  const Scalar k1 = k3_inv * (asqm12sq + abm12sq + a * m013 + (-1.0 + b) * m01sq);
  const Scalar k0 = k3_inv * (asqm12sq + a * m01sq);

  Scalar s;
  bool G = solve_cubic_single_real(k2, k1, k0, s);

  Matrix3x3<Scalar> C;
  C(0, 0) = -a + s * (1 - b);
  C(0, 1) = -m02 * s;
  C(0, 2) = a * m12 + b * m12 * s;
  C(1, 0) = C(0, 1);
  C(1, 1) = s + 1;
  C(1, 2) = -m01;
  C(2, 0) = C(0, 2);
  C(2, 1) = C(1, 2);
  C(2, 2) = -a - b * s + 1;

  std::array<Vec3<Scalar>, 2> pq = compute_pq(C);

  Scalar d0, d1, d2;
  CameraPose<Scalar> pose;
  output->clear();
  Matrix3x3<Scalar> XX;

  XX << X01, X02, X01.cross(X02);
  XX = XX.inverse().eval();

  Vec3<Scalar> v1, v2;
  Matrix3x3<Scalar> YY;

  int n_sols = 0;

  for (int i = 0; i < 2; ++i) {
      // [p0 p1 p2] * [1; xw; y] = 0, or [p0 p1 p2] * [d2; d0; d1] = 0
      Scalar p0 = pq[i](0);
      Scalar p1 = pq[i](1);
      Scalar p2 = pq[i](2);

      // here we run into trouble if p0 is zero,
      // so depending on which is larger, we solve for either d0 or d1
      // The case p0 = p1 = 0 is degenerate and can be ignored
      bool switch_12 = std::abs(p0) <= std::abs(p1);

      if (switch_12) {
          // eliminate d0
          Scalar w0 = -p0 / p1;
          Scalar w1 = -p2 / p1;
          Scalar ca = 1.0 / (w1 * w1 - b);
          Scalar cb = 2.0 * (b * m12 - m02 * w1 + w0 * w1) * ca;
          Scalar cc = (w0 * w0 - 2 * m02 * w0 - b + 1.0) * ca;
          Scalar taus[2];
          if (!root2real(cb, cc, taus[0], taus[1]))
              continue;
          for (Scalar tau : taus) {
              if (tau <= 0)
                  continue;
              // positive only
              d2 = std::sqrt(a12 / (tau * (tau - 2.0 * m12) + 1.0));
              d1 = tau * d2;
              d0 = (w0 * d2 + w1 * d1);
              if (d0 < 0)
                  continue;

              refine_lambda<Scalar>(d0, d1, d2, a01, a02, a12, m01, m02, m12);
              v1 = d0 * xw[0] - d1 * xw[1];
              v2 = d0 * xw[0] - d2 * xw[2];
              YY << v1, v2, v1.cross(v2);
              Matrix3x3<Scalar> R = YY * XX;
              output->emplace_back(R, d0 * xw[0] - R * Xw[0]);
              ++n_sols;
          }
      } else {
          Scalar w0 = -p1 / p0;
          Scalar w1 = -p2 / p0;
          Scalar ca = 1.0 / (-a * w1 * w1 + 2 * a * m12 * w1 - a + 1);
          Scalar cb = 2 * (a * m12 * w0 - m01 - a * w0 * w1) * ca;
          Scalar cc = (1 - a * w0 * w0) * ca;

          Scalar taus[2];
          if (!root2real(cb, cc, taus[0], taus[1]))
              continue;
          for (Scalar tau : taus) {
              if (tau <= 0)
                  continue;
              d0 = std::sqrt(a01 / (tau * (tau - 2.0 * m01) + 1.0));
              d1 = tau * d0;
              d2 = w0 * d0 + w1 * d1;

              if (d2 < 0)
                  continue;

              refine_lambda<Scalar>(d0, d1, d2, a01, a02, a12, m01, m02, m12);
              v1 = d0 * xw[0] - d1 * xw[1];
              v2 = d0 * xw[0] - d2 * xw[2];
              YY << v1, v2, v1.cross(v2);
              Matrix3x3<Scalar> R = YY * XX;
              output->emplace_back(R, d0 * xw[0] - R * Xw[0]);
              ++n_sols;
          }
      }

      if (n_sols > 0 && G)
          break;
  }
}

template <typename Scalar>
inline std::array<Vec3<Scalar>, 2>
compute_pq(Matrix3x3<Scalar> C)
{
  std::array<Vec3<Scalar>, 2> pq;
  Matrix3x3<Scalar> C_adj;

  C_adj(0, 0) = C(1, 2) * C(2, 1) - C(1, 1) * C(2, 2);
  C_adj(1, 1) = C(0, 2) * C(2, 0) - C(0, 0) * C(2, 2);
  C_adj(2, 2) = C(0, 1) * C(1, 0) - C(0, 0) * C(1, 1);
  C_adj(0, 1) = C(0, 1) * C(2, 2) - C(0, 2) * C(2, 1);
  C_adj(0, 2) = C(0, 2) * C(1, 1) - C(0, 1) * C(1, 2);
  C_adj(1, 0) = C_adj(0, 1);
  C_adj(1, 2) = C(0, 0) * C(1, 2) - C(0, 2) * C(1, 0);
  C_adj(2, 0) = C_adj(0, 2);
  C_adj(2, 1) = C_adj(1, 2);

  Vec3<Scalar> v;
  if (C_adj(0, 0) > C_adj(1, 1))
  {
    if (C_adj(0, 0) > C_adj(2, 2))
    {
      v = C_adj.col(0) / std::sqrt(C_adj(0, 0));
    }
    else
    {
      v = C_adj.col(2) / std::sqrt(C_adj(2, 2));
    }
  }
  else if (C_adj(1, 1) > C_adj(2, 2))
  {
    v = C_adj.col(1) / std::sqrt(C_adj(1, 1));
  }
  else
  {
    v = C_adj.col(2) / std::sqrt(C_adj(2, 2));
  }

  C(0, 1) -= v(2);
  C(0, 2) += v(1);
  C(1, 2) -= v(0);
  C(1, 0) += v(2);
  C(2, 0) -= v(1);
  C(2, 1) += v(0);

  pq[0] = C.col(0);
  pq[1] = C.row(0);

  return pq;
}

#if defined(NATIVE)
template <typename Scalar>
int p3p(const std::vector<Vec3<Scalar>> &x,
        const std::vector<Vec3<Scalar>> &X,
              std::vector<CameraPose<Scalar>>* output)
{
  Vec3<Scalar> X01 = X[0] - X[1];
  Vec3<Scalar> X02 = X[0] - X[2];
  Vec3<Scalar> X12 = X[1] - X[2];

  Scalar a01 = X01.squaredNorm();
  Scalar a02 = X02.squaredNorm();
  Scalar a12 = X12.squaredNorm();

  std::array<Vec3<Scalar>, 3> Xw = {X[0], X[1], X[2]};
  std::array<Vec3<Scalar>, 3> xw = {x[0], x[1], x[2]};

  // Switch Xw,xw so that BC is the largest distance among {X01, X02, X12}
  if (a01 > a02) {
      if (a01 > a12) {
          std::swap(xw[0], xw[2]);
          std::swap(Xw[0], Xw[2]);
          std::swap(a01, a12);
          X01 = -X12;
          X02 = -X02;
      }
  } else if (a02 > a12) {
      std::swap(xw[0], xw[1]);
      std::swap(Xw[0], Xw[1]);
      std::swap(a02, a12);
      X01 = -X01;
      X02 = X12;
  }

  const Scalar a12d = 1.0 / a12;
  const Scalar a = a01 * a12d;
  const Scalar b = a02 * a12d;

  const Scalar m01 = xw[0].dot(xw[1]);
  const Scalar m02 = xw[0].dot(xw[2]);
  const Scalar m12 = xw[1].dot(xw[2]);

  // Ugly parameters to simplify the calculation
  const Scalar m12sq = -m12 * m12 + 1.0;
  const Scalar m02sq = -1.0 + m02 * m02;
  const Scalar m01sq = -1.0 + m01 * m01;
  const Scalar ab = a * b;
  const Scalar bsq = b * b;
  const Scalar asq = a * a;
  const Scalar m013 = -2.0 + 2.0 * m01 * m02 * m12;
  const Scalar bsqm12sq = bsq * m12sq;
  const Scalar asqm12sq = asq * m12sq;
  const Scalar abm12sq = 2.0 * ab * m12sq;

  const Scalar k3_inv = 1.0 / (bsqm12sq + b * m02sq);
  const Scalar k2 = k3_inv * ((-1.0 + a) * m02sq + abm12sq + bsqm12sq + b * m013);
  const Scalar k1 = k3_inv * (asqm12sq + abm12sq + a * m013 + (-1.0 + b) * m01sq);
  const Scalar k0 = k3_inv * (asqm12sq + a * m01sq);

  Scalar s;
  bool G = solve_cubic_single_real(k2, k1, k0, s);

  Matrix3x3<Scalar> C;
  C(0, 0) = -a + s * (1 - b);
  C(0, 1) = -m02 * s;
  C(0, 2) = a * m12 + b * m12 * s;
  C(1, 0) = C(0, 1);
  C(1, 1) = s + 1;
  C(1, 2) = -m01;
  C(2, 0) = C(0, 2);
  C(2, 1) = C(1, 2);
  C(2, 2) = -a - b * s + 1;

  std::array<Vec3<Scalar>, 2> pq = compute_pq(C);

  Scalar d0, d1, d2;
  CameraPose<Scalar> pose;
  output->clear();
  Matrix3x3<Scalar> XX;

  XX << X01, X02, X01.cross(X02);
  XX = XX.inverse().eval();

  Vec3<Scalar> v1, v2;
  Matrix3x3<Scalar> YY;

  int n_sols = 0;

  for (int i = 0; i < 2; ++i) {
      // [p0 p1 p2] * [1; xw; y] = 0, or [p0 p1 p2] * [d2; d0; d1] = 0
      Scalar p0 = pq[i](0);
      Scalar p1 = pq[i](1);
      Scalar p2 = pq[i](2);

      // here we run into trouble if p0 is zero,
      // so depending on which is larger, we solve for either d0 or d1
      // The case p0 = p1 = 0 is degenerate and can be ignored
      bool switch_12 = std::abs(p0) <= std::abs(p1);

      if (switch_12) {
          // eliminate d0
          Scalar w0 = -p0 / p1;
          Scalar w1 = -p2 / p1;
          Scalar ca = 1.0 / (w1 * w1 - b);
          Scalar cb = 2.0 * (b * m12 - m02 * w1 + w0 * w1) * ca;
          Scalar cc = (w0 * w0 - 2 * m02 * w0 - b + 1.0) * ca;
          Scalar taus[2];
          if (!root2real(cb, cc, taus[0], taus[1]))
              continue;
          for (Scalar tau : taus) {
              if (tau <= 0)
                  continue;
              // positive only
              d2 = std::sqrt(a12 / (tau * (tau - 2.0 * m12) + 1.0));
              d1 = tau * d2;
              d0 = (w0 * d2 + w1 * d1);
              if (d0 < 0)
                  continue;

              refine_lambda(d0, d1, d2, a01, a02, a12, m01, m02, m12);
              v1 = d0 * xw[0] - d1 * xw[1];
              v2 = d0 * xw[0] - d2 * xw[2];
              YY << v1, v2, v1.cross(v2);
              Matrix3x3<Scalar> R = YY * XX;
              output->emplace_back(R, d0 * xw[0] - R * Xw[0]);
              ++n_sols;
          }
      } else {
          Scalar w0 = -p1 / p0;
          Scalar w1 = -p2 / p0;
          Scalar ca = 1.0 / (-a * w1 * w1 + 2 * a * m12 * w1 - a + 1);
          Scalar cb = 2 * (a * m12 * w0 - m01 - a * w0 * w1) * ca;
          Scalar cc = (1 - a * w0 * w0) * ca;

          Scalar taus[2];
          if (!root2real(cb, cc, taus[0], taus[1]))
              continue;
          for (Scalar tau : taus) {
              if (tau <= 0)
                  continue;
              d0 = std::sqrt(a01 / (tau * (tau - 2.0 * m01) + 1.0));
              d1 = tau * d0;
              d2 = w0 * d0 + w1 * d1;

              if (d2 < 0)
                  continue;

              refine_lambda(d0, d1, d2, a01, a02, a12, m01, m02, m12);
              v1 = d0 * xw[0] - d1 * xw[1];
              v2 = d0 * xw[0] - d2 * xw[2];
              YY << v1, v2, v1.cross(v2);
              Matrix3x3<Scalar> R = YY * XX;
              output->emplace_back(R, d0 * xw[0] - R * Xw[0]);
              ++n_sols;
          }
      }

      if (n_sols > 0 && G)
          break;
  }
  return output->size();
}
#endif // if defined(NATIVE)

} // namespace EntoPose

#endif // P3P_H
