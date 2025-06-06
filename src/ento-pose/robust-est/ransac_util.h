#ifndef ENTO_RANSAC_UTIL_H
#define ENTO_RANSAC_UTIL_H

#include <cstddef>
#include <limits>
#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>
#include <ento-math/core.h>
#include <ento-util/debug.h>

namespace EntoPose
{

using namespace EntoUtil;
using namespace EntoMath;

template <typename Scalar>
struct RansacStats
{
  size_t refinements = 0;
  size_t iters = 0;
  size_t num_inliers = 0;
  Scalar inlier_ratio = 0;
  Scalar model_score = std::numeric_limits<Scalar>::max();
};

// === LO-RANSAC Local Refinement Options ===
enum class LocalRefinementType {
    None,         // No local refinement
    Linear,       // Linear refinement (e.g., 8pt, DLT, upright planar 3pt)
    BundleAdjust, // Nonlinear refinement (bundle adjustment)
    Both          // Linear + Nonlinear (do both in sequence)
};

enum class LinearRefinementMethod {
    EightPoint,        // For general relative pose
    UprightPlanar3pt,  // For upright planar relpose
    DLT                // For absolute pose, homography, etc.
};

template <typename Scalar, bool ProgressiveSampling = false, size_t ProgressiveSamplingIters=100000>
struct RansacOptions
{
  size_t max_iters = 0;
  size_t min_iters = 0;
  Scalar dynamic_trials_mult = 3.0;
  Scalar success_prob = 0.9999;
  Scalar max_reproj_error = 12.0;
  Scalar max_epipolar_error = 1.0;
  unsigned long seed = 0;
  bool score_initial_model = false;
  /**
   * Local optimization (LO) refinement type for LO-RANSAC:
   *   None:         No local refinement
   *   Linear:       Linear refinement (e.g., 8pt, DLT, upright planar 3pt)
   *   BundleAdjust: Nonlinear refinement (bundle adjustment)
   */
  LocalRefinementType lo_type = LocalRefinementType::None;
  /**
   * Linear refinement method to use if lo_type == Linear:
   *   EightPoint:        General relative pose (8pt)
   *   UprightPlanar3pt:  Upright planar relpose (3pt, N >= 3)
   *   DLT:               Absolute pose, homography.
   */
  LinearRefinementMethod linear_method = LinearRefinementMethod::EightPoint;

  // IRLS (Iteratively Reweighted Least Squares) options for linear refinement
  bool use_irls = false;                    // Enable IRLS for linear refinement
  int irls_max_iters = 5;                   // Maximum IRLS iterations
  Scalar irls_huber_threshold = Scalar(1.0); // Huber loss threshold for IRLS

  bool final_refinement = true;
  static constexpr bool progressive_sampling = ProgressiveSampling;
  static constexpr size_t max_prosac_iterations = ProgressiveSamplingIters;
};

template <typename Scalar, size_t N = 0>
Scalar compute_msac_score(const CameraPose<Scalar> &pose,
                          const EntoContainer<Vec2<Scalar>, N> &x,
                          const EntoContainer<Vec3<Scalar>, N> &X,
                          Scalar threshold_squared,
                          size_t *inlier_count = nullptr) {
    *inlier_count = 0;
    Scalar score = 0.0;
    const Matrix3x3<Scalar> R = pose.R();
    const Scalar P0_0 = R(0, 0), P0_1 = R(0, 1), P0_2 = R(0, 2), P0_3 = pose.t(0);
    const Scalar P1_0 = R(1, 0), P1_1 = R(1, 1), P1_2 = R(1, 2), P1_3 = pose.t(1);
    const Scalar P2_0 = R(2, 0), P2_1 = R(2, 1), P2_2 = R(2, 2), P2_3 = pose.t(2);

    for (size_t k = 0; k < x.size(); ++k) {
        const Scalar X0 = X[k](0), X1 = X[k](1), X2 = X[k](2);
        const Scalar x0 = x[k](0), x1 = x[k](1);
        const Scalar z0 = P0_0 * X0 + P0_1 * X1 + P0_2 * X2 + P0_3;
        const Scalar z1 = P1_0 * X0 + P1_1 * X1 + P1_2 * X2 + P1_3;
        const Scalar z2 = P2_0 * X0 + P2_1 * X1 + P2_2 * X2 + P2_3;
        const Scalar inv_z2 = 1.0 / z2;

        const Scalar r_0 = z0 * inv_z2 - x0;
        const Scalar r_1 = z1 * inv_z2 - x1;
        const Scalar r_sq = r_0 * r_0 + r_1 * r_1;
        if (r_sq < threshold_squared && z2 > 0.0) 
        {
            (*inlier_count)++;
            score += r_sq;
        }
    }
    score += (x.size() - *inlier_count) * threshold_squared;
    return score;
}


// Returns MSAC score of the Sampson error (checks cheirality of points as well)
template <typename Scalar, size_t N = 0>
Scalar compute_sampson_msac_score(const CameraPose<Scalar> &pose,
                                  const EntoContainer<Vec2<Scalar>, N> &x1,
                                  const EntoContainer<Vec2<Scalar>, N> &x2,
                                  Scalar sq_threshold,
                                  size_t *inlier_count)
{
  *inlier_count = 0;
  Matrix3x3<Scalar> E;
  essential_from_motion(pose, &E);

  // For some reason this is a lot faster than just using nice Eigen expressions...
  const Scalar E0_0 = E(0, 0), E0_1 = E(0, 1), E0_2 = E(0, 2);
  const Scalar E1_0 = E(1, 0), E1_1 = E(1, 1), E1_2 = E(1, 2);
  const Scalar E2_0 = E(2, 0), E2_1 = E(2, 1), E2_2 = E(2, 2);

  Scalar score = 0.0;
  for (size_t k = 0; k < x1.size(); ++k) {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Ex1_0 = E0_0 * x1_0 + E0_1 * x1_1 + E0_2;
    const Scalar Ex1_1 = E1_0 * x1_0 + E1_1 * x1_1 + E1_2;
    const Scalar Ex1_2 = E2_0 * x1_0 + E2_1 * x1_1 + E2_2;

    const Scalar Ex2_0 = E0_0 * x2_0 + E1_0 * x2_1 + E2_0;
    const Scalar Ex2_1 = E0_1 * x2_0 + E1_1 * x2_1 + E2_1;
    // const Scalar Ex2_2 = E0_2 * x2_0 + E1_2 * x2_1 + E2_2;

    const Scalar C = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;
    const Scalar Cx = Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1;
    const Scalar Cy = Ex2_0 * Ex2_0 + Ex2_1 * Ex2_1;
    const Scalar r2 = C * C / (Cx + Cy);

    //ENTO_DEBUG("Point %zu - Sampson error: %f, threshold: %f", k, r2, sq_threshold);

    if (r2 < sq_threshold) {
        bool cheirality =
            check_cheirality(pose, x1[k].homogeneous().normalized(), x2[k].homogeneous().normalized(), Scalar(0.01));
        if (cheirality) {
            (*inlier_count)++;
            score += r2;
            //ENTO_DEBUG("Point %zu - ACCEPTED as inlier", k);
        } else {
            score += sq_threshold;
            //ENTO_DEBUG("Point %zu - REJECTED by cheirality check", k);
        }
    } else {
        score += sq_threshold;
        //ENTO_DEBUG("Point %zu - REJECTED by Sampson error", k);
    }
  }
  return score;
}

// Returns MSAC score of the Sampson error (no cheirality check)
template <typename Scalar, size_t N = 0>
Scalar compute_sampson_msac_score(const Matrix3x3<Scalar> &E,
                                  const EntoContainer<Vec2<Scalar>, N> &x1,
                                  const EntoContainer<Vec2<Scalar>, N> &x2,
                                  Scalar sq_threshold,
                                  size_t *inlier_count)
{
  *inlier_count = 0;

  // For some reason this is a lot faster than just using nice Eigen expressions...
  const Scalar E0_0 = E(0, 0), E0_1 = E(0, 1), E0_2 = E(0, 2);
  const Scalar E1_0 = E(1, 0), E1_1 = E(1, 1), E1_2 = E(1, 2);
  const Scalar E2_0 = E(2, 0), E2_1 = E(2, 1), E2_2 = E(2, 2);

  Scalar score = 0.0;
  for (size_t k = 0; k < x1.size(); ++k)
  {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Ex1_0 = E0_0 * x1_0 + E0_1 * x1_1 + E0_2;
    const Scalar Ex1_1 = E1_0 * x1_0 + E1_1 * x1_1 + E1_2;
    const Scalar Ex1_2 = E2_0 * x1_0 + E2_1 * x1_1 + E2_2;

    const Scalar Ex2_0 = E0_0 * x2_0 + E1_0 * x2_1 + E2_0;
    const Scalar Ex2_1 = E0_1 * x2_0 + E1_1 * x2_1 + E2_1;
    // const Scalar Ex2_2 = E0_2 * x2_0 + E1_2 * x2_1 + E2_2;

    const Scalar C = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;
    const Scalar Cx = Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1;
    const Scalar Cy = Ex2_0 * Ex2_0 + Ex2_1 * Ex2_1;
    const Scalar r2 = C * C / (Cx + Cy);

    if (r2 < sq_threshold)
    {
      (*inlier_count)++;
      score += r2;
    }
    else
    {
      score += sq_threshold;
    }
  }
  return score;
}

template <typename Scalar, size_t N = 0>
Scalar compute_homography_msac_score(const Matrix3x3<Scalar> &H,
                                     const EntoContainer<Vec2<Scalar>, N> &x1,
                                     const EntoContainer<Vec2<Scalar>, N> &x2,
                                     Scalar sq_threshold,
                                     size_t *inlier_count)
{
  *inlier_count = 0;
  Scalar score = 0;

  const Scalar H0_0 = H(0, 0), H0_1 = H(0, 1), H0_2 = H(0, 2);
  const Scalar H1_0 = H(1, 0), H1_1 = H(1, 1), H1_2 = H(1, 2);
  const Scalar H2_0 = H(2, 0), H2_1 = H(2, 1), H2_2 = H(2, 2);

  for (size_t k = 0; k < x1.size(); ++k)
  {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Hx1_0 = H0_0 * x1_0 + H0_1 * x1_1 + H0_2;
    const Scalar Hx1_1 = H1_0 * x1_0 + H1_1 * x1_1 + H1_2;
    const Scalar inv_Hx1_2 = 1.0 / (H2_0 * x1_0 + H2_1 * x1_1 + H2_2);

    const Scalar r0 = Hx1_0 * inv_Hx1_2 - x2_0;
    const Scalar r1 = Hx1_1 * inv_Hx1_2 - x2_1;
    const Scalar r2 = r0 * r0 + r1 * r1;

    if (r2 < sq_threshold)
    {
      (*inlier_count)++;
      score += r2;
    }
    else
    {
      score += sq_threshold;
    }
  }
  return score;
}

template <typename Scalar, size_t N = 0>
void get_homography_inliers(const Matrix3x3<Scalar> &H,
                            const EntoContainer<Vec2<Scalar>, N> &x1,
                            const EntoContainer<Vec2<Scalar>, N> &x2,
                            Scalar sq_threshold,
                            EntoContainer<char, N> *inliers)
{
  const Scalar H0_0 = H(0, 0), H0_1 = H(0, 1), H0_2 = H(0, 2);
  const Scalar H1_0 = H(1, 0), H1_1 = H(1, 1), H1_2 = H(1, 2);
  const Scalar H2_0 = H(2, 0), H2_1 = H(2, 1), H2_2 = H(2, 2);

  if constexpr (N == 0) 
    inliers->resize(x1.size());
  inliers->clear();

  for (size_t k = 0; k < x1.size(); ++k)
  {
      const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
      const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

      const Scalar Hx1_0 = H0_0 * x1_0 + H0_1 * x1_1 + H0_2;
      const Scalar Hx1_1 = H1_0 * x1_0 + H1_1 * x1_1 + H1_2;
      const Scalar inv_Hx1_2 = 1.0 / (H2_0 * x1_0 + H2_1 * x1_1 + H2_2);

      const Scalar r0 = Hx1_0 * inv_Hx1_2 - x2_0;
      const Scalar r1 = Hx1_1 * inv_Hx1_2 - x2_1;
      const Scalar r2 = r0 * r0 + r1 * r1;
      (*inliers)[k] = (r2 < sq_threshold);
  }
}

// Compute inliers for absolute pose estimation (using reprojection error and cheirality check)
template <typename Scalar, size_t N = 0>
int get_inliers(const CameraPose<Scalar> &pose,
                const EntoUtil::EntoContainer<Vec2<Scalar>, N> &x,
                const EntoUtil::EntoContainer<Vec3<Scalar>, N> &X,
                Scalar threshold_squared,
                EntoUtil::EntoContainer<uint8_t, N> *inliers)
{
    // Only resize for dynamic containers (N=0)
    if constexpr (N == 0)
      inliers->reserve(x.size());

    inliers->clear();
    
    const Matrix3x3<Scalar> R = pose.R();
    
    ENTO_DEBUG("In get_inliers! x.size: %i, X.size: %i", x.size(), x.size());
    for (size_t k = 0; k < x.size(); ++k) {
        Vec3<Scalar> Z = R * X[k] + pose.t;
        Scalar r2 = (Z.hnormalized() - x[k]).squaredNorm();
        bool flag = (r2 < threshold_squared && Z(2) > 0.0);
        ENTO_DEBUG("Inliers[%d] = (%f < %f && Z(2) [%f] > 0.0 ? %i)",
                   k, r2, threshold_squared, Z(2), flag);
        (*inliers).push_back(r2 < threshold_squared && Z(2) > 0.0);
    }

  return inliers->size();
}

// Compute inliers for relative pose estimation (using Sampson error)
template <typename Scalar, size_t N = 0>
int get_inliers(const CameraPose<Scalar> &pose,
                const EntoContainer<Vec2<Scalar>, N> &x1,
                const EntoContainer<Vec2<Scalar>, N> &x2,
                Scalar sq_threshold,
                EntoContainer<uint8_t, N> *inliers) {
  if constexpr(N == 0)
    inliers->reserve(x1.size());

  inliers->clear();

  Matrix3x3<Scalar> E;
  essential_from_motion(pose, &E);
  const Scalar E0_0 = E(0, 0), E0_1 = E(0, 1), E0_2 = E(0, 2);
  const Scalar E1_0 = E(1, 0), E1_1 = E(1, 1), E1_2 = E(1, 2);
  const Scalar E2_0 = E(2, 0), E2_1 = E(2, 1), E2_2 = E(2, 2);

  size_t inlier_count = 0.0;
  for (size_t k = 0; k < x1.size(); ++k)
  {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Ex1_0 = E0_0 * x1_0 + E0_1 * x1_1 + E0_2;
    const Scalar Ex1_1 = E1_0 * x1_0 + E1_1 * x1_1 + E1_2;
    const Scalar Ex1_2 = E2_0 * x1_0 + E2_1 * x1_1 + E2_2;

    const Scalar Ex2_0 = E0_0 * x2_0 + E1_0 * x2_1 + E2_0;
    const Scalar Ex2_1 = E0_1 * x2_0 + E1_1 * x2_1 + E2_1;
    // const Scalar Ex2_2 = E0_2 * x2_0 + E1_2 * x2_1 + E2_2;

    const Scalar C = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

    const Scalar Cx = Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1;
    const Scalar Cy = Ex2_0 * Ex2_0 + Ex2_1 * Ex2_1;

    const Scalar r2 = C * C / (Cx + Cy);

    bool inlier = (r2 < sq_threshold);
    if (inlier)
    {
      bool cheirality =
          check_cheirality(pose, x1[k].homogeneous().normalized(), x2[k].homogeneous().normalized(), Scalar(0.01));

      if (cheirality)
      {
        inlier_count++;
      }
      else
      {
        inlier = false;
      }
    }
    (*inliers).push_back(inlier);
  }
  return inlier_count;
}

// Compute inliers for relative pose estimation (using Sampson error)
template <typename Scalar, size_t N = 0>
int get_inliers(const Matrix3x3<Scalar> &E,
                const EntoContainer<Vec2<Scalar>, N> &x1,
                const EntoContainer<Vec2<Scalar>, N> &x2,
                Scalar sq_threshold,
                EntoContainer<uint8_t, N> *inliers) {
  if constexpr (N == 0)
    inliers->reserve(x1.size());

  inliers->clear();

  const Scalar E0_0 = E(0, 0), E0_1 = E(0, 1), E0_2 = E(0, 2);
  const Scalar E1_0 = E(1, 0), E1_1 = E(1, 1), E1_2 = E(1, 2);
  const Scalar E2_0 = E(2, 0), E2_1 = E(2, 1), E2_2 = E(2, 2);

  size_t inlier_count = 0.0;
  for (size_t k = 0; k < x1.size(); ++k)
  {
    const Scalar x1_0 = x1[k](0), x1_1 = x1[k](1);
    const Scalar x2_0 = x2[k](0), x2_1 = x2[k](1);

    const Scalar Ex1_0 = E0_0 * x1_0 + E0_1 * x1_1 + E0_2;
    const Scalar Ex1_1 = E1_0 * x1_0 + E1_1 * x1_1 + E1_2;
    const Scalar Ex1_2 = E2_0 * x1_0 + E2_1 * x1_1 + E2_2;

    const Scalar Ex2_0 = E0_0 * x2_0 + E1_0 * x2_1 + E2_0;
    const Scalar Ex2_1 = E0_1 * x2_0 + E1_1 * x2_1 + E2_1;
    // const Scalar Ex2_2 = E0_2 * x2_0 + E1_2 * x2_1 + E2_2;

    const Scalar C = x2_0 * Ex1_0 + x2_1 * Ex1_1 + Ex1_2;

    const Scalar Cx = Ex1_0 * Ex1_0 + Ex1_1 * Ex1_1;
    const Scalar Cy = Ex2_0 * Ex2_0 + Ex2_1 * Ex2_1;

    const Scalar r2 = C * C / (Cx + Cy);

    bool inlier = (r2 < sq_threshold);

    if (inlier) inlier_count++;

    (*inliers).push_back(inlier);
  }
  return inlier_count;
}

}

#endif // ENTO_RANSAC_UTIL_H
