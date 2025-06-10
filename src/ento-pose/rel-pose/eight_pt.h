#ifndef EIGHT_PT_H
#define EIGHT_PT_H

#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>

using namespace EntoUtil;

namespace EntoPose
{
// Relative pose from eight to n bearing vector correspondences.
// Port from OpenMVG (Essential_matrix computation then decomposition in 4 pose [R|t]).
template <typename Scalar, std::size_t N=0>
int relpose_8pt(const EntoContainer<Vec3<Scalar>, N> &x1,
                const EntoContainer<Vec3<Scalar>, N> &x2,
                EntoArray<CameraPose<Scalar>, 4>* output);

// Computation of essential matrix from eight to n bearing vector correspondences.
// See page 294 in [HZ] Result 11.1.
// [HZ] Multiple View Geometry - Richard Hartley, Andrew Zisserman - second edition
// Port from OpenMVG
template <typename Scalar>
void essential_matrix_8pt(const std::vector<Vec3<Scalar>> &x1,
                          const std::vector<Vec3<Scalar>> &x2,
                          Matrix3x3<Scalar>*              essential_matrix);

template <typename Scalar, std::size_t N>
void essential_matrix_8pt(const EntoArray<Vec3<Scalar>, N> &x1,
                          const EntoArray<Vec3<Scalar>, N> &x2,
                          Matrix3x3<Scalar>*              essential_matrix);

// ==================================================
/**
 * Build a 9 x n matrix from bearing vector matches, where each row is equivalent to the
 * equation x'T*F*x = 0 for a single correspondence pair (x', x).
 *
 * Note that this does not resize the matrix A; it is expected to have the
 * appropriate size already (n x 9).
 */

template <typename Scalar>
void encode_epipolar_equation(const std::vector<Vec3<Scalar>> &x1,
                              const std::vector<Vec3<Scalar>> &x2,
                              Eigen::Matrix<Scalar, Eigen::Dynamic, 9> *A)
{
  for (size_t i = 0; i < x1.size(); ++i)
  {
    A->row(i) << x2[i].x() * x1[i].transpose(), x2[i].y() * x1[i].transpose(), x2[i].z() * x1[i].transpose();
  }
}

template <typename Scalar, std::size_t N>
void encode_epipolar_equation(const EntoContainer<Vec3<Scalar>, N> &x1,
                              const EntoContainer<Vec3<Scalar>, N> &x2,
                              Eigen::Matrix<Scalar, N, 9> *A) {
  for (size_t i = 0; i < N; ++i)
  {
    A->row(i) << x2[i].x() * x1[i].transpose(), x2[i].y() * x1[i].transpose(), x2[i].z() * x1[i].transpose();
  }
}

// Local isotropic normalization utility for bearing vectors
// Normalizes bearing vectors in image plane (x/z, y/z) so mean distance from centroid is sqrt(2)
template <typename Scalar, size_t N>
static inline void iso_normalize_bearing_vectors(const EntoUtil::EntoContainer<Vec3<Scalar>, N>& bearing_vectors,
                                               EntoUtil::EntoContainer<Vec3<Scalar>, N>& normed_vectors,
                                               Matrix3x3<Scalar>& T)
{
    const size_t n = bearing_vectors.size();
    normed_vectors.clear();
    // For dynamic-size EntoContainer (N==0), reserve to avoid reallocations
    if constexpr (N == 0)
        normed_vectors.reserve(n);
        
    // Project bearing vectors to image plane (x/z, y/z) and compute centroid
    Vec2<Scalar> centroid = Vec2<Scalar>::Zero();
    for (size_t i = 0; i < n; ++i) {
        const Vec3<Scalar>& v = bearing_vectors[i];
        Vec2<Scalar> img_pt(v.x() / v.z(), v.y() / v.z());  // Project to image plane
        centroid += img_pt;
    }
    centroid /= Scalar(n);
    
    // Compute mean distance from centroid in image plane
    Scalar mean_dist = 0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3<Scalar>& v = bearing_vectors[i];
        Vec2<Scalar> img_pt(v.x() / v.z(), v.y() / v.z());
        mean_dist += (img_pt - centroid).norm();
    }
    mean_dist /= Scalar(n);
    Scalar scale = std::sqrt(Scalar(2)) / (mean_dist > Scalar(1e-12) ? mean_dist : Scalar(1));
    
    // Build normalization matrix T (3x3 homogeneous)
    T.setIdentity();
    T(0,0) = scale; T(1,1) = scale;
    T(0,2) = -scale * centroid(0);
    T(1,2) = -scale * centroid(1);
    
    // Apply transformation and renormalize to unit bearing vectors
    for (size_t i = 0; i < n; ++i) {
        Vec3<Scalar> transformed = T * bearing_vectors[i];
        normed_vectors.push_back(transformed.normalized());
    }
}

template <typename Scalar>
void essential_matrix_8pt(const std::vector<Vec3<Scalar>> &x1,
                          const std::vector<Vec3<Scalar>> &x2,
                          Matrix3x3<Scalar>* essential_matrix)
{
  // COMMENTED OUT: Isotropic normalization for numerical stability
  /*
  std::vector<Vec3<Scalar>> x1_norm, x2_norm;
  Matrix3x3<Scalar> T1, T2;
  
  // Convert to EntoContainer for normalization
  EntoUtil::EntoContainer<Vec3<Scalar>, 0> x1_container, x2_container;
  for (const auto& x : x1) x1_container.push_back(x);
  for (const auto& x : x2) x2_container.push_back(x);
  
  EntoUtil::EntoContainer<Vec3<Scalar>, 0> x1_norm_container, x2_norm_container;
  iso_normalize_bearing_vectors<Scalar, 0>(x1_container, x1_norm_container, T1);
  iso_normalize_bearing_vectors<Scalar, 0>(x2_container, x2_norm_container, T2);
  
  // Convert back to std::vector
  x1_norm.reserve(x1.size());
  x2_norm.reserve(x2.size());
  for (const auto& x : x1_norm_container) x1_norm.push_back(x);
  for (const auto& x : x2_norm_container) x2_norm.push_back(x);
  */
  
  // Original logic with original coordinates (no normalization)
  using MatX9 = Eigen::Matrix<Scalar, Eigen::Dynamic, 9>;
  MatX9 epipolar_constraint(x1.size(), 9);
  encode_epipolar_equation<Scalar>(x1, x2, &epipolar_constraint);

  using RMat3 = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;
  Matrix3x3<Scalar> E;
  if (x1.size() == 8)
  {
    Eigen::Matrix<Scalar, 9, 9> Q = epipolar_constraint.transpose().householderQr().householderQ();
    Eigen::Matrix<Scalar, 9, 1> e = Q.col(8);
    E = Eigen::Map<const RMat3>(e.data());
  }
  else
  {
    // Use Jacobi SVD directly on A (more accurate than Gram matrix A^T*A)
    Eigen::JacobiSVD<MatX9> svd(epipolar_constraint, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 9, 1> e = svd.matrixV().col(8);  // Smallest singular vector
    E = Eigen::Map<const RMat3>(e.data());
  }

  // Find the closest essential matrix to E in frobenius norm
  Eigen::JacobiSVD<Matrix3x3<Scalar>> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec3<Scalar> d = USV.singularValues();
  const Scalar a = d[0];
  const Scalar b = d[1];
  d << (a + b) / 2., (a + b) / 2., 0.0;
  E = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();

  // No denormalization needed since we're not using normalization
  (*essential_matrix) = E;
}

template <typename Scalar, std::size_t N>
void essential_matrix_8pt(const EntoArray<Vec3<Scalar>, N> &x1,
                          const EntoArray<Vec3<Scalar>, N> &x2,
                          Matrix3x3<Scalar>* essential_matrix)
{
  // COMMENTED OUT: Isotropic normalization for numerical stability
  /*
  EntoUtil::EntoContainer<Vec3<Scalar>, N> x1_norm, x2_norm;
  Matrix3x3<Scalar> T1, T2;
  
  // Convert to EntoContainer for normalization
  EntoUtil::EntoContainer<Vec3<Scalar>, N> x1_container, x2_container;
  for (size_t i = 0; i < x1.size(); ++i) {
    x1_container.push_back(x1[i]);
    x2_container.push_back(x2[i]);
  }
  
  iso_normalize_bearing_vectors<Scalar, N>(x1_container, x1_norm, T1);
  iso_normalize_bearing_vectors<Scalar, N>(x2_container, x2_norm, T2);
  */
  
  // Original logic with original coordinates (no normalization)
  using MatX9 = Eigen::Matrix<Scalar, N, 9>;
  MatX9 epipolar_constraint;
  encode_epipolar_equation<Scalar, N>(x1, x2, &epipolar_constraint);

  using RMat3 = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;
  Matrix3x3<Scalar> E;
  if (x1.size() == 8)
  {
    Eigen::Matrix<Scalar, 9, 9> Q = epipolar_constraint.transpose().householderQr().householderQ();
    Eigen::Matrix<Scalar, 9, 1> e = Q.col(8);
    E = Eigen::Map<const RMat3>(e.data());
  }
  else
  {
    // Use Jacobi SVD directly on A (more accurate than Gram matrix A^T*A)
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, N, 9>> svd(epipolar_constraint, Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 9, 1> e = svd.matrixV().col(8);  // Smallest singular vector
    E = Eigen::Map<const RMat3>(e.data());
  }

  // Find the closest essential matrix to E in frobenius norm
  Eigen::JacobiSVD<Matrix3x3<Scalar>> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec3<Scalar> d = USV.singularValues();
  const Scalar a = d[0];
  const Scalar b = d[1];
  d << (a + b) / 2., (a + b) / 2., 0.0;
  E = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();

  // No denormalization needed since we're not using normalization
  (*essential_matrix) = E;
}

template <typename Scalar, std::size_t N>
int relpose_8pt(const EntoContainer<Vec3<Scalar>, N> &x1,
                const EntoContainer<Vec3<Scalar>, N> &x2,
                EntoArray<CameraPose<Scalar>, 4> *output) {

  Matrix3x3<Scalar> essential_matrix;
  if constexpr(N == 0)
    essential_matrix_8pt(x1, x2, &essential_matrix);
  else
    essential_matrix_8pt<Scalar, N>(x1, x2, &essential_matrix);
  // Debug: print essential matrix

  // Generate plausible relative motion from E
  output->clear();
  motion_from_essential<Scalar, N, 4>(essential_matrix, x1, x2, output);
  return output->size();
}

}

#endif // EIGHT_PT_H
