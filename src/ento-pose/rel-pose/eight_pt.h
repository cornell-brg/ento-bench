#ifndef EIGHT_PT_H
#define EIGHT_PT_H

#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>

using namespace EntoUtil;

namespace EntoPose
{
// Relative pose from eight to n bearing vector correspondences.
// Port from OpenMVG (Essential_matrix computation then decomposition in 4 pose [R|t]).
template <typename Scalar, std::size_t N>
int relpose_8pt(const EntoArray<Vec3<Scalar>, N> &x1,
                const EntoArray<Vec3<Scalar>, N> &x2,
                EntoArray<CameraPose<Scalar>, N>* output);

// Computation of essential matrix from eight to n bearing vector correspondences.
// See page 294 in [HZ] Result 11.1.
// [HZ] Multiple View Geometry - Richard Hartley, Andrew Zisserman - second edition
// Port from OpenMVG
template <typename Scalar, std::size_t N>
void essential_matrix_8pt(const EntoArray<Vec3<Scalar>, N> &x1,
                          const EntoArray<Vec3<Scalar>, N> &x2,
                          Matrix3x3<Scalar>*               essential_matrix);


// ==================================================
/**
 * Build a 9 x n matrix from bearing vector matches, where each row is equivalent to the
 * equation x'T*F*x = 0 for a single correspondence pair (x', x).
 *
 * Note that this does not resize the matrix A; it is expected to have the
 * appropriate size already (n x 9).
 */
template <typename Scalar, std::size_t N>
void encode_epipolar_equation(const EntoArray<Vec3<Scalar>, N> &x1,
                              const EntoArray<Vec3<Scalar>, N> &x2,
                              Eigen::Matrix<Scalar, N, 9> *A) {
  for (size_t i = 0; i < x1.size(); ++i)
  {
    A->row(i) << x2[i].x() * x1[i].transpose(), x2[i].y() * x1[i].transpose(), x2[i].z() * x1[i].transpose();
  }
}

template <typename Scalar, std::size_t N>
void essential_matrix_8pt(const std::vector<Eigen::Vector3d> &x1,
                          const std::vector<Eigen::Vector3d> &x2,
                          Matrix3x3<Scalar> *essential_matrix)
{
  using MatX9 = Eigen::Matrix<Scalar, N, 9>;
  MatX9 epipolar_constraint(x1.size(), 9);
  encode_epipolar_equation(x1, x2, &epipolar_constraint);

  using RMat3 = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;
  Matrix3x3<Scalar> E;
  if (x1.size() == 8)
  {
    // In the case where we have exactly 8 correspondences, there is no need to compute the SVD
    Eigen::Matrix<double, 9, 9> Q = epipolar_constraint.transpose().householderQr().householderQ();
    Eigen::Matrix<double, 9, 1> e = Q.col(8);
    E = Eigen::Map<const RMat3>(e.data());
  }
  else
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 9, 9>> solver(epipolar_constraint.transpose() *
                                                                      epipolar_constraint);
    E = Eigen::Map<const RMat3>(solver.eigenvectors().template leftCols<1>().data());
  }

  // Find the closest essential matrix to E in frobenius norm
  // E = UD'VT
  Eigen::JacobiSVD<Eigen::Matrix3d> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d d = USV.singularValues();
  const double a = d[0];
  const double b = d[1];
  d << (a + b) / 2., (a + b) / 2., 0.0;
  E = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();

  (*essential_matrix) = E;
}

template <typename Scalar, std::size_t N>
int relpose_8pt(const std::vector<Eigen::Vector3d> &x1, const std::vector<Eigen::Vector3d> &x2,
                EntoArray<CameraPose<Scalar>, N> *output) {

  Matrix3x3<Scalar> essential_matrix;
  essential_matrix_8pt(x1, x2, &essential_matrix);

  // Generate plausible relative motion from E
  output->clear();
  motion_from_essential(essential_matrix, x1, x2, output);
  return output->size();
}

}

#endif // EIGHT_PT_H
