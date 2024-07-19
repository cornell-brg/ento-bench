#ifndef LINALG_KERNELS_HH
#define LINALG_KERNELS_HH

#include "linalg/blas.h"

/* Eigen includes */
#include <Eigen/Core>
#include <Eigen/SVD>

namespace linalg_kernels
{
/* Useful constexprs */
constexpr int ThinSVDOpt = Eigen::ComputeThinU | Eigen::ComputeThinV;
constexpr int FullSVDOpt = Eigen::ComputeFullU | Eigen::ComputeFullV;

/* Prototypes */

/* [Eigen] Two-Sided Jacobi Kernel
 * This function calls Eigen's implementation of Two-Sided Jacobi SVD.
 */
template <typename Scalar, int Rows, int Cols, int Opts>
inline void
__attribute__((always_inline))
eigen_tsjacobi_svd(Eigen::JacobiSVD<Eigen::Matrix<Scalar, Rows, Cols>, Opts>& svd,
                   const Eigen::Matrix<Scalar, Rows, Cols>& mat);

/*=============================================================================*/
/* Definitions */
template<typename Scalar, int Rows, int Cols, int Opts>
inline void eigen_tsjacobi_svd(Eigen::JacobiSVD<Eigen::Matrix<Scalar, Rows, Cols>, Opts>& svd,
                               const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  svd.compute(A);
}
}
#endif // LINALG_KERNELS_HH
