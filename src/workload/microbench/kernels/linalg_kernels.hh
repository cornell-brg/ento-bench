#ifndef LINALG_KERNELS_HH
#define LINALG_KERNELS_HH

#include "linalg/blas.h"

/* GSL Includes */
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

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

/* [GSL] Golub-Reinsch SVD Kernel
 * This function calls GSL's impl of Golub-Reinsch
 */
inline void
__attribute__((always_inline))
gsl_golubreinsch_svd(gsl_matrix* mat,
                     gsl_matrix* V,
                     gsl_vector* S,
                     gsl_vector* work);

/* [GSL] Modified Golub-Reinsch SVD Kernel
 * This function calls GSL's impl of Golub-Reinsch using the
 * R-SVD modification which is faster for M >> N.
 */
inline void
__attribute__((always_inline))
gsl_golubreinsch_mod_svd(gsl_matrix* mat,
                         gsl_matrix* V,
                         gsl_vector* S,
                         gsl_vector* work,
                         gsl_matrix* X);

/* [GSL] One Sided Jacobi SVD Kernel
 * This function calls GSL's impl of One Sided Jacobi SVD.
 */
inline void
__attribute__((always_inline))
gsl_osjacobi_svd(gsl_matrix* mat, gsl_matrix* V, gsl_vector* S);

/*=============================================================================*/
/* Definitions */
template<typename Scalar, int Rows, int Cols, int Opts>
inline void eigen_tsjacobi_svd(Eigen::JacobiSVD<Eigen::Matrix<Scalar, Rows, Cols>, Opts>& svd,
                               const Eigen::Matrix<Scalar, Rows, Cols>& A)
{
  svd.compute(A);
}

inline void gsl_golubreinsch_svd(gsl_matrix* A,
                                 gsl_matrix* V,
                                 gsl_vector* S,
                                 gsl_vector* work)
{
  gsl_linalg_SV_decomp(A, V, S, work);
}

inline void gsl_golubreinsch_mod_svd(gsl_matrix* A,
                                     gsl_matrix* V,
                                     gsl_vector* S,
                                     gsl_vector* work,
                                     gsl_matrix* X)
{
  gsl_linalg_SV_decomp_mod(A, X, V, S, work);
}

inline void gsl_osjacobi_svd(gsl_matrix* A,
                             gsl_matrix* V,
                             gsl_vector* S)
{
  gsl_linalg_SV_decomp_jacobi(A, V, S);
}

}

#endif // LINALG_KERNELS_HH
