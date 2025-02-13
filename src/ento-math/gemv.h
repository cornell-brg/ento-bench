#ifndef ENTO_GEMV_H 
#define ENTO_GEMV_H 

#include <Eigen/Core>
#include <type_traits>

namespace Eigen {
namespace internal {

template<
    typename Index,
    typename LhsScalar,
    typename LhsMapper,
    int LhsStorageOrder,
    bool ConjugateLhs,
    typename RhsScalar,
    typename RhsMapper,
    bool ConjugateRhs,
    int Version
>
struct general_matrix_vector_product<
    Index, LhsScalar, LhsMapper, LhsStorageOrder, ConjugateLhs,
    RhsScalar, RhsMapper, ConjugateRhs, Version
>
{
    // Use SFINAE inside the struct
    static void run(
        Index rows, Index cols, const LhsMapper& lhs, const RhsMapper& rhs,
        RhsScalar* res, const LhsScalar& alpha,
        typename std::enable_if<(LhsMapper::RowsAtCompileTime <= 64 &&
                                 LhsMapper::ColsAtCompileTime <= 64)>::type* = nullptr)
    {
        // Custom implementation for small fixed-size matrices
        for (Index i = 0; i < rows; ++i) {
            RhsScalar sum = RhsScalar(0);
            for (Index j = 0; j < cols; ++j) {
                sum += lhs(i, j) * rhs(j);
            }
            res[i] = alpha * sum;
        }
    }
};
}  // namespace internal
}  // namespace Eigen

#endif  // ENTO_GEMV_H

