#include <Eigen/Core>

template <typename T>
concept IsEigenMatrix = std::is_convertible_v<T,
                                              Eigen::Matrix<typename T::Scalar,
                                              T::RowsAtCompileTime, T::ColsAtCompileTime>>;

template <typename T>
concept IsEigenVector = IsEigenMatrix<T> &&
                        T::RowsAtCompileTime > 1 &&
                        T::ColsAtCompileTime == 1;

template <typename LHS, typename RHS>
concept CompatibleMatrixMultiplication =  IsEigenMatrix<LHS> && IsEigenMatrix<RHS> &&
                                          LHS::RowsAtCompileTime == RHS::ColsAtCompileTime;

template <typename LHS, typename RHS>
concept CompatibleMatrixAddition = IsEigenMatrix<LHS> && IsEigenMatrix<RHS> &&
                                   LHS::RowsAtCompileTime == RHS::RowsAtCompileTime && 
                                   LHS::ColsAtCompileTime == RHS::ColsAtCompileTime;

template<typename LHS, typename RHS>
concept CompatibleMatrixVectorMultiplication = IsEigenMatrix<LHS> && IsEigenVector<RHS> &&
                                               LHS::ColsAtCompileTime == RHS::RowsAtCompileTime;

template<typename LHS, typename RHS>
concept CompatibleDotProduct = IsEigenVector<LHS> && IsEigenVector<RHS> &&
                               LHS::RowsAtCompileTime == RHS::RowsAtCompileTime;

