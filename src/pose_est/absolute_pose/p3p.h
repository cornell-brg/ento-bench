#ifndef P3P_HH
#define P3P_HH

template <typename Scalar>
[[no discard]] 
int lambda_twist(const Eigen::Matrix<Scalar, 2, 3>& xprime,
                 const Eigen::Matrix<Scalar, 3, 3>& x,
                 std:array<Eigen::Matrix<Scalar, 3, 3>, 2> *output);

