#ifndef P3P_HH
#define P3P_HH

namespace PoseEst
{
namespace P3P
{

template <typename Scalar>
int lambda_twist(const Eigen::Matrix<Scalar, 2, 3>& points2d,
                 const Eigen::Matrix<Scalar, 3, 3>& points3d,
                 std:array<Eigen::Matrix<Scalar, 3, 3>, 2> *output);
}
}

template <typename Scalar>
int kneip_p3p(const Eigen::Matrix<Scalar, 2, 3>& points2d,
              const Eigen::Matrix<Scalar, 3, 3>& points3d,
              std::array<Eigen::Matrix<Scalar, 3, 3>, 2> *output);
