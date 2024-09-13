#include <Eigen/Dense>
#include <entomoton_math/svd.hh>

template <typename Scalar, typename N>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(Eigen::Matrix<Scalar, N, 2>& points2d, 
           Eigen::Matrix<Scalar, N, 3>& points3d);

template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, Eigen::Dynamic, 9, 0, 2 * MaxPoints, 9>
dlt_planar(Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxPoints, 2>& points2d,
           Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxPoints, 3>& points3d,
           const int N);

template <typename Scalar, int N, int Order, int useOSJ=0>
Eigen::Matrix<Scalar, 3, 3> dlt_planar_ho(Eigen::Matrix<Scalar, N, 2, Order>& points2d,
                                          Eigen::Matrix<Scalar, N, 2, Order>& points3d);

