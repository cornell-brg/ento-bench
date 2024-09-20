#ifndef P4P_HH
#define P4P_HH

#include <Eigen/Dense>

namespace pose_est
{


template <typename Scalar, typename N, int Order>
Eigen::Matrix<Scalar, 3, 3>
p4p_dlt(const Eigen::Matrix<Scalar, N, 2>& points2d, 
        const Eigen::Matrix<Scalar, N, 3>& points3d);


template <typename Scalar, int MaxN, int Order>
Eigen::Matrix<Scalar, 3, 3>
p4p_dlt(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxPoints, 2>& points2d,
        const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxPoints, 3>& points3d,
        const int N);


template <typename Scalar, int N, int Order>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar_ho(Eigen::Matrix<Scalar, N, 2, Order>& points2d,
              Eigen::Matrix<Scalar, N, 2, Order>& points3d);


template <typename Scalar, int MaxN, int Order>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar_ho(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, Order, MaxPoints, 2>& points2d,
              const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxPoints, 3>& points3d);


template <typename Scalar, int Order>
Eigen::Matrix<Scalar, 3, 3>
p4p_aca(const Eigen::Matrix<Scalar, 4, 2, Order>& xprime,
        const Eigen::Matrix<Scalar, 4, 3, Order>& x);

template <typename Scalar, int Order>
Eigen::Matrix<Scalar, 3, 3>
p4p_sks(const Eigen::Matrix<Scalar, 4, 2, Order> &xprime,
        const Eigen::Mtarix<Scalar, 4, 3, Order> &x);


}




#endif

