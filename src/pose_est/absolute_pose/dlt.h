#include <Eigen/Dense>
#include <entomoton_math/svd.hh>


template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 3> dlt(const Eigen::Matrix<Scalar, N, 2>& points2d,
                                const Eigen::Matrix<Scalar, N, 3>& points3d);


template <typename Scalar, typename MaxN>
Eigen::Matrix<Scalar, 3, 3>
dlt(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, MaxN, 3>& points3d);

template <typename Derived>
Eigen::Matrix<Scalar, 3, 3>
dlt(const Eigen::DenseBase<Derived>& points2d,
    const Eigen::DenseBase<Derived>& points3d);


template <typename Scalar, typename N>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(const Eigen::Matrix<Scalar, N, 2>& points2d, 
           const Eigen::Matrix<Scalar, N, 3>& points3d);


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
           const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points3d);

template <typename Derived>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar(const Eigen::DenseBase<Derived>& points2d,
           const Eigen::DenseBase<Derived>& points3d);


template <typename Scalar, int N>
Eigen::Matrix<Scalar, 3, 3> dlt_planar_ho(const Eigen::Matrix<Scalar, N, 2>& points2d,
                                          const Eigen::Matrix<Scalar, N, 2>& points3d);


template <typename Scalar, int MaxN>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar_ho(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points2d,
              const Eigen::Matrix<Scalar, Eigen::Dynamic, 2, 0, MaxN, 2>& points3d);

template <typename Derived>
Eigen::Matrix<Scalar, 3, 3>
dlt_planar_ho(const Eigen::DenseBase<Derived>& points2d,
              const Eigen::DenseBase<Derived>& points3d);


// Nonlinear Optimizations Based on the DLT methods above.
// These LM impls will be hardcoded to the Homography or DLT.
bool levenberg_marquardt_dlt();
bool levenberg_marquardt_dlt_planar();
bool levenberg_marquardt_dlt_planar_ho();
