#ifndef POSE_EST_UTIL_HH
#define POSE_EST_UTIL_HH

#include <Eigen/Dense>
#include "ento-math/core.h"

using namespace EntoMath;


template <typename Scalar>
struct DistortionCoeffs
{
  Scalar k1;
  Scalar k2;
  Scalar p1;
  Scalar p2;
  Scalar k3;
};

// =======================================================
// Normalization/Unnormalization
template <typename Scalar, int MaxM, int Order=0>
bool normalize_points(const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxM, 3>& P,
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxM, 3>* Q,
                      Eigen::Matrix<Scalar, 3, 3, Order>* T);

template <typename Derived>
bool normalize_points(const Eigen::DenseBase<Derived>& P,
                      const int& num_samples,
                      Eigen::DenseBase<Derived>* Q,
                      Eigen::DenseBase<Derived>* T);


template <typename Scalar, int Order=0>
void unnormalize_homography(Eigen::Matrix<Scalar, 3, 3, Order>* H ,
                            const Eigen::Matrix<Scalar, 3, 3, Order>& T1,
                            const Eigen::Matrix<Scalar, 3, 3, Order>& T2);
// =======================================================

// =======================================================
// Undistortion Points
template <typename Scalar, MaxM, int Order=0>
void undistort_points(const Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxM, 3>& P,
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Order, MaxM, 3>* Q,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const DistortionCoeffs& dist);

template <typename Derived>
void undistort_points(const Eigen::DenseBase<Derived>& P,
                      Eigen::DenseBase<Derived>* Q,
                      const Eigen::Matrix<Scalar, 3, 3>& K,
                      const DistortionCoeffs& dist);
// =======================================================


// =======================================================
// Homography Decomposition
template <typename Scalar, int Order=0>
void decompose_homography(Scalar* h,
                          Eigen::Matrix<Scalar, 3, 3, Order>* R,
                          Eigen::Matrix<Scalar, 3, 1, Order>* t);

template <typename Scalar, int Order=0>
void decompose_homography(Eigen::Matrix<Scalar, 3, 3, Order>* H,
                          Eigen::Matrix<Scalar, 3, 3, Order>* R,
                          Eigen::Matrix<Scalar, 3, 1, Order>* t);
// =======================================================

#endif // POSE_EST_UTIL_HH
