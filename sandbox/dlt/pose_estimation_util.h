#ifndef POSE_EST_UTIL_H
#define POSE_EST_UTIL_H

#include "blas.h"

#define SQRT2 1.4142135f

typedef struct {
    Mat_f32* R;
    Vec_f32* tvec;
} RigidTransformation_f32;

typedef struct {
  Mat_f32* K;     // Camera calibration matrix
  Vec_f32* dist; // distortion coefficients, using OpenCV conventions
} Calib_f32;

void undistort_points_f32(const Mat_f32* features, const Calib_f32* calib, Mat_f32* undistorted_features);
int  normalize_points_iso_f32(const Mat_f32* points, Mat_f32* normalized, Mat_f32* T);
int  unnormalize_H_f32(Vec_f32* h, Mat_f32* T1, Mat_f32* T2);
void simple_decompose_homography(const Vec_f32* h, RigidTransformation_f32* T);
void malis_decompose_homography();

#endif
