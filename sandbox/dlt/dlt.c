#include "dlt.h"
#include "linalg_core.h"
#include "pose_estimation_util.h"

void dlt_planar_f32(Mat_f32 *features,
                    Mat_f32 *landmarks,
                    Mat_f32 *landmarks_T,
                    Calib_f32 *calib,
                    RigidTransformation_f32 *T)
{
  uint32_t N = features->n;

  Mat_f32 undistorted_features;
  float32_t undistorted_features_data[N*3];

  Mat_f32 normalized_features;
  float32_t normalized_features_data[N*3];
  zeros_f32(normalized_features_data, N*3);

  Mat_f32 feats_T;
  float32_t feats_T_data[9] = { 0 };

  Mat_f32 A;
  float32_t A_data[2*N*9];
  zeros_f32(A_data, 2*N*9);

  mat_init_f32(undistorted_features_data, 3, N, &undistorted_features);
  mat_init_f32(normalized_features_data, 3, N, &normalized_features);
  mat_init_f32(feats_T_data, 3, 3, &feats_T);
  mat_init_f32(A_data, 2*N, 9, &A);

  undistort_points_f32(features, calib, &undistorted_features);
  uint32_t i;
  for (i = 0; i < N; i++) undistorted_features_data[2*N+i] = 1.0f;

  normalize_points_iso_f32(&undistorted_features, &normalized_features, &feats_T);

  dlt_planar_setup_f32(N, &normalized_features, landmarks, &A);

  float32_t h_data[9] = {0};
  Vec_f32 h;
  vec_init_f32(h_data, 9, &h);
  if (N == 4)
    svd_osj_transpose_f32(&A, &h);
  else {
    Mat_f32 V;
    float32_t V_data[9*9];
    Mat_f32 At;
    float32_t At_data[2*N*9];
    mat_init_f32(At_data, 9, 2*N, &At);
    mat_trans_f32(&A, &At);

    mat_init_f32(V_data, 9, 9, &V);
    eye_f32(&V);
    svd_osj_f32(&A, &V, &h);
  }
    
  if (h_data[8] < 0) v_scale_f32(&h, -1, &h);
  unnormalize_H_f32(&h, &feats_T, landmarks_T);
  simple_decompose_homography(&h, T);
}

void dlt_planar_HO_f32(Mat_f32 *features,
                       Mat_f32 *landmarks,
                       Mat_f32 *landmarks_T,
                       Calib_f32 *calib,
                       RigidTransformation_f32 *T)
{
  uint32_t N = features->n;

  Mat_f32 undistorted_features;
  float32_t undistorted_features_data[N*3];

  Mat_f32 normalized_features;
  float32_t normalized_features_data[N*3];
  zeros_f32(normalized_features_data, N*3);

  Mat_f32 feats_T;
  float32_t feats_T_data[9] = { 0 };

  Mat_f32 A;
  float32_t A_data[2*N*3];
  zeros_f32(A_data, 2*N*3);

  mat_init_f32(undistorted_features_data, 3, N, &undistorted_features);
  mat_init_f32(normalized_features_data, 3, N, &normalized_features);
  mat_init_f32(feats_T_data, 3, 3, &feats_T);
  mat_init_f32(A_data, 2*N, 3, &A);

  undistort_points_f32(features, calib, &undistorted_features);
  uint32_t i;
  for (i = 0; i < N; i++) undistorted_features_data[2*N+i] = 1.0f;

  normalize_points_iso_f32(&undistorted_features, &normalized_features, &feats_T);

  Mat_f32* xprime = &normalized_features;
  Mat_f32* x = landmarks;
  float32_t mC1 = 0.0f;
  float32_t mC2 = 0.0f;
  float32_t mC3 = 0.0f;
  float32_t mC4 = 0.0f;

  Vec_f32 C1, C2, C3, C4;
  float32_t C1_data[N], C2_data[N], C3_data[N], C4_data[N];
  vec_init_f32(C1_data, N, &C1);
  vec_init_f32(C2_data, N, &C2);
  vec_init_f32(C3_data, N, &C3);
  vec_init_f32(C4_data, N, &C4);
  
  Mat_f32 Mt, M, Mx, My, P, Pt, PPt, Pp;
  float32_t Mt_data[3*2*N], M_data[3*2*N], Pt_data[N*2], PPt_data[2*2], Pp_data[N*2];
  zeros_f32(Mt_data, 3*2*N);
  zeros_f32(M_data, 3*2*N);
  zeros_f32(Pt_data, N*2);
  zeros_f32(PPt_data, 2*2);
  zeros_f32(Pp_data, N*2);
  mat_init_f32(Mt_data, 3, 2*N, &Mt);
  mat_init_f32(M_data, 2*N, 3, &M);
  mat_init_f32(&M_data[0], N, 3, &Mx);
  mat_init_f32(&M_data[N*3], N, 3, &My);
  mat_init_f32(&x->data[0], 2, N, &P);
  mat_init_f32(Pt_data, N, 2, &Pt);
  mat_init_f32(PPt_data, 2, 2, &PPt);
  mat_init_f32(Pp_data, 2, N, &Pp);
  
  f32_scale(&xprime->data[0], -1, N, &Mt.data[4*N]);
  f32_scale(&xprime->data[N], -1, N, &Mt.data[5*N]);
  f32_f32_mult(&Mt.data[4*N], &x->data[0], N, C1.data);
  f32_f32_mult(&Mt.data[4*N], &x->data[N], N, C2.data);
  f32_f32_mult(&Mt.data[5*N], &x->data[0], N, C3.data);
  f32_f32_mult(&Mt.data[5*N], &x->data[N], N, C4.data);
  v_mean_f32(&C1, &mC1);
  v_mean_f32(&C2, &mC2);
  v_mean_f32(&C3, &mC3);
  v_mean_f32(&C4, &mC4);
  f32_offset(C1.data, -mC1, N, Mt.data);
  f32_offset(C2.data, -mC2, N, &Mt.data[2*N]);
  f32_offset(C3.data, -mC3, N, &Mt.data[N]);
  f32_offset(C4.data, -mC4, N, &Mt.data[3*N]);
  mat_trans_f32(&Mt, &M);

  // calculation of psuedoinverse of Phat
  mat_trans_f32(&P, &Pt);
  mm_mult_f32(&P, &Pt, &PPt);
  float32_t dt = PPt_data[0] * PPt_data[3] - PPt_data[1] * PPt_data[2];
  f32_scale(PPt_data, 1.0f/dt, 2*2, PPt_data);

  Mat_f32 Bx, By, Ex, Ey, A1, A2;
  float32_t Bx_data[2*3], By_data[2*3], Ex_data[3*N], Ey_data[3*N];
  mat_init_f32(Bx_data, 2, 3, &Bx);
  mat_init_f32(By_data, 2, 3, &By);
  mat_init_f32(Ex_data, N, 3, &Ex);
  mat_init_f32(Ey_data, N, 3, &Ey);
  mat_init_f32(&A.data[0], N, 3, &A1);
  mat_init_f32(&A.data[3*N], N, 3, &A2);
  zeros_f32(Bx_data, 2*3);
  zeros_f32(By_data, 2*3);
  zeros_f32(Ex_data, 3*N);
  zeros_f32(Ey_data, 3*N);

  mm_mult_f32(&PPt, &P, &Pp);
  mm_mult_f32(&Pp, &Mx, &Bx);
  mm_mult_f32(&Pp, &My, &By);
  mm_mult_f32(&Pt, &Bx, &Ex);
  mm_mult_f32(&Pt, &By, &Ey);

  mm_sub_f32(&Mx, &Ex, &A1);
  mm_sub_f32(&My, &Ey, &A2);
  
  
  float32_t V_data[3*3];
  Mat_f32 V;
  mat_init_f32(V_data, 3, 3, &V);
  eye_f32(&V);

  float32_t h_data[9];
  Vec_f32 h;
  vec_init_f32(&h_data[6], 3, &h);

  Mat_f32 At;
  float32_t At_data[2*N*3];
  mat_init_f32(At_data, 3, 2*N, &At);
  mat_trans_f32(&A, &At);
  svd_osj_f32(&At, &V, &h);
  
  h.data = h_data;
  if (h_data[8] < 0) {
    h_data[6] *= -1;
    h_data[7] *= -1;
    h_data[8] *= -1;
  }

  f32_f32_dot(Bx_data, &h_data[6], 3, &h_data[0]);
  h_data[0] *= -1;
  f32_f32_dot(&Bx_data[3], &h_data[6], 3, &h_data[1]);
  h_data[1] *= -1;

  f32_f32_dot(By_data, &h_data[6], 3, &h_data[3]);
  h_data[3] *= -1;
  f32_f32_dot(&By_data[3], &h_data[6], 3, &h_data[4]);
  h_data[4] *= -1;


  h_data[2] = -(mC1 * h_data[6] + mC2 * h_data[7]);
  h_data[5] = -(mC3 * h_data[6] + mC4 * h_data[7]);

  unnormalize_H_f32(&h, &feats_T, landmarks_T);
  simple_decompose_homography(&h, T);
}
