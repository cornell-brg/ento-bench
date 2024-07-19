#include "blas.h"
#include "pose_estimation_util.h"
#include "linalg_core.h"
#include "svd.h"

void dlt_planar_f32(Mat_f32* features, Mat_f32* landmarks, Mat_f32* landmarks_T, Calib_f32* calib, RigidTransformation_f32* T);
void dlt_planar_HO_f32(Mat_f32* features, Mat_f32* landmarks, Mat_f32* landmarks_T, Calib_f32* calib, RigidTransformation_f32* T);

// Inlines Functions
static inline void dlt_planar_setup_f32(uint32_t N, Mat_f32* xprime, Mat_f32* x, Mat_f32* A)
{
  Mat_f32 P, XP, YP;
  float32_t P_data[N*3], XP_data[N*3], YP_data[N*3];
  mat_init_f32(P_data, N, 3, &P);
  mat_trans_f32(x, &P);
  mat_init_f32(XP_data, N, 3, &XP);
  mat_init_f32(YP_data, N, 3, &YP);

  Vec_f32 Xdiag, Ydiag;
  vec_init_f32(&xprime->data[0], N, &Xdiag);
  vec_init_f32(&xprime->data[N], N, &Ydiag);
  v_scale_f32(&Xdiag, -1, &Xdiag);
  v_scale_f32(&Ydiag, -1, &Ydiag);
  diag_mult_mat_f32(&Xdiag, &P, &XP);
  diag_mult_mat_f32(&Ydiag, &P, &YP);
  
  float32_t* pAP1 = A->data;
  float32_t* pAP2 = &A->data[N*9 + 3];
  float32_t* pXP = &A->data[6];
  float32_t* pYP = &A->data[N*9 + 6];

  // Construct matrix A
  for (uint16_t i = 0; i < N; ++i)
  {
      for (uint16_t j = 0; j < 3; ++j)
      {
          pAP1[i*9+j] = x->data[i+N*j];
          pAP2[i*9+j] = x->data[i+N*j];
          pXP[i*9+j]  = XP.data[i*3+j];
          pYP[i*9+j]  = YP.data[i*3+j];
      }
  }
}

static inline void dlt_planar_HO_setup_f32(uint32_t N, Mat_f32* xprime, Mat_f32* x, Mat_f32* A)
{
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
  mat_init_f32(&A->data[0], N, 3, &A1);
  mat_init_f32(&A->data[3*N], N, 3, &A2);
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
}

