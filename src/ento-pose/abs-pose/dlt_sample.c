#include "dlt.h"

void dlt_mat_init(float32_t* dP_f32,
                  float32_t* dPhat_f32,
                  float32_t* dB_f32,
                  float32_t* dBhat_f32)
{
    // Set 3rd rows to 1s -> homogeneous coordinates.
    for (int i = 0; i < NUM_LANDMARKS; ++i)
    {
        dP_f32[2*NUM_LANDMARKS+i]    = 1;
        dPhat_f32[2*NUM_LANDMARKS+i] = 1;
        dB_f32[2*NUM_LANDMARKS+i]    = 1;
        dBhat_f32[2*NUM_LANDMARKS+i] = 1;
    }
}

void dlt_planar_f32(arm_matrix_instance_f32* feats,
                    arm_matrix_instance_f32* landmarks,
                    arm_matrix_instance_f32* landmarksT,
                    arm_matrix_instance_f32* K,
                    arm_matrix_instance_f32* R,
                    float32_t* t)
{
    //GPIOB->BSRR |= 1 << 1U;
#if PE_TEST == 1
	GPIOB->BSRR = (uint32_t)GPIO_PIN_11;

    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
#endif

    static float32_t dP_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 P_f32 = {3, NUM_LANDMARKS, dP_f32};
    // Feature data (Homogeneous coordinates).
    static float32_t dB_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 B_f32 = {3, NUM_LANDMARKS, dB_f32};

    static float32_t dFeatsNorm[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 featsNorm = {3, NUM_LANDMARKS, dFeatsNorm};

    // A matrix (Normal Planar DLT)

    static float32_t dA_f32[(2 * NUM_LANDMARKS) * 9] = {0};
    static arm_matrix_instance_f32 A_f32 = {2 * NUM_LANDMARKS, 9, dA_f32};

    static float32_t dAcm_f32[(2*NUM_LANDMARKS)*9] = {0};
    static arm_matrix_instance_f32 Acm_F32 = {9, 2*NUM_LANDMARKS, dAcm_f32};

    static float32_t dBhat_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 Bhat_f32 = {3, NUM_LANDMARKS, dBhat_f32};
    
    static float32_t dPhat_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 Phat_f32 = {NUM_LANDMARKS, 3, dPhat_f32};

    static float32_t dPhatT_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 PhatT_f32 = {3, NUM_LANDMARKS, dPhatT_f32};

    float32_t dV[9*9] = {0};
    arm_matrix_instance_f32 V = {9, 9, dV};

    arm_status status;
    // Normalize features following procedure laid out by Hartley+Zisserman
    static float32_t dFeatsT[9] = {0};
    static arm_matrix_instance_f32 featsT = {3, 3, dFeatsT}; //similarity Transformation for features

    static const DistortionCoeffs dist= {1.75557945e-01f, -2.51519544e-01f, -2.26855526e-04f, -7.53399980e-04f, 6.81196833e-02f};

    float32_t dUndistorted[NUM_LANDMARKS*3];
    arm_matrix_instance_f32 undistorted = {3, NUM_LANDMARKS, dUndistorted};

    undistort_points_iterative_f32(feats, &undistorted, K, &dist);
    normalize_points_f32(&undistorted.pData[0],
                         &undistorted.pData[NUM_LANDMARKS],
                         NUM_LANDMARKS,
                         &undistorted,
                         &Bhat_f32,
                         &featsT);
    /*normalize_points_f32(&landmarks->pData[0],
                         &landmarks->pData[NUM_LANDMARKS],
                         NUM_LANDMARKS,
                         landmarks,
                         &PhatT_f32,
                         &T2);
    */
    //GPIOB->BSRR |= 1 << (1U + 16);
    //GPIOB->BSRR |= 1 << (2U);

    //arm_mat_trans_f32(landmarks, &Phat_f32);

    // Setup A matrix for DLT
    dlt_planar_setup_f32(landmarks,
                         &dBhat_f32[0],
                         &dBhat_f32[NUM_LANDMARKS],
                         &A_f32);
    //GPIOB->BSRR |= 1 << (2U + 16);

    // Run SVD OSBJ Algorithm on A matrix. Dest mat is R
#if NUM_LANDMARKS == 4 && SVD_METHOD == 0
    static float32_t hvec[9] = {0};
    uint8_t min_idx = svd_osj_left_f32(&A_f32, hvec);
    hvec[0] = A_f32.pData[9*min_idx];
    hvec[1] = A_f32.pData[9*min_idx+1];
    hvec[2] = A_f32.pData[9*min_idx+2];
    hvec[3] = A_f32.pData[9*min_idx+3];
    hvec[4] = A_f32.pData[9*min_idx+4];
    hvec[5] = A_f32.pData[9*min_idx+5];
    hvec[6] = A_f32.pData[9*min_idx+6];
    hvec[7] = A_f32.pData[9*min_idx+7];
    hvec[8] = A_f32.pData[9*min_idx+8];
#elif SVD_METHOD == 1
    float32_t dVT[9*9] = {0};
	//float32_t dV[9*9] = {0};
	arm_matrix_instance_f32 VT = {9, 9, dVT};
	//arm_matrix_instance_f32 V = {9, 9, dV};
	float32_t work[9] = { 0 };
	float32_t s[9] = { 0 };
    static float32_t hvec[9] = {0};

	uint8_t min_idx = gr_svd(&A_f32, &VT, s, work);
	arm_mat_trans_f32(&VT, &V);
	hvec[0] = V.pData[9*min_idx];
	hvec[1] = V.pData[9*min_idx+1];
	hvec[2] = V.pData[9*min_idx+2];
	hvec[3] = V.pData[9*min_idx+3];
	hvec[4] = V.pData[9*min_idx+4];
	hvec[5] = V.pData[9*min_idx+5];
	hvec[6] = V.pData[9*min_idx+6];
	hvec[7] = V.pData[9*min_idx+7];
	hvec[8] = V.pData[9*min_idx+8];

#else
    eye_f32(&V);
    status = arm_mat_trans_f32(&A_f32, &Acm_F32);
    uint8_t min_idx = svd_osj_f32(&Acm_F32, &V);
    //GPIOB->BSRR |= 1 << (2U);
    // Reshape H, apply similarity transformations, scale H such that H[2,2]=1
    static float32_t hvec[9] = {0};
    hvec[0] = V.pData[9*min_idx];
    hvec[1] = V.pData[9*min_idx+1];
    hvec[2] = V.pData[9*min_idx+2];
    hvec[3] = V.pData[9*min_idx+3];
    hvec[4] = V.pData[9*min_idx+4];
    hvec[5] = V.pData[9*min_idx+5];
    hvec[6] = V.pData[9*min_idx+6];
    hvec[7] = V.pData[9*min_idx+7];
    hvec[8] = V.pData[9*min_idx+8];
#endif
    if (hvec[8] < 0) {
    	hvec[0] *= -1; hvec[1] *= -1; hvec[2] *= -1;
		hvec[3] *= -1; hvec[4] *= -1; hvec[5] *= -1;
    	hvec[6] *= -1; hvec[7] *= -1; hvec[8] *= -1;
    }
    unnormalize_points(hvec, &featsT, landmarksT);
    //GPIOB->BSRR |= 1 << (2U + 16);

    //GPIOB->BSRR |= 1 << (1U);

    fast_decompose_homography_f32(hvec, R, t);
    //GPIOB->BSRR |= 1 << (1U + 16);
#if PE_TEST == 1
	GPIOB->BRR = (uint32_t)GPIO_PIN_11;

    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
#endif
}

void dlt_planar_setup_f32(arm_matrix_instance_f32* P,
                          float32_t* Xdiag,
                          float32_t* Ydiag,
                          arm_matrix_instance_f32* A)
{
    /* DLT Matrix Setup
     *  Ah = r , X' = diag(Xdiag), Y' = diag(Ydiag)
     *  A = |P , 0, X'P|
     *      |P,  0, Y'P|
     */
    static float32_t dXP[NUM_LANDMARKS * 3] = {0};
    static float32_t dYP[NUM_LANDMARKS * 3] = {0};
    static arm_matrix_instance_f32 XP = {NUM_LANDMARKS, 3, dXP};
    static arm_matrix_instance_f32 YP = {NUM_LANDMARKS, 3, dYP};
    diag_mult_mat(Xdiag, P, &XP);
    diag_mult_mat(Ydiag, P, &YP);


    /*for (int i = 0; i < NUM_LANDMARKS; i++)
    //{
    //	dXP[i] = Xdiag[i] * P->pData[i*NUM_LANDMARKS];
    	dXP[2+i] = Xdiag[i] * P->pData[i*NUM_LANDMARKS+1];
    	dXP[4*NUM_LANDMARKS+i] = Xdiag[i] * P->pData[i*NUM_LANDMARKS+2];
    	dXP[]
    	dYP[i] = Ydiag[i] * P->pData[i*NUM_LANDMARKS];
		dYP[i+NUM_LANDMARKS+1] = Ydiag[i] * P->pData[i*NUM_LANDMARKS+1];
		dYP[i+NUM_LANDMARKS+2] = Ydiag[i] * P->pData[i*NUM_LANDMARKS+2];
    }*/

    float32_t* pAP1 = A->pData;
    float32_t* pAP2 = &A->pData[NUM_LANDMARKS*9 + 3];
    float32_t* pXP = &A->pData[6];
    float32_t* pYP = &A->pData[NUM_LANDMARKS*9 + 6];

    // Construct matrix A
    for (uint16_t i = 0; i < NUM_LANDMARKS; ++i)
    {
        for (uint16_t j = 0; j < 3; ++j)
        {
            pAP1[i*9+j] = P->pData[i+NUM_LANDMARKS*j];
            pAP2[i*9+j] = P->pData[i+NUM_LANDMARKS*j];
            pXP[i*9+j]  = -1 * XP.pData[i*3+j];
            pYP[i*9+j]  = -1 * YP.pData[i*3+j];
        }
    }
}

void dlt_HO_fixed_f32(arm_matrix_instance_f32* feats,
                      arm_matrix_instance_f32* landmarks,
                      arm_matrix_instance_f32* landmarksT,
                      arm_matrix_instance_f32* K,
                      arm_matrix_instance_f32* R,
                      float32_t* t)
{
#if PE_TEST == 1
    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	GPIOB->BSRR = (uint32_t)GPIO_PIN_11;

#endif

    //static float32_t dMx[3*NUM_LANDMARKS];
    //static float32_t dMy[3*NUM_LANDMARKS];
    static float32_t dMt[3*2*NUM_LANDMARKS];
    static float32_t dM[3*2*NUM_LANDMARKS];
    //static arm_matrix_instance_f32 Mtx = {3, NUM_LANDMARKS, &dMt[0]};
    //static arm_matrix_instance_f32 Mty = {3, NUM_LANDMARKS, &dMt[3*NUM_LANDMARKS]};
    arm_matrix_instance_f32 Mt = {3, NUM_LANDMARKS*2, &dMt[0]};
    arm_matrix_instance_f32 M = {NUM_LANDMARKS*2, 3, &dM[0]};
    arm_matrix_instance_f32 Mx = {NUM_LANDMARKS, 3, &dM[0]};
    arm_matrix_instance_f32 My = {NUM_LANDMARKS, 3, &dM[NUM_LANDMARKS*3]};

    
    static float32_t dAt[NUM_LANDMARKS*2];
    static float32_t dAAt[2*2];
    static arm_matrix_instance_f32 AAt = {2, 2, dAAt};

    float32_t C1[NUM_LANDMARKS], C2[NUM_LANDMARKS], C3[NUM_LANDMARKS], C4[NUM_LANDMARKS];


    static float32_t dBhat_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 Bhat_f32 = {3, NUM_LANDMARKS, dBhat_f32};
    
    static float32_t dPhat_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 Phat_f32 = {NUM_LANDMARKS, 3, dPhat_f32};

    static float32_t dPhatT_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 PhatT_f32 = {3, NUM_LANDMARKS, dPhatT_f32};

    static float32_t dFeatsT[9] = {0};
    static arm_matrix_instance_f32 featsT = {3, 3, dFeatsT}; //similarity Transformation for features

    arm_matrix_instance_f32 A = {2, NUM_LANDMARKS, &landmarks->pData[0]};
    static arm_matrix_instance_f32 At = {NUM_LANDMARKS, 2,  dAt};
    static float32_t dPp[2*NUM_LANDMARKS];
    static float32_t dBx[2*3];
    static float32_t dBy[2*3];
    static float32_t dD[2*NUM_LANDMARKS*3];
    static arm_matrix_instance_f32 Pp = {2, NUM_LANDMARKS, dPp};
    static arm_matrix_instance_f32 Bx = {2, 3, dBx};
    static arm_matrix_instance_f32 By = {2, 3, dBy};
    static arm_matrix_instance_f32 D1 = {NUM_LANDMARKS, 3, dD};
    static arm_matrix_instance_f32 D2 = {NUM_LANDMARKS, 3, &dD[NUM_LANDMARKS*3]};
    static arm_matrix_instance_f32 D = {2*NUM_LANDMARKS, 3, dD};

    static float32_t dEx[NUM_LANDMARKS * 3];
    static float32_t dEy[NUM_LANDMARKS * 3];
    static arm_matrix_instance_f32 Ex = {NUM_LANDMARKS, 3, dEx};
    static arm_matrix_instance_f32 Ey = {NUM_LANDMARKS, 3, dEy};

    static const DistortionCoeffs dist= {1.75557945e-01f, -2.51519544e-01f, -2.26855526e-04f, -7.53399980e-04f, 6.81196833e-02f};

    float32_t dUndistorted[NUM_LANDMARKS*3];
    arm_matrix_instance_f32 undistorted = {3, NUM_LANDMARKS, dUndistorted};

    undistort_points_iterative_f32(feats, &undistorted, K, &dist);
    normalize_points_f32(&undistorted.pData[0],
                         &undistorted.pData[NUM_LANDMARKS],
                         NUM_LANDMARKS,
                         &undistorted,
                         &Bhat_f32,
                         &featsT);
    
    /*normalize_points_f32(&landmarks->pData[0],
                         &landmarks->pData[NUM_LANDMARKS],
                         NUM_LANDMARKS,
                         landmarks,
                         &PhatT_f32,
                         &T2);
    */
    float32_t mC1 = 0.0f;
    float32_t mC2 = 0.0f;
    float32_t mC3 = 0.0f;
    float32_t mC4 = 0.0f;
    for (int i = 0; i < NUM_LANDMARKS; i++)
    {
        C1[i] = -Bhat_f32.pData[i] * landmarks->pData[i];
        C2[i] = -Bhat_f32.pData[i] * landmarks->pData[NUM_LANDMARKS+i];
        C3[i] = -Bhat_f32.pData[NUM_LANDMARKS+i] * landmarks->pData[i];
        C4[i] = -Bhat_f32.pData[NUM_LANDMARKS+i] * landmarks->pData[NUM_LANDMARKS+i];

        mC1 += C1[i];
        mC2 += C2[i];
        mC3 += C3[i];
        mC4 += C4[i];
    }

    mC1 /= NUM_LANDMARKS; mC2 /= NUM_LANDMARKS; mC3 /= NUM_LANDMARKS; mC4 /= NUM_LANDMARKS;
    
    arm_offset_f32(C1, -mC1, Mt.pData, NUM_LANDMARKS);
    arm_offset_f32(C2, -mC2, &Mt.pData[2*NUM_LANDMARKS], NUM_LANDMARKS);
    arm_offset_f32(C3, -mC3, &Mt.pData[NUM_LANDMARKS], NUM_LANDMARKS);
    arm_offset_f32(C4, -mC4, &Mt.pData[3*NUM_LANDMARKS], NUM_LANDMARKS);
    arm_scale_f32(&Bhat_f32.pData[0], -1, &Mt.pData[4*NUM_LANDMARKS], NUM_LANDMARKS);
    arm_scale_f32(&Bhat_f32.pData[NUM_LANDMARKS], -1, &Mt.pData[5*NUM_LANDMARKS], NUM_LANDMARKS);
    arm_mat_trans_f32(&Mt, &M);

    // calculation of psuedoinverse of Phat
    arm_mat_trans_f32(&A, &At);
    arm_mat_mult_f32(&A, &At, &AAt);
    float32_t dt = dAAt[0] * dAAt[3] - dAAt[1] * dAAt[2];
    arm_scale_f32(dAAt, 1.0f/dt, dAAt, NUM_LANDMARKS);
    
    arm_mat_mult_f32(&AAt, &A, &Pp);
    arm_mat_mult_f32(&Pp, &Mx, &Bx);
    arm_mat_mult_f32(&Pp, &My, &By);

    arm_mat_mult_f32(&At, &Bx, &Ex);
    arm_mat_mult_f32(&At, &By, &Ey);

    arm_mat_sub_f32(&Mx, &Ex, &D1);
    arm_mat_sub_f32(&My, &Ey, &D2);

#if SVD_METHOD == 0
    float32_t dV[3*3] = {0};
    arm_matrix_instance_f32 V = {3, 3, dV};
    eye_f32(&V);
    static float32_t dDcm_f32[3*2*NUM_LANDMARKS] = {0};
    static arm_matrix_instance_f32 Dcm_f32 = {3, 2 * NUM_LANDMARKS, dDcm_f32};
    arm_mat_trans_f32(&D, &Dcm_f32);
    uint8_t min_idx = svd_osj_f32(&Dcm_f32, &V); // SVD of 2mx3 matrix results in 3x3 V
#elif SVD_METHOD == 1
    float32_t dVT[3*3] = {0};
    float32_t dV[3*3] = {0};
    arm_matrix_instance_f32 VT = {3, 3, dVT};
    arm_matrix_instance_f32 V = {3, 3, dV};
    float32_t work[3] = { 0 };
    float32_t s[3] = { 0 };

    uint8_t min_idx = gr_svd(&D, &VT, s, work);
    arm_mat_trans_f32(&VT, &V);


#endif
    //GPIOB->BSRR |= 1 << 1U;
    // Reshape H, apply similarity transformations, scale H such that H[2,2]=1
    float32_t hvec[9] = {0};
    
    hvec[6] = V.pData[3*min_idx];
    hvec[7] = V.pData[3*min_idx+1];
    hvec[8] = V.pData[3*min_idx+2];
    if (hvec[8] < 0) {
    	arm_scale_f32(&hvec[6], -1, &hvec[6], 3);
    }

    arm_dot_prod_f32(Bx.pData, &hvec[6], 3, &hvec[0]);
    hvec[0] *= -1;
    arm_dot_prod_f32(&Bx.pData[3], &hvec[6], 3, &hvec[1]);
    hvec[1] *= -1;

    arm_dot_prod_f32(By.pData, &hvec[6], 3, &hvec[3]);
    hvec[3] *= -1; 
    arm_dot_prod_f32(&By.pData[3], &hvec[6], 3, &hvec[4]);
    hvec[4] *= -1;

    hvec[2] = -(mC1 * hvec[6] + mC2 * hvec[7]);
    hvec[5] = -(mC3 * hvec[6] + mC4 * hvec[7]);

    //GPIOB->BSRR |= 1 << (1U + 16);

    // Construct Rotation matrix and translation vector}
    //GPIOB->BSRR |= 1 << (2U);
    unnormalize_points(hvec, &featsT, landmarksT);
    //GPIOB->BSRR |= 1 << (2U + 16);
    
    //GPIOB->BSRR |= 1 << (1U);
    fast_decompose_homography_f32(hvec, R, t);
    //GPIOB->BSRR |= 1 << (1U + 16);
#if PE_TEST == 1
    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
	GPIOB->BRR = (uint32_t)GPIO_PIN_11;

#endif


}

void dlt_HO_f32(arm_matrix_instance_f32* feats,
                arm_matrix_instance_f32* Lh, // landmarks homogeneous
                arm_matrix_instance_f32* LhT, // landmarks homogeneous transpose
                arm_matrix_instance_f32* K,
                arm_matrix_instance_f32* R,
                float32_t* t)
{
    GPIOB->BSRR |= 1 << 1U;
    static float32_t dP_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 P_f32 = {3, NUM_LANDMARKS, dP_f32};
    // Feature data (Homogeneous coordinates).
    static float32_t dB_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 B_f32 = {3, NUM_LANDMARKS, dB_f32};
    static float32_t dFeatsNorm[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 featsNorm = {3, NUM_LANDMARKS, dFeatsNorm};
    // A matrix (Normal Planar DLT)
    static float32_t dA_f32[(2 * NUM_LANDMARKS) * 9] = {0};
    static arm_matrix_instance_f32 A_f32 = {2 * NUM_LANDMARKS, 9, dA_f32};

    static float32_t dBhat_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 Bhat_f32 = {3, NUM_LANDMARKS, dBhat_f32};
    
    static float32_t dPhat_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 Phat_f32 = {NUM_LANDMARKS, 3, dPhat_f32};

    static float32_t dPhatT_f32[NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 PhatT_f32 = {3, NUM_LANDMARKS, dPhatT_f32};

    static float32_t dV[2*NUM_LANDMARKS*3] = {0};
    static arm_matrix_instance_f32 V = {2*NUM_LANDMARKS, 3, dV};

    static float32_t dPinv[2*NUM_LANDMARKS] = {0};
    static arm_matrix_instance_f32 Pinv = {2, NUM_LANDMARKS, dPinv};
    // Normalize features following procedure laid out by Hartley+Zisserman
    static float32_t Tfeat_data[9] = {0}; Tfeat_data[8] = 1;
    static float32_t Tlndmrk_data[9] = {0}; Tlndmrk_data[8] = 1;
    static arm_matrix_instance_f32 T1 = {3, 3, Tfeat_data}; //similarity Transformation for features
    static arm_matrix_instance_f32 T2 = {3, 3, Tlndmrk_data};

    static const DistortionCoeffs dist= {1.75557945e-01f, -2.51519544e-01f, -2.26855526e-04f, -7.53399980e-04f, 6.81196833e-02f};
    
    undistort_points_f32(feats, K, &dist);
    normalize_points_f32(feats->pData,
                         &feats->pData[NUM_LANDMARKS],
                         NUM_LANDMARKS,
                         feats,
                         &Bhat_f32,
                         &T1);
    normalize_points_f32(&LhT->pData[0],
                         &LhT->pData[NUM_LANDMARKS],
                         NUM_LANDMARKS,
                         Lh,
                         &PhatT_f32,
                         &T2);
    //GPIOB->BSRR |= 1 << (1U + 16);
    //GPIOB->BSRR |= 1 << 2U;
    arm_mat_trans_f32(&PhatT_f32, &Phat_f32);
    psuedoinverse_f32(&Phat_f32, &PhatT_f32, &Pinv);

    // Setup A matrix for DLT
    dlt_HO_setup_f32(&Phat_f32,
                     &PhatT_f32,
                     &dBhat_f32[0],
                     &dBhat_f32[NUM_LANDMARKS],
                     &Pinv,
                     &A_f32);
    GPIOB->BSRR |= 1 << (2U + 16);

    // Run SVD OSBJ Algorithm on A matrix. Dest mat is R
    eye_f32(&V);
    static float32_t dAcm_f32[9*2*NUM_LANDMARKS] = {0};
    static arm_matrix_instance_f32 Acm_F32 = {9, 2 * NUM_LANDMARKS, dAcm_f32};
    arm_mat_trans_f32(&A_f32, &Acm_F32);
    uint8_t min_idx = svd_osj_f32(&Acm_F32, &V); // SVD of 2mx3 matrix results in 3x3 V
    //GPIOB->BSRR |= 1 << 1U;
    // Reshape H, apply similarity transformations, scale H such that H[2,2]=1
    float32_t hvec[9] = {0};
    
    hvec[6] = V.pData[3*min_idx];
    hvec[7] = V.pData[3*min_idx+1];
    hvec[8] = V.pData[3*min_idx+2];
    dlt_HO_backsubstitution_f32(&Pinv, &Phat_f32, &dBhat_f32[0], &hvec[6], hvec);
    dlt_HO_backsubstitution_f32(&Pinv, &Phat_f32, &dBhat_f32[NUM_LANDMARKS], &hvec[6], &hvec[3]);
    GPIOB->BSRR |= 1 << (1U + 16);

    // Construct Rotation matrix and translation vector}
    GPIOB->BSRR |= 1 << (2U);
    unnormalize_points(hvec, &T1, &T2);
    GPIOB->BSRR |= 1 << (2U + 16);
    
    GPIOB->BSRR |= 1 << (1U);
    fast_decompose_homography_f32(hvec, R, t);
    GPIOB->BSRR |= 1 << (1U + 16);

}

void dlt_HO_setup_f32(arm_matrix_instance_f32* Phat,
                      arm_matrix_instance_f32* Phat_T,
                      float32_t* Xdiag,
                      float32_t* Ydiag,
                      arm_matrix_instance_f32* Pinv,
                      arm_matrix_instance_f32* A)
{
    /* DLT Matrix Setup
     *  Ah = r , X' = diag(Xdiag), Y' = diag(Ydiag)
     *  A = |X'P - 1Mx - P(P^+)X'P , X'1 - P(P^+)X'1|
     *      |Y'P - 1My - P(P^+)Y'P , Y'1 - P(P^+)Y'1|
     *
     *  P^+ = psuedo-inverse of P
     *  Mx = second moment of x coord of features and landmarks
     *  My = second moment of y coord of features and landmarks
     *  P = Phat variable (not homogeneous coordinates)
     * 
     *  A = | G - H - J*K, L - J*  
     */
    float32_t* pA = A->pData;

    static float32_t dXP[NUM_LANDMARKS * 2] = {0};
    static float32_t dYP[NUM_LANDMARKS * 2] = {0};
    static arm_matrix_instance_f32 XP = {NUM_LANDMARKS, 2, dXP};
    static arm_matrix_instance_f32 YP = {NUM_LANDMARKS, 2, dYP};

    static float32_t x_moment[2] = {0};
    static float32_t y_moment[2] = {0};

    static float32_t dOnes[NUM_LANDMARKS] = {1,1,1,1};
    static arm_matrix_instance_f32 ones = {NUM_LANDMARKS, 1, dOnes};
    static arm_matrix_instance_f32 Mx = {1, 2, x_moment};
    static arm_matrix_instance_f32 My = {1, 2, y_moment};

    static float32_t dM1x[NUM_LANDMARKS * 2] = {1};
    static float32_t dM1y[NUM_LANDMARKS * 2] = {1};
    static arm_matrix_instance_f32 M1x = {NUM_LANDMARKS, 2, dM1x};
    static arm_matrix_instance_f32 M1y = {NUM_LANDMARKS, 2, dM1y};

    second_order_moment_f32(Xdiag,
                            Phat_T->pData,
                            &Phat_T->pData[NUM_LANDMARKS],
                            x_moment,
                            NUM_LANDMARKS);
    second_order_moment_f32(Ydiag,
                            Phat_T->pData,
                            &Phat_T->pData[NUM_LANDMARKS],
                            y_moment,
                            NUM_LANDMARKS);
    arm_status status;

    diag_mult_mat(Xdiag, Phat, &XP);
    diag_mult_mat(Ydiag, Phat, &YP);

    status = arm_mat_mult_f32(&ones, &Mx, &M1x);
    status = arm_mat_mult_f32(&ones, &My, &M1y);

    float32_t dPPinv[NUM_LANDMARKS*NUM_LANDMARKS] = {0};
    arm_matrix_instance_f32 PPinv = {NUM_LANDMARKS, NUM_LANDMARKS, dPPinv};
    arm_mat_mult_f32(Phat, Pinv, &PPinv);

    float32_t dI1[NUM_LANDMARKS * 2] = {0};
    float32_t dI2[NUM_LANDMARKS * 2] = {0};
    float32_t dI3[NUM_LANDMARKS * 2] = {0};
    arm_matrix_instance_f32 I1 = {NUM_LANDMARKS, 2, dI1};
    arm_matrix_instance_f32 I2 = {NUM_LANDMARKS, 2, dI2};
    arm_matrix_instance_f32 I3 = {NUM_LANDMARKS, 2, dI3};
    
    // Calculate first block
    arm_mat_sub_f32(&XP, &M1x, &I1);
    status = arm_mat_mult_f32(&PPinv, &XP, &I2);
    arm_mat_sub_f32(&I1, &I2, &I3);

    // Copy into destination mat.
    for (uint16_t i = 0; i < NUM_LANDMARKS; ++i)
    {
        pA[i*3] = dI3[2*i];
        pA[i*3 + 1] = dI3[2*i+1];
    }

    // Calculate 2nd Block
    float32_t dI4[NUM_LANDMARKS] = {0};
    float32_t dI5[NUM_LANDMARKS] = {0};
    mat_vec_mult_f32(&PPinv, Xdiag, dI4, NUM_LANDMARKS);
    arm_sub_f32(Xdiag, dI4, dI5, NUM_LANDMARKS);
    
    // Copy into destination mat.
    for (uint16_t i = 0; i < NUM_LANDMARKS; ++i)
    {
        pA[i*3+2] = dI5[i];
    }

    // Calculate 3rd block.
    arm_mat_sub_f32(&YP, &M1y, &I1);
    arm_mat_mult_f32(&PPinv, &YP, &I2);
    arm_mat_sub_f32(&I1, &I2, &I3);

    // Copy into destination mat.
    for (uint16_t i = 0; i < NUM_LANDMARKS; ++i)
    {
        pA[i*3 + 3*NUM_LANDMARKS] = dI3[2*i];
        pA[i*3 + 3*NUM_LANDMARKS] = dI3[2*i+1];
    }

    // Calculate 4th block.
    mat_vec_mult_f32(&PPinv, Ydiag, dI4, NUM_LANDMARKS);
    arm_sub_f32(Ydiag, dI4, dI5, NUM_LANDMARKS);

    // Copy into destination mat.
    for (uint16_t i = 0; i < NUM_LANDMARKS; ++i)
    {
        pA[i*3 + (2+3*NUM_LANDMARKS)] = dI5[i];
    }

} 

void dlt_HO_backsubstitution_f32(arm_matrix_instance_f32* Pinv,
                                 arm_matrix_instance_f32* P,
                                 float32_t* Xdiag,
                                 float32_t* h789,
                                 float32_t* dst)
{
    static float32_t dI1[NUM_LANDMARKS*3] = {0};
    static float32_t dI2[2*3] = {0};
    static arm_matrix_instance_f32 I1 = {NUM_LANDMARKS, 3, dI1};
    static arm_matrix_instance_f32 I2 = {2, 3, dI2};
    diag_mult_mat(Xdiag, P, &I1);
    arm_mat_mult_f32(Pinv, &I1, &I2);
    arm_dot_prod_f32(dI2, h789, 3, dst);
    arm_dot_prod_f32(&dI2[3], h789, 3, &dst[1]);

}

int normalize_points_f32(float32_t* x,
                         float32_t* y,
                         const uint8_t num_samples,
                         arm_matrix_instance_f32* A,
                         arm_matrix_instance_f32* A_hat,
                         arm_matrix_instance_f32* T)
{
    static float32_t x_centroid[NUM_LANDMARKS], x_centroidSq[NUM_LANDMARKS];
    static float32_t y_centroid[NUM_LANDMARKS], y_centroidSq[NUM_LANDMARKS];
    float32_t s, xc, yc, denom = 0;
    arm_status status;
    arm_mean_f32(x, num_samples, &xc); // xc is a value
    arm_mean_f32(y, num_samples, &yc); // yc is a value

    /* Dont need this. There is a fill method!
    for (int i = 0; i < num_samples; ++i)
    {
        xc_vec[i] = xc;
        yc_vec[i] = yc;
    }
    */
    
    arm_offset_f32(x, -xc, x_centroid, num_samples); // calculating Centroid
    arm_offset_f32(y, -yc, y_centroid, num_samples);
    arm_mult_f32(x_centroid, x_centroid, x_centroidSq, num_samples);
    arm_mult_f32(y_centroid, y_centroid, y_centroidSq, num_samples);
    arm_add_f32(x_centroidSq, y_centroidSq, x_centroid, num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        arm_sqrt_f32(x_centroid[i], &y_centroid[i]);
        denom += y_centroid[i];
    }
    // alternative is to use arm_weighted_sum but the include header
    // and lib I have for CMSIS-DSP currently does not include it :/
    if (denom > 0)
    {
        s = (num_samples * SQRT2) / denom;
        T->pData[0] = s; T->pData[2] = -s * xc;
        T->pData[4] = s; T->pData[5] = -s * yc;
        T->pData[8] = 1;
        
        status = arm_mat_mult_f32(T, A, A_hat);
        return 1;
    }
    else 
    {
        return 0;
    }
}

void unnormalize_points(float32_t* h,
                        arm_matrix_instance_f32* T1,
                        arm_matrix_instance_f32* T2)
{
    static arm_matrix_instance_f32 H;
    static float32_t dI1[9] = {0};
    static float32_t dI2[9] = {0};
    static arm_matrix_instance_f32 I1 = {3, 3, dI1};
    static arm_matrix_instance_f32 I2 = {3, 3, dI2};

    arm_mat_init_f32(&H, 3, 3, h);
    arm_mat_inverse_f32(T1, &I1);
    arm_mat_mult_f32(&I1, &H, &I2);
    arm_mat_mult_f32(&I2, T2, &H);
    //float32_t h22 = H
    //arm_scale_f32(I2->pData, )
}

void fast_decompose_homography_f32(float32_t* h,
                                   arm_matrix_instance_f32* R,
                                   float32_t* t)
{
    static arm_matrix_instance_f32 H;
    static float32_t temp[9];
    arm_mat_init_f32(&H, 3, 3, h);
    float32_t h22 = 1.0f / h[8];
    arm_scale_f32(h, h22, temp, 9);
    

    float32_t norm;
    arm_sqrt_f32(temp[0]*temp[0] + temp[3]*temp[3] + temp[6]*temp[6], &norm);
    norm = 1.0f / norm;

    arm_scale_f32(temp, norm, h, 9);
    float32_t* pR = R->pData;
    // Last column is the cross product of first two columns.
    pR[0] = h[0]; pR[1] = h[1]; pR[2] = h[3]*h[7] - h[6]*h[4];
    pR[3] = h[3]; pR[4] = h[4]; pR[5] = h[6]*h[1] - h[0]*h[7];
    pR[6] = h[6]; pR[7] = h[7]; pR[8] = h[0]*h[4] - h[3]*h[1];
    
    t[0] = h[2];
    t[1] = h[5];
    t[2] = h[8];
}
