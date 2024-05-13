#ifndef BLAS_H
#define BLAS_H

#include <stdint.h>

typedef int32_t blas_status;
typedef float float32_t;
typedef double float64_t;

typedef struct {
    uint32_t m;
    uint32_t n;
    float32_t* data;
} Mat_f32;

typedef struct {
    uint32_t n;
    float32_t* data;
} Vec_f32;


// Vector Initialization
Vec_f32*    vec_new_f32(uint32_t N);
void        vec_del_f32(Vec_f32* vec);
void        vec_init_f32(float32_t* data, uint32_t N, Vec_f32* dst);
blas_status vec_copy_f32(Vec_f32* in, Vec_f32* dst);
blas_status vec_copy_from_f32(Vec_f32* vec, float32_t* data, uint32_t N);
// Matrix Initialization
Mat_f32*    mat_new_f32(uint32_t m, uint32_t n);
void        mat_del_f32(Mat_f32* mat);
void        mat_init_f32(float32_t* data, uint32_t m, uint32_t n, Mat_f32* dst);
blas_status mat_copy_f32(Mat_f32* in, Mat_f32* dst);
blas_status mat_copy_from_f32(Mat_f32* mat, float32_t* data, float32_t sz);
blas_status mat_trans_from_f32(Mat_f32* mat, float32_t* data, float32_t sz);
blas_status mat_trans_f32(Mat_f32* mat, Mat_f32* transp);

// BLAS Level 1
blas_status vv_add_f32(Vec_f32* v1, Vec_f32* v2, Vec_f32* dst); 
blas_status vv_sub_f32(Vec_f32* v1, Vec_f32* v2, Vec_f32* dst);
blas_status vv_mult_f32(Vec_f32* v1, Vec_f32* v2, Vec_f32* dst);
blas_status vv_dot_f32(const Vec_f32* v1, const Vec_f32* v2, float32_t* dst);

blas_status v_offset_f32(Vec_f32* vec, const float32_t offset, Vec_f32* dst);
blas_status v_scale_f32(Vec_f32* vec, float32_t scale, Vec_f32* dst);
float32_t   v_l2norm_f32(Vec_f32* vec1);


void        f32_scale(float32_t* vec, float32_t scale, uint32_t N, float32_t* dst);
void        f32_offset(float32_t* vec, float32_t offset, uint32_t N, float32_t* dst);
void        f32_f32_mult(float32_t* v1, float32_t* v2, uint32_t N, float32_t* dst);
void        f32_f32_add(float32_t* v1, float32_t* v2, uint32_t N, float32_t* dst);
void        f32_f32_sub(float32_t* v1, float32_t* v2, uint32_t N, float32_t* dst);
void        f32_f32_dot(float32_t* v1, float32_t* v2, uint32_t N, float32_t* dst);
float32_t   f32_l2norm(float32_t* v1, uint32_t N);


// BLAS Level 2 
blas_status mv_mult_f32(const Mat_f32* in1, const Vec_f32* in2, Vec_f32* dst);

// BLAS Level 3
blas_status mm_mult_f32(const Mat_f32* in1, const Mat_f32* in2, Mat_f32* dst);
blas_status mm_add_f32(Mat_f32* mat1, Mat_f32* mat2, Mat_f32* dst);
blas_status mm_sub_f32(Mat_f32* mat1, Mat_f32* mat2, Mat_f32* dst);
blas_status m_scale_f32(Mat_f32* in1, float32_t scale, Mat_f32* dst);



#endif
