#ifndef LK_OPTICAL_FLOW_ITER_H
#define LK_OPTICAL_FLOW_ITER_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <ento-feature2d/fixed_point.h>
#include <ento-feature2d/raw_image.h>
// g++ -std=c++11 $(pkg-config --cflags eigen3 --libs opencv4) lk_optical_flow_fp_gem5.cpp -o lk_optical_flow_fp_gem5  -I/opt/homebrew/include
// ./lk_optical_flow_fp_gem5 -MAIN_ITERATIONS=150 -NUM_LEVELS=1 -MAX_COUNT=5 -WIN_DIM=15 -FRAME_RATE=150 -TEST_FOLDER=5 -FIRST_IMG=32

using namespace std;

#define MAX_WIN_DIM 15
#define IMG_SAVE
// Algorithm parameters
// TODO: constexpr

const short scharr_x_arr[] = {-3, 0, 3, -10, 0, 10, -3, 0, 3};
const short scharr_y_arr[] = {-3, -10, -3, 0, 0, 0, 3, 10, 3};

// TODO: long vs longlong on cortex m4
const int decimal_bits = 20;
using fp_t = FixedPoint<64-decimal_bits, decimal_bits, int64_t>;
using PointFP = std::tuple<fp_t, fp_t>;

const fp_t fp_1(1);
const fp_t fp_2(2);

// window dimension used for the gradient calculation
// not the same as the window dimension used for the optical flow
template<size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM>
void calc_gradient(const PointFP & pt, const RawImage<IMG_WIDTH, IMG_HEIGHT> & src, short* Ix_arr, short* Iy_arr, const int halfWin) {
    
    // Indices for top left corner of window in src
    int32_t x_i = get<0>(pt).to_int32()-halfWin;
    int32_t y_i = get<1>(pt).to_int32()-halfWin;

    for (size_t i = 0; i < WIN_DIM; i++) { // row / height / y
        for (size_t j = 0; j < WIN_DIM; j++) { // col / width / x
            // kernel loop
            int Ix = 0;
            int Iy = 0;
            for (int k = 0; k < 3; k++) { // row / height / y
                for (int l = 0; l < 3; l++) { // col / width / x
                    size_t y_index = i+k+y_i - 1;
                    if (y_index < 0) y_index = 0;
                    else if (y_index >= src.height) y_index = src.height-1;

                    size_t x_index = j+l+x_i - 1;
                    if (x_index < 0) x_index = 0;
                    else if (x_index >= src.width) x_index = src.width-1;
                    Ix += ((short) src.data[src.height*y_index + x_index])*scharr_x_arr[k*3+l];
                    Iy += ((short) src.data[src.height*y_index + x_index])*scharr_y_arr[k*3+l];
                }
            }
            Ix_arr[WIN_DIM*i + j] = round(Ix / 32);
            Iy_arr[WIN_DIM*i + j] = round(Iy / 32);
        }
    }
}

// WIN_DIM is dimension of output window
template <typename SrcT, size_t WIN_DIM>
void interpolate(const PointFP & pt, const SrcT * src, Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> & dst, int src_width, int src_height) {
    int32_t x_i = get<0>(pt).to_int32();
    int32_t y_i = get<1>(pt).to_int32();
    fp_t a = get<0>(pt) - fp_t(x_i);
    fp_t b = get<1>(pt) - fp_t(y_i);

    fp_t iw00((fp_1 - a) * (fp_1 - b));
    fp_t iw01(a*(fp_1-b));
    fp_t iw10((fp_1-a)*b);
    fp_t iw11(fp_1- iw00 - iw01 - iw10);

    for (size_t k = 0; k < WIN_DIM; k++) { // row / height / y
        for (size_t l = 0; l < WIN_DIM; l++) { // col / width / x
            int l_plus = l+1;
            if (l_plus >= src_width) l_plus = src_width - 1;

            int k_plus = k+1;
            if (k_plus >= src_height) l_plus = src_height - 1;

            if (l_plus >= src_width) l_plus = src_width - 1;
            fp_t ival(iw00 * fp_t(src[src_width*(k)     + l    ]) + 
                      iw01 * fp_t(src[src_width*(k)     + l + 1]) +
                      iw10 * fp_t(src[src_width*(k + 1) + l    ]) + 
                      iw11 * fp_t(src[src_width*(k + 1) + l + 1]));
            dst(k, l) = ival;
        }
    }
}

template <size_t WIN_DIM>
fp_t a_transpose_a(const Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> & Ix_win_square, 
                    const Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> & Iy_win_square, 
                    Eigen::Matrix<fp_t, 2, 2> & ATA_inv,
                    int DET_EPSILON) {                
    // Calculate ATA components and inverse
    fp_t IxIx(0);
    fp_t IyIy(0);
    fp_t IxIy(0);
    for (size_t j = 0; j < WIN_DIM; j++) {
        for (size_t k = 0; k < WIN_DIM; k++) {
            IxIx += (Ix_win_square(j, k) * Ix_win_square(j, k));
            IyIy += (Iy_win_square(j, k) * Iy_win_square(j, k));
            IxIy += (Ix_win_square(j, k) * Iy_win_square(j, k));
        }
    }

    // Separate int and decimal calculations because determinant needs 20 integer bits
    int IxIx_int = IxIx.to_int32();
    int IyIy_int = IyIy.to_int32();
    int IxIy_int = IxIy.to_int32();
    fp_t det = IxIx * IyIy - IxIy * IxIy;
    long det_int = ((long)IxIx_int) * ((long)IyIy_int) - ((long)IxIy_int) * ((long)IxIy_int);
    float full_det = ((float) det_int) + det.get_decimal(); // restore the integer bits
    det = fp_t(full_det); 

    // If matrix is nonsingular, mark status as failed
    if (det.to_int32() < DET_EPSILON && det.to_int32() > (-DET_EPSILON)) {
        return det;
    }

    // Calculate ATA_inv since the determinant is valid
    ATA_inv << IxIx / det, -IxIy / det, -IxIy / det, IyIy / det;
    return fp_t(0);
}

template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM>
void calcOpticalFlowIterLK( const RawImage<IMG_WIDTH, IMG_HEIGHT> & prevImg, 
                                const RawImage<IMG_WIDTH, IMG_HEIGHT> & nextImg,
                               const PointFP* prevPts, PointFP* nextPts,
                               bool* status, int num_good_points, int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA )
{
    int halfWin = (int) WIN_DIM / 2;

    for ( int i = 0; i < num_good_points; i++ ) {

        // Get previous point (fixed point and int)
        fp_t x(get<0>(nextPts[i]));
        fp_t y(get<1>(nextPts[i]));
        int x_i = x.to_int32();
        int y_i = y.to_int32();
        
        PointFP prevPt = prevPts[i];

        // Initalize guess pt, uv, and success
        PointFP guessPt = make_tuple(x, y);
        fp_t u(0);
        fp_t v(0);
        bool success = false;

        // If point is out of bounds, status is failed
        if ( x_i >= (int) prevImg.width || y_i >= (int) prevImg.height || x_i < 0 || y_i < 0) {
            // cout << "Point out of bounds " << "(" << x_i << ", " << y_i << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate gradients
        short Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        short Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Ix_win_square;
        Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> Iy_win_square;
        calc_gradient<IMG_WIDTH, IMG_HEIGHT, WIN_DIM+1>(prevPt, prevImg, Ix_arr, Iy_arr, halfWin);
        interpolate<short, WIN_DIM>(prevPt, Ix_arr, Ix_win_square, WIN_DIM+1, WIN_DIM+1);
        interpolate<short, WIN_DIM>(prevPt, Iy_arr, Iy_win_square, WIN_DIM+1, WIN_DIM+1);

        // Calculate ATA_inv
        Eigen::Matrix<fp_t, 2, 2> ATA_inv;
        fp_t det = a_transpose_a<WIN_DIM>(Ix_win_square, Iy_win_square, ATA_inv, DET_EPSILON);
        if (det.to_int32() != 0) {
            // cout << "Singular matrix at point (" << x << ", " << y << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate prev win
        Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> It_win_p;
        interpolate<uint8_t, WIN_DIM>(prevPt, ((uint8_t *)prevImg.data.data())+(y_i - halfWin)*prevImg.width+(x_i-halfWin), It_win_p, prevImg.width, prevImg.height-y_i);

        for (int j = 0; j < MAX_COUNT; j++) {

            // Calculate the time derivative
            Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> It_win_n;
            int guessPt_y_i = get<1>(guessPt).to_int32();
            int guessPt_x_i = get<0>(guessPt).to_int32();
            interpolate<uint8_t, WIN_DIM>(guessPt, ((uint8_t *)nextImg.data.data())+(guessPt_y_i - halfWin)*nextImg.width+(guessPt_x_i-halfWin), It_win_n, nextImg.width, nextImg.height-guessPt_y_i); 
            Eigen::Matrix<fp_t, WIN_DIM, WIN_DIM> It_win = It_win_n - It_win_p;

            // Compute b
            fp_t b0(0), b1(0);
            for (size_t k = 0; k < WIN_DIM; k++) {
                for (size_t l = 0; l < WIN_DIM; l++) {
                    b0 = b0 + Ix_win_square(k, l) * It_win(k, l);
                    b1 = b1 + Iy_win_square(k, l) * It_win(k, l);
                }
            }
            Eigen::Matrix<fp_t, 2, 1> b;
            b << b0 , b1;

            // Compute the least squares solution
            Eigen::Matrix<fp_t, 2, 1> soln = ATA_inv * (-b);
            u = soln(0, 0);
            v = soln(1, 0);
            success = true;
            get<0>(guessPt) += u;
            get<1>(guessPt) += v;

            if ((u * u + v * v) <= fp_t(CRITERIA)) break;        
        }

        status[i] = success;
        nextPts[i] = guessPt;
    }
}

#endif