#ifndef LK_OPTICAL_FLOW_ITER_H
#define LK_OPTICAL_FLOW_ITER_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <math/FixedPoint.hh>
#include <ento-feature2d/feat2d_util.h>
#include <image_io/Image.h>
// g++ -std=c++11 $(pkg-config --cflags eigen3 --libs opencv4) lk_optical_flow_fp_gem5.cpp -o lk_optical_flow_fp_gem5  -I/opt/homebrew/include
// ./lk_optical_flow_fp_gem5 -MAIN_ITERATIONS=150 -NUM_LEVELS=1 -MAX_COUNT=5 -WIN_DIM=15 -FRAME_RATE=150 -TEST_FOLDER=5 -FIRST_IMG=32

using namespace std;
using namespace EntoFeature2D;

const int scharr_x_arr[] = {-3, 0, 3, -10, 0, 10, -3, 0, 3};
const int scharr_y_arr[] = {-3, -10, -3, 0, 0, 0, 3, 10, 3};

// TODO: long vs longlong on cortex m4
// using PointFP = std::tuple<CoordT, CoordT>;

// window dimension used for the gradient calculation
// not the same as the window dimension used for the optical flow
template<size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT>
void calc_gradient(const Keypoint<CoordT> & pt,
                   const Image<IMG_HEIGHT, IMG_WIDTH, PixelT> & src,
                   int* Ix_arr, int* Iy_arr, const int halfWin) {
    
    // Indices for top left corner of window in src
    int32_t x_i = static_cast<int32_t>(pt.x)-halfWin;
    int32_t y_i = static_cast<int32_t>(pt.y)-halfWin;

    for (size_t i = 0; i < WIN_DIM; i++) { // row / height / y
        for (size_t j = 0; j < WIN_DIM; j++) { // col / width / x
            // kernel loop
            int Ix = 0;
            int Iy = 0;
            for (int k = 0; k < 3; k++) { // row / height / y
                for (int l = 0; l < 3; l++) { // col / width / x
                    size_t y_index = i+k+y_i - 1;
                    if (y_index < 0) y_index = 0;
                    else if (y_index >= src.rows) y_index = src.rows-1;

                    size_t x_index = j+l+x_i - 1;
                    if (x_index < 0) x_index = 0;
                    else if (x_index >= src.cols) x_index = src.cols-1;
                    Ix += ((int) src.data[src.rows*y_index + x_index])*scharr_x_arr[k*3+l];
                    Iy += ((int) src.data[src.rows*y_index + x_index])*scharr_y_arr[k*3+l];
                }
            }
            Ix_arr[WIN_DIM*i + j] = round(Ix / 32);
            Iy_arr[WIN_DIM*i + j] = round(Iy / 32);
        }
    }
}

// WIN_DIM is dimension of output window
template <typename SrcT, size_t WIN_DIM, typename CoordT>
void interpolate(const Keypoint<CoordT> & pt,
                 const SrcT * src,
                 Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> & dst,
                 int src_width,
                 int src_height) {
    int32_t x_i = static_cast<int32_t>(pt.x);
    int32_t y_i = static_cast<int32_t>(pt.y);
    CoordT a = pt.x - CoordT(x_i);
    CoordT b = pt.y - CoordT(y_i);

    CoordT one(1);

    CoordT iw00((one - a) * (one - b));
    CoordT iw01(a*(one-b));
    CoordT iw10((one-a)*b);
    CoordT iw11(one- iw00 - iw01 - iw10);

    for (size_t k = 0; k < WIN_DIM; k++) { // row / height / y
        for (size_t l = 0; l < WIN_DIM; l++) { // col / width / x
            int l_plus = l+1;
            if (l_plus >= src_width) l_plus = src_width - 1;

            int k_plus = k+1;
            if (k_plus >= src_height) l_plus = src_height - 1;

            if (l_plus >= src_width) l_plus = src_width - 1;
            CoordT ival(iw00 * CoordT(src[src_width*(k)     + l    ]) + 
                      iw01 * CoordT(src[src_width*(k)     + l + 1]) +
                      iw10 * CoordT(src[src_width*(k + 1) + l    ]) + 
                      iw11 * CoordT(src[src_width*(k + 1) + l + 1]));
            dst(k, l) = ival;
        }
    }
}

template <size_t WIN_DIM, typename CoordT>
CoordT a_transpose_a(const Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> & Ix_win_square, 
                    const Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> & Iy_win_square, 
                    Eigen::Matrix<CoordT, 2, 2> & ATA_inv,
                    int DET_EPSILON) {                
    // Calculate ATA components and inverse
    CoordT IxIx(0);
    CoordT IyIy(0);
    CoordT IxIy(0);
    for (size_t j = 0; j < WIN_DIM; j++) {
        for (size_t k = 0; k < WIN_DIM; k++) {
            IxIx += (Ix_win_square(j, k) * Ix_win_square(j, k));
            IyIy += (Iy_win_square(j, k) * Iy_win_square(j, k));
            IxIy += (Ix_win_square(j, k) * Iy_win_square(j, k));
        }
    }

    // Separate int and decimal calculations because determinant needs 20 integer bits
    int IxIx_int = static_cast<int32_t>(IxIx);
    int IyIy_int = static_cast<int32_t>(IyIy);
    int IxIy_int = static_cast<int32_t>(IxIy);
    CoordT det = IxIx * IyIy - IxIy * IxIy;
    long det_int = ((long)IxIx_int) * ((long)IyIy_int) - ((long)IxIy_int) * ((long)IxIy_int);
    float full_det = ((float) det_int) + (static_cast<float>(det) - static_cast<long>(det)); // restore the integer bits
    det = CoordT(full_det); 

    // If matrix is nonsingular, mark status as failed
    if (static_cast<int32_t>(det) < DET_EPSILON && static_cast<int32_t>(det) > (-DET_EPSILON)) {
        return det;
    }

    // Calculate ATA_inv since the determinant is valid
    ATA_inv << IxIx / det, -IxIy / det, -IxIy / det, IyIy / det;
    return CoordT(0);
}

template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT>
void calcOpticalFlowIterLK( const Image<IMG_HEIGHT, IMG_WIDTH, PixelT> & prevImg, 
                                const Image<IMG_HEIGHT, IMG_WIDTH, PixelT> & nextImg,
                               const Keypoint<CoordT>* prevPts, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA )
{
    int halfWin = (int) WIN_DIM / 2;

    for ( int i = 0; i < num_good_points; i++ ) {

        // Get previous point (fixed point and int)
        CoordT x(nextPts[i].x);
        CoordT y(nextPts[i].y);
        int x_i = static_cast<int32_t>(x);
        int y_i = static_cast<int32_t>(x);
        
        Keypoint<CoordT> prevPt = prevPts[i];

        // Initalize guess pt, uv, and success
        Keypoint<CoordT> guessPt(x, y);
        CoordT u(0);
        CoordT v(0);
        bool success = false;

        // If point is out of bounds, status is failed
        if ( x_i >= (int) prevImg.cols || y_i >= (int) prevImg.rows || x_i < 0 || y_i < 0) {
            // cout << "Point out of bounds " << "(" << x_i << ", " << y_i << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate gradients
        int Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        int Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> Ix_win_square;
        Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> Iy_win_square;
        calc_gradient<IMG_WIDTH, IMG_HEIGHT, WIN_DIM+1, CoordT, PixelT>(prevPt, prevImg, Ix_arr, Iy_arr, halfWin);
        interpolate<int, WIN_DIM, CoordT>(prevPt, Ix_arr, Ix_win_square, WIN_DIM+1, WIN_DIM+1);
        interpolate<int, WIN_DIM, CoordT>(prevPt, Iy_arr, Iy_win_square, WIN_DIM+1, WIN_DIM+1);

        // Calculate ATA_inv
        Eigen::Matrix<CoordT, 2, 2> ATA_inv;
        CoordT det = a_transpose_a<WIN_DIM, CoordT>(Ix_win_square, Iy_win_square, ATA_inv, DET_EPSILON);
        if (static_cast<int32_t>(det) != 0) {
            // cout << "Singular matrix at point (" << x << ", " << y << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate prev win
        Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win_p;
        interpolate<PixelT, WIN_DIM, CoordT>(prevPt, ((PixelT *)prevImg.data)+(y_i - halfWin)*prevImg.cols+(x_i-halfWin), It_win_p, prevImg.cols, prevImg.rows-y_i);

        for (int j = 0; j < MAX_COUNT; j++) {

            // Calculate the time derivative
            Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win_n;
            int guessPt_y_i = static_cast<int32_t>(guessPt.y);
            int guessPt_x_i = static_cast<int32_t>(guessPt.x);
            interpolate<PixelT, WIN_DIM, CoordT>(guessPt, ((PixelT *)nextImg.data)+(guessPt_y_i - halfWin)*nextImg.cols+(guessPt_x_i-halfWin), It_win_n, nextImg.cols, nextImg.rows-guessPt_y_i); 
            Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win = It_win_n - It_win_p;

            // Compute b
            CoordT b0(0), b1(0);
            for (size_t k = 0; k < WIN_DIM; k++) {
                for (size_t l = 0; l < WIN_DIM; l++) {
                    b0 = b0 + Ix_win_square(k, l) * It_win(k, l);
                    b1 = b1 + Iy_win_square(k, l) * It_win(k, l);
                }
            }
            Eigen::Matrix<CoordT, 2, 1> b;
            b << b0 , b1;

            // Compute the least squares solution
            Eigen::Matrix<CoordT, 2, 1> soln = ATA_inv * (-b);
            u = soln(0, 0);
            v = soln(1, 0);
            success = true;
            guessPt.x += u;
            guessPt.y += v;

            if ((u * u + v * v) <= CoordT(CRITERIA)) break;        
        }

        status[i] = success;
        nextPts[i] = guessPt;
    }
}

#endif
