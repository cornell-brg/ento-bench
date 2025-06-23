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

static const int scharr_x_arr[9] = {-3, 0, 3, -10, 0, 10, -3, 0, 3};
static const int scharr_y_arr[9] = {-3, -10, -3, 0, 0, 0, 3, 10, 3};

// TODO: long vs longlong on cortex m4
// using PointFP = std::tuple<CoordT, CoordT>;

// window dimension used for the gradient calculation
// not the same as the window dimension used for the optical flow
template<int IMG_WIDTH, int IMG_HEIGHT, int WIN_DIM, typename CoordT, typename PixelT>
void calc_gradient(const Keypoint<CoordT>& pt, const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& src, int* Ix_arr, int* Iy_arr, const int halfWin) {
    
    // Indices for top left corner of window in src
    int32_t x_i = static_cast<int32_t>(pt.x)-halfWin;
    int32_t y_i = static_cast<int32_t>(pt.y)-halfWin;

    for (int i = 0; i < WIN_DIM; i++) // row / height / y
    {
        for (int j = 0; j < WIN_DIM; j++) { // col / width / x
            // kernel loop
            int Ix = 0;
            int Iy = 0;
            for (int k = 0; k < 3; k++) { // row / height / y
                for (int l = 0; l < 3; l++) { // col / width / x
                    int y_index = i+k+y_i - 1;
                    if (y_index < 0) y_index = 0;
                    else if (y_index >= src.rows_) y_index = src.rows_-1;

                    int x_index = j+l+x_i - 1;
                    if (x_index < 0) x_index = 0;
                    else if (x_index >= src.cols_) x_index = src.cols_-1;
                    Ix += ((int) src.data[src.rows_*y_index + x_index])*scharr_x_arr[k*3+l];
                    Iy += ((int) src.data[src.rows_*y_index + x_index])*scharr_y_arr[k*3+l];
                }
            }
            Ix_arr[WIN_DIM*i + j] = round(Ix / 32);
            Iy_arr[WIN_DIM*i + j] = round(Iy / 32);
        }
    }
}

// WIN_DIM is dimension of output window
template <typename SrcT, int WIN_DIM, typename CoordT>
void interpolate(const Keypoint<CoordT> & pt, const SrcT * src, Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> & dst, int src_width, int src_height) {
    int32_t x_i = static_cast<int32_t>(pt.x);
    int32_t y_i = static_cast<int32_t>(pt.y);
    CoordT a = pt.x - CoordT(x_i);
    CoordT b = pt.y - CoordT(y_i);

    CoordT one(1);

    CoordT iw00((one - a) * (one - b));
    CoordT iw01(a*(one-b));
    CoordT iw10((one-a)*b);
    CoordT iw11(one- iw00 - iw01 - iw10);

    for (int k = 0; k < WIN_DIM; k++) { // row / height / y
        for (int l = 0; l < WIN_DIM; l++) { // col / width / x
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

template <int WIN_DIM, typename CoordT>
CoordT a_transpose_a(const Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM>& Ix_win_square, 
                     const Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM>& Iy_win_square, 
                     Eigen::Matrix<CoordT, 2, 2> & ATA_inv,
                     int DET_EPSILON) {                
    // Calculate ATA components and inverse
    CoordT IxIx(0);
    CoordT IyIy(0);
    CoordT IxIy(0);
    for (int j = 0; j < WIN_DIM; j++) {
        for (int k = 0; k < WIN_DIM; k++) {
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

template <int IMG_WIDTH, int IMG_HEIGHT, int WIN_DIM, typename CoordT, typename PixelT>
void calcOpticalFlowIterLK( const Image<IMG_HEIGHT, IMG_WIDTH, PixelT> & prevImg, 
                                const Image<IMG_HEIGHT, IMG_WIDTH, PixelT> & nextImg,
                               const Keypoint<CoordT>* prevPts, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA )
{
    int halfWin = (int) WIN_DIM / 2;
    //alignas(4) static int Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
    //alignas(4) static int Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];
    //alignas(4) static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> Ix_win_square;
    //alignas(4) static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> Iy_win_square;
    //alignas(4) static Eigen::Matrix<CoordT, 2, 2> ATA_inv;
    //alignas(4) static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win_p;
    //alignas(4) static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win_n;
    //alignas(4) static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win;
    ENTO_DEBUG("Started calcOpticalFlowIterLK for (%d/%d, %d/%d, %d)!", prevImg.cols_, IMG_WIDTH, prevImg.rows_, IMG_HEIGHT, WIN_DIM);

    for ( int i = 0; i < num_good_points; i++ ) {

        // Get previous point (fixed point and int)
        CoordT x(nextPts[i].x);
        CoordT y(nextPts[i].y);
        int x_i = static_cast<int32_t>(x);
        int y_i = static_cast<int32_t>(y);
        
        Keypoint<CoordT> prevPt = prevPts[i];

        // Initalize guess pt, uv, and success
        Keypoint<CoordT> guessPt(x, y);
        CoordT u(0);
        CoordT v(0);
        bool success = false;

        // If point is out of bounds, status is failed
        if ( x_i >= (int) prevImg.cols_ || y_i >= (int) prevImg.rows_ || x_i < 0 || y_i < 0) {
            // cout << "Point out of bounds " << "(" << x_i << ", " << y_i << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate gradients
        static int Ix_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        static int Iy_arr[(WIN_DIM+1)*(WIN_DIM+1)];
        static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> Ix_win_square;
        static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> Iy_win_square;
        ENTO_DEBUG("Calculating gradient!");
        calc_gradient<IMG_WIDTH, IMG_HEIGHT, WIN_DIM+1, CoordT, PixelT>(prevPt, prevImg, Ix_arr, Iy_arr, halfWin);
        ENTO_DEBUG("Finished calculating gradient!");
        interpolate<int, WIN_DIM, CoordT>(prevPt, Ix_arr, Ix_win_square, WIN_DIM+1, WIN_DIM+1);
        interpolate<int, WIN_DIM, CoordT>(prevPt, Iy_arr, Iy_win_square, WIN_DIM+1, WIN_DIM+1);
        ENTO_DEBUG("Computing Ix_win_square");
        ENTO_DEBUG("Computing Iy_win_square");

        // Calculate ATA_inv
        static Eigen::Matrix<CoordT, 2, 2> ATA_inv;
        ENTO_DEBUG("Computing ATA");
        CoordT det = a_transpose_a<WIN_DIM, CoordT>(Ix_win_square, Iy_win_square, ATA_inv, DET_EPSILON);
        if (static_cast<int32_t>(det) != 0) {
            // cout << "Singular matrix at point (" << x << ", " << y << ")" << endl;
            status[i] = 0;
            nextPts[i] = guessPt;
            continue;
        }

        // Calculate prev win
        ENTO_DEBUG("Computing It_win_p");
        static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win_p;
        interpolate<PixelT, WIN_DIM, CoordT>(prevPt, ((PixelT *)prevImg.data)+(y_i - halfWin)*prevImg.cols_+(x_i-halfWin), It_win_p, prevImg.cols_, prevImg.rows_-y_i);

        ENTO_DEBUG("Entering refinement loop...");
        for (int j = 0; j < MAX_COUNT; j++) {

            ENTO_DEBUG("refinement loop (%d/%d)...", j, MAX_COUNT);
            // Calculate the time derivative
            static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win_n;
            int guessPt_y_i = static_cast<int32_t>(guessPt.y);
            int guessPt_x_i = static_cast<int32_t>(guessPt.x);
            interpolate<PixelT, WIN_DIM, CoordT>(guessPt, ((PixelT *)nextImg.data)+(guessPt_y_i - halfWin)*nextImg.cols_+(guessPt_x_i-halfWin), It_win_n, nextImg.cols_, nextImg.rows_-guessPt_y_i); 
            static Eigen::Matrix<CoordT, WIN_DIM, WIN_DIM> It_win;
            It_win = (It_win_n - It_win_p).eval();

            // Compute b
            CoordT b0(0), b1(0);
            for (int k = 0; k < WIN_DIM; k++) {
                for (int l = 0; l < WIN_DIM; l++) {
                    b0 = b0 + Ix_win_square(k, l) * It_win(k, l);
                    b1 = b1 + Iy_win_square(k, l) * It_win(k, l);
                }
            }
            static Eigen::Matrix<CoordT, 2, 1> b;
            b << b0 , b1;

            // Compute the least squares solution
            static Eigen::Matrix<CoordT, 2, 1> soln = (ATA_inv * (-b)).eval();
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
    ENTO_DEBUG("Finished calcOpticalFlowIterLK for (%d, %d, %d)!", IMG_WIDTH, IMG_HEIGHT, WIN_DIM);
}

#endif
