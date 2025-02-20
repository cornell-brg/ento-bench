#ifndef LK_OPTICAL_FLOW_H
#define LK_OPTICAL_FLOW_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <ento-feature2d/fixed_point.h>
#include <ento-feature2d/raw_image.h>
#include <ento-feature2d/image_pyramid.h>
#include <ento-feature2d/lk_optical_flow_iter.h>
// g++ -std=c++11 $(pkg-config --cflags eigen3 --libs opencv4) lk_optical_flow_fp_gem5.cpp -o lk_optical_flow_fp_gem5  -I/opt/homebrew/include
// ./lk_optical_flow_fp_gem5 -MAIN_ITERATIONS=150 -NUM_LEVELS=1 -MAX_COUNT=5 -WIN_DIM=15 -FRAME_RATE=150 -TEST_FOLDER=5 -FIRST_IMG=32

using namespace std;

#define MAX_WIN_DIM 15
#define IMG_SAVE
// Algorithm parameters
// TODO: constexpr


template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM>
void calcOpticalFlowPyrLKPyr(const ImagePyramid<IMG_WIDTH, IMG_HEIGHT>* prevPyramid, const ImagePyramid<IMG_WIDTH, IMG_HEIGHT>* nextPyramid,
                                PointFP* prevPts, PointFP* nextPts,
                               bool* status, int num_good_points, int NUM_LEVELS, 
                               int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA)
{

    fp_t divFactor(1 << NUM_LEVELS);

    // Initialize highest layer points
    PointFP prevPyrPoints[num_good_points];
    for (int i = 0; i < num_good_points; i++) {
        prevPyrPoints[i] = PointFP(get<0>(prevPts[i]) / divFactor, get<1>(prevPts[i]) / divFactor);
        nextPts[i] = PointFP(get<0>(prevPts[i]) / divFactor, get<1>(prevPts[i]) / divFactor);
    }
    
    for (int i = NUM_LEVELS; i >= 0; i--) {
        calcOpticalFlowPyrLKSimpleIter<IMG_WIDTH, IMG_HEIGHT, WIN_DIM>(prevPyramid->get_level(i), nextPyramid->get_level(i), prevPyrPoints, 
                                        nextPts, status, num_good_points, MAX_COUNT,
                                        DET_EPSILON, CRITERIA );
        if (i != 0) {
            for (int j = 0; j < num_good_points; j ++) {
            prevPyrPoints[j] = PointFP(get<0>(prevPyrPoints[j]) * fp_2, get<1>(prevPyrPoints[j]) * fp_2);
            nextPts[j] = PointFP(get<0>(nextPts[j]) * fp_2, get<1>(nextPts[j]) * fp_2);
            }
        }
    }
}

#endif // LK_OPTICAL_FLOW_H
