#ifndef LK_OPTICAL_FLOW_H
#define LK_OPTICAL_FLOW_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <ento-feature2d/fixed_point.h>
#include <ento-feature2d/raw_image.h>
#include <ento-feature2d/image_pyramid_template.h>
#include <ento-feature2d/lk_optical_flow_iter.h>
// g++ -std=c++11 $(pkg-config --cflags eigen3 --libs opencv4) lk_optical_flow_fp_gem5.cpp -o lk_optical_flow_fp_gem5  -I/opt/homebrew/include
// ./lk_optical_flow_fp_gem5 -MAIN_ITERATIONS=150 -NUM_LEVELS=1 -MAX_COUNT=5 -WIN_DIM=15 -FRAME_RATE=150 -TEST_FOLDER=5 -FIRST_IMG=32

using namespace std;

#define MAX_WIN_DIM 15
#define IMG_SAVE
// Algorithm parameters
// TODO: constexpr

template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, size_t LEVEL>
void calcOpticalFlowPyrLKSingleIter(const RawImage<IMG_WIDTH, IMG_HEIGHT>& prevImg, 
                                const RawImage<IMG_WIDTH, IMG_HEIGHT>& nextImg, 
                                Keypoint<CoordT>* prevPyrPoints, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, 
                               int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA) {

    calcOpticalFlowIterLK<IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT>(prevImg, 
                                        nextImg, 
                                        prevPyrPoints, 
                                        nextPts, status, num_good_points, MAX_COUNT,
                                        DET_EPSILON, CRITERIA );
    CoordT two(2);
    if (LEVEL != 0) {
        for (int j = 0; j < num_good_points; j ++) {
        prevPyrPoints[j] = Keypoint<CoordT>(prevPyrPoints[j].x * two, prevPyrPoints[j].y * two);
        nextPts[j] = Keypoint<CoordT>(nextPts[j].x * two, nextPts[j].y * two);
        }
    }
}

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, size_t... Is>
void calcOpticalFlowPyrLKHelper(const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT>& prevPyramid, 
                                const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT>& nextPyramid,
                                Keypoint<CoordT>* prevPyrPoints, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, 
                               int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA,
                               std::index_sequence<Is...>) {
    (calcOpticalFlowPyrLKSingleIter<(IMG_WIDTH >> (NUM_LEVELS-Is)), (IMG_HEIGHT >> (NUM_LEVELS-Is)), WIN_DIM, CoordT, Is>(
                                        std::get<(NUM_LEVELS-Is)>(prevPyramid.pyramid), 
                                        std::get<(NUM_LEVELS-Is)>(nextPyramid.pyramid), 
                                        prevPyrPoints, 
                                        nextPts, status, num_good_points, MAX_COUNT,
                                        DET_EPSILON, CRITERIA ), ...
                                        );
}

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT>
void calcOpticalFlowPyrLK(const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT>& prevPyramid, 
                                const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT>& nextPyramid,
                                Keypoint<CoordT>* prevPts, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, 
                               int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA)
{

    CoordT divFactor(1 << NUM_LEVELS);

    // Initialize highest layer points
    Keypoint<CoordT> prevPyrPoints[num_good_points];
    for (int i = 0; i < num_good_points; i++) {
        prevPyrPoints[i] = Keypoint<CoordT>(prevPts[i].x / divFactor, prevPts[i].y / divFactor);
        nextPts[i]       = Keypoint<CoordT>(prevPts[i].x / divFactor, prevPts[i].y / divFactor);
    }

    calcOpticalFlowPyrLKHelper<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT>(prevPyramid, 
                                nextPyramid,
                                prevPyrPoints, nextPts,
                                status, num_good_points, 
                                MAX_COUNT,
                                DET_EPSILON, CRITERIA,
                                std::make_index_sequence<NUM_LEVELS+1>{});
    
}

#endif // LK_OPTICAL_FLOW_H
