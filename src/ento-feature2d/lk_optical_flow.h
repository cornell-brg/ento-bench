#ifndef LK_OPTICAL_FLOW_H
#define LK_OPTICAL_FLOW_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <math/FixedPoint.hh>
#include <ento-feature2d/image_pyramid.h>
#include <ento-feature2d/lk_optical_flow_iter.h>
#include <image_io/Image.h>
// g++ -std=c++11 $(pkg-config --cflags eigen3 --libs opencv4) lk_optical_flow_fp_gem5.cpp -o lk_optical_flow_fp_gem5  -I/opt/homebrew/include
// ./lk_optical_flow_fp_gem5 -MAIN_ITERATIONS=150 -NUM_LEVELS=1 -MAX_COUNT=5 -WIN_DIM=15 -FRAME_RATE=150 -TEST_FOLDER=5 -FIRST_IMG=32

using namespace std;

#define MAX_WIN_DIM 15
#define IMG_SAVE
// Algorithm parameters

template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT, size_t LEVEL>
void calcOpticalFlowPyrLKSingleIter(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& prevImg, 
                                const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& nextImg, 
                                Keypoint<CoordT>* prevPyrPoints, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, 
                               int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA) {

    calcOpticalFlowIterLK<IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT>(prevImg, 
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

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT, size_t... Is>
void calcOpticalFlowPyrLKHelper(const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT>& prevPyramid, 
                                const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT>& nextPyramid,
                                Keypoint<CoordT>* prevPyrPoints, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points, 
                               int MAX_COUNT,
                               int DET_EPSILON, float CRITERIA,
                               std::index_sequence<Is...>) {
    (calcOpticalFlowPyrLKSingleIter<(IMG_WIDTH >> (NUM_LEVELS-Is)), (IMG_HEIGHT >> (NUM_LEVELS-Is)), WIN_DIM, CoordT, PixelT, Is>(
                                        std::get<(NUM_LEVELS-Is)>(prevPyramid.pyramid), 
                                        std::get<(NUM_LEVELS-Is)>(nextPyramid.pyramid), 
                                        prevPyrPoints, 
                                        nextPts, status, num_good_points, MAX_COUNT,
                                        DET_EPSILON, CRITERIA ), ...
                                        );
}

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT>
void calcOpticalFlowPyrLK(const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT>& prevPyramid, 
                                const ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT>& nextPyramid,
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

    calcOpticalFlowPyrLKHelper<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT>(prevPyramid, 
                                nextPyramid,
                                prevPyrPoints, nextPts,
                                status, num_good_points, 
                                MAX_COUNT,
                                DET_EPSILON, CRITERIA,
                                std::make_index_sequence<NUM_LEVELS+1>{});
    
}

template <size_t NUM_LEVELS,
          size_t IMG_WIDTH,
          size_t IMG_HEIGHT,
          size_t WIN_DIM,
          typename CoordT,
          typename PixelT,
          size_t NumFeats>
class LucasKanadeOFKernel
{
private:
  ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT> prevPyramid_;
  ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT> nextPyramid_;
  bool status_[NumFeats];
  int num_good_points_;
  int MAX_COUNT_;
  int DET_EPSILON_;
  float CRITERIA_;

public:
  using CoordT_ = CoordT;
  using PixelT_ = PixelT;
  static constexpr size_t NumLevels_ = NUM_LEVELS;
  static constexpr size_t Width_ = IMG_WIDTH;
  static constexpr size_t Height_ = IMG_HEIGHT;

  LucasKanadeOFKernel(int num_good_points, 
                      int MAX_COUNT,
                      int DET_EPSILON,
                      float CRITERIA)
      : num_good_points_(num_good_points), MAX_COUNT_(MAX_COUNT), 
        DET_EPSILON_(DET_EPSILON), CRITERIA_(CRITERIA) {}
  
  void operator()(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img1,
                  const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img2,
                        FeatureArray<Keypoint<CoordT>, NumFeats>& feats,
                        FeatureArray<Keypoint<CoordT>, NumFeats>* feats_next)
  {
    prevPyramid_.set_top_image(img1);
    prevPyramid_.initialize_pyramid();

    nextPyramid_.set_top_image(img2);
    nextPyramid_.initialize_pyramid();

    // TODO: have function take in Feature Array 
    feats_next->num_features = num_good_points_;

    calcOpticalFlowPyrLK<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT>(prevPyramid_, nextPyramid_, 
                                                          (Keypoint<CoordT> *)feats.keypoints.data(), (Keypoint<CoordT> *)feats_next->keypoints.data(), status_, 
                                                          num_good_points_, MAX_COUNT_, DET_EPSILON_, CRITERIA_);

  }
  
  // Name method for identification
  static constexpr const char* name()
  {
    return "Lukas Kanade Sparse Optical Flow";
  }
};

// Static member definitions - required for static member variables
//template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT, size_t NumFeats>
//ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT> LucasKanadeOFKernel<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT, NumFeats>::prevPyramid_;
//
//template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t WIN_DIM, typename CoordT, typename PixelT, size_t NumFeats>
//ImagePyramid<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, PixelT> LucasKanadeOFKernel<NUM_LEVELS, IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT, NumFeats>::nextPyramid_;

#endif // LK_OPTICAL_FLOW_H
