#ifndef BB_OPTICAL_FLOW_H
#define BB_OPTICAL_FLOW_H
// Block-based optical flow
// Paper: An open source and open hardware embedded metric optical flow CMOS camera for indoor and outdoor applications

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <math/FixedPoint.hh>
#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>
#include <ento-feature2d/sad.h>

#ifdef NATIVE
#include <ento-feature2d/bb_optical_flow_debug.h>
#endif

using namespace EntoFeature2D;

template <int IMG_WIDTH, int IMG_HEIGHT, int WIN_DIM, typename CoordT, typename PixelT, int SEARCH_AREA, typename FlowT>
Keypoint<FlowT> calcOpticalFlowBB(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& prevImg, 
                                const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& nextImg,
                                Keypoint<CoordT>* prevPts, int num_good_points
                               )
{
    constexpr int halfWin = WIN_DIM / 2;

    // For histogram: bin size and range
    constexpr int hist_size = 2 * SEARCH_AREA + 1;
    int hist[hist_size][hist_size] = {0};

    // last corresponding point to the histogram
    int hist_pts[hist_size][hist_size] = {0};

    // iterate over num points
    for (int i = 0; i < num_good_points; i++ )
    {
        int min_sad = INT_MAX;
        int best_dx = 0;
        int best_dy = 0;

        Keypoint<CoordT> pt = prevPts[i];

        // Indices for top left corner of window
        int x_i = static_cast<int32_t>(pt.x)-halfWin;
        int y_i = static_cast<int32_t>(pt.y)-halfWin;

        for (int x_offset = -SEARCH_AREA; x_offset <= SEARCH_AREA; x_offset++) {
            for (int y_offset = -SEARCH_AREA; y_offset <= SEARCH_AREA; y_offset++) {

                // Indices for top left corner of offset window
                int x_i_win = x_i + x_offset;
                int y_i_win = y_i + y_offset;

                int sad_val = 0;

                // loop through window
                // encapsulate this? 
                // no clamp
                #ifdef NATIVE
                #ifdef ENTO_DEBUG
                // row * Cols + col
                const uint8_t* frame1 = &prevImg.data[y_i * IMG_WIDTH + x_i];
                const uint8_t* frame2 = &nextImg.data[y_i_win * IMG_WIDTH + x_i_win];
                // PixelT = uint8
                // IMG_WIDTH = IMG_HEIGHT = 320
                // WIN_DIM = 8
                sad_val = absdiff(frame1, frame2, IMG_WIDTH);
                #else
                for (int wx = 0; wx < WIN_DIM; wx++) {
                    for (int wy = 0; wy < WIN_DIM; wy++) {

                        
                        int x1 = x_i + wx;
                        int y1 = y_i + wy;

                        int x2 = x_i_win + wx;
                        int y2 = y_i_win + wy;

                        sad_val += std::abs(
                            (nextImg.get_pixel(y2, x2)) -
                            (prevImg.get_pixel(y1, x1))
                        );
                    }
                }
                #endif
                #else
                    // row * Cols + col
                    const uint8_t* frame1 = &prevImg.data[y_i * IMG_WIDTH + x_i];
                    const uint8_t* frame2 = &nextImg.data[y_i_win * IMG_WIDTH + x_i_win];
                    // PixelT = uint8
                    // IMG_WIDTH = IMG_HEIGHT = 320
                    // WIN_DIM = 8
                    sad_val = sad(frame1, frame2, IMG_WIDTH);
                #endif


                if (sad_val < min_sad) {
                    min_sad = sad_val;
                    best_dx = x_offset;
                    best_dy = y_offset;
                }
                
            }
        }
    
        // Populate histogram
        int bin_x = best_dx + SEARCH_AREA;
        int bin_y = best_dy + SEARCH_AREA;
        hist[bin_x][bin_y]++;
        hist_pts[bin_x][bin_y] = i;
    
    }

    // find the most frequent flow vector
    int max_freq = -1;
    Keypoint<CoordT> best_pt;
    int global_dx = 0;
    int global_dy = 0;
    for (int i = 0; i < hist_size; i++) {
        for (int j = 0; j < hist_size; j++) {
            int freq = hist[i][j];
        
            if (freq > max_freq) {
                max_freq = freq;
                best_pt = prevPts[hist_pts[i][j]];
                global_dx = i - SEARCH_AREA;
                global_dy = j - SEARCH_AREA;
            }
        }
    }

    // --------------------------------------//
    // refinement step
    // --------------------------------------//

    // Indices for top left corner of window
    int x_i = static_cast<int32_t>(best_pt.x)-halfWin;
    int y_i = static_cast<int32_t>(best_pt.y)-halfWin;

    // initialize best interp sad, and x/y flows
    float best_interp_sad = std::numeric_limits<float>::max();
    float best_x_offset = float(global_dx);
    float best_y_offset = float(global_dy);


    for (float x_offset = -0.5; x_offset <= 0.5; x_offset += 0.5) {
        for (float y_offset = -0.5; y_offset <= 0.5; y_offset += 0.5) {

            // float part of x coord and y coord
            float a = std::abs(x_offset);
            float b = std::abs(y_offset);

            // interp params
            float iw00((1.f - a) * (1.f - b));
            float iw01(a*(1.f-b));
            float iw10((1.f-a)*b);
            float iw11(1.f- iw00 - iw01 - iw10);

            // top left corner of next window
            int x_i_win = x_i + std::floor(x_offset);
            int y_i_win = y_i + std::floor(y_offset);

            float interp_sad = 0.f;
            for (int wx = 0; wx < WIN_DIM; wx++) {
                for (int wy = 0; wy < WIN_DIM; wy++) {

                    int x1 = x_i + wx;
                    int y1 = y_i + wy;

                    int x2 = x_i_win + wx;
                    int y2 = y_i_win + wy;

                    interp_sad += std::abs(float(prevImg.get_pixel(y1, x1)) - 
                        (iw00 * float(nextImg.get_pixel(y2, x2)) + 
                        iw01 * float(nextImg.get_pixel(y2, x2 + 1)) + 
                        iw10 * float(nextImg.get_pixel(y2 + 1, x2)) + 
                        iw11 * float(nextImg.get_pixel(y2 + 1, x2 + 1))));

                }
            }
            if (interp_sad <= best_interp_sad) {
                best_interp_sad = interp_sad;
                best_x_offset = x_offset + float(global_dx);
                best_y_offset = y_offset + float(global_dy);

            }
        }
    }

    Keypoint<FlowT> flow;
    flow.x = best_x_offset;
    flow.y = best_y_offset;

    return flow;
}

template <size_t IMG_WIDTH,
          size_t IMG_HEIGHT,
          size_t WIN_DIM,
          typename CoordT,
          typename PixelT,
          size_t NumFeats,
          size_t SEARCH_AREA>
class BlockBasedOFKernel
{
public:
  using CoordT_ = CoordT;
  using PixelT_ = PixelT;
  static constexpr size_t Width_ = IMG_WIDTH;
  static constexpr size_t Height_ = IMG_HEIGHT;

  BlockBasedOFKernel() {}

  void operator()(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img1,
                  const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img2,
                        FeatureArray<Keypoint<CoordT>, NumFeats>& feats,
                        FeatureArray<Keypoint<CoordT>, NumFeats>* feats_next)
  {
    // Block-based optical flow returns a single global flow vector
    // We'll apply this to all features
    Keypoint<CoordT> global_flow = calcOpticalFlowBB<IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT, SEARCH_AREA, CoordT>(
      img1,
      img2,
      feats.keypoints.data(),
      feats.num_features
    );

    feats_next->num_features = feats.num_features;
    
    // Apply global flow to all features
    for (size_t i = 0; i < feats.num_features; ++i) {
      feats_next->keypoints[i].x = feats.keypoints[i].x + global_flow.x;
      feats_next->keypoints[i].y = feats.keypoints[i].y + global_flow.y;
    }
  }

  static constexpr const char* name()
  {
    return "Block-Based Optical Flow";
  }
};

#endif 