#ifndef II_OPTICAL_FLOW_H
#define II_OPTICAL_FLOW_H
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <math/FixedPoint.hh>
#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>



using namespace EntoFeature2D;


// x_shamt is the reference amount to shift prevImg along the +-x direction
// y_shamt is the reference amount to shift prevImg along the +-y direction
// Let f be the nextImg and f0 be the prevImg
// Define f1 = f0(x + x_shamt) and f2 = f0(x - x_shamt)
// Define f3 = f0(y + y_shamt) and f4 = f0(y - y_shamt)
// We define the velocity vector as (u,v) in pixel units
template <int IMG_WIDTH, int IMG_HEIGHT, int WIN_DIM, typename CoordT, typename PixelT>
void calcOpticalFlowII(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& prevImg, 
                                const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& nextImg,
                                Keypoint<CoordT>* prevPts, Keypoint<CoordT>* nextPts,
                               bool* status, int num_good_points,
                               int x_shamt, 
                               int y_shamt,
                               int DET_EPSILON)
{
  int halfWin = (int) WIN_DIM / 2;

  // Compute (f2 - f1), (f4-f3), 

  // iterate over num points
  for (int i = 0; i < num_good_points; i++ )
  {

    Keypoint<CoordT> pt = prevPts[i];

    // Indices for top left corner of window
    int x_i = static_cast<int32_t>(pt.x)-halfWin;
    int y_i = static_cast<int32_t>(pt.y)-halfWin;

    // Ix = f2 - f1; Iy = f4-f3; It = f-f0
    float IxIx = 0;
    float IyIy = 0;
    float IxIy = 0;
    float ItIx = 0;
    float ItIy = 0;

    // y coord
    for (int j = 0; j < WIN_DIM; j++)
    {
      // x coord
      for (int k = 0; k < WIN_DIM; k++)
      {
        int x_index = std::min(std::max(x_i + j, 0), prevImg.cols - 1);
        int x_l_index = std::min(std::max(x_index + j - x_shamt, 0), prevImg.cols - 1);
        int x_r_index = std::min(std::max(x_index + j + x_shamt, 0), prevImg.cols - 1);

        int y_index = std::min(std::max(y_i + k, 0), prevImg.rows - 1);
        int y_b_index = std::min(std::max(y_index + k - y_shamt, 0), prevImg.rows - 1);
        int y_t_index = std::min(std::max(y_index + k + y_shamt, 0), prevImg.rows - 1);


        // float Ix = ((float) prevImg.get_pixel(y_index, x_r_index) - prevImg.get_pixel(y_index, x_l_index));
        // float Iy = ((float) prevImg.get_pixel(y_t_index, x_index) - prevImg.get_pixel(y_b_index, x_index));
        // float It = ((float) nextImg.get_pixel(y_index, x_index) - prevImg.get_pixel(y_index, x_index)    );
        
        float Ix = ((float) prevImg.get_pixel(y_index, x_r_index) - prevImg.get_pixel(y_index, x_l_index)) / (WIN_DIM*WIN_DIM);
        float Iy = ((float) prevImg.get_pixel(y_t_index, x_index) - prevImg.get_pixel(y_b_index, x_index)) / (WIN_DIM*WIN_DIM);
        float It = ((float) nextImg.get_pixel(y_index, x_index) - prevImg.get_pixel(y_index, x_index)    ) / (WIN_DIM*WIN_DIM);
        

        IxIx += Ix * Ix;
        IyIy += Iy * Iy;
        IxIy += Ix * Iy;
        ItIx += It * Ix;
        ItIy += It * Iy;
      }
    }

    // IxIx = IxIx / (WIN_DIM * WIN_DIM);
    // IyIy = IyIy / (WIN_DIM * WIN_DIM);
    // IxIy = IxIy / (WIN_DIM * WIN_DIM);
    // ItIx = ItIx / (WIN_DIM * WIN_DIM);
    // ItIy = ItIy / (WIN_DIM * WIN_DIM);

    // We get the system of equations:
    // | Ix^2   IxIy | | u/x_shamt |   | 2ItIx |
    // |             | |           | = |       |
    // | IxIy   Iy^2 | | v/y_shamt | = | 2ItIy |

    // The determinant of the left matrix is (IxIx * IyIy - IxIy * IxIy)
    // The inverse is 
    // | Iy^2   -IxIy |
    // |              | * 1/det
    // | -IxIy   Ix^2 |


    float det = IxIx * IyIy - IxIy * IxIy;
    if (static_cast<int32_t>(det) < DET_EPSILON && static_cast<int32_t>(det) > (-DET_EPSILON))
    {
      status[i] = 0;
      nextPts[i] = pt;
      continue;
    }

    CoordT u = (((CoordT) IyIy * (CoordT) (2*ItIx)) - ((CoordT) IxIy * (CoordT) (2*ItIy))) / (CoordT) det;
    CoordT v = (((CoordT) IxIx * (CoordT) (2*ItIy)) - ((CoordT) IxIy * (CoordT) (2*ItIx)))  / (CoordT) det;
    
    Keypoint<CoordT> nextPt(pt.x + u * (CoordT)x_shamt, pt.y + v * (CoordT)y_shamt);
    nextPts[i] = nextPt;
    status[i] = 1;

  }
}

template <size_t IMG_WIDTH,
          size_t IMG_HEIGHT,
          size_t WIN_DIM,
          typename CoordT,
          typename PixelT,
          size_t NumFeats>
class ImageInterpolationOFKernel
{
private:
  bool status_[NumFeats];

  int search_radius_;
  int det_epsilon_;

public:
  using CoordT_ = CoordT;
  using PixelT_ = PixelT;
  static constexpr size_t Width_ = IMG_WIDTH;
  static constexpr size_t Height_ = IMG_HEIGHT;

  ImageInterpolationOFKernel(int search_radius,
                             int det_epsilon)
    : search_radius_(search_radius),
      det_epsilon_(det_epsilon)
  {}

  void operator()(const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img1,
                  const Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& img2,
                        FeatureArray<Keypoint<CoordT>, NumFeats>& feats,
                        FeatureArray<Keypoint<CoordT>, NumFeats>* feats_next)
  {
    feats_next->num_features = feats.num_features;

    calcOpticalFlowII<IMG_WIDTH, IMG_HEIGHT, WIN_DIM, CoordT, PixelT>(
      img1,
      img2,
      feats.keypoints.data(),
      feats_next->keypoints.data(),
      status_,
      feats.num_features,
      search_radius_,
      search_radius_,
      det_epsilon_
    );
  }

  static constexpr const char* name()
  {
    return "Image Interpolation Sparse Optical Flow";
  }
};

#endif
