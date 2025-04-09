#ifndef SIFT_H
#define SIFT_H

#include <image_io/Image.h>
#include <ento-feature2d/feat2d_util.h>
#include <math/FixedPoint.hh>

namespace EntoFeature2D
{

using SIFT_FP = FixedPoint<8, 8, int16_t>; // Q8.8

template <typename DoGImageT>
struct DoGTriplet
{
  const DoGImageT* prev;
  const DoGImageT* curr;
  const DoGImageT* next;
};

template <typename ImageT, typename DoGPixelT = SIFT_FP, int RowChunk = 0>
class SIFTDoGOctave
{
  using ImageT_ = ImageT;
  using PixelT_ = ImageT::pixel_type;
  using DoGPixelT_   = DoGPixelT;
  static constexpr int Height_ = ImageT::rows;
  static constexpr int Width_  = ImageT::cols;
  static constexpr int RowChunk_ = (RowChunk == 0) ? Height_ : RowChunk;

  static_assert(RowChunk_ <= Height_, "Block size must fit in image.");
  static_assert(RowChunk_ >= 0, "Row chunk must be non-negative.");
  static_assert((Height_ % RowChunk_) == 0, "Block size must be an divisor of height.");

  ImageT& input_img_;

  using DoGImageT_ = Image<Height_, Width_, DoGPixelT_>;
  DoGImageT_ gaussian_buffers_[2];
  DoGImageT_* gaussian_prev_p_;
  DoGImageT_* gaussian_curr_p_;
  DoGImageT_ dog_ring_[3];

  int dog_head      = 0;
  int current_scale = 0;

  SIFTDoGOctave(ImageT& img)
    : input_img_(img),
      gaussian_curr_p_(&gaussian_buffers_[0]),
      gaussian_prev_p_(&gaussian_buffers_[1])
  {}

  void initialize()
  {
    auto& gprev = gaussian_prev();
    auto& gcurr = gaussian_curr();
    for (int y = 0; y < Height_; y++)
    {
      for (int x = 0; x < Width_; x++)
      {
        gprev(y, x) = static_cast<DoGPixelT_>(input_img_(y, x));
      }
    }

    gaussian_blur_in_place<DoGImageT_, 5>(gprev);
    gaussian_blur_in_place<DoGImageT_, 5>(gcurr);

    compute_DoG(dog_ring_[dog_head], gcurr, gprev);
    // @TODO: Continue impl
  }

  bool step()
  {

  }
  
  DoGTriplet<DoGImageT_>
  get_current_DoG_triplet() const
  {
    return {
      &dog_ring_[(dog_head + 1) % 3], // prev
      &dog_ring_[(dog_head + 0) % 3], // curr
      &dog_ring_[(dog_head + 2) % 3], // next
    };
  }

  DoGImageT_& gaussian_prev() { return *gaussian_prev_p_; }
  DoGImageT_& gaussian_curr() { return *gaussian_curr_p_; }

  const DoGImageT_& gaussian_prev() const { return *gaussian_prev_p_; }
  const DoGImageT_& gaussian_curr() const { return *gaussian_curr_p_; }

private:
  void swap_gaussians() { std::swap(gaussian_prev_p_, gaussian_curr_p_); }
  void gaussian_blur();
  void compute_DoG();
};


template <typename ImageT, int MaxKeypoints>
class SIFTDriver
{
public:
  using KeypointT_ = SIFTKeypoint<>;
  using PixelT_ = ImageT::pixel_type;
  static constexpr int Height_ = ImageT::rows;
  static constexpr int Width_ = ImageT::cols;
  
  FeatureArray<KeypointT_, MaxKeypoints>& feats;


  void run(const Image<Height_, Width_, PixelT_>& base_img);

};

}


#endif
