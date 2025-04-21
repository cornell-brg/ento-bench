#ifndef SIFT_H
#define SIFT_H

#include <image_io/Image.h>
#include <image_io/image_util.h>
#include <ento-feature2d/feat2d_util.h>
#include <math/FixedPoint.hh>
#include <ento-math/core.h>

namespace EntoFeature2D
{

using SIFT_FP = FixedPoint<8, 8, int16_t>; // Q8.8

template <typename DoGImageT>
struct DoGTriplet
{
  const DoGImageT* prev;
  const DoGImageT* curr;
  const DoGImageT* next;

  const DoGImageT& prev_image() const { return *prev; }
  const DoGImageT& curr_image() const { return *curr; }
  const DoGImageT& next_image() const { return *next; }
};

template <typename ImageT,
          typename DoGPixelT = float,
          int NumDoGLayers = 4,
          int RingSize = 3,
          int RowChunk = 0>
class SIFTDoGOctave
{
  using ImageT_ = ImageT;
  using PixelT_ = ImageT::pixel_type_;
  using DoGPixelT_   = DoGPixelT;
  static constexpr int Height_ = ImageT::rows_;
  static constexpr int Width_  = ImageT::cols_;
  static constexpr int RowChunk_ = (RowChunk == 0) ? Height_ : RowChunk;
  static constexpr int NumDoGLayers_ = NumDoGLayers;
  static constexpr int NumBlurLevels_ = NumDoGLayers + 1;
  static constexpr int RingSize_ = RingSize;

  static_assert(RowChunk_ <= Height_, "Block size must fit in image.");
  static_assert(RowChunk_ >= 0, "Row chunk must be non-negative.");
  static_assert((Height_ % RowChunk_) == 0, "Block size must be an divisor of height.");

  ImageT& input_img_;

  using DoGImageT_ = Image<Height_, Width_, DoGPixelT_>;
  DoGImageT_ gaussian_buffers_[2];
  DoGImageT_* gaussian_prev_p_;
  DoGImageT_* gaussian_curr_p_;
  DoGImageT_ dog_ring_[RingSize];

  int dog_head_      = 0;
  int current_scale_ = 0;

public:
  SIFTDoGOctave(ImageT& img)
    : input_img_(img),
      gaussian_prev_p_(&gaussian_buffers_[1]),
      gaussian_curr_p_(&gaussian_buffers_[0])
  {}

  void initialize()
  {
    auto& gprev = gaussian_prev();
    for (int y = 0; y < Height_; y++)
    {
      for (int x = 0; x < Width_; x++)
      {
        gprev(y, x) = static_cast<DoGPixelT_>(input_img_(y, x));
      }
    }

    gaussian_blur<DoGImageT_, DoGImageT_, 5>(gaussian_prev(), gaussian_curr());
    compute_DoG(dog_ring_[0], gaussian_curr(), gaussian_prev());
    swap_gaussians();

    gaussian_blur<DoGImageT_, DoGImageT_, 5>(gaussian_prev(), gaussian_curr());
    compute_DoG(dog_ring_[1], gaussian_curr(), gaussian_prev());
    swap_gaussians();

    gaussian_blur<DoGImageT_, DoGImageT_, 5>(gaussian_prev(), gaussian_curr());
    compute_DoG(dog_ring_[2], gaussian_curr(), gaussian_prev());

    ENTO_DEBUG_IMAGE(gaussian_prev());
    ENTO_DEBUG_IMAGE(gaussian_curr());

    ENTO_DEBUG_IMAGE(dog_ring_[0]);
    ENTO_DEBUG_IMAGE(dog_ring_[1]);
    ENTO_DEBUG_IMAGE(dog_ring_[2]);

    dog_head_ = 2;
    current_scale_ = 3;
  }

  bool step()
  {
    if (current_scale_ >= NumDoGLayers_ - 1)
      return false;

    swap_gaussians();

    gaussian_blur<DoGImageT_, DoGImageT_, 5>(gaussian_prev(), gaussian_curr());

    int next = (dog_head_ + 1) % 3; // next in dog_ring_
    compute_DoG(dog_ring_[next], gaussian_curr(), gaussian_prev());

    dog_head_ = next;  // next is now current
    current_scale_ += 1;

    return true;
  }
  
  DoGTriplet<DoGImageT_>
  get_current_DoG_triplet() const
  {
    return {
      &dog_ring_[(dog_head_ + 1) % 3], // prev
      &dog_ring_[(dog_head_ + 2) % 3], // curr
      &dog_ring_[(dog_head_ + 0) % 3], // next
    };
  }

  DoGImageT_& gaussian_prev() { return *gaussian_prev_p_; }
  DoGImageT_& gaussian_curr() { return *gaussian_curr_p_; }

  const DoGImageT_& gaussian_prev() const { return *gaussian_prev_p_; }
  const DoGImageT_& gaussian_curr() const { return *gaussian_curr_p_; }

private:
  void swap_gaussians() { std::swap(gaussian_prev_p_, gaussian_curr_p_); }

  void compute_DoG(DoGImageT_& dst,
                   const DoGImageT_ curr,
                   const DoGImageT_ prev)
  {
    for (int y = 0; y < Height_; ++y)
      for (int x = 0; x < Width_; ++x)
        dst(y, x) = curr(y, x) - prev(y, x);
  }
};

template <typename InterpCoordT = float>
struct SIFTInterpolationResult
{
  InterpCoordT dx;
  InterpCoordT dy;
  InterpCoordT dscale;
  float interpolated_value;
  bool success;
};

struct DerivativesAtExtremum
{
  EntoMath::Vec3<float> grad;   // Dx, Dy, Ds
  EntoMath::Matrix3x3<float> H; // Full Hessian (Dxx, Dyy, Dss, and cross terms)

  float Dxx, Dyy, Dxy;          // For edge response test
};


template <typename ImageT,
          int MaxKeypoints,
          typename KeypointT = SIFTKeypoint<uint16_t>,
          int NumDoGLayers = 4,
          typename DoGPixelT = float,
          int RowChunk = 0>
class SIFTDriver
{
public:
  using ImageT_         = ImageT;
  using PixelT_         = ImageT::pixel_type_;
  using DoGPixelT_      = DoGPixelT;
  using KeypointT_      = KeypointT;
  using KeypointCoordT_ = KeypointT::CoordT_;
  using DescriptorT_    = KeypointT::DescriptorT_;
  using ScaleT_         = KeypointT::ScaleT_;
  using OrientationT_   = KeypointT::OrientationT_;
  static constexpr int   Height_ = ImageT::rows_;
  static constexpr int   Width_  = ImageT::cols_;
  static constexpr int   RowChunk_ = (RowChunk == 0) ? Height_ : RowChunk;
  static constexpr int   NumDoGLayers_ = NumDoGLayers;
  static constexpr int   NumBlurLevels_ = NumDoGLayers + 1;
  static constexpr int   MaxInterpIters_ = 5;
  static constexpr float ContrastThreshold_ = 0.03;

  static_assert(RowChunk_ <= Height_, "Block size must fit in image.");
  static_assert(RowChunk_ >= 0, "Row chunk must be non-negative.");
  static_assert((Height_ % RowChunk_) == 0, "Block size must be an divisor of height.");

  using DoGImageT_ = Image<Height_, Width_, DoGPixelT_>;

  ImageT& input_img_;
  
  FeatureArray<KeypointT_, MaxKeypoints>& feats_;


  void run(const Image<Height_, Width_, PixelT_>& base_img);

  SIFTDriver(ImageT& img, FeatureArray<KeypointT_, MaxKeypoints>& feats) : input_img_{img}, feats_{feats} {}

  void detect_extrema_in_triplet(const DoGTriplet<DoGImageT_>& triplet,
                                 int scale_idx, int octave)
  {
    constexpr int H = DoGImageT_::rows_;
    constexpr int W = DoGImageT_::cols;

    for (int y = 1; y < H - 1; ++y)
    {
      for (int x = 1; x < W - 1; ++x)
      {
        float center_val = triplet.curr_image()(y,x);
        ENTO_DEBUG("Center val: %f", center_val);
        bool is_max = true;
        bool is_min = true;

        for (int dz = -1; dz <= 1; ++dz)
        {
          const DoGImageT_* img =
            (dz == -1) ? &triplet.prev_image() :
            (dz == 0)  ? &triplet.curr_image() :
                         &triplet.next_image();

          for (int dy = -1; dy <= 1; ++dy)
          {
            for (int dx = -1; dx <= 1; ++dx)
            {
              if (dz == 0 && dy == 0 && dx == 0)
                continue; // skip center


              float val = (*img)(y + dy, x + dx);
              ENTO_DEBUG("DoGTriplet(%d, %d) = %f", y+dy, x+dx, val);
              if (center_val <= val)
                is_max = false;
              if (center_val >= val)
                is_min = false;
            }
          }
        }

        if ((is_max || is_min) && !feats_.full())
        {
          ENTO_DEBUG("Interpolating candidate!");
          SIFTInterpolationResult interp_result;
          if (!interpolate_extremum(triplet, x, y, scale_idx, interp_result))
            continue;

          feats_.add_keypoint(KeypointT{
            static_cast<KeypointCoordT_>(x),
            static_cast<KeypointCoordT_>(y),
            static_cast<uint8_t>(scale_idx),
            0,
            interp_result.interpolated_value 
          });
        }
      }
    }
  }

  DescriptorT_ score_current_extrema(const DoGTriplet<DoGImageT_>& triplet,
                                     KeypointCoordT_ x, KeypointCoordT_ y, ScaleT_ scale);
  
  bool interpolate_extremum(const DoGTriplet<DoGImageT_>& triplet,
                            KeypointCoordT_ x, KeypointCoordT_ y, ScaleT_ scale,
                            SIFTInterpolationResult<>& result)
  {
    for (int iter = 0; iter < MaxInterpIters_; ++iter)
    {
      DerivativesAtExtremum d = compute_gradient_and_hessian(triplet, x, y, scale);

      EntoMath::Vec3<float> x_hat;
      solve_offset(d.H, d.grad, x_hat);
      if ((x_hat.array().abs() < 0.5f).all())
      {
        result.success = true;
        result.dx      = x_hat[0];
        result.dy      = x_hat[1];
        result.dscale  = x_hat[2];

        float D = triplet.curr_image()(y,x);

        // Equation on page 11 in original sift paper
        result.interpolated_value = D + 0.5f * d.grad.dot(x_hat);

        // Evaluate wrt to ContrastThreshold
        if (std::abs(result.interpolated_value) < ContrastThreshold_)
          result.success = false;

        if (!passes_edge_response_check(d))
          result.success = false;

        return result.success;
      }

      // Update x/y as explained in section 4 of the original Lowe paper.
      // We do not update scale due to only having access to a DoG triplet.
      x += round(x_hat[0]);
      y += round(x_hat[1]);

      // If our x, y update move us out of bounds, this is a bad keypoint.
      // Reject it if so...
      if (x <= 0 || x >= Width_ - 1 || y <= 0 || y >= Height_ - 1) 
      {
        result.success = false;
        return false;
      }
    }
    result.success = false;
    return false;
  }

  // @TODO: Add support for full DoG octaves, that store generally 5 images in scale space
  //        although coule be more. This function would be used when we want to be able
  //        to interpolate extrema exactly how it is done in the original paper.
  //
  //bool interpolate_extremum(const DoGOctave<DoGImageT_>& octave,
  //                          KeypointCoordT_ x, KeypointCoordT_ y, ScaleT_ scale)
  //{

  //}

  DerivativesAtExtremum compute_gradient_and_hessian(const DoGTriplet<DoGImageT_>& triplet,
                                    int x, int y, int s)
  {
    DerivativesAtExtremum d;
    // Gradient ∇D
    float Dx = (triplet.curr_image()(y, x + 1) - triplet.curr_image()(y, x - 1)) * 0.5f;
    float Dy = (triplet.curr_image()(y + 1, x) - triplet.curr_image()(y - 1, x)) * 0.5f;
    float Ds = (triplet.next_image()(y, x) - triplet.prev_image()(y, x)) * 0.5f;

    d.grad = {Dx, Dy, Ds};

    // Hessian Calculation
    d.Dxx = triplet.curr_image()(y, x + 1) 
              - 2 * triplet.curr_image()(y, x) + triplet.curr_image()(y, x - 1);
    d.Dyy = triplet.curr_image()(y + 1, x)
              - 2 * triplet.curr_image()(y, x) + triplet.curr_image()(y - 1, x);

    float Dss = triplet.next_image()(y, x) 
              - 2 * triplet.curr_image()(y, x) + triplet.prev_image()(y, x);

    d.Dxy = (triplet.curr_image()(y + 1, x + 1) - triplet.curr_image()(y + 1, x - 1)
               - triplet.curr_image()(y - 1, x + 1) + triplet.curr_image()(y - 1, x - 1)) * 0.25f;

    float Dxs = (triplet.next_image()(y, x + 1) - triplet.next_image()(y, x - 1)
               - triplet.prev_image()(y, x + 1) + triplet.prev_image()(y, x - 1)) * 0.25f;

    float Dys = (triplet.next_image()(y + 1, x) - triplet.next_image()(y - 1, x)
               - triplet.prev_image()(y + 1, x) + triplet.prev_image()(y - 1, x)) * 0.25f;

    // Construct Hessian
    d.H(0, 0) = d.Dxx;  d.H(0, 1) = d.Dxy;  d.H(0, 2) = Dxs;
    d.H(1, 0) = d.Dxy;  d.H(1, 1) = d.Dyy;  d.H(1, 2) = Dys;
    d.H(2, 0) =   Dxs;  d.H(2, 1) =   Dys;  d.H(2, 2) = Dss;
    return d;
  }

  inline void solve_offset(const EntoMath::Matrix3x3<float>& H,
                         const EntoMath::Vec3<float>& grad,
                         EntoMath::Vec3<float>& x_hat)
  { 
    EntoMath::Matrix3x3<float> H_inv = H.inverse(); 
    x_hat = -H_inv * grad;
  }

  bool passes_edge_response_check(const DerivativesAtExtremum& d,
                                  float edge_threshold = 10.0f)
  {
    float tr  = d.Dxx + d.Dyy;
    float det = d.Dxx * d.Dyy - d.Dxy * d.Dxy;

    if (det <= 0.0f)
      return false;

    float r = edge_threshold;
    float ratio = (tr * tr) / det;
    float r_thresh = ((r + 1) * (r + 1)) / r;

    return ratio < r_thresh;
  }
};


} // namespace EntoFeature2D

#endif
