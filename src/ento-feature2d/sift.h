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
          int NumDoGLayers = 3,
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
  static constexpr int NumDoGLayers_ = NumDoGLayers + 2;
  static constexpr int NumBlurLevels_ = NumDoGLayers + 1;
  static constexpr int RingSize_ = RingSize;
  static constexpr int KernelSize = 5;

  static_assert(RowChunk_ <= Height_, "Block size must fit in image.");
  static_assert(RowChunk_ >= 0, "Row chunk must be non-negative.");
  static_assert((Height_ % RowChunk_) == 0, "Block size must be an divisor of height.");

  ImageT& input_img_;

  using DoGImageT_ = Image<Height_, Width_, DoGPixelT_>;
  DoGImageT_ gaussian_buffers_[2];
  DoGImageT_* gaussian_prev_p_;
  DoGImageT_* gaussian_curr_p_;
  DoGImageT_ dog_ring_[RingSize];
  std::array<std::array<float,KernelSize>, NumDoGLayers_ + 2> kernels_;
  DoGImageT_ orientation_map_;

  // Note: We compute DoGs incrementally, no need to store all Gaussians

  // SIFT++ parameters
  static constexpr float base_sigma = 1.6f;

  int dog_head_      = 0;
  int current_scale_ = 0;

public:
  SIFTDoGOctave(ImageT& img)
    : input_img_(img),
      gaussian_prev_p_(&gaussian_buffers_[1]),
      gaussian_curr_p_(&gaussian_buffers_[0])
  {
    precompute_kernels();
  }

  void precompute_kernels() {
    constexpr float base_sigma        = 1.6f;
    constexpr float sigman            = 0.5f;  // Input assumed to have this much blur
    constexpr int   levels_per_octave = 3;     // Standard SIFT uses 3 levels per octave (not NumDoGLayers_)
    const float     k                = std::pow(2.0f, 1.0f/levels_per_octave);  // k = 2^(1/3) = 1.26
    const float     sigma0           = base_sigma * k;  // 1.6 * 1.26 = 2.016 (like SIFT++)

    ENTO_DEBUG("SIFT kernel computation (fixed to match SIFT++):");
    ENTO_DEBUG("  base_sigma = %f", base_sigma);
    ENTO_DEBUG("  sigman = %f", sigman);
    ENTO_DEBUG("  levels_per_octave = %d", levels_per_octave);
    ENTO_DEBUG("  k = %f", k);
    ENTO_DEBUG("  sigma0 = %f", sigma0);

    // First kernel: Apply initial blur to get from sigman (0.5) to sigma0 (2.016)
    float sigma_curr = sigma0;
    float delta = std::sqrt(sigma_curr*sigma_curr - sigman*sigman);
    ENTO_DEBUG("  kernel[1]: sigma_curr=%f, sigma_prev=%f, delta=%f", sigma_curr, sigman, delta);
    kernels_[1] = make_gaussian_kernel<float, KernelSize>(delta);
    
    // Subsequent kernels: incremental blurring
    // For 3 DoG layers, we need 4 Gaussian levels (G0, G1, G2, G3), so 4 kernels total
    float sigma_prev = sigma_curr;
    for (int i = 2; i <= NumDoGLayers_ + 1; ++i) {
      sigma_curr = sigma0 * std::pow(k, i-1);  // sigma0*k^0, sigma0*k^1, sigma0*k^2
      delta = std::sqrt(sigma_curr*sigma_curr - sigma_prev*sigma_prev);
      ENTO_DEBUG("  kernel[%d]: sigma_curr=%f, sigma_prev=%f, delta=%f", i, sigma_curr, sigma_prev, delta);
      kernels_[i] = make_gaussian_kernel<float, KernelSize>(delta);
      sigma_prev = sigma_curr;
    }
  }

  void initialize()
  {
    // Step 1: Normalize input image (like test 3 and SIFT++)
    for (int y = 0; y < Height_; ++y)
      for (int x = 0; x < Width_; ++x)
        gaussian_prev()(y, x) = static_cast<DoGPixelT_>(input_img_(y, x)) / 255.0f;

    ENTO_DEBUG("Input image normalized:");
    //ENTO_DEBUG_IMAGE(make_centered_view(gaussian_prev(), 17, 16, 15, 15));

    // Step 2: SIFT++ parameters (exactly from sift.cpp)
    // SIFT++ uses S=3 for levels per octave, smin=-1, smax=S+1=4
    constexpr int S = 3;           // levels per octave (SIFT++ constant)
    constexpr int smin = -1;       // first level (SIFT++ default) 
    constexpr int omin = 0;        // first octave (for --first-octave 0)
    constexpr float sigman = 0.5f; // assumed input sigma (SIFT++ default)
    
    float sigmak = powf(2.0f, 1.0f / S);  // SIFT++: powf(2.0f, 1.0 / S)
    float sigma0 = base_sigma * sigmak;    // SIFT++ driver: sigma0 = 1.6 * k = 2.015874
    float dsigma0 = sigma0 * sqrtf(1.0f - 1.0f / (sigmak*sigmak));  // SIFT++ dsigma0
    
    ENTO_DEBUG("SIFT++ parameters: S=%d, smin=%d, sigmak=%.6f, sigma0=%.6f, dsigma0=%.6f", S, smin, sigmak, sigma0, dsigma0);
    printf("SIFT++ parameters: S=%d, smin=%d, sigmak=%.6f, sigma0=%.6f, dsigma0=%.6f\n", S, smin, sigmak, sigma0, dsigma0);

    // Step 3: Initial smoothing (exactly from SIFT++ process())
    // From SIFT++: sa = sigma0 * powf(sigmak, smin), sb = sigman / powf(2.0f, omin)
    float sa = sigma0 * powf(sigmak, smin);  // sigma0 * sigmak^(-1) = 2.015874 / 1.259921 = 1.6
    float sb = sigman / powf(2.0f, omin);        // sigman / 2^0 = 0.5
    float initial_sigma = sqrtf(sa*sa - sb*sb);  // sqrt(sa^2 - sb^2)
    
    ENTO_DEBUG("Initial smoothing: sa=%.6f, sb=%.6f, initial_sigma=%.6f", sa, sb, initial_sigma);
    printf("Initial smoothing: sa=%.6f, sb=%.6f, initial_sigma=%.6f\n", sa, sb, initial_sigma);
    
    sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), initial_sigma);
    // G(-1) is now in gaussian_curr()
    
    ENTO_DEBUG("After initial smoothing G(smin=%d): sigma=%.6f", smin, initial_sigma);
    ENTO_DEBUG_IMAGE(make_centered_view(gaussian_curr(), 17, 16, 15, 15));

    // Step 4: Incremental inter-level smoothing with DoG computation
    // From SIFT++: for(int s = smin+1 ; s <= smax ; ++s) { float sd = dsigma0 * powf(sigmak, s); }
    // We need to compute enough Gaussians to fill the 3-element ring buffer and provide first triplet
    
    // G(0) = smooth(G(-1), dsigma0 * sigmak^0)
    float sd0 = dsigma0 * powf(sigmak, 0);  // s=0
    swap_gaussians();  // Move G(-1) to gaussian_prev()
    sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), sd0);
    // Now: gaussian_prev() = G(-1), gaussian_curr() = G(0)
    compute_DoG(dog_ring_[0], gaussian_curr(), gaussian_prev());  // G(0) - G(-1)
    
    ENTO_DEBUG("After inter-level G(0): sd=%.6f", sd0);
    printf("Inter-level G(0): sd=%.6f\n", sd0);
    ENTO_DEBUG_IMAGE(make_centered_view(gaussian_curr(), 17, 16, 15, 15));

    // G(1) = smooth(G(0), dsigma0 * sigmak^1)  
    float sd1 = dsigma0 * powf(sigmak, 1);  // s=1
    swap_gaussians();  // Move G(0) to gaussian_prev()
    sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), sd1);
    // Now: gaussian_prev() = G(0), gaussian_curr() = G(1)
    
    // Compute DoG(0) = G(1) - G(0) as soon as we have both
    compute_DoG(dog_ring_[1], gaussian_curr(), gaussian_prev());  // G(1) - G(0)
    ENTO_DEBUG("DoG[0] (G1 - G0):");
    ENTO_DEBUG_IMAGE(make_centered_view(dog_ring_[0], 17, 16, 7, 7));
    
    ENTO_DEBUG("After inter-level G(1): sd=%.6f", sd1);
    ENTO_DEBUG_IMAGE(make_centered_view(gaussian_curr(), 17, 16, 15, 15));

    // G(2) = smooth(G(1), dsigma0 * sigmak^2)
    float sd2 = dsigma0 * powf(sigmak, 2);  // s=2
    swap_gaussians();  // Move G(1) to gaussian_prev()
    sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), sd2);
    // Now: gaussian_prev() = G(1), gaussian_curr() = G(2)
    
    // Compute DoG(1) = G(2) - G(1) as soon as we have both
    compute_DoG(dog_ring_[2], gaussian_curr(), gaussian_prev());  // G(2) - G(1)
    ENTO_DEBUG("DoG[1] (G2 - G1):");
    ENTO_DEBUG_IMAGE(make_centered_view(dog_ring_[1], 17, 16, 7, 7));
    
    ENTO_DEBUG("After inter-level G(2): sd=%.6f", sd2);
    ENTO_DEBUG_IMAGE(make_centered_view(gaussian_curr(), 17, 16, 15, 15));

    // G(3) = smooth(G(2), dsigma0 * sigmak^3) 
    // float sd3 = dsigma0 * powf(sigmak, 3);  // s=3
    // swap_gaussians();  // Move G(2) to gaussian_prev()
    // sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), sd3);
    // // Now: gaussian_prev() = G(2), gaussian_curr() = G(3)
    
    // // Compute DoG(2) = G(3) - G(2) as soon as we have both
    // compute_DoG(dog_ring_[2], gaussian_curr(), gaussian_prev());  // G(3) - G(2)
    // ENTO_DEBUG("DoG[2] (G3 - G2):");
    // ENTO_DEBUG_IMAGE(make_centered_view(dog_ring_[2], 17, 16, 7, 7));
    
    // ENTO_DEBUG("After inter-level G(3): sd=%.6f", sd3);
    // ENTO_DEBUG_IMAGE(make_centered_view(gaussian_curr(), 17, 16, 15, 15));

    // ENTO_DEBUG("\nComputed DoG triplet for extrema detection.\n");

    // Step 5: Set up for extrema detection
    dog_head_ = 2;  // Point to DoG2
    current_scale_ = 2;  // Next scale would be 3, but we stop here for 3-layer config
    
    // After initialization: gaussian_prev() = G(2), gaussian_curr() = G(3)
    // This is perfect for potential step() calls
  }

  bool step()
  {
    if (current_scale_ >= NumDoGLayers_ + 2)
      return false;

    ENTO_DEBUG("\nSwapping gaussians.\n");
    int kernel_idx = current_scale_ + 1;

    // Compute the appropriate delta sigma for this step using SIFT++ method
    constexpr int S = 3;           // levels per octave (SIFT++ constant)
    float sigmak = powf(2.0f, 1.0f / S);  // SIFT++: powf(2.0f, 1.0 / S)
    float sigma0 = base_sigma * sigmak;    // SIFT++ driver: sigma0 = 1.6 * k = 2.015874
    float dsigma0 = sigma0 * sqrtf(1.0f - 1.0f / (sigmak*sigmak));  // SIFT++ dsigma0
    float delta_sigma = dsigma0 * std::pow(sigmak, kernel_idx);  // SIFT++ inter-level
    //float sd3 = dsigma0 * powf(sigmak, 3);
    swap_gaussians();
    sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), delta_sigma);
    ENTO_DEBUG_IMAGE(make_centered_view(gaussian_prev(), 16, 16, 5, 5));
    ENTO_DEBUG_IMAGE(make_centered_view(gaussian_curr(), 16, 16, 5, 5));

    int next = (dog_head_ + 1) % 3; // next in dog_ring_
    compute_DoG(dog_ring_[next], gaussian_curr(), gaussian_prev());
    ENTO_DEBUG("\nComputed a DoG triplet.\n");

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

  // Add this method to recompute specific Gaussian levels
  const DoGImageT_& recompute_gaussian_for_scale(int target_scale) {
    
    // Step 1: Reset to initial state and normalize input
    normalize_input_image();  // Use gaussian_prev() as temp
    
    // Step 2: Initial smoothing to G(-1) 
    float initial_sigma = sqrtf(1.6f*1.6f - 0.5f*0.5f);
    sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), initial_sigma);
    // Now: gaussian_curr() = G(-1)
    
    if (target_scale == -1) {
      return gaussian_curr();  // Return G(-1)
    }
    
    // Step 3: Incremental smoothing to target scale
    constexpr int S = 3;
    float sigmak = powf(2.0f, 1.0f / S);
    float sigma0 = base_sigma * sigmak;
    float dsigma0 = sigma0 * sqrtf(1.0f - 1.0f / (sigmak*sigmak));
    
    // Apply incremental smoothing: G(-1) → G(0) → G(1) → ... → G(target_scale)
    for (int s = 0; s <= target_scale; s++) {
      float delta_sigma = dsigma0 * powf(sigmak, s);
      swap_gaussians();  // Move current to prev
      sift_smooth_efficient<DoGImageT_, 17>(gaussian_prev(), gaussian_curr(), delta_sigma);
      // Now: gaussian_curr() = G(s)
    }
    
    return gaussian_curr();  // Return G(target_scale)
  }

  DoGImageT_& gaussian_prev() { return *gaussian_prev_p_; }
  DoGImageT_& gaussian_curr() { return *gaussian_curr_p_; }

  const DoGImageT_& gaussian_prev() const { return *gaussian_prev_p_; }
  const DoGImageT_& gaussian_curr() const { return *gaussian_curr_p_; }
  
  // Access current Gaussians for debugging
  const DoGImageT_& get_current_gaussian_prev() const { 
    return gaussian_prev(); 
  }
  const DoGImageT_& get_current_gaussian_curr() const { 
    return gaussian_curr(); 
  }

private:
  void swap_gaussians() { std::swap(gaussian_prev_p_, gaussian_curr_p_); }

  void normalize_input_image() {
    // Normalize input image into gaussian_prev()
    for (int y = 0; y < Height_; ++y) {
      for (int x = 0; x < Width_; ++x) {
        gaussian_prev()(y, x) = static_cast<DoGPixelT_>(input_img_(y, x)) / 255.0f;
      }
    }
  }

  void compute_DoG(DoGImageT_& dst,
                   const DoGImageT_ curr,
                   const DoGImageT_ prev)
  {
    for (int y = 0; y < Height_; ++y)
      for (int x = 0; x < Width_; ++x)
        dst(y, x) = curr(y, x) - prev(y, x);
  }

public:
  void compute_gradients_for_current_gaussian() {
    const DoGImageT_& gaussian = gaussian_curr();  // Source Gaussian
    DoGImageT_& magnitude = gaussian_prev();       // Reuse prev buffer
    DoGImageT_& orientation = orientation_map_;    // Use dedicated buffer
    
    // Compute gradients for entire image
    for (int y = 1; y < Height_ - 1; y++) {
      for (int x = 1; x < Width_ - 1; x++) {
        float Gx = 0.5f * (gaussian(y, x+1) - gaussian(y, x-1));
        float Gy = 0.5f * (gaussian(y+1, x) - gaussian(y-1, x));
        
        magnitude(y, x) = sqrtf(Gx*Gx + Gy*Gy);
        orientation(y, x) = atan2f(Gy, Gx);
        if (orientation(y, x) < 0) {
          orientation(y, x) += 2.0f * M_PI;
        }
      }
    }
  }

  // Access methods for gradients
  const DoGImageT_& get_gradient_magnitude() const { return gaussian_prev(); }
  const DoGImageT_& get_gradient_orientation() const { return orientation_map_; }

private:
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
          typename KeypointT = SIFTKeypoint<>,
          int NumDoGLayers = 3,
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
  static constexpr int   NumBlurLevels_ = NumDoGLayers_ + 1;
  static constexpr int   MaxInterpIters_ = 5;
  static constexpr float ContrastThreshold_ = 0.04f / 3.0f / 2.0f;  // Match SIFT++: 0.04/S/2 = 0.00667
  static constexpr float InitialThreshold_ = 0.8f * ContrastThreshold_;  // 0.8 * 0.00667 = 0.00533

  static_assert(RowChunk_ <= Height_, "Block size must fit in image.");
  static_assert(RowChunk_ >= 0, "Row chunk must be non-negative.");
  static_assert((Height_ % RowChunk_) == 0, "Block size must be an divisor of height.");

  using DoGImageT_ = Image<Height_, Width_, DoGPixelT_>;

  ImageT& input_img_;
  
  FeatureArray<KeypointT_, MaxKeypoints>& feats_;

  // Pointer to current octave for external access (valid only during/after run())
  SIFTDoGOctave<ImageT, DoGPixelT, NumDoGLayers>* current_octave_ = nullptr;


  bool run(const Image<Height_, Width_, PixelT_>& base_img)
  {
    // 1. Compute DoG pyramid
    SIFTDoGOctave<ImageT, DoGPixelT, NumDoGLayers> octave(input_img_);
    octave.initialize();
    current_octave_ = &octave; // Store reference for external access

    int scale_idx = 0;
    do {
      detect_extrema_in_triplet(octave.get_current_DoG_triplet(), scale_idx, 0);
      scale_idx++;
    } while (octave.step());

    if (feats_.size() < 1)
    {
      ENTO_DEBUG("No extrema found");
      return false;
    }

    constexpr int MaxPerScale = 50;
    struct ScaleGroup {
      KeypointT_ keypoints_[MaxPerScale];
      int count = 0;
    };

    ScaleGroup groups[NumDoGLayers_ + 2];

    for (int i = 0; i < feats_.size(); ++i)
    {
      int scale_index = determine_scale_index(feats_[i].scale);
      if (scale_index >= 0 && scale_index < NumDoGLayers_ + 2) {
        if (groups[scale_index].count < MaxPerScale) {
          groups[scale_index].keypoints_[groups[scale_index].count] = feats_[i];
          groups[scale_index].count++;
        }
      }
    }

    // Clear the initial detection keypoints - we'll replace them with processed ones
    feats_.clear();
    
    // 3. Orientation and Descriptor Computation
    int total_features = 0;
    for (int scale_idx = 0; scale_idx < NumDoGLayers_ + 2; ++scale_idx)
    {
      if (groups[scale_idx].count == 0)
        continue; // Skip empty scale groups

      const DoGImageT_& gaussian = octave.recompute_gaussian_for_scale(scale_idx);

      octave.compute_gradients_for_current_gaussian();

      for (int kp_idx = 0; kp_idx < groups[scale_idx].count; ++kp_idx)
      {
        const KeypointT_& kp = groups[scale_idx].keypoints_[kp_idx];
        float mag = octave.get_gradient_magnitude()(kp.y, kp.x);
        float ori = octave.get_gradient_orientation()(kp.y, kp.x);
        process_keypoint(kp, octave.get_gradient_magnitude(), octave.get_gradient_orientation());
      }

    }
    return true;
  }

  int determine_scale_index(ScaleT_ scale)
  {
    // Convert fractional scale to discrete index
    return static_cast<int>(std::round(scale));
  }

  void process_keypoint(const KeypointT_& kp, 
                     const DoGImageT_& gradient_magnitude,
                     const DoGImageT_& gradient_orientation)
  {

  
    // Step 1: Orientation Assignment
    float orientations[4];
    int num_orientations = compute_keypoint_orientations(kp, gradient_magnitude, gradient_orientation, orientations);
    
    // Step 2: Descriptor Computation for each orientation
    for (int ori_idx = 0; ori_idx < num_orientations; ori_idx++) {
      if (feats_.full()) return;  // Check capacity using FeatureArray method
      
      float descriptor[128];
      compute_keypoint_descriptor(kp, orientations[ori_idx], gradient_magnitude, gradient_orientation, descriptor);
      
      // Step 3: Create final SIFT keypoint with orientation and descriptor
      KeypointT_ final_kp(kp.x, kp.y, kp.scale, 0, orientations[ori_idx], kp.response, 0.0f);
      final_kp.set_descriptor(descriptor);
      
      // Add to feature array
      feats_.add_keypoint(final_kp);
    }
  }

  SIFTDriver(ImageT& img, FeatureArray<KeypointT_, MaxKeypoints>& feats) : input_img_{img}, feats_{feats} {}
  
  // Get access to the current octave (valid only after run() is called)
  SIFTDoGOctave<ImageT, DoGPixelT, NumDoGLayers>* get_current_octave() { return current_octave_; }

  void detect_extrema_in_triplet(const DoGTriplet<DoGImageT_>& triplet,
                                 int scale_idx, int octave)
  {
    constexpr int H = DoGImageT_::rows_;
    constexpr int W = DoGImageT_::cols_;

    int extrema_candidates = 0;
    int interpolation_attempts = 0;
    int contrast_failures = 0;
    int edge_failures = 0;
    int points_checked = 0;
    int local_extrema_found = 0;
    int zero_value_extrema = 0;

    // Debug: Check values in different regions
    ENTO_DEBUG("DoG values in different regions:");
    ENTO_DEBUG("  Center (16,16): prev=%f, curr=%f, next=%f", 
               triplet.prev_image()(16,16), triplet.curr_image()(16,16), triplet.next_image()(16,16));
    ENTO_DEBUG("  Border (1,1): prev=%f, curr=%f, next=%f", 
               triplet.prev_image()(1,1), triplet.curr_image()(1,1), triplet.next_image()(1,1));
    ENTO_DEBUG("  Border (30,30): prev=%f, curr=%f, next=%f", 
               triplet.prev_image()(30,30), triplet.curr_image()(30,30), triplet.next_image()(30,30));

    for (int y = 1; y < H - 1; ++y)
    {
      for (int x = 1; x < W - 1; ++x)
      {
        points_checked++;
        float center_val = triplet.curr_image()(y,x);
        
        bool is_max = true;
        bool is_min = true;

        // Check all 26 neighbors (3x3x3 - 1 center)
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
              if (!is_max && !is_min) 
                break; // Early exit if neither max nor min
              if (dz == 0 && dy == 0 && dx == 0)
                continue; // Skip center point

              float val = (*img)(y + dy, x + dx);

              if (center_val < val)
                is_max = false;
              if (center_val > val)
                is_min = false;
              }
            if (!is_max && !is_min) break;
          }
          if (!is_max && !is_min) break;
        }

        if (is_max || is_min) {
          // Apply initial threshold check like SIFT++
          if (std::abs(center_val) < InitialThreshold_) {
            continue; // Skip low-contrast candidates
          }
          
          local_extrema_found++;
          if (center_val == 0.0f) zero_value_extrema++;
          
          // Debug: Print DoG values around suspected region for 32x32 images
          if (Width_ == 32 && Height_ == 32 && x >= 5 && x <= 9 && y >= 5 && y <= 9) {
            printf("  DoG extrema @ (%d,%d) = %.6f (is_max=%d, is_min=%d)\n", 
                   x, y, center_val, is_max, is_min);
          }
          
          if (local_extrema_found <= 5) { // Only show first few
            ENTO_DEBUG("Local extrema found @ (%d, %d) = %f (is_max=%d, is_min=%d)", 
                       x, y, center_val, is_max, is_min);
          }

        }
        
        if ((is_max || is_min) && !feats_.full())
        {
          extrema_candidates++;

          interpolation_attempts++;
          SIFTInterpolationResult interp_result;
          if (!interpolate_extremum(triplet, x, y, scale_idx, interp_result))
          {
            continue;
          }

          // Use the interpolated response value from subpixel refinement
          float raw_dog_value = center_val;
          float interpolated_response = interp_result.interpolated_value;
          ENTO_DEBUG("Keypoint accepted @ (%f, %f, %f): raw_DoG=%f, interpolated=%f", 
                     x+interp_result.dx, y+interp_result.dy, scale_idx+interp_result.dscale, 
                     raw_dog_value, interpolated_response);
          feats_.add_keypoint(KeypointT{
            static_cast<KeypointCoordT_>(x),
            static_cast<KeypointCoordT_>(y),
            static_cast<ScaleT_>(scale_idx + interp_result.dscale),  // Apply scale interpolation
            0,                                         // octave
            static_cast<OrientationT_>(0.0f),         // orientation (will be set later)
            interpolated_response,                    // response (interpolated DoG value)
            static_cast<DescriptorT_>(0.0f)           // descriptor fill value
          });
        }
      }
    }
    
    ENTO_DEBUG("Extrema detection summary: points_checked=%d, local_extrema_found=%d, zero_value_extrema=%d, candidates=%d", 
               points_checked, local_extrema_found, zero_value_extrema, extrema_candidates);
  }

  int compute_keypoint_orientations(const KeypointT_& kp,
                                 const DoGImageT_& grad_mag, 
                                 const DoGImageT_& grad_ori,
                                 float orientations[4])
  {
    // Create 36-bin orientation histogram
    float hist[36] = {0};
    
    // Sample gradients in circular window around keypoint
    float sigma = get_sigma_for_keypoint(kp);
    float sigmaw = 1.5f * sigma;  // winFactor = 1.5
    int W = (int) ceilf(3.0f * sigmaw);
    
    for (int dy = -W; dy <= W; dy++) {
      for (int dx = -W; dx <= W; dx++) {
        int x = kp.x + dx, y = kp.y + dy;
        float dist_sq = dx*dx + dy*dy;
        
        if (dist_sq <= W*W + 0.5f && x >= 0 && x < Width_ && y >= 0 && y < Height_) {
          float weight = expf(-dist_sq / (2 * sigmaw * sigmaw));
          float magnitude = grad_mag(y, x);
          float orientation = grad_ori(y, x);
          
          int bin = (int)(36 * orientation / (2*M_PI)) % 36;
          hist[bin] += magnitude * weight;
        }
      }
    }
    
    // Smooth histogram + find peaks > 80% of max
    smooth_histogram(hist, 36);
    return find_orientation_peaks(hist, orientations);
  }

  void compute_keypoint_descriptor(const KeypointT_& kp, 
                                float keypoint_orientation,
                                const DoGImageT_& grad_mag,
                                const DoGImageT_& grad_ori,
                                float descriptor[128])
  {
    
    // 4×4 spatial bins × 8 orientation bins = 128D descriptor
    constexpr int NBP = 4;  // spatial bins per dimension  
    constexpr int NBO = 8;  // orientation bins
    
    float sigma = get_sigma_for_keypoint(kp);
    const float magnif = 3.0f;
    const float SBP = magnif * sigma;
    int W = (int) floorf(sqrtf(2.0f) * SBP * (NBP + 1) / 2.0f + 0.5f);
    
    std::fill(descriptor, descriptor + 128, 0.0f);
    
    // Sample gradients in rotated window
    for (int dy = -W; dy <= W; dy++) {
      for (int dx = -W; dx <= W; dx++) {
        
        // Rotate sampling coordinates by keypoint orientation
        float rotated_x = dx * cosf(-keypoint_orientation) - dy * sinf(-keypoint_orientation);
        float rotated_y = dx * sinf(-keypoint_orientation) + dy * cosf(-keypoint_orientation);
        
        // Determine which spatial bin this sample belongs to  
        float nx = rotated_x / SBP;
        float ny = rotated_y / SBP;
        float bin_x = nx + NBP/2.0f;
        float bin_y = ny + NBP/2.0f;
        
        if (bin_x >= 0 && bin_x < NBP && bin_y >= 0 && bin_y < NBP) {
          
          int img_x = kp.x + dx, img_y = kp.y + dy;
          if (img_x >= 0 && img_x < Width_ && img_y >= 0 && img_y < Height_) {
            
            float magnitude = grad_mag(img_y, img_x);
            float orientation = grad_ori(img_y, img_x) - keypoint_orientation;  // Make relative
            
            // Trilinear interpolation into 4×4×8 histogram
            trilinear_interpolate(descriptor, bin_x, bin_y, orientation, magnitude);
          }
        }
      }
    }
    
    // Normalize to unit length + clamp + re-normalize
    normalize_descriptor(descriptor, 128);
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
        {
          ENTO_DEBUG("Failed contrast threshold: |%f| < %f", 
                     result.interpolated_value, ContrastThreshold_);
          result.success = false;
        }

        if (result.success && !passes_edge_response_check(d))
        {
          ENTO_DEBUG("Failed edge response check");
          result.success = false;
        }

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
        ENTO_DEBUG("Position out of bounds: (%d, %d)", x, y);
        result.success = false;
        return false;
      }
    }
    
    ENTO_DEBUG("Interpolation failed: exceeded max iterations");
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
                                  float edge_threshold = 3.0f)
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

private:
  // Missing function implementations - temporary stubs
  float get_sigma_for_keypoint(const KeypointT_& kp) {
    // Convert scale index to sigma using SIFT++ formula
    const float base_sigma = 1.6f;
    const float k = powf(2.0f, 1.0f/3.0f);  // 2^(1/3)
    return base_sigma * powf(k, kp.scale);
  }

  void smooth_histogram(float hist[], int size) {
    // SIFT++ 6-iteration smoothing
    for (int iter = 0; iter < 6; iter++) {
      float prev = hist[size-1];
      for (int i = 0; i < size; i++) {
        float newh = (prev + hist[i] + hist[(i+1) % size]) / 3.0f;
        prev = hist[i];
        hist[i] = newh;
      }
    }
  }

  int find_orientation_peaks(float hist[], float orientations[4]) {
    // Find maximum
    float maxh = *std::max_element(hist, hist + 36);
    
    // Find peaks > 80% of max with quadratic interpolation
    int nangles = 0;
    for(int i = 0; i < 36 && nangles < 4; ++i) {
      float h0 = hist[i];
      float hm = hist[(i-1+36) % 36];
      float hp = hist[(i+1+36) % 36];
      
      if(h0 > 0.8f*maxh && h0 > hm && h0 > hp) {
        float di = -0.5f * (hp-hm) / (hp+hm-2*h0);
        float th = 2*M_PI*(i+di+0.5f)/36;
        orientations[nangles++] = th;
      }
    }
    return nangles;
  }

  void trilinear_interpolate(float descriptor[], float bin_x, float bin_y, float orientation, float magnitude) {
    const int NBP = 4;
    const int NBO = 8;
    
    // Normalize orientation to [0, 2π) and convert to bin space
    while(orientation < 0) orientation += 2*M_PI;
    while(orientation >= 2*M_PI) orientation -= 2*M_PI;
    float nt = NBO * orientation / (2*M_PI);
    
    // Get the "lower-left" bins for trilinear interpolation
    int binx = (int) floorf(bin_x - 0.5f);
    int biny = (int) floorf(bin_y - 0.5f);  
    int bint = (int) floorf(nt);
    
    // Compute interpolation weights
    float rbinx = bin_x - (binx + 0.5f);
    float rbiny = bin_y - (biny + 0.5f);
    float rbint = nt - bint;
    
    // Distribute to 8 adjacent bins
    for(int dbinx = 0; dbinx < 2; ++dbinx) {
      for(int dbiny = 0; dbiny < 2; ++dbiny) {
        for(int dbint = 0; dbint < 2; ++dbint) {
          
          int fx = binx + dbinx;
          int fy = biny + dbiny;
          int ft = (bint + dbint) % NBO;
          
          // Check bounds (centered bins from -NBP/2 to +NBP/2)
          if(fx >= -(NBP/2) && fx < (NBP/2) && fy >= -(NBP/2) && fy < (NBP/2)) {
            
            float weight = magnitude
              * fabsf(1 - dbinx - rbinx)
              * fabsf(1 - dbiny - rbiny) 
              * fabsf(1 - dbint - rbint);
            
            // Convert to descriptor array index (Lowe's convention)
            int idx = ((fy + NBP/2) * NBP + (fx + NBP/2)) * NBO + ft;
            if(idx >= 0 && idx < 128) {
              descriptor[idx] += weight;
            }
          }
        }
      }
    }
  }

  void normalize_descriptor(float descriptor[], int size) {
    // First normalization
    float norm = 0.0f;
    for(int i = 0; i < size; i++) norm += descriptor[i] * descriptor[i];
    norm = sqrtf(norm);
    if (norm > 0.0f) {
      for(int i = 0; i < size; i++) descriptor[i] /= norm;
    }
    
    // Clamp values > 0.2
    for(int i = 0; i < size; i++) {
      if(descriptor[i] > 0.2f) descriptor[i] = 0.2f;
    }
    
    // Second normalization
    norm = 0.0f;
    for(int i = 0; i < size; i++) norm += descriptor[i] * descriptor[i];
    norm = sqrtf(norm);
    if (norm > 0.0f) {
      for(int i = 0; i < size; i++) descriptor[i] /= norm;
    }
  }
};


} // namespace EntoFeature2D

#endif
