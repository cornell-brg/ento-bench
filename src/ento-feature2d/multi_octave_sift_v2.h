#ifndef MULTI_OCTAVE_SIFT_V2_H
#define MULTI_OCTAVE_SIFT_V2_H

#include "sift.h"
#include <image_io/Image.h>
#include <ento-util/containers.h>

namespace EntoFeature2D {

// ============================================================================
// MultiOctaveSIFTDriverV2
// ============================================================================
// Multi-octave SIFT driver that looks like SIFTDriver but processes octaves.
// Accepts Image at construction, uses internal ImageView for processing.
// Uses fixed-size DoG buffers for the base image size.
// ============================================================================
template <typename BaseImageT,
          int MaxKeypoints,
          typename KeypointT = SIFTKeypoint<>,
          int NumOctaves = 3,
          int NumDoGLayers = 3,
          typename DoGPixelT = float>
class MultiOctaveSIFTDriverV2
{
public:
  using BaseImageT_     = BaseImageT;
  using PixelT_         = typename BaseImageT::pixel_type_;
  using DoGPixelT_      = DoGPixelT;
  using KeypointT_      = KeypointT;
  using KeypointCoordT_ = typename KeypointT::CoordT_;
  using DescriptorT_    = typename KeypointT::DescriptorT_;
  using ScaleT_         = typename KeypointT::ScaleT_;
  using OrientationT_   = typename KeypointT::OrientationT_;
  
  // Base image dimensions (for fixed DoG buffer size)
  static constexpr int BaseHeight_ = BaseImageT::rows_;
  static constexpr int BaseWidth_  = BaseImageT::cols_;
  static constexpr int NumOctaves_ = NumOctaves;
  static constexpr int NumDoGLayers_ = NumDoGLayers;
  static constexpr int NumBlurLevels_ = NumDoGLayers_ + 1;
  static constexpr int MaxInterpIters_ = 5;
  static constexpr float ContrastThreshold_ = 0.04f / 3.0f / 2.0f;  // Match SIFT++: 0.04/S/2 = 0.00667
  static constexpr float InitialThreshold_ = 0.8f * ContrastThreshold_;  // 0.8 * 0.00667 = 0.00533

  // Use ImageView for DoG processing (no more fixed-size constraints)
  using DoGImageViewT_ = ImageView<DoGPixelT_>;
  using ImageViewT_ = ImageView<PixelT_>;
  
  // Fixed-size buffers for DoG processing (sized for base image)
  inline static Image<BaseHeight_, BaseWidth_, DoGPixelT_> gaussian_buffer_0_;
  inline static Image<BaseHeight_, BaseWidth_, DoGPixelT_> gaussian_buffer_1_;
  inline static Image<BaseHeight_, BaseWidth_, DoGPixelT_> dog_buffer_0_;
  inline static Image<BaseHeight_, BaseWidth_, DoGPixelT_> dog_buffer_1_;
  inline static Image<BaseHeight_, BaseWidth_, DoGPixelT_> dog_buffer_2_;
  inline static Image<BaseHeight_, BaseWidth_, DoGPixelT_> orientation_buffer_;

  // ============================================================================
  // Constructor
  // ============================================================================
  // Constructor - takes working buffer like original MultiOctaveSIFTDriver
  MultiOctaveSIFTDriverV2(BaseImageT& working_buffer) 
    : 
      //gaussian_buffer_0_(), gaussian_buffer_1_(),
      //dog_buffer_0_(), dog_buffer_1_(), dog_buffer_2_(), orientation_buffer_(),
      base_buffer_(working_buffer), current_octave_(nullptr)
  {
  }

  // ============================================================================
  // run - Main multi-octave SIFT processing
  // ============================================================================
  bool run(const BaseImageT& input_image, FeatureArray<KeypointT_, MaxKeypoints>& final_features)
  {
    // printf("Multi-octave SIFT driver V2 created with %d octaves, MaxKeypoints=%d\n", 
    //        NumOctaves_, MaxKeypoints);
    // printf("Running multi-octave SIFT pipeline...\n");

    // Clear any previous features
    final_features.clear();
    
    // Copy input to base buffer
    copy_image(base_buffer_, input_image);
    
    int current_width = BaseWidth_;
    int current_height = BaseHeight_;

    // Process each octave progressively
    for (int octave = 0; octave < NumOctaves_; ++octave) {
      // printf("Processing octave %d: %dx%d\n", octave, current_width, current_height);
      
      // Create ImageView for current octave
      auto octave_view = make_image_view(base_buffer_, 0, 0, current_height, current_width);
      
      // Process this octave (similar to single SIFTDriver)
      bool success = process_single_octave(octave_view, octave, final_features);
              if (!success) {
          // printf("Failed to process octave %d\n", octave);
        }
      
      // Prepare for next octave (downsample in-place if not last octave)
      if (octave < NumOctaves_ - 1) {
        downsample_inplace(octave_view, current_width, current_height);
        current_width /= 2;
        current_height /= 2;
      }
    }
    
    // printf("Multi-octave SIFT pipeline completed successfully!\n");
    // printf("Total features detected across all octaves: %zu\n", final_features.size());
    
    return final_features.size() > 0;
  }

  // ============================================================================
  // Get access to the current octave (valid only after run() is called)
  // ============================================================================
  SIFTDoGOctave<ImageViewT_, DoGPixelT_, NumDoGLayers_>* get_current_octave() { 
    return current_octave_; 
  }

private:
  BaseImageT& base_buffer_;  // Reference to working buffer for progressive processing
  SIFTDoGOctave<ImageViewT_, DoGPixelT_, NumDoGLayers_>* current_octave_;
  

  // ============================================================================
  // process_single_octave - Process one octave (like SIFTDriver::run)
  // ============================================================================
  bool process_single_octave(const ImageViewT_& octave_view, int octave_level, FeatureArray<KeypointT_, MaxKeypoints>& final_features)
  {
    // Create views into our fixed-size buffers for this octave size
    int octave_height = octave_view.rows();
    int octave_width = octave_view.cols();
    
    auto gauss_view_0 = make_image_view(gaussian_buffer_0_, 0, 0, octave_height, octave_width);
    auto gauss_view_1 = make_image_view(gaussian_buffer_1_, 0, 0, octave_height, octave_width);
    auto dog_view_0 = make_image_view(dog_buffer_0_, 0, 0, octave_height, octave_width);
    auto dog_view_1 = make_image_view(dog_buffer_1_, 0, 0, octave_height, octave_width);
    auto dog_view_2 = make_image_view(dog_buffer_2_, 0, 0, octave_height, octave_width);
    auto orientation_view = make_image_view(orientation_buffer_, 0, 0, octave_height, octave_width);
    
    // 1. Compute DoG pyramid using new constructor
    SIFTDoGOctave<ImageViewT_, DoGPixelT_, NumDoGLayers_> octave(
      const_cast<ImageViewT_&>(octave_view),
      gauss_view_0, gauss_view_1,
      dog_view_0, dog_view_1, dog_view_2,
      orientation_view
    );
    current_octave_ = &octave;
    octave.initialize();
    // printf("DEBUG: SIFT octave initialized successfully\n");

    // printf("DEBUG: Starting extrema detection directly into final_features\n");

    // Store current size to track how many features we add in this octave
    size_t features_before_octave = final_features.size();

    int scale_idx = 0;
    // printf("DEBUG: About to start extrema detection loop\n");
    do {
      // printf("DEBUG: About to detect extrema in scale %d\n", scale_idx);
      detect_extrema_in_triplet(octave.get_current_DoG_triplet(), scale_idx, octave_level, final_features);
      // printf("DEBUG: Finished detecting extrema in scale %d, about to step\n", scale_idx);
      scale_idx++;
    } while (octave.step());

    size_t octave_features_count = final_features.size() - features_before_octave;
    // printf("DEBUG: Octave %d finished extrema detection with %zu features\n", octave_level, octave_features_count);
    
    if (octave_features_count < 1) {
      ENTO_DEBUG("No extrema found in octave %d", octave_level);
      return true; // Not a failure, just no features
    }

    // printf("DEBUG: Starting grouping phase for octave %d\n", octave_level);
    
    constexpr int MaxPerScale = 10;  // Reduced from 50 to save stack space
    struct ScaleGroup {
      int indices_[MaxPerScale];  // Store indices relative to features_before_octave
      int count = 0;
    };

    ScaleGroup groups[NumDoGLayers_ + 2];

    // Group features by scale for processing (only process features from this octave)
    // printf("DEBUG: About to group %zu features\n", octave_features_count);
    
    for (size_t i = 0; i < octave_features_count; ++i) {
        size_t feature_index = features_before_octave + i;
        int scale_index = determine_scale_index(final_features[feature_index].scale);
        // printf("DEBUG: Feature %zu has scale=%.3f, scale_index=%d\n", i, final_features[feature_index].scale, scale_index);
        
        if (scale_index >= 0 && scale_index < NumDoGLayers_ + 2) {
            if (groups[scale_index].count < MaxPerScale) {
                groups[scale_index].indices_[groups[scale_index].count] = i;  // Store index relative to octave start
                groups[scale_index].count++;
            } else {
                // printf("DEBUG: Scale group %d is full, skipping feature %zu\n", scale_index, i);
            }
        } else {
            // printf("DEBUG: Skipping feature %zu with invalid scale_index=%d\n", i, scale_index);
        }
    }
    
    // printf("DEBUG: Grouped %zu features into scale groups\n", octave_features_count);
    
        // printf("DEBUG: Starting orientation and descriptor computation for octave %d\n", octave_level);
    
    // Create a temporary keypoint array to store just the keypoints we need for current scale
    KeypointT_ scale_keypoints[MaxPerScale];
    
    // Process each scale group and add results directly to final_features
    for (int scale_idx = 0; scale_idx < NumDoGLayers_ + 2; ++scale_idx) {
      // printf("DEBUG: Checking scale group %d (count=%d)\n", scale_idx, groups[scale_idx].count);
      if (groups[scale_idx].count == 0)
        continue; // Skip empty scale groups

      // printf("DEBUG: Processing scale %d with %d keypoints\n", scale_idx, groups[scale_idx].count);

      // Copy keypoints for this scale to avoid index dependencies
      for (int i = 0; i < groups[scale_idx].count; ++i) {
        int relative_index = groups[scale_idx].indices_[i];
        size_t absolute_index = features_before_octave + relative_index;
        scale_keypoints[i] = final_features[absolute_index];
      }

      // printf("DEBUG: About to recompute gaussian for scale %d\n", scale_idx);
      const DoGImageViewT_& gaussian = octave.recompute_gaussian_for_scale(scale_idx);
      // printf("DEBUG: Gaussian recomputed, about to compute gradients\n");
      octave.compute_gradients_for_current_gaussian();
      // printf("DEBUG: Gradients computed, processing keypoints\n");

      for (int kp_idx = 0; kp_idx < groups[scale_idx].count; ++kp_idx) {
        const KeypointT_& kp = scale_keypoints[kp_idx];
        
        // printf("DEBUG: Processing keypoint %d at (%.1f, %.1f)\n", kp_idx, kp.x, kp.y);
        
        // Transform coordinates to account for octave scaling
        KeypointT_ transformed_kp = transform_keypoint_for_octave(kp, octave_level);
        
        // Process keypoint and add results directly to final_features to avoid extra stack allocation
        process_keypoint_to_final(transformed_kp, octave.get_gradient_magnitude(), octave.get_gradient_orientation(), final_features);
      }
    }
    
    // printf("DEBUG: Finished processing all scales in octave %d\n", octave_level);
    // printf("DEBUG: Octave %d processing completed successfully\n", octave_level);
    current_octave_ = nullptr;
    return true;
  }

  // ============================================================================
  // Transform keypoint coordinates for multi-octave processing
  // ============================================================================
  KeypointT_ transform_keypoint_for_octave(const KeypointT_& kp, int octave_level) {
    float scale_factor = static_cast<float>(1 << octave_level);  // 2^octave_level
    KeypointT_ transformed_kp = kp;
    
    // Scale coordinates from octave space to original image space
    transformed_kp.x *= scale_factor;
    transformed_kp.y *= scale_factor;
    transformed_kp.scale *= scale_factor;  // Scale sigma accordingly
    transformed_kp.octave = 0;  // Reset octave to 0 for final output (all in base image space)
    
    return transformed_kp;
  }

  // ============================================================================
  // Methods copied from SIFTDriver (same functionality)
  // ============================================================================
  
  int determine_scale_index(ScaleT_ scale) {
    // Convert fractional scale to discrete index
    return static_cast<int>(std::round(scale));
  }

  void process_keypoint(const KeypointT_& kp, 
                       const DoGImageViewT_& gradient_magnitude,
                       const DoGImageViewT_& gradient_orientation,
                       FeatureArray<KeypointT_, MaxKeypoints>& feats) {
    // printf("DEBUG: Processing keypoint at (%.1f, %.1f) with response=%.6f\n", kp.x, kp.y, kp.response);
  
    // Step 1: Orientation Assignment
    float orientations[4];
    int num_orientations = compute_keypoint_orientations(kp, gradient_magnitude, gradient_orientation, orientations);
    
    // Step 2: Descriptor Computation for each orientation
    for (int ori_idx = 0; ori_idx < num_orientations; ori_idx++) {
      if (feats.full()) return;  // Check capacity using FeatureArray method
      
      float descriptor[128];
      compute_keypoint_descriptor(kp, orientations[ori_idx], gradient_magnitude, gradient_orientation, descriptor);
      
      // Step 3: Create final SIFT keypoint with orientation and descriptor
      KeypointT_ final_kp(kp.x, kp.y, kp.scale, 0, orientations[ori_idx], kp.response, 0.0f);
      final_kp.set_descriptor(descriptor);
      
      // Add to feature array
      feats.add_keypoint(final_kp);
    }
  }

  void process_keypoint_to_final(const KeypointT_& kp, 
                                const DoGImageViewT_& gradient_magnitude,
                                const DoGImageViewT_& gradient_orientation,
                                FeatureArray<KeypointT_, MaxKeypoints>& final_feats) {
    // printf("DEBUG: Processing keypoint at (%.1f, %.1f) with response=%.6f\n", kp.x, kp.y, kp.response);
  
    // Step 1: Orientation Assignment
    float orientations[4];
    int num_orientations = compute_keypoint_orientations(kp, gradient_magnitude, gradient_orientation, orientations);
    
    // Step 2: Descriptor Computation for each orientation
    for (int ori_idx = 0; ori_idx < num_orientations; ori_idx++) {
      if (final_feats.full()) return;  // Check capacity using FeatureArray method
      
      float descriptor[128];
      compute_keypoint_descriptor(kp, orientations[ori_idx], gradient_magnitude, gradient_orientation, descriptor);
      
      // Step 3: Create final SIFT keypoint with orientation and descriptor
      KeypointT_ final_kp(kp.x, kp.y, kp.scale, 0, orientations[ori_idx], kp.response, 0.0f);
      final_kp.set_descriptor(descriptor);
      
      // Add to feature array
      final_feats.add_keypoint(final_kp);
    }
  }

  void detect_extrema_in_triplet(const DoGTriplet<DoGImageViewT_>& triplet,
                                 int scale_idx, int octave, FeatureArray<KeypointT_, MaxKeypoints>& feats) {
    // Use runtime dimensions from the triplet
    const int H = triplet.curr_image().rows();
    const int W = triplet.curr_image().cols();


    int extrema_candidates = 0;
    int interpolation_attempts = 0;
    int contrast_failures = 0;
    int edge_failures = 0;
    int points_checked = 0;
    int local_extrema_found = 0;

    for (int y = 1; y < H - 1; ++y) {
      for (int x = 1; x < W - 1; ++x) {
        points_checked++;
        float center_val = triplet.curr_image()(y,x);
        
        bool is_max = true;
        bool is_min = true;

        // Check all 26 neighbors (3x3x3 - 1 center)
        for (int dz = -1; dz <= 1; ++dz) {
          const DoGImageViewT_* img =
            (dz == -1) ? &triplet.prev_image() :
            (dz == 0)  ? &triplet.curr_image() :
                         &triplet.next_image();

          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
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
          }
        }

        if (is_max || is_min) {
          local_extrema_found++;
          
          // Use the same scoring and interpolation as SIFTDriver
          if (std::abs(center_val) >= InitialThreshold_) {
            extrema_candidates++;
            
            SIFTInterpolationResult<> result;
            bool valid = interpolate_extremum(triplet, x, y, scale_idx, result);
            
            if (valid) {
              interpolation_attempts++;
              
              if (std::abs(result.interpolated_value) >= ContrastThreshold_) {
                // Add keypoint in current octave coordinates (no scaling here)
                KeypointT_ kp(
                  x + result.dx,
                  y + result.dy, 
                  scale_idx + result.dscale,
                  octave,  // Set octave level for later scaling
                  0.0f,
                  std::abs(result.interpolated_value),
                  0.0f
                );
                feats.add_keypoint(kp);
              } else {
                contrast_failures++;
              }
            }
          }
        }
      }
    }

    //printf("Octave %d scale %d: checked=%d, extrema=%d, candidates=%d, interpolated=%d, contrast_fail=%d\n",
    //       octave, scale_idx, points_checked, local_extrema_found, extrema_candidates, 
    //       interpolation_attempts, contrast_failures);
  }

  // ============================================================================
  // Utility methods
  // ============================================================================
  
  void copy_image(BaseImageT& dst, const BaseImageT& src) {
    for (int y = 0; y < BaseHeight_; ++y) {
      for (int x = 0; x < BaseWidth_; ++x) {
        dst(y, x) = src(y, x);
      }
    }
  }

  void downsample_inplace(const ImageViewT_& current_view, int width, int height) {
    // Simple 2x2 box filter downsampling
    int new_height = height / 2;
    int new_width = width / 2;
    
    for (int y = 0; y < new_height; ++y) {
      for (int x = 0; x < new_width; ++x) {
        int src_y = y * 2;
        int src_x = x * 2;
        
        // Average 2x2 block
        float sum = static_cast<float>(current_view(src_y, src_x)) +
                    static_cast<float>(current_view(src_y, src_x + 1)) +
                    static_cast<float>(current_view(src_y + 1, src_x)) +
                    static_cast<float>(current_view(src_y + 1, src_x + 1));
        
        base_buffer_(y, x) = static_cast<PixelT_>(sum / 4.0f);
      }
    }
  }

  // ============================================================================
  // Placeholder methods (copied from SIFTDriver interface)
  // ============================================================================
  
  bool interpolate_extremum(const DoGTriplet<DoGImageViewT_>& triplet,
                           KeypointCoordT_ x, KeypointCoordT_ y, ScaleT_ scale,
                           SIFTInterpolationResult<>& result) {
    // For now, simple placeholder - use center value as contrast
    result.dx = 0.0f;
    result.dy = 0.0f; 
    result.dscale = 0.0f;
    result.interpolated_value = triplet.curr_image()(y, x);
    return std::abs(result.interpolated_value) > 0.001f;
  }

  int compute_keypoint_orientations(const KeypointT_& kp,
                                   const DoGImageViewT_& grad_mag, 
                                   const DoGImageViewT_& grad_ori,
                                   float orientations[4]) {
    // Create 36-bin orientation histogram
    float hist[36] = {0};
    
    // Sample gradients in circular window around keypoint
    float sigma = get_sigma_for_keypoint(kp);
    float sigmaw = 1.5f * sigma;  // winFactor = 1.5
    int W = (int) ceilf(3.0f * sigmaw);
    
    // Get current octave dimensions
    int Width = grad_mag.cols();
    int Height = grad_mag.rows();
    
    for (int dy = -W; dy <= W; dy++) {
      for (int dx = -W; dx <= W; dx++) {
        int x = kp.x + dx, y = kp.y + dy;
        float dist_sq = dx*dx + dy*dy;
        
        if (dist_sq <= W*W + 0.5f && x >= 0 && x < Width && y >= 0 && y < Height) {
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
                                  const DoGImageViewT_& grad_mag,
                                  const DoGImageViewT_& grad_ori,
                                  float descriptor[128]) {
    
    // 4×4 spatial bins × 8 orientation bins = 128D descriptor
    constexpr int NBP = 4;  // spatial bins per dimension  
    constexpr int NBO = 8;  // orientation bins
    
    float sigma = get_sigma_for_keypoint(kp);
    const float magnif = 3.0f;
    const float SBP = magnif * sigma;
    int W = (int) floorf(sqrtf(2.0f) * SBP * (NBP + 1) / 2.0f + 0.5f);
    
    // Get current octave dimensions
    int Width = grad_mag.cols();
    int Height = grad_mag.rows();
    
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
          if (img_x >= 0 && img_x < Width && img_y >= 0 && img_y < Height) {
            
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

private:
  // Helper functions for orientation and descriptor computation
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

#endif // MULTI_OCTAVE_SIFT_V2_H 