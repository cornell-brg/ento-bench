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
    printf("Multi-octave SIFT driver V2 created with %d octaves, MaxKeypoints=%d\n", 
           NumOctaves_, MaxKeypoints);
    printf("Running multi-octave SIFT pipeline...\n");

    // Clear any previous features
    final_features.clear();
    
    // Copy input to base buffer
    copy_image(base_buffer_, input_image);
    
    int current_width = BaseWidth_;
    int current_height = BaseHeight_;

    // Process each octave progressively
    for (int octave = 0; octave < NumOctaves_; ++octave) {
      printf("Processing octave %d: %dx%d\n", octave, current_width, current_height);
      
      // Create ImageView for current octave
      auto octave_view = make_image_view(base_buffer_, 0, 0, current_height, current_width);
      
      // Process this octave (similar to single SIFTDriver)
      bool success = process_single_octave(octave_view, octave, final_features);
      if (!success) {
        printf("Failed to process octave %d\n", octave);
      }
      
      // Prepare for next octave (downsample in-place if not last octave)
      if (octave < NumOctaves_ - 1) {
        downsample_inplace(octave_view, current_width, current_height);
        current_width /= 2;
        current_height /= 2;
      }
    }
    
    printf("Multi-octave SIFT pipeline completed successfully!\n");
    printf("Total features detected across all octaves: %zu\n", final_features.size());
    
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

    // Use temporary feature array for this octave to avoid clearing final_features
    FeatureArray<KeypointT_, MaxKeypoints> octave_features;

    int scale_idx = 0;
    do {
      detect_extrema_in_triplet(octave.get_current_DoG_triplet(), scale_idx, octave_level, octave_features);
      scale_idx++;
    } while (octave.step());

    if (octave_features.size() < 1) {
      ENTO_DEBUG("No extrema found in octave %d", octave_level);
      return true; // Not a failure, just no features
    }

    constexpr int MaxPerScale = 50;
    struct ScaleGroup {
      KeypointT_ keypoints_[MaxPerScale];
      int count = 0;
    };

    ScaleGroup groups[NumDoGLayers_ + 2];

    // Group features by scale for processing
    size_t features_before = octave_features.size();
    for (size_t i = 0; i < octave_features.size(); ++i) {
              int scale_index = determine_scale_index(octave_features[i].scale);
        if (scale_index >= 0 && scale_index < NumDoGLayers_ + 2) {
          if (groups[scale_index].count < MaxPerScale) {
            groups[scale_index].keypoints_[groups[scale_index].count] = octave_features[i];
          groups[scale_index].count++;
        }
      }
    }

    // Clear the octave detection keypoints - we'll replace them with processed ones
    octave_features.clear();
    
    // 3. Orientation and Descriptor Computation
    for (int scale_idx = 0; scale_idx < NumDoGLayers_ + 2; ++scale_idx) {
      if (groups[scale_idx].count == 0)
        continue; // Skip empty scale groups

      const DoGImageViewT_& gaussian = octave.recompute_gaussian_for_scale(scale_idx);
      octave.compute_gradients_for_current_gaussian();

      for (int kp_idx = 0; kp_idx < groups[scale_idx].count; ++kp_idx) {
        const KeypointT_& kp = groups[scale_idx].keypoints_[kp_idx];
        
        // Transform coordinates to account for octave scaling
        KeypointT_ transformed_kp = transform_keypoint_for_octave(kp, octave_level);
        
        process_keypoint(transformed_kp, octave.get_gradient_magnitude(), octave.get_gradient_orientation(), octave_features);
      }
    }

    // Merge octave features into final features
    for (size_t i = 0; i < octave_features.size(); ++i) {
      if (final_features.full()) break;  // Stop if final array is full
      final_features.add_keypoint(octave_features[i]);
    }

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
    printf("DEBUG: Processing keypoint at (%.1f, %.1f) with response=%.6f\n", kp.x, kp.y, kp.response);
  
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

    printf("Octave %d scale %d: checked=%d, extrema=%d, candidates=%d, interpolated=%d, contrast_fail=%d\n",
           octave, scale_idx, points_checked, local_extrema_found, extrema_candidates, 
           interpolation_attempts, contrast_failures);
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
    // Placeholder - return single orientation
    orientations[0] = 0.0f;
    return 1;
  }

  void compute_keypoint_descriptor(const KeypointT_& kp, 
                                  float keypoint_orientation,
                                  const DoGImageViewT_& grad_mag,
                                  const DoGImageViewT_& grad_ori,
                                  float descriptor[128]) {
    // Placeholder - zero descriptor
    for (int i = 0; i < 128; ++i) {
      descriptor[i] = 0.0f;
    }
  }
};

} // namespace EntoFeature2D

#endif // MULTI_OCTAVE_SIFT_V2_H 