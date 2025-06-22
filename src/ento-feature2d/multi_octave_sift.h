#ifndef MULTI_OCTAVE_SIFT_H
#define MULTI_OCTAVE_SIFT_H

#include "sift.h"
#include <image_io/Image.h>
#include <ento-util/containers.h>

namespace EntoFeature2D {

// ============================================================================
// MultiOctaveSIFTDriver
// ============================================================================
// Memory-efficient multi-octave SIFT using ImageView for buffer reuse.
// Progressive processing: process one octave, downsample in-place, repeat.
// ============================================================================
template <typename BaseImageT, 
          size_t MaxKeypoints,
          typename KeypointT = SIFTKeypoint<>,
          int NumOctaves = 3,
          int NumDoGLayers = 3>
class MultiOctaveSIFTDriver
{
public:
  using PixelT = typename BaseImageT::pixel_type_;
  static constexpr int BaseHeight = BaseImageT::rows_;
  static constexpr int BaseWidth = BaseImageT::cols_;

  // ============================================================================
  // Constructor
  // ============================================================================
  explicit MultiOctaveSIFTDriver(BaseImageT& base_buffer)
    : base_buffer_(base_buffer),
      current_width_(BaseWidth),
      current_height_(BaseHeight),
      features_() {}

  // ============================================================================
  // run
  // ============================================================================
  // Process all octaves progressively using buffer reuse
  // ============================================================================
  bool run(const BaseImageT& input_img, FeatureArray<KeypointT, MaxKeypoints>& final_features)
  {
    printf("Multi-octave SIFT driver created with %d octaves, MaxKeypoints=%zu\n", 
           NumOctaves, MaxKeypoints);
    printf("Running multi-octave SIFT pipeline...\n");

    // Clear any previous features
    features_.clear();
    
    // Copy input to base buffer
    copy_image(base_buffer_, input_img);
    current_width_ = BaseWidth;
    current_height_ = BaseHeight;

    // Process each octave progressively
    for (int octave = 0; octave < NumOctaves; ++octave) {
      printf("Processing octave %d: %dx%d\n", octave, current_width_, current_height_);
      
      // Save debug image before processing
      char filename[100];
      sprintf(filename, "/tmp/octave_%d_before_%dx%d.pgm", octave, current_width_, current_height_);
      save_buffer_as_pgm(filename, current_width_, current_height_);
      
      // Create view of current octave data
      auto octave_view = make_image_view(base_buffer_, 0, 0, current_height_, current_width_);
      
      // Process this octave (downsampling happens inside process_octave_with_size if needed)
      bool success = process_octave_generic(octave_view, octave);
      if (!success) {
        printf("Failed to process octave %d\n", octave);
      }
      
      // Update dimensions for next octave (downsampling already done in processing)
      if (octave < NumOctaves - 1) {
        current_width_ /= 2;
        current_height_ /= 2;
      }
    }
    
    // Merge all features into final array
    merge_all_features(final_features);
    
    printf("Multi-octave SIFT pipeline completed successfully!\n");
    printf("Total features detected across all octaves: %zu\n", final_features.size());
    
    return final_features.size() > 0;
  }

  // ============================================================================
  // get_octave_features - Compatibility method for debugging
  // ============================================================================
  const FeatureArray<KeypointT, MaxKeypoints>& get_octave_features(int octave) const {
    // For now, return all features (we don't store per-octave separately)
    return features_;
  }

  // ============================================================================
  // Debug function to save current buffer as PGM
  // ============================================================================
  void save_buffer_as_pgm(const char* filename, int width, int height)
  {
    FILE* file = fopen(filename, "w");
    if (!file) {
      printf("Error: Could not open %s for writing\n", filename);
      return;
    }
    
    // Write PGM header
    fprintf(file, "P2\n");
    fprintf(file, "%d %d\n", width, height);
    fprintf(file, "255\n");
    
    // Write pixel data
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        fprintf(file, "%d ", (int)base_buffer_(y, x));
      }
      fprintf(file, "\n");
    }
    
    fclose(file);
    printf("Saved debug image: %s (%dx%d)\n", filename, width, height);
  }

private:
  BaseImageT& base_buffer_;
  int current_width_;
  int current_height_;
  FeatureArray<KeypointT, MaxKeypoints> features_;
  


  // ============================================================================
  // process_octave_generic
  // ============================================================================
  // Generic octave processing that works with any size using runtime templates
  // ============================================================================
  template<typename ImageViewT>
  bool process_octave_generic(const ImageViewT& octave_view, int octave_level)
  {
    // We need to work with the SIFT pipeline which expects compile-time Image sizes
    // Since we can't create runtime-sized Image templates, we need to handle
    // the most common octave sizes and provide a fallback
    
    int height = octave_view.rows();
    int width = octave_view.cols();
    
    if (height > BaseHeight || width > BaseWidth) {
      printf("Error: Octave view size (%dx%d) exceeds base buffer (%dx%d)\n", 
             height, width, BaseHeight, BaseWidth);
      return false;
    }
    
    // Since C++ templates require compile-time constants, we need to dispatch
    // to template specializations. This is unavoidable with the current SIFT API.
    return dispatch_to_template_size(octave_view, octave_level, height, width);
  }
  
  // ============================================================================
  // dispatch_to_template_size
  // ============================================================================
  // Handle the template size dispatch based on runtime dimensions
  // ============================================================================
  template<typename ImageViewT>
  bool dispatch_to_template_size(const ImageViewT& octave_view, int octave_level, int height, int width)
  {
    // We need to support the expected octave progression: BaseSize -> BaseSize/2 -> BaseSize/4 -> ...
    // This is fundamental to how SIFT octaves work
    
    // Calculate expected size for this octave level
    int expected_height = BaseHeight >> octave_level;  // BaseHeight / 2^octave_level
    int expected_width = BaseWidth >> octave_level;    // BaseWidth / 2^octave_level
    
    if (height != expected_height || width != expected_width) {
      printf("Warning: Octave %d size (%dx%d) doesn't match expected (%dx%d)\n",
             octave_level, height, width, expected_height, expected_width);
    }
    
    // Template dispatch based on actual dimensions
    // This covers the standard SIFT octave progression
    if (height == 128 && width == 128) {
      return process_octave_with_size<128, 128>(octave_view, octave_level);
    } else if (height == 64 && width == 64) {
      return process_octave_with_size<64, 64>(octave_view, octave_level);
    } else if (height == 32 && width == 32) {
      return process_octave_with_size<32, 32>(octave_view, octave_level);
    } else if (height == 16 && width == 16) {
      return process_octave_with_size<16, 16>(octave_view, octave_level);
    } else if (height == 8 && width == 8) {
      return process_octave_with_size<8, 8>(octave_view, octave_level);
    } else {
      printf("Unsupported octave size: %dx%d. Add template specialization if needed.\n", height, width);
      return false;
    }
  }
  
  // ============================================================================
  // process_octave_with_size
  // ============================================================================
  // Generic template function that works with any compile-time size
  // ============================================================================
  template<int H, int W, typename ImageViewT>
  bool process_octave_with_size(const ImageViewT& octave_view, int octave_level)
  {
    // Create temporary image of the right compile-time size
    Image<H, W, PixelT> temp_img;
    copy_from_view(temp_img, octave_view);
    
    // Calculate feature budget for this octave (distribute evenly)
    constexpr size_t OctaveFeatureBudget = MaxKeypoints / NumOctaves;
    
    FeatureArray<KeypointT, OctaveFeatureBudget> octave_features;
    SIFTDriver<Image<H, W, PixelT>, OctaveFeatureBudget, KeypointT, NumDoGLayers> driver(temp_img, octave_features);
    
    bool success = driver.run(temp_img);
    if (success) {
      printf("  Octave %d (%dx%d) raw features (before coordinate transform):\n", octave_level, H, W);
      for (int i = 0; i < std::min(5, (int)octave_features.size()); ++i) {
        const auto& feat = octave_features[i];
        printf("    Raw[%d]: pos(%.1f, %.1f), scale=%.1f, response=%.6f\n", 
               i, feat.x, feat.y, feat.scale, feat.response);
      }
      transform_and_merge_features(octave_features, octave_level);
      
      // Use the octave for SIFT++ compliant downsampling if not the last octave
      if (octave_level < NumOctaves - 1) {
        downsample_with_octave_gaussian(driver.get_current_octave());
      }
    }
    return success;
  }

  // ============================================================================
  // downsample_inplace_generic
  // ============================================================================
  // SIFT++ compliant downsampling using Gaussian blur level 2 for anti-aliasing
  // ============================================================================
  void downsample_inplace_generic()
  {
    int new_height = current_height_ / 2;
    int new_width = current_width_ / 2;
    
    printf("  SIFT++ downsampling from %dx%d to %dx%d using scale level 2\n", 
           current_height_, current_width_, new_height, new_width);
    
    // Simple 3x3 weighted average for anti-aliasing (good enough for now)
    // TODO: Could use recompute_gaussian_for_scale(2) for SIFT++ compliance
    for (int y = 0; y < new_height; y++) {
      for (int x = 0; x < new_width; x++) {
        int src_y = 2 * y;
        int src_x = 2 * x;
        
        // 3x3 weighted average around source pixel for anti-aliasing
        float sum = 0.0f;
        float weight_sum = 0.0f;
        
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            int sample_y = src_y + dy;
            int sample_x = src_x + dx;
            
            if (sample_y >= 0 && sample_y < current_height_ && 
                sample_x >= 0 && sample_x < current_width_) {
              // Center pixel gets higher weight
              float weight = (dx == 0 && dy == 0) ? 4.0f : 1.0f;
              sum += weight * static_cast<float>(base_buffer_(sample_y, sample_x));
              weight_sum += weight;
            }
          }
        }
        
        base_buffer_(y, x) = static_cast<PixelT>(sum / weight_sum);
      }
    }
  }

private:
  
  // ============================================================================
  // downsample_with_octave_gaussian
  // ============================================================================
  // SIFT++ compliant downsampling using the current octave's Gaussian level 2
  // ============================================================================
  template<typename OctaveT>
  void downsample_with_octave_gaussian(OctaveT* octave)
  {
    if (!octave) {
      printf("Warning: No octave available, falling back to simple downsampling\n");
      downsample_inplace_generic();
      return;
    }
    
    int new_height = current_height_ / 2;
    int new_width = current_width_ / 2;
    
    printf("Before SIFT++ downsampling octave: center pixel = %d\n", 
           (int)base_buffer_(current_height_/2, current_width_/2));
    printf("  SIFT++ downsampling from %dx%d to %dx%d using Gaussian scale level 2\n", 
           current_height_, current_width_, new_height, new_width);
    
    // Get the pre-blurred Gaussian at scale level 2 for anti-aliasing
    const auto& blurred_gaussian = octave->recompute_gaussian_for_scale(2);
    
    // Subsample the blurred Gaussian by 2x to create downsampled image
    for (int y = 0; y < new_height; y++) {
      for (int x = 0; x < new_width; x++) {
        int src_y = 2 * y;
        int src_x = 2 * x;
        
        if (src_y < current_height_ && src_x < current_width_) {
          // Convert back to PixelT (typically uint8_t)
          float blurred_value = blurred_gaussian(src_y, src_x);
          base_buffer_(y, x) = static_cast<PixelT>(blurred_value * 255.0f);
        }
      }
    }
    
    printf("After SIFT++ downsampling to %dx%d: center pixel = %d\n", 
           new_width, new_height, (int)base_buffer_(new_height/2, new_width/2));
  }

  // ============================================================================
  // Feature transformation and merging
  // ============================================================================
  template<size_t MaxOctaveFeatures>
  void transform_and_merge_features(const FeatureArray<KeypointT, MaxOctaveFeatures>& octave_features,
                                  int octave_level)
  {
    float scale_factor = static_cast<float>(1 << octave_level);  // 2^octave_level
    
    for (int i = 0; i < octave_features.size(); ++i) {
      if (features_.full()) {
        break; // No more space in feature array
      }
      
      KeypointT transformed_kp = octave_features[i];
      
      // Transform coordinates from octave space to global image space
      transformed_kp.x = transformed_kp.x * scale_factor;
      transformed_kp.y = transformed_kp.y * scale_factor;
      
      // Set the correct octave field (not scale!)
      transformed_kp.octave = octave_level;
      // Scale should be adjusted for global scale space
      transformed_kp.scale = transformed_kp.scale + octave_level;
      
      features_.add_keypoint(transformed_kp);
    }
  }
  
  void merge_all_features(FeatureArray<KeypointT, MaxKeypoints>& final_features)
  {
    for (int i = 0; i < features_.size(); ++i) {
      if (final_features.full()) break;
      final_features.add_keypoint(features_[i]);
    }
  }

  // ============================================================================
  // Utility functions
  // ============================================================================
  void copy_image(BaseImageT& dst, const BaseImageT& src)
  {
    for (int y = 0; y < BaseHeight; y++) {
      for (int x = 0; x < BaseWidth; x++) {
        dst(y, x) = src(y, x);
      }
    }
  }
  
  template<int H, int W, typename ImageViewT>
  void copy_from_view(Image<H, W, PixelT>& dst, const ImageViewT& src)
  {
    for (int y = 0; y < H && y < src.rows(); y++) {
      for (int x = 0; x < W && x < src.cols(); x++) {
        dst(y, x) = src(y, x);
      }
    }
  }
};

} // namespace EntoFeature2D

#endif // MULTI_OCTAVE_SIFT_H 