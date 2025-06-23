#ifndef MULTI_OCTAVE_SIFT_H
#define MULTI_OCTAVE_SIFT_H

#include "sift.h"
#include <image_io/Image.h>
#include <ento-util/containers.h>

namespace EntoFeature2D {

// ============================================================================
// MultiOctaveSIFTDriver
// ============================================================================
// Progressive multi-octave SIFT implementation using ImageView and in-place 
// downsampling for memory efficiency.
//
// Strategy:
// 1. Process octave 0 on full resolution base image
// 2. Downsample base image in-place (destroying original data)
// 3. Process octave 1 using ImageView of downsampled region
// 4. Repeat for additional octaves
//
// Template Parameters:
// - BaseImageT: The base image type (e.g., Image<128,128,uint8_t>)
// - MaxKeypoints: Maximum total keypoints across all octaves
// - KeypointT: Keypoint type (default: SIFTKeypoint)
// - NumOctaves: Number of octave levels to process
// - NumDoGLayers: Number of DoG layers per octave
// ============================================================================

template <typename BaseImageT, 
          int MaxKeypoints, 
          typename KeypointT = SIFTKeypoint<>,
          int NumOctaves = 3,
          int NumDoGLayers = 3>
class MultiOctaveSIFTDriver
{
public:
  using PixelT = typename BaseImageT::pixel_type_;
  using KeypointT_ = KeypointT;
  using DoGPixelT = float;
  using DoGImageViewT = ImageView<DoGPixelT>;
  
  static constexpr int BaseHeight = BaseImageT::rows_;
  static constexpr int BaseWidth = BaseImageT::cols_;

private:
  BaseImageT& base_buffer_;
  FeatureArray<KeypointT, MaxKeypoints> features_;
  
  // Current octave processing state
  int current_octave_;
  int current_width_;
  int current_height_;

public:
  // Constructor - takes reference to base image buffer
  MultiOctaveSIFTDriver(BaseImageT& base_buffer) 
    : base_buffer_(base_buffer), current_octave_(0) 
  {
    current_width_ = BaseWidth;
    current_height_ = BaseHeight;
  }

  // Main processing function
  bool run(const BaseImageT& input_img, FeatureArray<KeypointT, MaxKeypoints>& final_features)
  {
    final_features.clear();
    features_.clear();
    
    // Copy input to base buffer for in-place processing
    copy_image(base_buffer_, input_img);
    
    current_width_ = BaseWidth;
    current_height_ = BaseHeight;
    
    // Process each octave progressively
    for (int octave = 0; octave < NumOctaves; ++octave) {
      if (current_width_ < 8 || current_height_ < 8) {
        break; // Too small to process meaningfully
      }
      
      current_octave_ = octave;
      
      printf("Processing octave %d: %dx%d\n", octave, current_width_, current_height_);
      
      // Create ImageView for current octave size
      auto octave_view = make_image_view(base_buffer_, 0, 0, current_height_, current_width_);
      
      // Process this octave
      bool success = process_octave(octave_view, octave);
      if (!success) {
        printf("Failed to process octave %d\n", octave);
        // Continue to next octave even if this one fails
      }
      
      // Downsample for next octave (in-place, destroys current data)
      if (octave < NumOctaves - 1) {
        downsample_inplace();
        current_width_ /= 2;
        current_height_ /= 2;
      }
    }
    
    // Merge all features into final array
    merge_all_features(final_features);
    
    return final_features.size() > 0;
  }
  
  // Accessor for debugging
  const FeatureArray<KeypointT, MaxKeypoints>& get_features() const {
    return features_;
  }

private:
  // ============================================================================
  // process_octave
  // ============================================================================
  // Process a single octave using existing SIFT pipeline with ImageView
  // ============================================================================
  template<typename ImageViewT>
  bool process_octave(const ImageViewT& octave_view, int octave_level)
  {
    // We need to adapt the existing SIFT pipeline to work with ImageView
    // For now, let's create a temporary Image and copy data
    // TODO: This is inefficient, but gets us started. Later we'll modify
    // the SIFT pipeline to work directly with ImageView
    
    if (octave_view.rows() > BaseHeight || octave_view.cols() > BaseWidth) {
      printf("Error: Octave view size (%dx%d) exceeds base buffer (%dx%d)\n", 
             octave_view.rows(), octave_view.cols(), BaseHeight, BaseWidth);
      return false;
    }
    
    // Create a temporary single-octave driver for this size
    // This is a temporary approach - we'll optimize later
    if (octave_view.rows() == 128 && octave_view.cols() == 128) {
      return process_octave_128x128(octave_view, octave_level);
    } else if (octave_view.rows() == 64 && octave_view.cols() == 64) {
      return process_octave_64x64(octave_view, octave_level);
    } else if (octave_view.rows() == 32 && octave_view.cols() == 32) {
      return process_octave_32x32(octave_view, octave_level);
    } else if (octave_view.rows() == 16 && octave_view.cols() == 16) {
      return process_octave_16x16(octave_view, octave_level);
    } else {
      printf("Unsupported octave size: %dx%d\n", octave_view.rows(), octave_view.cols());
      return false;
    }
  }
  
  // ============================================================================
  // Octave processing for specific sizes (temporary solution)
  // ============================================================================
  template<typename ImageViewT>
  bool process_octave_128x128(const ImageViewT& octave_view, int octave_level)
  {
    Image<128, 128, PixelT> temp_img;
    copy_from_view(temp_img, octave_view);
    
    FeatureArray<KeypointT, MaxKeypoints/4> octave_features;
    SIFTDriver<Image<128, 128, PixelT>, MaxKeypoints/4, KeypointT, NumDoGLayers> driver(temp_img, octave_features);
    
    bool success = driver.run(temp_img);
    if (success) {
      transform_and_merge_features(octave_features, octave_level);
    }
    return success;
  }

  template<typename ImageViewT>
  bool process_octave_64x64(const ImageViewT& octave_view, int octave_level)
  {
    Image<64, 64, PixelT> temp_img;
    copy_from_view(temp_img, octave_view);
    
    FeatureArray<KeypointT, MaxKeypoints/4> octave_features;
    SIFTDriver<Image<64, 64, PixelT>, MaxKeypoints/4, KeypointT, NumDoGLayers> driver(temp_img, octave_features);
    
    bool success = driver.run(temp_img);
    if (success) {
      transform_and_merge_features(octave_features, octave_level);
    }
    return success;
  }
  
  template<typename ImageViewT>
  bool process_octave_32x32(const ImageViewT& octave_view, int octave_level)
  {
    Image<32, 32, PixelT> temp_img;
    copy_from_view(temp_img, octave_view);
    
    FeatureArray<KeypointT, MaxKeypoints/4> octave_features;
    SIFTDriver<Image<32, 32, PixelT>, MaxKeypoints/4, KeypointT, NumDoGLayers> driver(temp_img, octave_features);
    
    bool success = driver.run(temp_img);
    if (success) {
      transform_and_merge_features(octave_features, octave_level);
    }
    return success;
  }
  
  template<typename ImageViewT>
  bool process_octave_16x16(const ImageViewT& octave_view, int octave_level)
  {
    Image<16, 16, PixelT> temp_img;
    copy_from_view(temp_img, octave_view);
    
    FeatureArray<KeypointT, MaxKeypoints/4> octave_features;
    SIFTDriver<Image<16, 16, PixelT>, MaxKeypoints/4, KeypointT, NumDoGLayers> driver(temp_img, octave_features);
    
    bool success = driver.run(temp_img);
    if (success) {
      transform_and_merge_features(octave_features, octave_level);
    }
    return success;
  }

  // ============================================================================
  // downsample_inplace
  // ============================================================================
  // Downsamples the base buffer in-place by factor of 2
  // ============================================================================
  void downsample_inplace()
  {
    int new_height = current_height_ / 2;
    int new_width = current_width_ / 2;
    
    for (int y = 0; y < new_height; y++) {
      for (int x = 0; x < new_width; x++) {
        // Simple downsampling: take every 2nd pixel
        base_buffer_(y, x) = base_buffer_(2*y, 2*x);
        
        // Alternative: Average 2x2 blocks (more accurate but slower)
        // float sum = static_cast<float>(base_buffer_(2*y, 2*x)) + 
        //             static_cast<float>(base_buffer_(2*y, 2*x+1)) +
        //             static_cast<float>(base_buffer_(2*y+1, 2*x)) + 
        //             static_cast<float>(base_buffer_(2*y+1, 2*x+1));
        // base_buffer_(y, x) = static_cast<PixelT>(sum / 4.0f);
      }
    }
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
      
      // Adjust scale level to account for octave level
      transformed_kp.scale = transformed_kp.scale + octave_level;
      
      // The descriptor remains unchanged (invariant to scale)
      // The orientation remains unchanged (rotation invariant)
      
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