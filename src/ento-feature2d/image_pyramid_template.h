#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

#include <ento-feature2d/raw_image.h>
#include <assert.h>
#include <stdio.h>

// Helper function to generate a tuple of RawImages for levels 0..NUM_LEVELS-1
template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t... Is>
constexpr auto make_pyramid_tuple(std::index_sequence<Is...>) {
    // For each index Is, create a RawImage with dimensions scaled by (1 << Is)
    return std::make_tuple(RawImage<IMG_WIDTH / (1 << Is), IMG_HEIGHT / (1 << Is)>()...);
}

// Helper function that takes a tuple and returns an array of pointers to each element.
template<typename Tuple, size_t... Is>
auto make_images_ptr_array(const Tuple& tup, std::index_sequence<Is...>) {
    // We use const void* as the common pointer type since the tuple elements are different types.
    return std::array<const void*, sizeof...(Is)>{ (static_cast<const void*>(&std::get<Is>(tup)))... };
}

template<size_t NUM_LEVELS, size_t IMG_HEIGHT, size_t IMG_WIDTH>
class ImagePyramid {
public:
  // Constructor that takes a const reference to the base RawImage
  explicit ImagePyramid(const RawImage<IMG_WIDTH, IMG_HEIGHT>& topImage)
        : images_(make_pyramid_tuple<IMG_WIDTH, IMG_HEIGHT>(std::make_index_sequence<NUM_LEVELS + 1>{}))
  {
      // Set the 0th image to the input top-level image.
      std::get<0>(images_) = topImage;
      // create array of pointers to each image in the tuple
      images_ptr_ = make_images_ptr_array(images_, std::make_index_sequence<NUM_LEVELS + 1>{});
  }

  // Getter: Access level i (0 <= i <= NUM_LEVELS)
  const void* get_level(uint16_t level) const {
    assert(level <= NUM_LEVELS);
    return images_ptr_[level]; // where images_ptr_ is a homogeneous array of pointers
  }

  // Create pyramids all at once
  // void create_pyramids();

private:
  // Declare images_ using decltype to match the tuple returned by make_pyramid_tuple.
  decltype(make_pyramid_tuple<IMG_WIDTH, IMG_HEIGHT>(std::make_index_sequence<NUM_LEVELS + 1>{})) images_;

  // Array of pointers to each image in the tuple.
  std::array<const void*, NUM_LEVELS + 1> images_ptr_;


  const int left_shift_kernel_[9] = {0, 1, 0, 1, 2, 1, 0, 1, 0};
  // const float multiply_kernel_[25] = {1, 4, 6, 4, 1,
  //                                       4, 16, 24, 16, 4,
  //                                       6, 24, 36, 24, 6,
  //                                       4, 16, 24, 16, 4,
  //                                       1, 4, 6, 4, 1}; // divide by 256 after summing
};

// void ImagePyramid::create_pyramids() {
//   for (int i = 1; i <= num_levels_; i ++ ) {
//     auto prev_img = static_cast<const RawImage<IMG_WIDTH / (2 << i), IMG_HEIGHT  / (2 << i)>*>(images_ptr_[0]);
//     auto curr_img = static_cast<RawImage<IMG_WIDTH/2, IMG_HEIGHT/2>*>(images_ptr_[1]);
//     int height = pyramid_[i]->height;
//     int width = pyramid_[i]->width;

//     int prev_img_height = pyramid_[i-1]->height;
//     int prev_img_width = pyramid_[i-1]->width;
//     for (int y = 0; y < height; y++) {
//       for (int x = 0; x < width; x++) {
//         int sum = 0;
//         // Function to downsample an image using lowpass filter from
//         // http://robots.stanford.edu/cs223b04/algo_tracking.pdf
//         for (int j = 0; j < 3; j++) {
//           for (int k = 0; k < 3; k++) {
//             int y_index = 2*y+j-1;
//             if (y_index < 0) y_index = 0;
//             else if (y_index >= prev_img_height) y_index = prev_img_height-1;

//             int x_index = 2*x+k-1;
//             if (x_index < 0) x_index = 0;
//             else if (x_index >= prev_img_width) x_index = prev_img_width-1;

//             sum += ((int) pyramid_[i-1]->data[y_index*prev_img_height+x_index]) << left_shift_kernel_[j*3+k];
//             // sum += ((float) pyramid_[i-1]->data[y_index*prev_img_height+x_index]) * left_shift_kernel_[j*5+k];
//           }
//         }
//         pyramid_[i]->data[y*height+x] = (uint8_t) (sum >> 4);
//         // pyramid_[i]->data[y*height+x] = (uint8_t) (sum / 256);
//       }
//     }
//   }
// }

#endif // IMAGE_PYRAMID_H