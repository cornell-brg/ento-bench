#include "RawImage.cpp"
#include <assert.h>

class ImagePyramid {
public:
  // Constructor that takes a const reference to the base RawImage
  ImagePyramid( RawImage* topLevelImage, int IMG_HEIGHT, int IMG_WIDTH, int NUM_LEVELS);
  ~ImagePyramid();
  // Getter for the lower levels
  const RawImage& get_level(int level) const;

  // Create pyramids all at once
  void create_pyramids();
  
private:
  RawImage ** pyramid_; // Owned pyramid
  int num_levels_; // Number of levels
  const int left_shift_kernel_[9] = {0, 1, 0, 1, 2, 1, 0, 1, 0};
  // const float multiply_kernel_[25] = {1, 4, 6, 4, 1,
  //                                       4, 16, 24, 16, 4,
  //                                       6, 24, 36, 24, 6,
  //                                       4, 16, 24, 16, 4,
  //                                       1, 4, 6, 4, 1}; // divide by 256 after summing
};

ImagePyramid::ImagePyramid( RawImage* topLevelImage, int IMG_HEIGHT, int IMG_WIDTH, int NUM_LEVELS) {
  num_levels_ = NUM_LEVELS;
  
  pyramid_ = new RawImage*[NUM_LEVELS+1];

  pyramid_[0] = topLevelImage;

  for (int i = 1; i <= NUM_LEVELS; i++) {
    pyramid_[i] = new_image(IMG_HEIGHT / (1 << i), IMG_WIDTH / (1 << i));
  }
}

const RawImage& ImagePyramid::get_level(int level) const {
  assert(level >= 0 & level <= num_levels_);
  return *pyramid_[level];  // Return a reference to the RawImage
}

void ImagePyramid::create_pyramids() {
  for (int i = 1; i <= num_levels_; i ++ ) {
    int height = pyramid_[i]->height;
    int width = pyramid_[i]->width;

    int prev_img_height = pyramid_[i-1]->height;
    int prev_img_width = pyramid_[i-1]->width;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int sum = 0;
        // Function to downsample an image using lowpass filter from
        // http://robots.stanford.edu/cs223b04/algo_tracking.pdf
        for (int j = 0; j < 3; j++) {
          for (int k = 0; k < 3; k++) {
            int y_index = 2*y+j-1;
            if (y_index < 0) y_index = 0;
            else if (y_index >= prev_img_height) y_index = prev_img_height-1;

            int x_index = 2*x+k-1;
            if (x_index < 0) x_index = 0;
            else if (x_index >= prev_img_width) x_index = prev_img_width-1;

            sum += ((int) pyramid_[i-1]->data[y_index*prev_img_height+x_index]) << left_shift_kernel_[j*3+k];
            // sum += ((float) pyramid_[i-1]->data[y_index*prev_img_height+x_index]) * left_shift_kernel_[j*5+k];
          }
        }
        pyramid_[i]->data[y*height+x] = (u_char) (sum >> 4);
        // pyramid_[i]->data[y*height+x] = (u_char) (sum / 256);
      }
    }
  }
}

ImagePyramid::~ImagePyramid() {
  for (int i = 0; i < num_levels_; i++) {
    del_raw_image(pyramid_[i]);
  }
  delete [] pyramid_;    
}