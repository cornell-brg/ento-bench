#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

#include <assert.h>
#include <utility>
#include <tuple>
#include <stdio.h>
#include <image_io/Image.h>

// Now the function is templated only on IMG_WIDTH, IMG_HEIGHT.
// The indices come from a function parameter of type std::index_sequence<Is...>.
// do we need this in the template? and why size_t... Is
// TODO: look more into the ellipses stuff

// Create a tuple of images, with dimension halfed with increasing level
template <size_t IMG_WIDTH, size_t IMG_HEIGHT, typename PixelT, size_t... Is>
constexpr auto make_pyramid_tuple(std::index_sequence<Is...>)
{
  return std::make_tuple(
      (Image<(IMG_HEIGHT >> (Is + 1)), (IMG_WIDTH >> (Is + 1)), PixelT>())...
  );
}

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT, typename PixelT>
class ImagePyramid
{
public:
    // Define the tuple type that holds Image objects for levels 0..NUM_LEVELS-1.
    using PyramidTupleType = decltype(
        make_pyramid_tuple<IMG_WIDTH, IMG_HEIGHT, PixelT>(std::make_index_sequence<NUM_LEVELS>{})
        );

    // Top-level image
    Image<IMG_HEIGHT, IMG_WIDTH, PixelT>* top_level;

    // The tuple storing the lower pyramid images.
    PyramidTupleType pyramid;

    // Constructor that sets the 0th level to 'topLevelImage' and
    // default-constructs (or otherwise creates) the other levels.
    constexpr ImagePyramid()
    {
    }

    // Set top level
    constexpr void set_top_image(Image<IMG_HEIGHT, IMG_WIDTH, PixelT>& topLevelImage)
    {
        top_level = &topLevelImage;
    }

    // Downscales images from level 1 to level NUM_LEVELS.
    constexpr void initialize_pyramid()
    {
        // If the pyramid has more than one level, initialize from 0..NUM_LEVELS
        if constexpr (NUM_LEVELS > 0) {
            call_create_pyramids_n_times(std::make_index_sequence<NUM_LEVELS-1>{});
        }
    }

    template <std::size_t Index>
    auto& get_level() const {
        if constexpr (Index == 0) return *top_level;
        else return std::get<Index-1>(pyramid);
    }


private:
    const int left_shift_kernel_[9] = {0, 1, 0, 1, 2, 1, 0, 1, 0};

    template<size_t PREV_IMG_WIDTH, size_t PREV_IMG_HEIGHT>
    void create_pyramids(const Image<PREV_IMG_HEIGHT, PREV_IMG_WIDTH, PixelT>& prev_img,
                        Image<PREV_IMG_HEIGHT / 2, PREV_IMG_WIDTH / 2, PixelT>& img) {
        size_t width = PREV_IMG_WIDTH / 2;
        size_t height = PREV_IMG_HEIGHT / 2;
        for (size_t y = 0; y < height; y++) {
            for (size_t x = 0; x < width; x++) {
            int sum = 0;
            // Function to downsample an image using lowpass filter from
            // http://robots.stanford.edu/cs223b04/algo_tracking.pdf
            for (size_t j = 0; j < 3; j++) {
                for (size_t k = 0; k < 3; k++) {
                    size_t y_index = 2*y+j-1;

                    // check for overflow
                    if (y_index > PREV_IMG_HEIGHT+3) y_index = 0;
                    else if (y_index >= PREV_IMG_HEIGHT) y_index = PREV_IMG_HEIGHT-1;

                    size_t x_index = 2*x+k-1;
                    if (x_index > PREV_IMG_WIDTH+3) x_index = 0;
                    else if (x_index >= PREV_IMG_WIDTH) x_index = PREV_IMG_WIDTH-1;

                    sum += ((int) prev_img.data[y_index*PREV_IMG_HEIGHT+x_index]) << left_shift_kernel_[j*3+k];
                    // sum += ((float) prev_img->data[y_index*PREV_IMG_HEIGHT+x_index]) * left_shift_kernel_[j*5+k];
                }
            }
            img.data[y*height+x] = (uint8_t) (sum >> 4);

            // img->data[y*height+x] = (uint8_t) (sum / 256);
            }
        }
    }

    // Helper to initialize images at each level of pyramid
    // Use a fold expression to have compile-time loop without recursion 
    template <std::size_t... Is>
    void call_create_pyramids_n_times(std::index_sequence<Is...>)
    {

        create_pyramids<IMG_WIDTH, IMG_HEIGHT>(*top_level, std::get<0>(pyramid));

        ((
            (void)create_pyramids<(IMG_WIDTH >> (Is+1)), (IMG_HEIGHT >> (Is+1))>(std::get<Is>(pyramid), std::get<Is+1>(pyramid))
        ), ...);
    }

};

#endif // IMAGE_PYRAMID_H
