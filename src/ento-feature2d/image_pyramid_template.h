#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

#include <ento-feature2d/raw_image.h>
#include <assert.h>
#include <stdio.h>



// // 1) A small helper that returns a different type depending on the index:
// template <size_t LEVEL_WIDTH, size_t LEVEL_HEIGHT, size_t Level>
// constexpr auto make_level(const RawImage<LEVEL_WIDTH, LEVEL_HEIGHT>& topLevelImage)
// {
//     if constexpr (Level == 0) {
//         // Return the top-level image itself (by value).
//         return topLevelImage;
//     } else {
//         // Return a downscaled version (default-constructed).
//         return RawImage<(LEVEL_WIDTH >> Level), (LEVEL_HEIGHT >> Level)>();
//     }
// }

// // 2) Use that helper in the pack expansion:
// template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t... Is>
// constexpr auto make_pyramid_tuple_from_input(
//     const RawImage<IMG_WIDTH, IMG_HEIGHT>& topLevelImage,
//     std::index_sequence<Is...>)
// {
//     return std::make_tuple(
//         make_level<IMG_WIDTH, IMG_HEIGHT, Is>(topLevelImage)...  // expanded call
//     );
// }

// Now the function is templated only on IMG_WIDTH, IMG_HEIGHT.
// The indices come from a function parameter of type std::index_sequence<Is...>.
// do we need this in the template? and why size_t... Is
// TODO: look more into the ellipses stuff

// Create a tuple of images, with dimension halfed with increasing level
template <size_t IMG_WIDTH, size_t IMG_HEIGHT, size_t... Is>
constexpr auto make_pyramid_tuple(std::index_sequence<Is...>)
{
    return std::make_tuple(
        (RawImage<(IMG_WIDTH >> Is), (IMG_HEIGHT >> Is)>())...
    );
}

template <size_t NUM_LEVELS, size_t IMG_WIDTH, size_t IMG_HEIGHT>
class ImagePyramid {
public:
    // Define the tuple type that holds RawImage objects for levels 0..NUM_LEVELS-1.
    using PyramidTupleType = decltype(
        make_pyramid_tuple<IMG_WIDTH, IMG_HEIGHT>(std::make_index_sequence<NUM_LEVELS+1>{})
        );

    // The tuple storing the pyramid images.
    PyramidTupleType pyramid;

    // Constructor that sets the 0th level to 'topLevelImage' and
    // default-constructs (or otherwise creates) the other levels.
    constexpr ImagePyramid(const RawImage<IMG_WIDTH, IMG_HEIGHT>& topLevelImage)
        : pyramid(make_pyramid_tuple_from_input(
            topLevelImage,
            std::make_index_sequence<NUM_LEVELS+1>{}))
    {
    }

    // Downscales images from level 1 to level NUM_LEVELS.
    constexpr void initialize_pyramid()
    {
        // If the pyramid has more than one level, initialize from 0..NUM_LEVELS
        if constexpr (NUM_LEVELS > 0) {
            call_create_pyramids_n_times(std::make_index_sequence<NUM_LEVELS>{});
        }
    }

private:
    const int left_shift_kernel_[9] = {0, 1, 0, 1, 2, 1, 0, 1, 0};

    // Helper that returns the image type for a level
    template <size_t LEVEL_WIDTH, size_t LEVEL_HEIGHT, size_t Level>
    constexpr auto make_level(const RawImage<LEVEL_WIDTH, LEVEL_HEIGHT>& topLevelImage)
    {
        if constexpr (Level == 0) {
            // Return the top-level image itself (by value).
            return topLevelImage;
        } else {
            // Return a downscaled version (default-constructed).
            return RawImage<(LEVEL_WIDTH >> Level), (LEVEL_HEIGHT >> Level)>();
        }
    }

    // Create a tuple of images, with dimension halfed with increasing level
    // Set level 0 to the top level image
    template <size_t... Is>
    constexpr auto make_pyramid_tuple_from_input(
        const RawImage<IMG_WIDTH, IMG_HEIGHT>& topLevelImage,
        std::index_sequence<Is...>)
    {
        return std::make_tuple(
            make_level<IMG_WIDTH, IMG_HEIGHT, Is>(topLevelImage)...  // expanded call
        );
    }

    template<size_t PREV_IMG_WIDTH, size_t PREV_IMG_HEIGHT>
    void create_pyramids(const RawImage<PREV_IMG_WIDTH, PREV_IMG_HEIGHT>& prev_img,
                        RawImage<PREV_IMG_WIDTH / 2, PREV_IMG_HEIGHT / 2>& img) {
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
                if (PREV_IMG_WIDTH== 320 && PREV_IMG_HEIGHT == 320 && 
                    x == 0 && y == 0) {
                        std::cout << (size_t) y_index << std::endl;
                    }
                // check for overflow
                if (y_index > PREV_IMG_HEIGHT+3) y_index = 0;
                else if (y_index >= PREV_IMG_HEIGHT) y_index = PREV_IMG_HEIGHT-1;

                size_t x_index = 2*x+k-1;
                if (x_index > PREV_IMG_WIDTH+3) x_index = 0;
                else if (x_index >= PREV_IMG_WIDTH) x_index = PREV_IMG_WIDTH-1;

                if (PREV_IMG_WIDTH== 320 && PREV_IMG_HEIGHT == 320 && 
                    x == 0 && y == 0) {
                    std::cout << 
                    (int) x_index << " " <<
                    (int) y_index << " " <<
                    (int)prev_img.data[y_index*PREV_IMG_HEIGHT+x_index] << " " << 
                    left_shift_kernel_[j*3+k] << " " <<
                    (((int) prev_img.data[y_index*PREV_IMG_HEIGHT+x_index]) << left_shift_kernel_[j*3+k])
                    << std::endl;
                }
                
                sum += ((int) prev_img.data[y_index*PREV_IMG_HEIGHT+x_index]) << left_shift_kernel_[j*3+k];
                // sum += ((float) prev_img->data[y_index*PREV_IMG_HEIGHT+x_index]) * left_shift_kernel_[j*5+k];
                }
            }
            img.data[y*height+x] = (uint8_t) (sum >> 4);

            if (PREV_IMG_WIDTH== 320 && PREV_IMG_HEIGHT == 320 && 
                x == 0 && y == 0) {
                std::cout << (int) img.data[y*height+x]<< std::endl;
            }
            // img->data[y*height+x] = (uint8_t) (sum / 256);
            }
        }
    }

    // Helper to initialize images at each level of pyramid
    template <std::size_t... Is>
    void call_create_pyramids_n_times(std::index_sequence<Is...>)
    {
        ( (void)create_pyramids<(IMG_WIDTH >> Is), (IMG_HEIGHT >> Is)>(std::get<Is>(pyramid), std::get<Is+1>(pyramid)), ... );
    }

};

#endif // IMAGE_PYRAMID_H