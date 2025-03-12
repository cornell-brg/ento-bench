#include <stdlib.h>
#include <Eigen/Dense>

#include <ento-util/debug.h>
#include <ento-feature2d/raw_image.h>
#include <ento-feature2d/image_pyramid_template.h>

// #include <ento-feature2d/lk_optical_flow.h>
#include <ento-util/unittest.h>
#include <iostream>
#include <vector>
#include <png.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;

// DEBUG MESSAGES
template<size_t WIDTH, size_t HEIGHT>
void write_png_image(const char* imagepath, RawImage<WIDTH, HEIGHT>& image) {
    if (!imagepath) {
        cerr << "Invalid input to write_png_image!" << endl;
        return;
    }

    FILE *fp = fopen(imagepath, "wb");
    if (!fp) {
        cerr << "Error opening file for writing: " << imagepath << endl;
        return;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        cerr << "Failed to create PNG write structure!" << endl;
        fclose(fp);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        cerr << "Failed to create PNG info structure!" << endl;
        png_destroy_write_struct(&png, nullptr);
        fclose(fp);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        cerr << "Error during PNG creation!" << endl;
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return;
    }

    png_init_io(png, fp);

    // Set PNG metadata
    png_set_IHDR(png, info, image.width, image.height, 8, // 8-bit depth
                 PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png, info);

    // Write image data row-by-row
    png_bytep* row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * image.height);
    for (uint16_t i = 0; i < image.height; ++i) {
        row_pointers[i] = &image.data[i * image.width];
    }

    png_write_image(png, row_pointers);
    png_write_end(png, nullptr);

    // Cleanup
    free(row_pointers);
    png_destroy_write_struct(&png, &info);
    fclose(fp);

    cout << "PNG saved successfully: " << imagepath << endl;
}

// Tests
// void test_pyramid_zero_level()
// {
//     // initialize topImg 
//     string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
//     const char* prev_image_path = top_str.c_str();
//     RawImage<320, 320> topImg;
//     read_png_image<320, 320>(prev_image_path, topImg);

//     string dst_str = "//Users/acui21/Documents/brg/ento-bench/build/native/src/ento-feature2d/bin/image_2.png";
//     const char* dst_image_path = dst_str.c_str();
//     write_png_image<320, 320>(dst_image_path, topImg);

//     // int NUM_LEVELS = 0;
//     // ImagePyramid* pyramid = new ImagePyramid(topImg, 320, 320, NUM_LEVELS);
//     // pyramid->create_pyramids();

//     // ENTO_TEST_CHECK_ARRAY_INT_EQ(pyramid->get_level(0).data, topImg.data, 320*320);

// }

// void test_pyramid()
// {
//     // initialize topImg 
//     string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
//     const char* prev_image_path = top_str.c_str();
//     RawImage* topImg = new_image(320, 320);
//     if (!topImg) {
//         cerr << "Failed to allocate RawImage!" << endl;
//         return;
//     }
//     read_png_image(prev_image_path, topImg);

//     // initialize oneImg 
//     string one_str = "/Users/acui21/Documents/brg/code/gem5_pyr_prev_1.png";
//     const char* one_image_path = one_str.c_str();
//     RawImage* oneImg = new_image(160, 160);
//     if (!oneImg) {
//         cerr << "Failed to allocate RawImage!" << endl;
//         return;
//     }
//     read_png_image(one_image_path, oneImg);

//     // initialize oneImg 
//     string two_str = "/Users/acui21/Documents/brg/code/gem5_pyr_prev_2.png";
//     const char* two_image_path = two_str.c_str();
//     RawImage* twoImg = new_image(80, 80);
//     if (!twoImg) {
//         cerr << "Failed to allocate RawImage!" << endl;
//         return;
//     }
//     read_png_image(two_image_path, twoImg);

//     int NUM_LEVELS = 2;
//     ImagePyramid* pyramid = new ImagePyramid(topImg, 320, 320, NUM_LEVELS);
//     pyramid->create_pyramids();

//     ENTO_TEST_CHECK_ARRAY_INT_EQ(pyramid->get_level(0).data, topImg->data, 320*320);
//     ENTO_TEST_CHECK_ARRAY_INT_EQ(pyramid->get_level(1).data, oneImg->data, 160*160);
//     ENTO_TEST_CHECK_ARRAY_INT_EQ(pyramid->get_level(2).data, twoImg->data, 80*80);

//     delete pyramid;
// }
template <size_t IMG_WIDTH, size_t IMG_HEIGHT>
void check_pyramid(RawImage<IMG_WIDTH, IMG_HEIGHT>& img, size_t Is) {
  // initialize oneImg 
  string str = "/Users/acui21/Documents/brg/code/gem5_pyr_prev_" + to_string(Is) + ".png";
  const char* image_path = str.c_str();
  RawImage<IMG_WIDTH, IMG_HEIGHT> golden_img;
  read_png_image<IMG_WIDTH, IMG_HEIGHT>(image_path, golden_img);

  ENTO_TEST_CHECK_ARRAY_INT_EQ(img.data, golden_img.data,  IMG_WIDTH * IMG_HEIGHT);
}

template <typename Tuple, size_t TOP_WIDTH, size_t TOP_HEIGHT, std::size_t... Is>
void test_pyramid_helper(Tuple& pyramid, std::index_sequence<Is...>)
{
    // Use a fold expression to print each level’s type in one go.
    ((check_pyramid<(TOP_WIDTH >> Is), (TOP_HEIGHT >> Is)>(get<Is>(pyramid), Is)),
     ...);

}

// void test() {
//     // initialize topImg 
//     string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
//     const char* prev_image_path = top_str.c_str();
//     constexpr size_t DIM = (size_t) 320;
//     RawImage<DIM, DIM> topImg;
//     read_png_image<DIM, DIM>(prev_image_path, topImg);
//     ImagePyramid<(size_t) 1, DIM, DIM> pyramid(topImg);

//     // Using the templated getter:
//     const void* level0 = pyramid.get_level(0);  // RawImage<640,480>
//     const RawImage<DIM,DIM>* level1Image = static_cast<const RawImage<DIM,DIM>*>(level0);
//     string dst_str = "//Users/acui21/Documents/brg/ento-bench/build/native/src/ento-feature2d/bin/image_2.png";
//     const char* dst_image_path = dst_str.c_str();
//     write_png_image<DIM, DIM>(dst_image_path, level1Image);

// }

// A helper function that iterates over the levels at compile time and prints each type.
template <typename Tuple, std::size_t... Is>
void printPyramidLevelTypes(Tuple& pyramid, std::index_sequence<Is...>)
{
    // Use a fold expression to print each level’s type in one go.
    ((std::cout << "Level " << Is << " type: "
                << typeid(std::tuple_element_t<Is, Tuple>).name() << "\n"),
     ...);

    ((void)write_png_image<(320 >> Is), (320 >> Is)>(
      ("pyramid_" + to_string(Is) + ".png").c_str(), 
      get<Is>(pyramid)),
      ...);

}

void basic() {
    // initialize topImg 
    string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
    const char* prev_image_path = top_str.c_str();
    constexpr size_t DIM = (size_t) 320;
    RawImage<DIM, DIM> topImg;
    read_png_image<DIM, DIM>(prev_image_path, topImg);


  ImagePyramid<2, 320, 320> pyramid(topImg);
  using MyTupleType = decltype(pyramid.pyramid);
  // Print the types of each level in the pyramid:
  pyramid.initialize_pyramid();
  printPyramidLevelTypes<MyTupleType>(
      pyramid.pyramid,
      std::make_index_sequence<3>{}
  );
}

void test_pyramid_zero_level() {
  // initialize topImg 
  string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
  const char* prev_image_path = top_str.c_str();
  constexpr size_t DIM = (size_t) 320;
  RawImage<DIM, DIM> topImg;
  read_png_image<DIM, DIM>(prev_image_path, topImg);

  constexpr size_t NUM_LEVELS = 0;
  ImagePyramid<NUM_LEVELS, 320, 320> pyramid(topImg);
  using MyTupleType = decltype(pyramid.pyramid);
  // Print the types of each level in the pyramid:
  pyramid.initialize_pyramid();
  test_pyramid_helper<MyTupleType, DIM, DIM>(
      pyramid.pyramid,
      std::make_index_sequence<NUM_LEVELS+1>{}
  );
}

void test_pyramid_one_level() {
  // initialize topImg 
  string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
  const char* prev_image_path = top_str.c_str();
  constexpr size_t DIM = (size_t) 320;
  RawImage<DIM, DIM> topImg;
  read_png_image<DIM, DIM>(prev_image_path, topImg);

  constexpr size_t NUM_LEVELS = 1;
  ImagePyramid<NUM_LEVELS, 320, 320> pyramid(topImg);
  using MyTupleType = decltype(pyramid.pyramid);
  pyramid.initialize_pyramid();
  test_pyramid_helper<MyTupleType, DIM, DIM>(
      pyramid.pyramid,
      std::make_index_sequence<NUM_LEVELS+1>{}
  );
}

void test_pyramid_three_level() {
  string top_str = "/Users/acui21/Documents/brg/FigureEight_test4_images/image_2.png";
  const char* prev_image_path = top_str.c_str();
  constexpr size_t DIM = (size_t) 320;
  RawImage<DIM, DIM> topImg;
  read_png_image<DIM, DIM>(prev_image_path, topImg);


  ImagePyramid<2, 320, 320> pyramid(topImg);
  pyramid.initialize_pyramid();
  using MyTupleType = decltype(pyramid.pyramid);
  test_pyramid_helper<MyTupleType, DIM, DIM>(
      pyramid.pyramid,
      std::make_index_sequence<3>{}
  );
}


int main ( int argc, char ** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_image_pyramid_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_pyramid_zero_level();
  if (__ento_test_num(__n, 2)) test_pyramid_one_level();
  if (__ento_test_num(__n, 3)) test_pyramid_three_level();

}
