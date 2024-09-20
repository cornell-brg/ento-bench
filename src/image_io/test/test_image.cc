#include <cstdio>
#include "image_io/Pixel.h"
#include "image_io/Image.h"

void checkCondition(bool condition, const char* testName) {
  if (condition) {
    printf("%s passed.\n", testName);
  } else {
    printf("%s not passed.\n", testName);
  }
}

void test_pixel_types() {
  printf("Running test_pixel_types...\n");

  // Test PixelType<8>
  Pixel<8> pixel8 = 255;
  checkCondition(pixel8 == 255, "PixelType<8>");

  // Test PixelType<10>
  Pixel<10> pixel10 = 1023;
  checkCondition(pixel10 == 1023, "PixelType<10>");

  // Test PixelType<16>
  Pixel<16> pixel16 = 65535;
  checkCondition(pixel16 == 65535, "PixelType<16>");
}

void test_packed_pixel() {
  printf("Running test_packed_pixel...\n");

  // Create a 10-bit packed pixel with start bit = 1, stop bit = 0
  PackedPixel<10> packedPixel;
  packedPixel.set_pixel(512);

  // Test getPixel with 10-bit depth
  uint16_t pixelValue10 = packedPixel.get_pixel<10>();
  printf("PackedPixel get_pixel<10>: %i\n", pixelValue10);
  checkCondition(pixelValue10 == 512, "PackedPixel get_pixel<10>");

  // Test getPixel with 8-bit depth
  uint8_t pixelValue8 = packedPixel.get_pixel<8>();
  printf("PackedPixel get_pixel<8>: %i\n", pixelValue8);
  checkCondition(pixelValue8 == (512 >> 2), "PackedPixel get_pixel<8>");

  printf("PackedPixel test finished.\n");
}

void test_image() {
  printf("Running test_image...\n");

  // Create a normal Image with 8-bit pixels
  Image<4, 4, uint8_t> img8Bit;
  img8Bit.set_pixel(0, 0, 255);
  checkCondition(img8Bit(0, 0) == 255, "Image get_pixel(0, 0)");

  img8Bit.set_pixel(2, 2, 128);
  checkCondition(img8Bit(2, 2) == 128, "Image get_pixel(2, 2)");

  printf("Image test finished.\n");
}

void test_packed_image() {
  printf("Running test_packed_image...\n");

  // Create a PackedImage with 10-bit pixels (8-bit return, start bit = 1, stop bit = 0)
  PackedImage<4, 4, 10, 8, 1, 0> packed_img_10Bit;

  packed_img_10Bit.set_packed_pixel(0, 0, PixelType<10, 1, 0>::type(512));

  // Test getPixel with user-specified 8-bit depth
  uint8_t pixelValue8 = packed_img_10Bit(0, 0);
  checkCondition(pixelValue8 == (512 >> 2), "PackedImage getPixel(0, 0)");

  // Test raw packed data
  auto rawPacket = packed_img_10Bit.get_raw_packed_pixel(0, 0);
  checkCondition(rawPacket == 1024, "PackedImage getRawPackedPixel(0, 0)");

  printf("PackedImage test finished.\n");
}

void test_image_from_pgm() {
  printf("Running test_image_from_pgm...\n");

  // Create an Image object with the appropriate size (adjust dimensions as needed)
  Image<5, 5, uint16_t> image;

  // Test loading a PGM file into the Image object
  const char* pgm_path = "/home/ddo26/workspace/entomoton-bench/datasets/unittest/test_pgm.pgm";
  printf("Attempting image from pgm load!\n");
  uint16_t expected[25] = {
    65535,     0, 65535,      0, 65535,
        0, 65535,      0, 65535, 0    ,
    65535,     0, 65535,      0, 65535,
        0, 65535,      0, 65535, 0    ,
    65535,     0, 65535,      0, 65535
  };
  if (image.image_from_pgm(pgm_path)) {
    printf("PGM file loaded successfully.\n");
    char test_name[50];

    for (int i = 0; i < 5; i++)
    {
      for (int j= 0; j < 5; j++)
      {
        uint16_t pixel = image.get_pixel(i, j);
        uint16_t expected_pixel = expected[5*i + j];
        sprintf(test_name, "Image get_pixel(%d, %d)", i, j);
        checkCondition(pixel == expected_pixel, test_name);
      }
    }
  } else {
    printf("Failed to load PGM file.\n");
  }

  printf("test_image_from_pgm finished.\n");
}

int main() {
  test_pixel_types();
  test_packed_pixel();
  test_image();
  test_packed_image();
  test_image_from_pgm();

  printf("All tests executed!\n");
  return 0;
}
