#ifndef RAW_IMAGE_H
#define RAW_IMAGE_H

#include <math.h>
#include <png.h>
#include <iostream>
#include <cstdlib>
#include <cstring>

typedef struct {
  uint8_t* data;
  uint16_t width;
  uint16_t height;
} RawImage;

void read_png_image(const char* imagepath, RawImage* image) {
    if (!imagepath || !image) {
        std::cerr << "Invalid input to read_png_image!" << std::endl;
        return;
    }

    FILE *fp = fopen(imagepath, "rb");
    if (!fp) {
        std::cerr << "Error opening file: " << imagepath << std::endl;
        return;
    }

    // Read the PNG header
    png_byte header[8];
    fread(header, 1, 8, fp);
    if (png_sig_cmp(header, 0, 8)) {
        std::cerr << "File is not a valid PNG: " << imagepath << std::endl;
        fclose(fp);
        return;
    }

    // Initialize libpng read structures
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        std::cerr << "Failed to create PNG read struct!" << std::endl;
        fclose(fp);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        std::cerr << "Failed to create PNG info struct!" << std::endl;
        png_destroy_read_struct(&png, nullptr, nullptr);
        fclose(fp);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        std::cerr << "Error during PNG reading!" << std::endl;
        png_destroy_read_struct(&png, &info, nullptr);
        fclose(fp);
        return;
    }

    png_init_io(png, fp);
    png_set_sig_bytes(png, 8);  // We already read 8 bytes

    png_read_info(png, info);

    // Get image details
    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth = png_get_bit_depth(png, info);
    uint16_t height = image->height;
    uint16_t width = image->width;

    // Ensure the image is in grayscale format
    if (color_type != PNG_COLOR_TYPE_GRAY || bit_depth != 8) {
        std::cerr << "Unsupported PNG format. Expected 8-bit grayscale." << std::endl;
        png_destroy_read_struct(&png, &info, nullptr);
        fclose(fp);
        return;
    }

    // Allocate memory in the RawImage structure
    if (!image->data) {
        std::cerr << "Memory allocation failed for image data!" << std::endl;
        png_destroy_read_struct(&png, &info, nullptr);
        fclose(fp);
        return;
    }

    // Read image row-by-row
    png_bytep* row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
    for (uint16_t i = 0; i < height; ++i) {
        row_pointers[i] = image->data + (i * width);
    }

    png_read_image(png, row_pointers);
    png_read_end(png, nullptr);

    // Cleanup
    free(row_pointers);
    png_destroy_read_struct(&png, &info, nullptr);
    fclose(fp);

    std::cout << "PNG loaded successfully: " << imagepath << std::endl;
}


RawImage* new_image(uint16_t height, uint16_t width)
{
  RawImage* img = (RawImage *)malloc(sizeof(RawImage));
  if (!img) return NULL;
  img->width = width;
  img->height = height;
  img->data = (uint8_t *)malloc(img->width * img->height * sizeof(uint8_t));
  if (!img->data) return NULL;
  return img;
}

void del_raw_image(RawImage *img)
{
  if (!img) return;
  if (img->data) free((void *)img->data);
  free((void *)img);
}

#endif 