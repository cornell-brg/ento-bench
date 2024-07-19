#ifndef IMAGEIO_H
#define IMAGEIO_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>


//====================================================
// PNG Image Handling
#ifndef STM32
#include <png.h>
#endif
typedef struct PNGImg {
    png_bytep *rowPointers;
    png_infop       info;
    png_structp      png;
} PNGImage;
PNGImage* new_png_image();
void del_png_image(PNGImage *img);
void read_png_file(char *filename, PNGImage *image);
void write_png_file(char *filename, PNGImage *image);
void display_image_info(PNGImage *image);
//====================================================
// PGN Image Handling
Image* new_image();

//====================================================

#endif
