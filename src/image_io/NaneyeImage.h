#ifndef NANEYE_IMAGE_H
#define NANEYE_IMAGE_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
//#include "MonoImage16.h"

typedef struct {
  uint16_t data[320][320];
  uint16_t width = 320 / 2 ; // 320
  const uint16_t height = 320 ; // 320
} NaneyeImage;

typedef struct {
  uint32_t rows_per_part;
  uint32_t parts_per_img;
  uint32_t row_stride;
  uint32_t image_size;
} BufferParams;

typedef struct {
  uint16_t* data;
  NaneyeImage* image;
  int32_t buffer_num;
  int32_t frame_num;
  BufferParams* params;
} RawNaneyeBuffer;

NaneyeImage* new_naneye_image();
//int naneye_image_from_mono_image16(NaneyeImage* naneye_img, MonoImage16* mono_img);
int naneye_image_from_pgm(NaneyeImage* naneye_img, const char* pgm_path);
void del_naneye_image(NaneyeImage* naneye_img);

void buffer_params_init(BufferParams* pparams, uint32_t rows_per_part, uint32_t parts_per_img, uint32_t row_stride, uint32_t image_size);
RawNaneyeBuffer* initialize_pixel_buffer();
void raw_naneye_buffer_init(RawNaneyeBuffer* pbuff, NaneyeImage* image, BufferParams* params);
int next_partition(RawNaneyeBuffer *pbuff);
int next_image(RawNaneyeBuffer* pbuff, NaneyeImage* pimg);
int update_pixel_buffer(RawNaneyeBuffer* pbuff, NaneyeImage* img);
void del_pixel_buffer(RawNaneyeBuffer* pbuff);

#endif
