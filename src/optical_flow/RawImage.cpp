#ifndef RAW_IMAGE
#define RAW_IMAGE

#include <math.h>

typedef struct {
  uint8_t* data;
  uint16_t width;
  uint16_t height;
} RawImage;

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