#include "MonoImage16.h"

/*MonoImage16* new_mono_image16()
{
    MonoImage16 *mono_image = malloc(sizeof(MonoImage16));
    if (!mono_image) return NULL;
    mono_image->rowPointers = NULL;
    mono_image->width = 0;
    mono_image->height = 0;
    return mono_image;
}*/

MonoImage16* new_mono_image16(uint32_t width, uint32_t height)
{

    MonoImage16 *mono_image = malloc(sizeof(MonoImage16));
    if (!mono_image) return NULL;
    mono_image->width = width;
    mono_image->height = height;

    mono_image->rowPointers = malloc(height*sizeof(uint16_t *));
    
    mono_image->rowPointers[0] = malloc(height*width*sizeof(uint16_t));
    for(int i = 0; i < height; i++)
    {
        mono_image->rowPointers[i] = mono_image->rowPointers[0] + (i * width);
    }
}

MonoImage16* new_mono_image16_from_png(PNGImage *image)
{
    if (!image->info) {
        printf("Image info does not exist!");
        return NULL;
    }
    if (!image->png) {
        printf("Image png struct does not exists!");
        return NULL;
    }

    if (png_get_bit_depth(image->png, image->info) != 16) {
        printf("PNG Image is not 16 bit depth!");
        return NULL;
    }

    unsigned int width = png_get_image_width(image->png, image->info);
    unsigned int height = png_get_image_height(image->png, image->info);

    MonoImage16 *mono_image = malloc(sizeof(MonoImage16));
    if (!mono_image) return NULL;
    mono_image->width = width;
    mono_image->height = height;

    mono_image->rowPointers = malloc(height*sizeof(uint16_t *));
    
    mono_image->rowPointers[0] = malloc(height*width*sizeof(uint16_t));
    for(int i = 0; i < height; i++)
    {
        mono_image->rowPointers[i] = mono_image->rowPointers[0] + (i * width);
    }
    
    png_bytep png_row;
    uint16_t *row;
    //uint16_t pix;
    for(int r = 0; r < height; r++)
    {
        png_row = image->rowPointers[r];
        row = mono_image->rowPointers[r];
        for (int c = 0; c < width; c++)
        {
            //*(row[c]) = *(png_row[c]);
            memcpy(&(row[c]), &(png_row[2*c]), 2);
        }
    }
    return mono_image;
}

void del_mono_image16(MonoImage16 *img)
{
    if (!img) return;
    if (img->rowPointers)
    {
        if (img->rowPointers[0]) free((void *)img->rowPointers[0]);
        free((void *)img->rowPointers);
    }
    free((void *)img);
}

int mono_image_from_png(MonoImage16* img, PNGImage* pngimg)
{
  if (!img) return 0;
  if (!pngimg) return 0;
  if (!img->rowPointers) return 0;
  
  unsigned int width = png_get_image_width(pngimg->png, pngimg->info);
  unsigned int height = png_get_image_height(pngimg->png, pngimg->info);
  png_byte bit_depth = png_get_bit_depth(pngimg->png, pngimg->info);

  if (img->width != width && img->width != height) return 0;  
  if (bit_depth != 16) return 0;

  png_bytep png_row;
  uint16_t *row;
  //uint16_t pix;
  for(uint32_t r = 0; r < img->height; r++)
  {
      png_row = pngimg->rowPointers[r];
      row = img->rowPointers[r];
      for (uint32_t c = 0; c < img->width; c++)
      {
          //*(row[c]) = *(png_row[c]);
          memcpy(&(row[c]), &(png_row[2*c]), 2);
      }
  }
  return 1;
}
