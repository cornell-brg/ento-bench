#include "MonoImage8.h"

MonoImage8* new_mono_image8()
{
    MonoImage8 *mono_image = malloc(sizeof(MonoImage8));
    if (!mono_image) return NULL;
    mono_image->rowPointers = NULL;
    mono_image->width = 0;
    mono_image->height = 0;
    return mono_image;
}

MonoImage8* new_mono_image8_from_png(PNGImage *image)
{
    if (!image->info)
    {
        printf("Image info does not exist!");
        return NULL;
    }

    if (!image->png)
    {
        printf("Image png struct does not exists!");
        return NULL;
    }

    if (png_get_bit_depth(image->png, image->info) != 8)
    {
        printf("PNG Image is not 8 bit depth!");
        return NULL;
    }

    unsigned int width = png_get_image_width(image->png, image->info);
    unsigned int height = png_get_image_height(image->png, image->info);

    MonoImage8 *mono_image = malloc(sizeof(MonoImage8));
    if (!mono_image) return NULL;
    mono_image->width = width;
    mono_image->height = height;
    mono_image->rowPointers = malloc(height*sizeof(uint8_t *));
    
    mono_image->rowPointers[0] = malloc(height*width*sizeof(uint8_t));
    for(int i = 0; i < height; i++)
    {
        mono_image->rowPointers[i] = mono_image->rowPointers[0] + (i * width);
    }
    
    png_bytep png_row;
    uint8_t *row;
    for(int r = 0; r < height; r++)
    {
        png_row = image->rowPointers[r];
        row = mono_image->rowPointers[r];
        for (int c = 0; c < width; c++)
        {
            //*(row[c]) = *(png_row[c]);
            memcpy(&(row[c]), &(png_row[c]), 1);
        }
    }
    return mono_image;
}

void del_mono_image8(MonoImage8 *img)
{
    if (!img) return;
    if (img->rowPointers)
    {
        if (img->rowPointers[0]) free((void *)img->rowPointers[0]);
        free((void *)img->rowPointers);
    }
    free((void *)img);
    return;
}
