#include <stdio.h>
#include <png.h>
#include "imageIO.h"


typedef struct MonoImg16
{
    uint16_t **rowPointers;
    unsigned int     width;
    unsigned int    height;
} MonoImage16;

MonoImage16* new_mono_image16(uint32_t width, uint32_t height);
MonoImage16* new_mono_image16_from_png(PNGImage *img);
int          init_mono_image16_from_pgm(MonoImage16* img, char *path);
void         del_mono_image16(MonoImage16 *img);
