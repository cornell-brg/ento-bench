#include <stdio.h>
#include <png.h>
#include <stdlib.h>

#include "imageIO.h"

typedef struct MonoImg8 {
    uint8_t **rowPointers;
    unsigned int    width;
    unsigned int   height;
} MonoImage8;
MonoImage8* new_mono_image8();
MonoImage8* new_mono_image8_from_png(PNGImage *img);
void del_mono_image8(MonoImage8 *img);

template<typename t>
void del_mono_image8(t *img);
