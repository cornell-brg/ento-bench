#include "imageIO.h"

PNGImage* new_png_image()
{
    PNGImage *newImage = malloc(sizeof(PNGImage));
    if (!newImage) return NULL;
    newImage->png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!newImage->png) printf("png_create_read_struct returned NULL!");
    newImage->info = png_create_info_struct(newImage->png);
    if (!newImage->info) printf("png_create_info_struct returned NULL!");

    newImage->rowPointers = NULL;

    return newImage;
}

void read_png_file(char* file_name, PNGImage *image)
{
    FILE *fp = fopen(file_name, "rb");

    if (setjmp(png_jmpbuf(image->png))) abort();

    png_init_io(image->png, fp);

    png_read_info(image->png, image->info);

    unsigned int width = png_get_image_width(image->png, image->info);
    unsigned int height = png_get_image_height(image->png, image->info);
    png_byte colorType = png_get_color_type(image->png, image->info);

    if (colorType == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(image->png);

    if (png_get_valid(image->png, image->info, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(image->png);

    png_read_update_info(image->png, image->info);
    if (image->rowPointers) exit(1);

    image->rowPointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
    for (int row = 0; row < height; row++)
    {
        image->rowPointers[row] = (png_byte*)malloc(png_get_rowbytes(image->png, image->info));
    }

    png_read_image(image->png, image->rowPointers);

    fclose(fp);
}

void del_png_image(PNGImage *image)
{
    if (image != NULL)
    {
        if (image->png && image->info)
            png_destroy_read_struct(&(image->png),
                                    &(image->info),
                                    NULL);
        if (image->rowPointers) free(image->rowPointers);
    }
    return;
}

void display_image_info(PNGImage *image)
{
    if (!image->info) {
        printf("Image info does not exist!");
        return;
    }
    if (!image->png) {
        printf("Image pnf struct does not exists!");
        return;
    }

    png_byte width = png_get_image_width(image->png, image->info);
    png_byte height = png_get_image_height(image->png, image->info);
    png_byte color_type = png_get_color_type(image->png, image->info);
    png_byte bit_depth = png_get_bit_depth(image->png, image->info);
}

void write_png_file(char* file_name, PNGImage *image)
{
    FILE *fp = fopen(file_name, "wb");
    if (!fp) exit(1);

    if (setjmp(png_jmpbuf(image->png))) exit(1);

    png_init_io(image->png, fp);

    png_set_IHDR(
            image->png,
            image->info,
            png_get_image_width(image->png, image->info),
            png_get_image_height(image->png, image->info),
            png_get_bit_depth(image->png, image->info),
            PNG_COLOR_TYPE_GRAY,
            PNG_INTERLACE_NONE,
            PNG_COMPRESSION_TYPE_DEFAULT,
            PNG_FILTER_TYPE_DEFAULT
    );
    png_write_info(image->png, image->info);

    if (!image->rowPointers)
    {
        printf("No image data to write!");
        fclose(fp);
    }
    else
    {
        png_write_image(image->png, image->rowPointers);
        png_write_end(image->png, NULL);
        fclose(fp);
    }
}
