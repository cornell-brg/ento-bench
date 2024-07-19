#include "NaneyeImage.h"

NaneyeImage* new_naneye_image()
{
  NaneyeImage* img = malloc(sizeof(NaneyeImage));
  if (!img) return NULL;
  img->width = (uint16_t) 320;
  img->height = (uint16_t) 320;
  img->data = malloc(img->width * img->height * sizeof(uint16_t));
  if (!img->data) return NULL;
  return img;
}

void del_naneye_image(NaneyeImage *img)
{
  if (!img) return;
  if (img->data) free((void *)img->data);
  free((void *)img);
}

/*int naneye_image_from_mono_image16(NaneyeImage *naneye_img, MonoImage16 *mono_img)
{
  if (!naneye_img || !naneye_img->data) return 0;
  if (!mono_img || !mono_img->rowPointers || !mono_img->rowPointers[0]) return 0;
  
  if (mono_img->width != naneye_img->width || mono_img->height != naneye_img->height) return 0;
  uint16_t* row;
  uint16_t pp;
  uint16_t row_stride = naneye_img->width;
  for (int r = 0; r < naneye_img->height; r++)
  {
    row = mono_img->rowPointers[r];
    for (int c = 0; c < naneye_img->width; c++)
    {
      // Pack pixel in Naneye Pixel packet
      pp = row[c] << 1;
      pp &= 0xF7FF;
      pp |= 0x0001;
      naneye_img->data[r*row_stride + c] = pp;
    }
  }
  return 1;
}*/

int naneye_image_from_pgm(NaneyeImage *naneye_img, const char *pgm_path)
{
  if (!naneye_img || !naneye_img->data) return 0;
  uint16_t* img_buff = naneye_img->data;

  printf("pgm path: %s", pgm_path);
  FILE* file = fopen(pgm_path, "r");
  if (!file) {
      perror("Error opening file");
      return 0;
  }

  char magic[3];
  if (fscanf(file, "%2s", magic) != 1 || magic[0] != 'P' || magic[1] != '2') {
      fprintf(stderr, "Invalid PGM format\n");
      fclose(file);
      return 0;
  }

  uint16_t width, height;
  if (fscanf(file, "%hu %hu", &width, &height) != 2 || width != naneye_img->width || height != naneye_img->height)
  {
    fprintf(stderr, "Invalid PGM file. Are you sure this is a NaneyeImage?");
    fclose(file);
    return 0;
  }

  uint16_t maxval;
  if (fscanf(file, "%hu", &maxval) != 1) {
      fprintf(stderr, "Invalid PGM header\n");
      fclose(file);
      return 0;
  }

  if (maxval != 65535) {
      fprintf(stderr, "Only 16-bit PGM images are supported\n");
      fclose(file);
      return 0;
  }

  int status;
  uint32_t max_idx = 320 * 320;
  uint16_t pixel;
  uint16_t pp;
  for (uint32_t i = 0; i < max_idx; i++) {
      status = fscanf(file, "%hu", &pixel);
      if (status != 1) {
          fprintf(stderr, "Error reading image data. Failed at index %i\n", i);
          fclose(file);
          return 0;
      }
      pp = pixel << 1;
      pp &= 0xF7FF;
      pp |= 0x0001;
      naneye_img->data[i] = pp;
  }

  fclose(file);
  return 1;
}

void raw_naneye_buffer_init(RawNaneyeBuffer *pbuff,
                            NaneyeImage* image,
                            BufferParams* params)
{
  pbuff->image = image;
  pbuff->buffer_num = -1;
  pbuff->frame_num = 0;
  pbuff->params = params;
  pbuff->data = image->data;
}

void buffer_params_init(BufferParams* pparams,
                        uint32_t rows_per_part,
                        uint32_t parts_per_img,
                        uint32_t row_stride,
                        uint32_t image_size)
{
  pparams->rows_per_part = rows_per_part;
  pparams->parts_per_img = parts_per_img;
  pparams->row_stride = row_stride;
  pparams->image_size = image_size;
}

int next_partition(RawNaneyeBuffer *pbuff)
{
  pbuff->buffer_num++;
  if (pbuff->buffer_num == pbuff->params->parts_per_img) return 0;
  uint32_t row_index = pbuff->params->row_stride * pbuff->params->rows_per_part * pbuff->buffer_num;
  pbuff->data = &pbuff->image->data[row_index];
  return 1;
}

int next_image(RawNaneyeBuffer* pbuff, NaneyeImage* pimg)
{
  if (!pimg->data) return 0;
  pbuff->buffer_num = -1;
  pbuff->frame_num++;
  pbuff->image = pimg;
  pbuff->data = pimg->data;
  return 1;
}

int update_pixel_buffer(RawNaneyeBuffer* buff, NaneyeImage* img)
{
  uint32_t row_idx = buff->buffer_num * buff->params->rows_per_part;
  uint32_t num_bytes = 2 * buff->params->rows_per_part * buff->params->row_stride;
  memcpy((void*) buff->data, (void*) &img->data[row_idx], num_bytes);
  if (buff->buffer_num == buff->params->parts_per_img) {
    buff->buffer_num = 0;
    buff->frame_num++;
    return 1;
  } else {
    return 0;
  }
}
