#ifndef DATASET_H
#define DATASET_H

#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/syslimits.h>
#include <dirent.h>

typedef struct {
  char* images_root;
  char* gt_path;
  char* current_image_path;
  struct dirent **image_paths;
  unsigned int idx;
  unsigned int size;
} ImageDataset;

int image_dataset_init(ImageDataset* dataset,
                       char* images_root,
                       char* gt_path);
int image_dataset_next(ImageDataset* dataset);
int image_dataset_prev(ImageDataset* dataset);
int get_current_image_path(ImageDataset* dataset, char* path, int char_sz);

#endif
