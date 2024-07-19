#include "dataset.h"

int compare_entries(const void* a, const void* b)
{
  const struct dirent *entry1 = *(const struct dirent **)a;
  const struct dirent *entry2 = *(const struct dirent **)b;
  return strcmp(entry1->d_name, entry2->d_name);
}

int image_dataset_init(ImageDataset *dataset, char* images_root, char* gt_path)
{
  if (!dataset) return 0;
  DIR *dir;
  struct dirent *entry;
  dir = opendir(images_root);
  if (dir == NULL)
  {
    perror("Opendir got NULL pointer! Is path correct?");
    return -1;
  }
  
  unsigned int entry_count = 0;
  unsigned int max_entries = 100;
  struct dirent **entries = NULL;
  entries = (struct dirent **)malloc(max_entries * sizeof(struct dirent *));
  if (entries == NULL)
  {
    perror("Could not allocate 100 dirent entries...");
    return -1;
  }

  while ((entry = readdir(dir)) != NULL)
  {
    if (entry_count >= max_entries)
    {
      max_entries *= 2;
    
      entries = (struct dirent **)realloc(entries, max_entries * sizeof(struct dirent *));
      if (entries == NULL)
      {
        perror("Could not allocate 100 dirent entries...");
        return -1;
      }
    }
    if (entry->d_type == DT_REG)
    {
      entry_count++;
      entries[entry_count] = entry;
    }
  }
  closedir(dir);

  qsort(entries, entry_count, sizeof(struct dirent *), compare_entries);
  dataset->size = entry_count;
  
}

int image_dataset_next(ImageDataset *dataset)
{
  if (!dataset) return 0;
  if (dataset->idx == dataset->size) return 0;
  else dataset->idx++;
  return 1;
}

int image_dataset_prev(ImageDataset* dataset)
{
  if (!dataset) return 0;
  if (dataset->idx == 0) return 0;
  else dataset->idx--;
  return 1;
}

int get_current_image_path(ImageDataset* dataset, char* path, int char_sz)
{
  if (!dataset) return 0;
  if (!dataset->image_paths) return 0;
  
  int res = snprintf(path, char_sz, "%s/%s", dataset->images_root, dataset->image_paths[dataset->idx]->d_name);
  if (res < 0) return 0;
  if (res >= char_sz) return 0;
  return 1;
}
