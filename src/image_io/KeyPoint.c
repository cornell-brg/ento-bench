#include "KeyPoint.h"

KeyPoint* new_keypoint(uint16_t x, uint16_t y, uint16_t response, uint8_t octave)
{
    KeyPoint *kpp = malloc(sizeof(KeyPoint));
    if (!kpp) return NULL;
    kpp->x = x;
    kpp->y = y;
    kpp->response = response;
    kpp->octave = octave;
    return kpp;
}

void del_keypoint(KeyPoint* kpp)
{
    if (!kpp) return;
    free((void *) kpp);
}

KeyPointList* new_keypoint_list(uint16_t size)
{
    KeyPointList *list = malloc(sizeof(KeyPointList));
    if (!list) return NULL;
    list->ptrs = malloc(size * sizeof(KeyPoint));
    if (!list->ptrs) return NULL;
    list->size = size;
    list->numElements = 0;
    return list;
}

void del_keypoint_list(KeyPointList* list)
{
    if (!list) return;
    if (list->ptrs) free((void *)list->ptrs);
    free((void *)list);
}