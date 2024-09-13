#ifndef FAST_H
#define FAST_H


template <typename PixelType, typename CicleSize, int RowStride>
void bressenham_circle(PixelType* circle[static_cast<int>(0.5 * CircleSize * M_PI)]);

template <typename Image, int CircleSize, int Threshold>
void fast(const Image& img);

#endif // FAST_H
