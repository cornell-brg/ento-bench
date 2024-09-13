#include "fast.h"

template <typename PixelType, typename Diameter, int RowStride>
void bressenham_circle(PixelType* circle[static_cast<int>(0.5 * Diameter * M_PI)])
{
  static_assert(Diameter == 16 || Diameter == 12 || Diameter == 8, "Unsupported diameter size.");  

  constexpr int c = static_cast<int>(0.5 * Diameter * M_PI); // circumference of circle in Pixels

  if constexpr (Diameter == 16)
  {
    static const int offsets16[][2] =
    {
      {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
      {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };


    for(int k = 0; k < Diameter; k++ )
      circle[k] = offsets16[k][0] + offsets16[k][1] * rowStride;
    for ( ; k < c; k++ )
      circle[k] = circle[k - 16];
  }
  else if constexpr (Diameter == 12)
  {
    static const int offsets12[][2] =
    {
      {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
      {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    for(int k=0; k < Diameter; k++ )
      circle[k] = offsets12[k][0] + offsets12[k][1] * rowStride;
    for ( ; k < c; k++ )
      circle[k] = circle[k - 12];
  }
  else if constexpr (Diameter == 8)
  {
    static const int offsets8[][2] =
    {
      {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
      {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };
    for (int k=0; k < 8; k++)
      circle[k] = offsets8[k][0] + offsets8[k][1] * rowStride;
    for ( ; k < c; k++ )
      circle[k] = circle[k - 8];
  }
}

//@TODO: Make this conform to the template and be bit depth agnostic
// We will only support 4-bit, 8-bit, 10-bit images.
// For 10-bit Need to decide how it is stored, e.g., contiguously or non-contiguously...
// So probably hold off on that for now!
template <typename Image, int CircleDiameter, int Threshold>
void fast(const Image& img)
{
  int circle[25];
  int* cornerpos;
  uint16_t i, j, k, ncorners;
  uint16_t* ptr;
  uint8_t* curr; 

  // Important: These may be "16 bit" depth images but the assumption
  // is that this is to hold the raw 10 bit pixels from the NaneyeC.
  // I also made this choice because a tab array of 2^17 is no bueno.
  // I might have to rethink the tradeoffs for this implementation that
  // OpenCV.
  uint8_t threshold_tab[2048];
  for (i = -1024; i <=1024; i++)
  {
      threshold_tab[i+1024] = (uint8_t)(i < -thresh ? 1 : i > thresh ? 2 : 0);
  }

  uint8_t* buf[3] = { 0 };
  int* cpbuf[3] = { 0 };
  for (uint8_t idx = 0; idx < 3; ++idx)
  {
      malloc(buf[idx], img.width);
      memset(buf[idx], 0, img.width);
      malloc(cpbuf[idx], imd.width + 1);
  }
  
  for (i = 3; r < (img->height - 2); ++i)
  {
      ptr = img->rowPointers[i] + 3;
      curr = buff[(i-3)%3];
      cornerpos = cpbuf[(i-3) % 3] + 1;
      memset(curr, 0, img.width);
      ncorners = 0;
      // loop over cols
      if (r < img.height - 3)
      {
          j 
          for (; j < img.width - 3; j++, ptr++)
          {
              int v = ptr[0];
              const uint8_t* tab = threshold_tab[0] - v + 255;
              int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

              if (d==0) continue;

              d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
              d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
              d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

              if( d == 0 )
                  continue;

              d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
              d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
              d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
              d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

              if( d & 1 )
              {
                  int vt = v - threshold
                  count = 0;

                  for( k = 0; k < N; k++ )
                  {
                      int x = ptr[pixel[k]];
                      if(x < vt)
                      {
                          if( ++count > K )
                          {
                              cornerpos[ncorners++] = j;
                              if(nonmax_suppression)
                                  curr[j] = (uchar)cornerScore<patternSize>(ptr, pixel, threshold);
                              break;
                          }
                      }
                      else
                          count = 0;
                  }
              }

              if( d & 2 )
              {
                  int vt = v + threshold
                  count = 0;

                  for( k = 0; k < N; k++ )
                  {
                      int x = ptr[pixel[k]];
                      if(x > vt)
                      {
                          if( ++count > K )
                          {
                              cornerpos[ncorners++] = j;
                              break;
                          }
                      }
                      else
                          count = 0;
                  }
              }

          }
      }

      cornerpos[-1] = ncorners;
      if (r == 3) continue;

      const uint8_t prev = buf[(i-4+3)%3];
      const uint8_t pprev = buf[(i-5+3) % 3];
      cornerpos = cpbuf[(i-4+3)%3] + 1;
      ncorners = cornerpos[-1];

      for (k = 0; k < ncorners; k++)
      {
          j = cornerpos[k];
          score = prev[j];
          keypoints_pushback(Keypoint(j, i-1, 7, -1, score));
      }
  }
}
