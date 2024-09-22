#ifndef FAST_H
#define FAST_H

#include <cstdint>

#include <numbers>
#include "feature2d/util.h"
#include "ento-util/debug.h"

constexpr float PI = std::numbers::pi_v<float>;

template <int BitDepth, int Threshold>
struct ThresholdTable {
  static constexpr int tab_size = (1 << (BitDepth + 1)) + 1;

  static constexpr auto create()
  {
    std::array<uint8_t, tab_size> table = {};
    constexpr int half = tab_size / 2;
    for (int i = -half; i <= half; ++i)
    {
      table[i + half] = (i < -Threshold ? 1 : i > Threshold ? 2 : 0);
    }
    return table;
  }

  static constexpr std::array<uint8_t, tab_size> table = create();
};

template <typename PixelType, int PatternSize, int RowStride, int ContiguityRequirement = 9>
constexpr std::array<PixelType, PatternSize + ContiguityRequirement>
generate_bresenham_circle();

template <typename Image,
          int PatternSize,
          int Threshold,
          int ContiguityRequirement = 9,
          size_t MaxFeatures = 100>
void fast(const Image& img, FeatureDetectorOutput<FastKeypoint, MaxFeatures>& fdo );


// ===========================================================
// Implementations
// ===========================================================

template <typename PixelType, int PatternSize, int RowStride, int ContiguityRequirement>
constexpr std::array<PixelType, PatternSize + ContiguityRequirement>
generate_bresenham_circle() {
  static_assert(PatternSize == 16 || PatternSize == 12 || PatternSize == 8, "Unsupported diameter size.");

  constexpr int offsets_size = PatternSize + ContiguityRequirement;  // Circle circumference

  std::array<PixelType, offsets_size> circle_offsets{};
  int k = 0;

  if constexpr (PatternSize == 16) {
    const int offsets16[][2] = {
      {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
      {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    for ( ; k < PatternSize; k++) {
      circle_offsets[k] = static_cast<PixelType>(offsets16[k][0] + offsets16[k][1] * RowStride);
    }
  }
  else if constexpr (PatternSize == 12) {
    const int offsets12[][2] = {
      {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
      {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    for ( ; k < PatternSize; k++) {
      circle_offsets[k] = static_cast<PixelType>(offsets12[k][0] + offsets12[k][1] * RowStride);
    }
  }
  else if constexpr (PatternSize == 8) {
    const int offsets8[][2] = {
      {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
      {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    for ( ; k < PatternSize; k++) {
      circle_offsets[k] = static_cast<PixelType>(offsets8[k][0] + offsets8[k][1] * RowStride);
    }
  }
  for ( ; k < PatternSize + ContiguityRequirement; ++k)
  {
    circle_offsets[k] = circle_offsets[k - PatternSize];
  }

  return circle_offsets;
}

//@TODO: Make this conform to the template and be bit depth agnostic
// We will only support 4-bit, 8-bit, 10-bit images.
// For 10-bit Need to decide how it is stored, e.g., contiguously or non-contiguously...
// So probably hold off on that for now!
template <typename ImageType,
          int PatternSize,
          int Threshold,
          int ContiguityRequirement,
          size_t MaxFeatures>
void fast(const ImageType& img, FeatureDetectorOutput<FastKeypoint, MaxFeatures>& fdo)
{
  // Compile-time access to number of columns
  constexpr int img_width   = ImageType::cols;
  constexpr int img_height  = ImageType::rows;
  constexpr int max_dim     = (img_width > img_height) ? img_width : img_height;

  DPRINTF("[FAST] img W, H: (%i, %i)\n", img_width, img_height);

  // Extract pixel type info
  using PixelType = ImageType::pixel_type;
  constexpr int bit_depth = ImageType::bit_depth;
  constexpr int middle_value = (1 << (bit_depth + 1)) / 2;


  // Coordinate type
  using CoordType = typename std::conditional<
      (max_dim <= 255), uint8_t, typename std::conditional<
        (max_dim <= 65535), uint16_t, uint32_t
      >::type
    >::type;

  using CircleType = typename std::conditional<
      (max_dim <= 255), int16_t, int32_t>::type;

  // Create a circle of offsets
  constexpr int circle_buff_sz = PatternSize + ContiguityRequirement;
  //static PixelType circle[circle_buff_sz];
  //bressenham_circle<PixelType, CircleDiameter, img_width>(circle);
  static constexpr auto circle = generate_bresenham_circle<CircleType, PatternSize, img_width, ContiguityRequirement>();
  //DPRINTF("[FAST] Circle size: %i\n", circle_buff_sz);
  //DPRINTF("[FAST] Circle offsets: ");
  const PixelType* ptemp = &img.data[3*img_width] + 3;
  //DPRINTF("[FAST] ptemp[0]: %i\n", ptemp[0]);
  for (int i = 0; i < circle_buff_sz; ++i)
  {
    DPRINTF("Pixel[%i]: %i\n", circle[i], ptemp[circle[i]]);
  }
  DPRINTF("\n");



  CoordType* cornerpos;
  CoordType i, j, k, ncorners;
  const PixelType* ptr;
  PixelType* curr; 

  // Important: These may be "16 bit" depth images but the assumption
  // is that this is to hold the raw 10 bit pixels from the NaneyeC.
  // I also made this choice because a tab array of 2^17 is no bueno.
  // I might have to rethink the tradeoffs for this implementation that
  // OpenCV.
  constexpr auto threshold_tab = ThresholdTable<bit_depth, Threshold>::table;
  constexpr int tab_size = (1 << (bit_depth + 1));
  constexpr int half = tab_size / 2;
  
  DPRINTF("[FAST] Threshold table[%i]: \n", tab_size);
  for (int i = 0; i < tab_size/16; ++i)
  {
    for (int j = 0; j < 16; ++j )
    {
      if (i * 16 + j == half)
      {
        DPRINTF(" {%i} ", threshold_tab[i*16+j]);
      }
      else
      {
        DPRINTF("  %i  ", threshold_tab[i*16 + j]);
      }
    }
    DPRINTF("\n");
  }
  DPRINTF("\n");

  static PixelType buff1[img_width];
  static PixelType buff2[img_width];
  static PixelType buff3[img_width];

  static CoordType cpbuff1[img_width+1];
  static CoordType cpbuff2[img_width+1];
  static CoordType cpbuff3[img_width+1];

  static PixelType* buf[3] = { buff1, buff2, buff3 };
  static CoordType* cpbuf[3] = { cpbuff1, cpbuff2, cpbuff3 };
  
  for (i = 3; i < (img_height - 2); ++i)
  {
    ptr = &img.data[i*img_width] + 3;
    curr = buf[(i-3)%3];
    cornerpos = cpbuf[(i-3) % 3] + 1;
    std::fill(curr, curr+img_width, 0);
    ncorners = 0;

    if (i < img_height - 3)
    {
      j = 3;
      for (; j < img_width - 3; j++, ptr++)
      {
        DPRINTF("[FAST] Current Pixel Coord for test: (%i, %i)\n", i, j);
        int v = ptr[0];
        DPRINTF("[FAST] Pixel Value: (%i)\n", v);

        const PixelType* tab = &threshold_tab[0] - v + middle_value;
        DPRINTF("[FAST] &threshold_tab[0], minus v, minus v + middle: %u, %u, %u\n", &threshold_tab[0], &threshold_tab[0] - v, tab);
        DPRINTF("[FAST] &threshold_tab[half]: %u", &threshold_tab[half]);
        int d = tab[ptr[circle[0]]] | tab[ptr[circle[8]]];

        DPRINTF("[FAST] circle[0], circle[8]: %i, %i\n", circle[0], circle[8]);
        DPRINTF("[FAST] ptr[circle[0]], ptr[circle[8]]: %i, %i\n", ptr[circle[0]], ptr[circle[8]]);
        DPRINTF("[FAST] tab[ptr[circle[0]]], tab[ptr[circle[8]]: %i, %i\n", tab[ptr[circle[0]]], tab[ptr[circle[8]]]);


        DPRINTF("[FAST] First test d: %i\n", d);
        if ( d==0 ) continue;

        d &= tab[ptr[circle[2]]] | tab[ptr[circle[10]]];
        d &= tab[ptr[circle[4]]] | tab[ptr[circle[12]]];
        d &= tab[ptr[circle[6]]] | tab[ptr[circle[14]]];

        DPRINTF("[FAST] circle[2], circle[10]: %i, %i\n", circle[2], circle[10]);
        DPRINTF("[FAST] ptr[circle[2]], ptr[circle[10]]: %i, %i\n", ptr[circle[2]], ptr[circle[10]]);
        DPRINTF("[FAST] circle[4], circle[12]: %i, %i\n", circle[4], circle[12]);
        DPRINTF("[FAST] ptr[circle[4]], ptr[circle[12]]: %i, %i\n", ptr[circle[4]], ptr[circle[12]]);
        DPRINTF("[FAST] circle[6], circle[14]: %i, %i\n", circle[6], circle[14]);
        DPRINTF("[FAST] ptr[circle[6]], ptr[circle[14]]: %i, %i\n", ptr[circle[6]], ptr[circle[14]]);
        DPRINTF("[FAST] Second test d: %i\n", d);
        if( d == 0 )
            continue;

        d &= tab[ptr[circle[1]]] | tab[ptr[circle[9]]];
        d &= tab[ptr[circle[3]]] | tab[ptr[circle[11]]];
        d &= tab[ptr[circle[5]]] | tab[ptr[circle[13]]];
        d &= tab[ptr[circle[7]]] | tab[ptr[circle[15]]];

        DPRINTF("[FAST] third test d: %i\n", d);
        if( d & 1 )
        {
          int vt = v - Threshold;
          int count = 0;

          for( k = 0; k < circle_buff_sz; k++ )
          {
            int x = ptr[circle[k]];
            if(x < vt)
            {
              if( ++count > ContiguityRequirement )
              {
                cornerpos[ncorners++] = j;
                //if(nonmax_suppression)
                    //curr[j] = (uchar)cornerScore<patternSize>(ptr, pixel, threshold);
                break;
              }
            }
            else
            {
              count = 0;
            }
          }
        }

        if( d & 2 )
        {
          int vt = v + Threshold;
          int count = 0;

          for( k = 0; k < circle_buff_sz; k++ )
          {
            int x = ptr[circle[k]];
            if(x > vt)
            {
              if( ++count > ContiguityRequirement )
              {
                cornerpos[ncorners++] = j;
                break;
              }
            }
            else
            {
              count = 0;
            }
          }
        }

      }
    }

    cornerpos[-1] = ncorners;
    if (i == 3) continue;

    const PixelType* prev = buf[(i-4+3)%3];
    //const PixelType* pprev = buf[(i-5+3) % 3];
    cornerpos = cpbuf[(i-4+3)%3] + 1;
    ncorners = cornerpos[-1];

    for (k = 0; k < ncorners; k++)
    {
      j = cornerpos[k];
      int score = prev[j];
      FastKeypoint kp(j, i-1, score);
      DPRINTF("Found feature: %i, %i, %i", j, i-1, score);
      fdo.add_keypoint(kp);
    }
  }
}

#endif // FAST_H
