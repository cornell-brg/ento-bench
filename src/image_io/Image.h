#ifndef IMAGE_IO_IMAGE_HH
#define IMAGE_IO_IMAGE_HH

#include <cstdint>
#include <cstdio>
//#include <Eigen/Dense>
#include <image_io/Pixel.h>
#include <ento-util/debug.h>


template <int Rows, int Cols, typename PixelT>
class Image
{
public:
  static constexpr int rows = Rows;
  static constexpr int cols = Cols;
  static constexpr int bit_depth = sizeof(PixelT) * 8;
  using pixel_type = PixelT;

  PixelT data[Rows * Cols];

  Image(): data{} {}

  void set_pixel(int row, int col, PixelT pixel)
  {
    data[row * Cols + col] = pixel;
  }

  PixelT get_pixel(int row, int col) const
  {
    return data[row * Cols + col];
  }

  PixelT& operator()(int row, int col)
  {
    return data[row * Cols + col];
  }

  const PixelT& operator()(int row, int col) const
  {
    return data[row * Cols + col];
  }

  int image_from_pgm(const char* pgm_path) {
    DPRINTF("PGM path: %s\n", pgm_path);
    FILE* file = fopen(pgm_path, "r");
    DPRINTF("Successfully fopened pgm_path: %s\n", pgm_path);
    if (!file) {
        DPRINTF("Error opening file\n");
        return 0;
    }

    char magic[3];
    if (fscanf(file, "%2s", magic) != 1 || magic[0] != 'P' || magic[1] != '2') {
        DPRINTF("Invalid PGM format\n");
        fclose(file);
        return 0;
    }

    uint16_t width, height;
    if (fscanf(file, "%hu %hu", &width, &height) != 2 ||
        width != Cols || height != Rows) {
        DPRINTF("Invalid PGM file. Expected %dx%d but got %dx%d\n", 
                Cols, Rows, width, height);
        fclose(file);
        return 0;
    }

    uint16_t maxval;
    if (fscanf(file, "%hu", &maxval) != 1) {
      fprintf(stderr, "Error reading maxval\n");
      fclose(file);
      return 0;
    }

    // Check if the maxval matches the expected range of PixelT
    if (maxval != ((1 << (sizeof(PixelT) * 8)) - 1)) {
      fprintf(stderr, "Error: maxval (%hu) does not match the expected bit depth for PixelT (%zu-bit)\n",
              maxval, sizeof(PixelT) * 8);
      fclose(file);
      return 0;
    }
    // Read pixel data
    for (int row = 0; row < Rows; ++row)
    {
      for (int col = 0; col < Cols; ++col)
      {
        int pixel_value;
        if (fscanf(file, "%d", &pixel_value) != 1)
        {
          DPRINTF("Error reading pixel data at [%d, %d]\n", row, col);
          fclose(file);
          return 0;
        }
        set_pixel(row, col, static_cast<PixelT>(pixel_value));
      }
    }
    fclose(file);
    return 1;  // Success
  }
};

template <int Rows, int Cols, typename PixelT>
class DoubleBufferedImage : public Image<Rows, Cols, PixelT>
{
public:
  PixelT buffer1[Rows * Cols];
  PixelT buffer2[Rows * Cols];

  PixelT* active_buffer;
  PixelT* dma_buffer;

  // ....

  DoubleBufferedImage()
  {
    active_buffer = buffer1;
    dma_buffer = buffer2;
  }

  void swap_buffers()
  {
    PixelT* tmp = active_buffer;
    active_buffer = dma_buffer;
    dma_buffer = tmp;
  }

  PixelT& operator()(int row, int col)
  {
    return active_buffer[row * Cols + col];
  }

  PixelT& get_pixel(int row, int col)
  {
    return active_buffer[row * Cols + col];
  }

  void set_pixel(int row, int col, PixelT pixel)
  {
    active_buffer[row * Cols + col] = pixel;
  }
};


// We assume here that partitions are double buffered
template <int Rows, int Cols, int RowsPerPartition, typename PixelT>
class PartitionedImage : public DoubleBufferedImage<Rows, RowsPerPartition, PixelT>
{
public:
  static constexpr int part_size = RowsPerPartition;
  static constexpr int total_parts = Rows / RowsPerPartition;
  int part_num = 0;
};



template <int Rows, int Cols, int BitDepth, int UserBitDepth, int StartBit = 1, int StopBit = 0>
class PackedImage : public Image<Rows, Cols, typename PixelType<BitDepth, StartBit, StopBit>::type>
{
public:
  using PackedPixelT = typename PixelType<BitDepth, StartBit, StopBit>::type;
  using PacketType = typename PackedPixelT::PacketType;
  using PixelReturnType = typename std::conditional<BitDepth <= 8, uint8_t, uint16_t>::type;

  static constexpr int bit_depth = UserBitDepth;

  PackedImage() : Image<Rows, Cols, PackedPixelT>() {}

  // Operator to access pixel value at specified user bit depth
  PixelReturnType operator()(int row, int col) const
  {
    return this->data[row * Cols + col].template get_pixel<UserBitDepth>();
  }

  // Set a packed pixel value
  void set_packed_pixel(int row, int col, PackedPixelT pixel)
  {
    this->data[row * Cols + col] = pixel;
  }

  // Get raw packed data for a pixel (row, col)
  PacketType get_raw_packed_pixel(int row, int col) const
  {
    return this->data[row * Cols + col].get_packet();
  }

  // Set raw packed data directly (for packed pixel manipulation)
  void set_raw_packed_pixel(int row, int col, PacketType packet)
  {
    this->data[row * Cols + col] = PackedPixelT(packet);
  }

};




template <int Rows, int Cols, int RowsPerPartition, int BitDepth, int UserBitDepth, int StartBit = 1, int StopBit = 0>
class PartitionedPackedImage : public PackedImage<Rows, Cols, BitDepth, UserBitDepth, StartBit, StopBit>
{
public:
  using PackedPixelT = typename PixelType<BitDepth, StartBit, StopBit>::type;
  using PacketType = typename PackedPixelT::PacketType;
  using PixelReturnType = typename std::conditional<BitDepth <= 8, uint8_t, uint16_t>::type;
  
};

#endif // IMAGE_IO_IMAGE_HH
