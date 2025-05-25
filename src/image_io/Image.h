#ifndef IMAGE_IO_IMAGE_HH
#define IMAGE_IO_IMAGE_HH

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <image_io/Pixel.h>
#include <ento-util/debug.h>
#include <type_traits>


template <int Rows, int Cols, typename PixelT>
class Image
{
public:
  static constexpr int rows_ = Rows;
  static constexpr int cols_ = Cols;
  static constexpr int bit_depth_ = sizeof(PixelT) * 8;
  using pixel_type_ = PixelT;

  PixelT data[Rows * Cols];

  Image(): data{} {}

  constexpr int rows() const { return rows_; }
  constexpr int cols() const { return cols_; }

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

  void clear()
  {
    if constexpr(std::is_integral_v<PixelT>)
    {
      std::memset(data, 0, sizeof(data));
    }
    else
    {
      for (size_t i = 0; i < static_cast<size_t>(Rows*Cols); ++i)
      {
        data[i] = PixelT{};
      }
    }
  }

  int image_from_pgm(const char* pgm_path) {
    ENTO_DEBUG("PGM path: %s\n", pgm_path);
    FILE* file = fopen(pgm_path, "r");
    ENTO_DEBUG("Successfully fopened pgm_path: %s", pgm_path);
    if (!file) {
        ENTO_ERROR("Error opening PGM file: %s", pgm_path);
        return 0;
    }

    char magic[3];
    if (fscanf(file, "%2s", magic) != 1 || magic[0] != 'P' || magic[1] != '2') {
        ENTO_ERROR("Invalid PGM format\n");
        fclose(file);
        return 0;
    }

    uint16_t width, height;
    if (fscanf(file, "%hu %hu", &width, &height) != 2 ||
        width != Cols || height != Rows) {
        ENTO_ERROR("Invalid PGM file. Expected %dx%d but got %dx%d\n", 
                Cols, Rows, width, height);
        fclose(file);
        return 0;
    }

    uint16_t maxval;
    if (fscanf(file, "%hu", &maxval) != 1) {
      ENTO_ERROR("Error reading maxval");
      fclose(file);
      return 0;
    }

    // Check if the maxval matches the expected range of PixelT
    if (maxval != ((1 << (sizeof(PixelT) * 8)) - 1)) {
      ENTO_ERROR("Error: maxval (%hu) does not match the expected bit depth for PixelT (%zu-bit)\n",
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
          ENTO_ERROR("Error reading pixel data at [%d, %d]\n", row, col);
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

////////////////////////////////////////////////////////////////////////////////
// ImageViewMode
// ============================================================================
// Enum to specify how the reference point passed to the ImageView constructor
// should be interpreted.
//
// - TopLeft: (row, col) is the top-left corner of the patch.
// - Center:  (row, col) is the center of the patch.
// ============================================================================
enum class ImageViewMode
{
  TopLeft,
  Center
};

////////////////////////////////////////////////////////////////////////////////
// ImageView
// ============================================================================
// A lightweight, non-owning view into a subregion of a statically-sized image.
//
// Template Parameters:
// - PixelT:       Type of each pixel (e.g., uint8_t, float).
// - BoundsCheck:  Enables bounds checking if true (default = false).
//
// This class is designed for memory-constrained systems like Cortex-M.
// No dynamic allocation, no virtuals, only direct pixel access.
// ============================================================================
template <typename PixelT, bool BoundsCheck = false>
class ImageView
{
public:

  // ============================================================================
  // Constructor
  // ============================================================================
  // Creates an ImageView into a subregion of a larger image buffer.
  //
  // Arguments:
  // - base_ptr:     Pointer to the full image buffer (row-major).
  // - image_cols:   Number of columns in the full image (i.e., stride).
  // - row_ref:      Reference row (top-left or center).
  // - col_ref:      Reference column (top-left or center).
  // - patch_rows:   Height of the view.
  // - patch_cols:   Width of the view.
  // - mode:         Whether (row_ref, col_ref) is TopLeft or Center.
  //
  // Notes:
  // - No bounds checking on the original image bounds.
  // - Bounds checking *within the view* is enabled if BoundsCheck = true.
  // ============================================================================
  ImageView(PixelT* base_ptr,
            int image_cols,
            int row_ref,
            int col_ref,
            int patch_rows,
            int patch_cols,
            ImageViewMode mode)
    : data_(base_ptr),
      image_cols_(image_cols),
      rows_(patch_rows),
      cols_(patch_cols)
  {
    if (mode == ImageViewMode::TopLeft)
    {
      row_offset_ = row_ref;
      col_offset_ = col_ref;
    }
    else // Center
    {
      row_offset_ = row_ref - patch_rows / 2;
      col_offset_ = col_ref - patch_cols / 2;
    }
  }

  // ============================================================================
  // operator()(row, col)
  // ============================================================================
  // Returns a reference to the pixel at (row, col) in the view.
  // Bounds checking is done if BoundsCheck is true.
  // ============================================================================
  PixelT& operator()(int row, int col)
  {
    if constexpr (BoundsCheck)
    {
      if (row < 0 || row >= rows_ || col < 0 || col >= cols_)
      {
        printf("ImageView access out of bounds at (%d, %d)\n", row, col);
        __builtin_trap();
      }
    }

    return data_[(row + row_offset_) * image_cols_ + (col + col_offset_)];
  }

  // ============================================================================
  // operator()(row, col) const
  // ============================================================================
  // Const-qualified version for read-only access to view pixels.
  // ============================================================================
  const PixelT& operator()(int row, int col) const
  {
    if constexpr (BoundsCheck)
    {
      if (row < 0 || row >= rows_ || col < 0 || col >= cols_)
      {
        printf("ImageView (const) access out of bounds at (%d, %d)\n", row, col);
        __builtin_trap();
      }
    }

    return data_[(row + row_offset_) * image_cols_ + (col + col_offset_)];
  }

  // ============================================================================
  // get_pixel
  // ============================================================================
  // Returns a copy of the pixel value at the specified row/col.
  // ============================================================================
  PixelT get_pixel(int row, int col) const
  {
    return (*this)(row, col);
  }

  // ============================================================================
  // rows
  // ============================================================================
  // Returns the number of rows in the view.
  // ============================================================================
  int rows() const
  {
    return rows_;
  }

  // ============================================================================
  // cols
  // ============================================================================
  // Returns the number of columns in the view.
  // ============================================================================
  int cols() const
  {
    return cols_;
  }

private:
  PixelT* data_;       // pointer to base image buffer
  int image_cols_;     // stride of base image (i.e., full image width)
  int row_offset_;     // top-left row index of the view
  int col_offset_;     // top-left col index of the view

public:
  using pixel_type_ = PixelT;
  int rows_;           // view height
  int cols_;           // view width
};

// ============================================================================
// make_image_view
// ----------------------------------------------------------------------------
// Returns a top-left-anchored ImageView from a mutable Image.
// ============================================================================
template <int Rows, int Cols, typename PixelT, bool BoundsCheck = false>
ImageView<PixelT, BoundsCheck>
make_image_view(Image<Rows, Cols, PixelT>& image,
                int row,
                int col,
                int view_rows,
                int view_cols)
{
  return ImageView<PixelT, BoundsCheck>(
    image.data, Cols,
    row, col,
    view_rows, view_cols,
    ImageViewMode::TopLeft
  );
}

// ============================================================================
// make_image_view (const Image version)
// ----------------------------------------------------------------------------
// Returns a top-left-anchored ImageView from a const-qualified Image.
// ============================================================================
template <int Rows, int Cols, typename PixelT, bool BoundsCheck = false>
ImageView<const PixelT, BoundsCheck>
make_image_view(const Image<Rows, Cols, PixelT>& image,
                int row,
                int col,
                int view_rows,
                int view_cols)
{
  return ImageView<const PixelT, BoundsCheck>(
    image.data, Cols,
    row, col,
    view_rows, view_cols,
    ImageViewMode::TopLeft
  );
}

// ============================================================================
// make_centered_view
// ----------------------------------------------------------------------------
// Returns a center-anchored ImageView from a mutable Image.
// ============================================================================
template <int Rows, int Cols, typename PixelT, bool BoundsCheck = false>
ImageView<PixelT, BoundsCheck>
make_centered_view(Image<Rows, Cols, PixelT>& image,
                   int center_row,
                   int center_col,
                   int view_rows,
                   int view_cols)
{
  return ImageView<PixelT, BoundsCheck>(
    image.data, Cols,
    center_row, center_col,
    view_rows, view_cols,
    ImageViewMode::Center
  );
}

// ============================================================================
// make_centered_view (const Image version)
// ----------------------------------------------------------------------------
// Returns a center-anchored ImageView from a const-qualified Image.
// ============================================================================
template <int Rows, int Cols, typename PixelT, bool BoundsCheck = false>
ImageView<const PixelT, BoundsCheck>
make_centered_view(const Image<Rows, Cols, PixelT>& image,
                   int center_row,
                   int center_col,
                   int view_rows,
                   int view_cols)
{
  return ImageView<const PixelT, BoundsCheck>(
    image.data, Cols,
    center_row, center_col,
    view_rows, view_cols,
    ImageViewMode::Center
  );
}



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
