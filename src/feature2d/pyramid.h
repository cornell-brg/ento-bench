#indef FEATURE2D_PYRAMID_H
#define FEATURE2D_PYRAMID_H

template <int KernelSize>
constexpr std::array<int, KernelSize>
generate_shift_kernel()
{
  static_assert(KernelSize == 3 || KernelSize == 5, "Unsupport kernel size");
  if constexpr (KernelSize == 3)
  {
    const int kernel[KernelSize] = { 0, 1, 0,
                                     1, 2, 1,
                                     0, 1, 0};
  }
  else if constexpr (KernelSize == 5)
  {
    const int kernel[KernelSize] = { 1,  4,  6,  4, 1,
                                     4, 16, 24, 16, 4,
                                     6, 24, 36, 24, 6,
                                     4, 16, 24, 16, 4,
                                     1,  4,  6,  4, 1 }; 
  }
}

template <typename Derived, typename ImageType, int NumLevels, int KernelSize = 9>
class BasePyramid
{
private:
  static constexpr int kernel_size = KernelSize;
  static constexpr int num_levels = Levels;
  static constexpr int shift_kernel[kernel_size];
protected:
  std::array<std::optional<ImageType>, NumLevels> levels;  // Pyramid levels

public:
  BasePyramid()
  {
    
  }

  // Build the pyramid from the top level (calls the derived class's downsample method)
  void buildPyramid()
  {
    for (int i = 1; i < NumLevels; ++i)
    {
      if (levels[i - 1])
      {
        levels[i] = static_cast<Derived*>(this)->downsample(*levels[i - 1]);
      }
    }
  }

  // Accessor for pyramid levels
  const ImageType& getLevel(uint8_t level)
  {
    // @TODO add static assert such that level does not exceed num_levels
    return *levels[level];
  }

  // Downsample method is implemented in the derived classes via CRTP
  ImageType downsample(const ImageType& img)
  {
    return static_cast<Derived*>(this)->downsample(img);
  }
};

template <typename ImageType, int NumLevels>
class Pyramid : public BasePyramid<Pyramid<ImageType, NumLevels>, ImageType, NumLevels>
{
public:
  // Constructor that initializes with the full image
  Pyramid(const ImageType& image)
  {
    this->levels[0] = full_image;  // Set the top level of the pyramid
    this->buildPyramid();  // Build the entire pyramid
  }

  // Downsampling function for normal pyramid
  ImageType downsample(const ImageType& img)
  {
    ImageType downsampled_img;
    // @TODO: Add in down sampling logic
    return downsampled_img;
  }
};

#endif // FEATURE2D_PYRAMID_H
