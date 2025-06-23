#ifndef IMAGE_UTIL_H
#define IMAGE_UTIL_H

#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <array>
#include <limits>

//------------------------------
// Factorial
//------------------------------
namespace EntoMath
{
template<typename Integer>
constexpr auto factorial(Integer n) -> Integer
{
  Integer res = 1;
  for (Integer i = 2; i <= n; ++i)
  {
    res *= i;
  }
  return res;
}

//------------------------------
// Power: exponentiation by squaring
//------------------------------
template<typename T, typename Unsigned>
constexpr auto pow_helper(T x, Unsigned exponent)
    -> std::common_type_t<T, Unsigned>
{
  return (exponent == 0) ? 1 :
         (exponent % 2 == 0) ? pow_helper(x * x, exponent / 2) :
         x * pow_helper(x * x, (exponent - 1) / 2);
}

template<typename Number, typename Integer>
constexpr auto pow(Number x, Integer exponent)
    -> std::common_type_t<Number, Integer>
{
  static_assert(std::is_integral<Integer>::value, "pow only accepts integer exponents");
  return (exponent == 0) ? 1 :
         (exponent > 0) ? pow_helper(x, exponent) :
         1 / pow_helper(x, -exponent);
}

//------------------------------
// x^n / n!
//------------------------------
template<typename T>
constexpr auto xn_nfac(T x, int n) -> T
{
  return pow(x, n) / factorial(n);
}

//------------------------------
// constexpr exp using a Taylor series approximation
//------------------------------
constexpr std::size_t exp_max_depth = 10;

template<std::size_t N>
struct exponential
{
  static_assert(N < exp_max_depth, "exceeded maximum recursion depth");

  template<typename T>
  static constexpr T compute(T x)
  {
    const T epsilon = std::numeric_limits<T>::epsilon() * x;
    const T term = xn_nfac(x, static_cast<int>(N));
    return (term < epsilon) ? term : term + exponential<N + 1>::compute(x);
  }
};

template<>
struct exponential<exp_max_depth>
{
  template<typename T>
  static constexpr T compute(T)
  {
    return T();
  }
};

template<std::size_t N, typename T>
constexpr T exp_helper(T x)
{
  return exponential<N>::compute(x);
}

template<typename Float>
constexpr auto exp(Float x) -> decltype(std::exp(x))
{
  return (x < 0.0) ?
      1.0 / exp(-x) :  // For negative x, use reciprocal.
      1 + x + exp_helper<2>(x);  // For x >= 0, use 1 + x + series starting at n=2.
}
} // namespace EntoMath

//------------------------------
// Gaussian Kernel Generator
//------------------------------
template <typename ScalarT, int N>
constexpr std::array<ScalarT, N> make_gaussian_kernel()
{
  constexpr ScalarT sigma = 2.0f;
  std::array<ScalarT, N> kernel{};
  constexpr int half = N / 2;
  ScalarT sum = 0.0;
  for (int i = 0; i < N; ++i)
  {
    int x = i - half;
    // Use our constexpr exp for computing the Gaussian weight.
    kernel[i] = EntoMath::exp(- (x * x) / (2 * sigma * sigma));
    sum += kernel[i];
  }
  // Normalize the kernel so that the sum is 1.
  for (auto& w : kernel)
  {
    w /= sum;
  }
  return kernel;
}

template <typename ScalarT, int N>
std::array<ScalarT, N> make_gaussian_kernel(ScalarT sigma)
{
  std::array<ScalarT, N> kernel{};
  constexpr int half = N / 2;
  ScalarT sum = 0;
  for (int i = 0; i < N; ++i)
  {
    ScalarT x = static_cast<ScalarT>(i - half);
    kernel[i] = std::exp(- (x * x) / (2 * sigma * sigma));
    sum += kernel[i];
  }
  // Normalize the kernel so that the sum is 1.
  for (auto& w : kernel)
  {
    w /= sum;
  }
  return kernel;
}

template <typename T>
std::vector<T> make_gauss_kernel(int N, T sigma) {
  int half = N/2;
  std::vector<T> K(N);
  T sum = 0;
  for (int i = 0; i < N; ++i) {
    T x = static_cast<T>(i - half);
    K[i] = std::exp(- (x*x) / (2 * sigma * sigma));
    sum += K[i];
  }
  for (auto &v : K) v /= sum;
  return K;
}

//------------------------------
// Variable Size Gaussian Kernel (for SIFT++)
//------------------------------
struct VariableKernel {
    static constexpr int MaxSize = 17;  // Max kernel size needed for SIFT++
    std::array<float, MaxSize> data;
    int half_width;  // W value (actual kernel size is 2*W+1)
    
    // Convenience methods
    int size() const { return 2 * half_width + 1; }
    float& operator[](int i) { return data[i]; }
    const float& operator[](int i) const { return data[i]; }
    
    // Iterator support for range-based loops
    float* begin() { return data.data(); }
    float* end() { return data.data() + size(); }
    const float* begin() const { return data.data(); }
    const float* end() const { return data.data() + size(); }
};

template<typename T>
VariableKernel make_variable_gaussian_kernel(T sigma) {
    VariableKernel kernel;
    kernel.half_width = static_cast<int>(std::ceil(4.0f * sigma));  // SIFT++ formula: W = ceil(4*sigma)
    
    // Generate kernel values
    T sum = 0;
    for(int j = 0; j < kernel.size(); ++j) {
        int offset = j - kernel.half_width;
        kernel.data[j] = std::exp(-0.5f * offset * offset / (sigma * sigma));
        sum += kernel.data[j];
    }
    
    // Normalize to sum to 1
    for(int j = 0; j < kernel.size(); ++j) {
        kernel.data[j] /= sum;
    }
    
    return kernel;
}

//------------------------------
// SIFT++ Exact Convolution Functions
//------------------------------
template<typename T>
void sift_convolve_transpose(T* dst_pt, const T* src_pt, int M, int N, 
                            const float* filter_pt, int W) {
    // Exact SIFT++ convolve implementation
    // convolve along columns, save transpose
    // image is M by N 
    // buffer is N by M 
    // filter is (2*W+1) by 1
    for(int j = 0; j < N; ++j) {
        int i = 0;

        // top
        for(; i <= std::min(W-1, M-1); ++i) {
            const T* start = src_pt;
            const T* stop = src_pt + std::min(i+W, M-1) + 1;
            const float* g = filter_pt + W-i;
            T acc = 0.0;
            while(stop != start) acc += (*g++) * (*start++);
            *dst_pt = acc;
            dst_pt += N;
        }

        // middle
        // run this for W <= i <= M-1-W, only if M >= 2*W+1
        for(; i <= M-1-W; ++i) {
            const T* start = src_pt + i-W;
            const T* stop = src_pt + i+W + 1;
            const float* g = filter_pt;
            T acc = 0.0;
            while(stop != start) acc += (*g++) * (*start++);
            *dst_pt = acc;
            dst_pt += N;
        }

        // bottom
        // run this for M-W <= i <= M-1, only if M >= 2*W+1
        for(; i <= M-1; ++i) {
            const T* start = src_pt + i-W;
            const T* stop = src_pt + std::min(i+W, M-1) + 1;
            const float* g = filter_pt;
            T acc = 0.0;
            while(stop != start) acc += (*g++) * (*start++);
            *dst_pt = acc;
            dst_pt += N;
        }
        
        // next column
        src_pt += M;
        dst_pt -= M*N - 1;
    }
}

template <typename ImageT>
void sift_smooth(const ImageT& src, ImageT& dst, float sigma) {
    // Exact SIFT++ implementation
    constexpr int width = ImageT::cols_;
    constexpr int height = ImageT::rows_;
    
    // Prepare filter buffer (SIFT++ approach)
    int W = static_cast<int>(std::ceil(4.0f * sigma));
    std::vector<float> filter(2 * W + 1);
    
    // Filter shape (SIFT++ kernel generation)
    for (int j = 0; j < 2 * W + 1; ++j) {
        int offset = j - W;
        filter[j] = std::exp(-0.5f * offset * offset / (sigma * sigma));
    }
    
    // Sum to one (SIFT++ normalization)
    float acc = 0.0f;
    for (int j = 0; j < 2 * W + 1; ++j) {
        acc += filter[j];
    }
    for (int j = 0; j < 2 * W + 1; ++j) {
        filter[j] /= acc;
    }
    
    // Create temporary buffer for separable convolution
    std::vector<float> temp(height * width);
    
    // First pass: convolve along columns, save transpose (SIFT++ approach)
    sift_convolve_transpose(temp.data(), src.data, width, height, filter.data(), W);
    
    // Second pass: convolve along columns of transposed image (which are rows of original)
    sift_convolve_transpose(dst.data, temp.data(), height, width, filter.data(), W);
}

// Memory-efficient version using compile-time ring buffer with SIFT++ boundary handling
// MaxKernelSize should be large enough for expected sigma values (e.g., 17 for sigma up to ~4.0)
template <typename ImageT, int MaxKernelSize = 17>
void sift_smooth_efficient(const ImageT& src, ImageT& dst, float sigma) {
    //constexpr int width = ImageT::cols_;
    //constexpr int height = ImageT::rows_;
    int width = src.cols();
    int height = src.rows();
    using PixelT = typename ImageT::pixel_type_;
    
    // Generate SIFT++ style kernel
    int W = static_cast<int>(std::ceil(4.0f * sigma));
    int kernel_size = 2 * W + 1;
    
    // Ensure kernel size doesn't exceed our compile-time limit
    if (kernel_size > MaxKernelSize) {
        // Fallback: truncate kernel size to maximum
        kernel_size = MaxKernelSize;
        W = MaxKernelSize / 2;
    }
    
    // Compile-time kernel array
    float kernel[MaxKernelSize];
    
    // Generate kernel values (SIFT++ approach)
    for (int j = 0; j < kernel_size; ++j) {
        int offset = j - W;
        kernel[j] = std::exp(-0.5f * offset * offset / (sigma * sigma));
    }
    
    // Normalize to sum to 1
    float sum = 0.0f;
    for (int j = 0; j < kernel_size; ++j) {
        sum += kernel[j];
    }
    for (int j = 0; j < kernel_size; ++j) {
        kernel[j] /= sum;
    }
    
    // Compile-time ring buffer for horizontal blur results
    // Only stores MaxKernelSize rows instead of full image
    float ring[MaxKernelSize][width];
    
    // Process each row
    for (int row = 0; row < height; ++row) {
        int ring_idx = row % kernel_size;
        
        // Horizontal blur for this row into ring buffer
        for (int col = 0; col < width; ++col) {
            float acc = 0.0f;
            
            // SIFT++ horizontal convolution with truncation
            for (int k = 0; k < kernel_size; ++k) {
                int src_col = col + k - W;
                // Truncate at boundaries (SIFT++ style)
                if (src_col >= 0 && src_col < width) {
                    acc += static_cast<float>(src(row, src_col)) * kernel[k];
                }
                // Note: pixels outside boundary contribute 0 (truncation/zero-padding)
            }
            ring[ring_idx][col] = acc;
        }
        
        // If we have enough rows filled, do vertical blur for center row
        if (row >= W) {
            int dst_row = row - W;
            
            for (int col = 0; col < width; ++col) {
                float acc = 0.0f;
                
                // SIFT++ vertical convolution with truncation
                for (int k = 0; k < kernel_size; ++k) {
                    int src_row = dst_row + k - W;
                    // Truncate at boundaries (SIFT++ style)
                    if (src_row >= 0 && src_row < height) {
                        int src_ring_idx = src_row % kernel_size;
                        acc += ring[src_ring_idx][col] * kernel[k];
                    }
                    // Note: pixels outside boundary contribute 0 (truncation/zero-padding)
                }
                dst(dst_row, col) = static_cast<PixelT>(acc);
            }
        }
    }
    
    // Handle final bottom rows (tail processing)
    for (int tail = 0; tail < W; ++tail) {
        int dst_row = height - W + tail;
        if (dst_row >= height) break;
        
        for (int col = 0; col < width; ++col) {
            float acc = 0.0f;
            
            // SIFT++ vertical convolution with truncation
            for (int k = 0; k < kernel_size; ++k) {
                int src_row = dst_row + k - W;
                // Truncate at boundaries (SIFT++ style)
                if (src_row >= 0 && src_row < height) {
                    int src_ring_idx = src_row % kernel_size;
                    acc += ring[src_ring_idx][col] * kernel[k];
                }
                // Note: pixels outside boundary contribute 0 (truncation/zero-padding)
            }
            dst(dst_row, col) = static_cast<PixelT>(acc);
        }
    }
}

template <typename KernelT, int KernelSize>
constexpr std::array<KernelT, KernelSize> get_fixed_gaussian_kernel()
{
  if constexpr (KernelSize == 3)
  {
    return { 0.25, 0.5, 0.25 };
  }
  else if constexpr (KernelSize == 5)
  {
    //return {0.15246914, 0.22184130, 0.25137912, 0.22184130, 0.15246914};
    return { 0.0625, 0.25  , 0.375 , 0.25  , 0.0625 };
  }
  else if constexpr (KernelSize == 7)
  {
    // return {0.07015933, 0.13107488, 0.19071282, 0.21610594, 0.19071282, 0.13107488, 0.07015933};
    return { 0.03125 , 0.109375, 0.21875 , 0.28125 , 0.21875 , 0.109375, 0.03125 };
  }
  else
  {
    static_assert(KernelSize == 3 || KernelSize == 5 || KernelSize == 7,
                  "Unsupported kernel size");
    return {}; // Dummy to satisfy compiler; static_assert will fire for other sizes.
  }
}

enum class GaussianCastMode
{
  Truncate,
  Round
};

template <typename ImageT, int KernelSize, typename KernelT = float,
          GaussianCastMode CastMode = GaussianCastMode::Round>
void gaussian_blur_in_place(ImageT& image)
{
  static_assert(KernelSize % 2 == 1, "Kernel size must be odd");
  constexpr int rows = ImageT::rows_;
  constexpr int cols = ImageT::cols_;
  constexpr int half = KernelSize / 2;
  constexpr auto kernel = get_fixed_gaussian_kernel<KernelT, KernelSize>();

  auto reflect = [](int x, int max) -> int {
    if (x < 0)
      return -x;
    else if (x >= max)
      return 2 * max - x - 2;
    else
      return x;
  };

  // Ring buffer of KernelSize float rows (each row is cols-wide)
  KernelT ring[KernelSize][cols];

  for (int row = 0; row < rows; ++row)
  {
    // Horizontal blur for this row into ring buffer
    for (int col = 0; col < cols; ++col)
    {
      KernelT sum = 0.0f;
      for (int k = 0; k < KernelSize; ++k)
      {
        int offset = k - half;
        int idx = reflect(col + offset, cols);
        sum += static_cast<KernelT>(image(row, idx)) * kernel[k];
      }
      ring[row % KernelSize][col] = sum;
    }

    // If we have enough rows filled, do vertical blur for center row
    if (row >= half)
    {
      int dst_row = row - half;
      for (int col = 0; col < cols; ++col)
      {
        KernelT sum = 0.0f;
        for (int k = 0; k < KernelSize; ++k)
        {
          int ring_row = reflect(dst_row + k - half, rows) % KernelSize;
          sum += ring[ring_row][col] * kernel[k];
        }
        if constexpr (CastMode == GaussianCastMode::Round &&
                      std::is_integral_v<typename ImageT::pixel_type_>)
        {
          image(dst_row, col) = static_cast<typename ImageT::pixel_type_>(sum + 0.5f);
        }
        else
        {
          image(dst_row, col) = static_cast<typename ImageT::pixel_type_>(sum);
        }
      }
    }
  }

  // Handle final bottom rows (tail)
  for (int tail = 0; tail < half; ++tail)
  {
    int dst_row = rows - half + tail;
    if (dst_row >= rows)
      break;

    for (int col = 0; col < cols; ++col)
    {
      KernelT sum = 0.0f;
      for (int k = 0; k < KernelSize; ++k)
      {
        int ring_row = reflect(dst_row + k - half, rows) % KernelSize;
        sum += ring[ring_row][col] * kernel[k];
      }
      if constexpr (CastMode == GaussianCastMode::Round &&
                    std::is_integral_v<typename ImageT::pixel_type_>)
      {
        image(dst_row, col) = static_cast<typename ImageT::pixel_type_>(sum + 0.5f);
      }
      else
      {
        image(dst_row, col) = static_cast<typename ImageT::pixel_type_>(sum);
      }
    }
  }
}

template <typename SrcImageT, typename DstImageT, int KernelSize, typename KernelT = float,
          GaussianCastMode CastMode = GaussianCastMode::Round>
void gaussian_blur(const SrcImageT& src, DstImageT& dst, const std::array<KernelT, KernelSize>& kernel)
{
  static_assert(KernelSize % 2 == 1, "Kernel size must be odd");
  static constexpr int rows = SrcImageT::rows_;
  static constexpr int cols = SrcImageT::cols_;
  static constexpr int half = KernelSize / 2;
  //static constexpr auto kernel = get_fixed_gaussian_kernel<KernelT, KernelSize>();

  auto reflect = [](int x, int max) -> int {
    if (x < 0)
      return -x;
    else if (x >= max)
      return 2 * max - x - 2;
    else
      return x;
  };

  KernelT ring[KernelSize][cols];

  for (int row = 0; row < rows; ++row)
  {
    // Horizontal blur pass into ring buffer
    for (int col = 0; col < cols; ++col)
    {
      KernelT sum = 0.0f;
      for (int k = 0; k < KernelSize; ++k)
      {
        int offset = k - half;
        int idx = reflect(col + offset, cols);
        sum += static_cast<KernelT>(src(row, idx)) * kernel[k];
      }
      ring[row % KernelSize][col] = sum;
    }

    // Vertical pass when ring buffer is sufficiently filled
    if (row >= half)
    {
      int dst_row = row - half;
      for (int col = 0; col < cols; ++col)
      {
        KernelT sum = 0.0f;
        for (int k = 0; k < KernelSize; ++k)
        {
          int ring_row = reflect(dst_row + k - half, rows) % KernelSize;
          sum += ring[ring_row][col] * kernel[k];
        }
        if constexpr (CastMode == GaussianCastMode::Round &&
                      std::is_integral_v<typename DstImageT::pixel_type_>)
        {
          dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum + 0.5f);
        }
        else
        {
          dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum);
        }
      }
    }
  }

  // Final vertical pass for bottom rows
  for (int tail = 0; tail < half; ++tail)
  {
    int dst_row = rows - half + tail;
    if (dst_row >= rows)
      break;

    for (int col = 0; col < cols; ++col)
    {
      KernelT sum = 0.0f;
      for (int k = 0; k < KernelSize; ++k)
      {
        int ring_row = reflect(dst_row + k - half, rows) % KernelSize;
        sum += ring[ring_row][col] * kernel[k];
      }
      if constexpr (CastMode == GaussianCastMode::Round &&
                    std::is_integral_v<typename DstImageT::pixel_type_>)
      {
        dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum + 0.5f);
      }
      else
      {
        dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum);
      }
    }
  }
}

template <typename SrcImageT, typename DstImageT, int KernelSize, typename KernelT = float,
          GaussianCastMode CastMode = GaussianCastMode::Round>
void gaussian_blur(const SrcImageT& src, DstImageT& dst)
{
  static_assert(KernelSize % 2 == 1, "Kernel size must be odd");
  static constexpr auto kernel = get_fixed_gaussian_kernel<KernelT, KernelSize>();
  gaussian_blur<SrcImageT, DstImageT, KernelSize, KernelT, CastMode>(src, dst, kernel);
}

//------------------------------
// Variable Kernel Gaussian Blur (for SIFT++)
//------------------------------
template <typename SrcImageT, typename DstImageT, typename KernelT = float,
          GaussianCastMode CastMode = GaussianCastMode::Round>
void gaussian_blur(const SrcImageT& src, DstImageT& dst, const VariableKernel& kernel)
{
  static constexpr int rows = SrcImageT::rows_;
  static constexpr int cols = SrcImageT::cols_;
  const int half = kernel.half_width;
  const int kernel_size = kernel.size();

  auto reflect = [](int x, int max) -> int {
    if (x < 0)
      return -x;
    else if (x >= max)
      return 2 * max - x - 2;
    else
      return x;
  };

  // Dynamic ring buffer based on actual kernel size
  std::vector<std::vector<KernelT>> ring(kernel_size, std::vector<KernelT>(cols));

  for (int row = 0; row < rows; ++row)
  {
    // Horizontal blur pass into ring buffer
    for (int col = 0; col < cols; ++col)
    {
      KernelT sum = 0.0f;
      for (int k = 0; k < kernel_size; ++k)
      {
        int offset = k - half;
        int idx = reflect(col + offset, cols);
        sum += static_cast<KernelT>(src(row, idx)) * kernel[k];
      }
      ring[row % kernel_size][col] = sum;
    }

    // Vertical pass when ring buffer is sufficiently filled
    if (row >= half)
    {
      int dst_row = row - half;
      for (int col = 0; col < cols; ++col)
      {
        KernelT sum = 0.0f;
        for (int k = 0; k < kernel_size; ++k)
        {
          int ring_row = reflect(dst_row + k - half, rows) % kernel_size;
          sum += ring[ring_row][col] * kernel[k];
        }
        if constexpr (CastMode == GaussianCastMode::Round &&
                      std::is_integral_v<typename DstImageT::pixel_type_>)
        {
          dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum + 0.5f);
        }
        else
        {
          dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum);
        }
      }
    }
  }

  // Final vertical pass for bottom rows
  for (int tail = 0; tail < half; ++tail)
  {
    int dst_row = rows - half + tail;
    if (dst_row >= rows)
      break;

    for (int col = 0; col < cols; ++col)
    {
      KernelT sum = 0.0f;
      for (int k = 0; k < kernel_size; ++k)
      {
        int ring_row = reflect(dst_row + k - half, rows) % kernel_size;
        sum += ring[ring_row][col] * kernel[k];
      }
      if constexpr (CastMode == GaussianCastMode::Round &&
                    std::is_integral_v<typename DstImageT::pixel_type_>)
      {
        dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum + 0.5f);
      }
      else
      {
        dst(dst_row, col) = static_cast<typename DstImageT::pixel_type_>(sum);
      }
    }
  }
}

#endif // IMAGE_UTIL_H
