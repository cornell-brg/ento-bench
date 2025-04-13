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
  constexpr int rows = ImageT::rows;
  constexpr int cols = ImageT::cols;
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
                      std::is_integral_v<typename ImageT::pixel_type>)
        {
          image(dst_row, col) = static_cast<typename ImageT::pixel_type>(sum + 0.5f);
        }
        else
        {
          image(dst_row, col) = static_cast<typename ImageT::pixel_type>(sum);
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
                    std::is_integral_v<typename ImageT::pixel_type>)
      {
        image(dst_row, col) = static_cast<typename ImageT::pixel_type>(sum + 0.5f);
      }
      else
      {
        image(dst_row, col) = static_cast<typename ImageT::pixel_type>(sum);
      }
    }
  }
}

template <typename SrcImageT, typename DstImageT, int KernelSize, typename KernelT = float,
          GaussianCastMode CastMode = GaussianCastMode::Round>
void gaussian_blur(const SrcImageT& src, DstImageT& dst)
{
  static_assert(KernelSize % 2 == 1, "Kernel size must be odd");
  static constexpr int rows = SrcImageT::rows;
  static constexpr int cols = SrcImageT::cols;
  static constexpr int half = KernelSize / 2;
  static constexpr auto kernel = get_fixed_gaussian_kernel<KernelT, KernelSize>();

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
                      std::is_integral_v<typename DstImageT::pixel_type>)
        {
          dst(dst_row, col) = static_cast<typename DstImageT::pixel_type>(sum + 0.5f);
        }
        else
        {
          dst(dst_row, col) = static_cast<typename DstImageT::pixel_type>(sum);
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
                    std::is_integral_v<typename DstImageT::pixel_type>)
      {
        dst(dst_row, col) = static_cast<typename DstImageT::pixel_type>(sum + 0.5f);
      }
      else
      {
        dst(dst_row, col) = static_cast<typename DstImageT::pixel_type>(sum);
      }
    }
  }
}

#endif // IMAGE_UTIL_H
