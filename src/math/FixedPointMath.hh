#pragma once

#include "FixedPoint.hh"
#include <cmath>

// Global math functions for FixedPoint types (required for Eigen compatibility)
// These functions are placed in the global namespace to be found by Eigen

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
sqrt(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType>& x)
{
  using FP = FixedPoint<IntegerBits, FractionalBits, UnderlyingType>;
  if (x <= FP(0.0f)) return FP(0.0f);
  
  // Initial guess using floating-point sqrt
  float x_float = static_cast<float>(x);
  float sqrt_float = std::sqrt(x_float);
  FP result(sqrt_float);
  
  // Newton-Raphson iteration: x_{n+1} = 0.5 * (x_n + a/x_n)
  for (int i = 0; i < 4; ++i) {
    result = FP(0.5f) * (result + x / result);
  }
  
  return result;
}

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
abs(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType>& x)
{
  return x < FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0) ? -x : x;
}

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
abs2(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return x * x;
}

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
conj(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return x; // Real numbers are their own conjugate
}

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
real(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &x)
{
  return x; // Real part of a real number is itself
}

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
imag(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> &)
{
  return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0); // Imaginary part of real number is zero
}

// acos function for FixedPoint types
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
FixedPoint<IntegerBits, FractionalBits, UnderlyingType>
acos(const FixedPoint<IntegerBits, FractionalBits, UnderlyingType>& x)
{
  // Convert to float, compute acos, convert back
  float fx = static_cast<float>(x);
  return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(std::acos(fx));
} 