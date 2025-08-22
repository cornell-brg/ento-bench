#ifndef FIXED_POINT_HH
#define FIXED_POINT_HH

#include <cstdint>
#include <type_traits>
#include <iostream>
#include <limits>
#include <concepts>

template <typename T>
concept Integral = std::is_integral_v<T>;

template <typename T>
concept SignedIntegral = Integral<T> && std::is_signed_v<T>;

template <typename T>
concept UnsignedIntegral = Integral<T> && std::is_unsigned_v<T>;

template <int IntegerBits, int FractionalBits, typename T>
concept FixedPointValid = 
  Integral<T> && 
  (IntegerBits > 0) && 
  (FractionalBits > 0) && 
  ((IntegerBits + FractionalBits) <= (sizeof(T) * 8));

template <typename T>
struct IntermediateType;

template <>
struct IntermediateType<uint8_t> {
  using type = uint16_t;
};

template <>
struct IntermediateType<uint16_t> {
  using type = uint32_t;
};

template <>
struct IntermediateType<uint32_t> {
  using type = uint64_t;
};

template <>
struct IntermediateType<int8_t> {
  using type = int16_t;
};

template <>
struct IntermediateType<int16_t> {
  using type = int32_t;
};

template <>
struct IntermediateType<int32_t> {
  using type = int64_t;
};

template <>
struct IntermediateType<int64_t> {
  using type = int64_t;
};

template <typename T>
using IntermediateType_t = typename IntermediateType<T>::type;

template <int IntegerBits, int FractionalBits, typename UnderlyingType>
requires FixedPointValid<IntegerBits, FractionalBits, UnderlyingType>
class FixedPoint {
  UnderlyingType value;

public:
  // Default constructor
  FixedPoint() : value(0) {}

  // Constructors - ordered by priority to resolve ambiguity
  
  // Highest priority: literal zero (resolves Scalar(0) ambiguity)
  FixedPoint(std::integral_constant<int, 0>) : value(0) {}
  
  // High priority: nullptr (for literal 0)
  FixedPoint(decltype(nullptr)) : value(0) {}
  
  // Medium priority: integer literals (non-explicit for convenience)
  template<typename T>
  FixedPoint(T i) requires std::is_integral_v<T> && (!std::is_same_v<T, UnderlyingType>) 
    : value(static_cast<UnderlyingType>(i << FractionalBits)) {}
  
  // Lower priority: floating point (explicit to avoid accidental conversions)
  explicit FixedPoint(float f) : value(static_cast<UnderlyingType>(f * (1 << FractionalBits))) {}
  explicit FixedPoint(double d) : value(static_cast<UnderlyingType>(d * (1 << FractionalBits))) {}
  
  // Lowest priority: raw value constructor (explicit)
  explicit FixedPoint(UnderlyingType raw_value) : value(raw_value) {}

  // Constructor for int32_t (only when UnderlyingType is NOT int32_t and NOT int16_t)
  explicit FixedPoint(int32_t i) requires 
    SignedIntegral<UnderlyingType> &&
    (!std::is_same_v<UnderlyingType, int32_t>) &&
    (!std::is_same_v<UnderlyingType, int16_t>) &&
    (!std::is_same_v<UnderlyingType, int>) &&
    std::is_same_v<int32_t, int> : value(static_cast<UnderlyingType>(i << FractionalBits)) {}

  // Constructor for uint32_t (only when UnderlyingType is NOT uint32_t and NOT int16_t)
  explicit FixedPoint(uint32_t u) requires UnsignedIntegral<UnderlyingType> && (!std::is_same_v<UnderlyingType, uint32_t>) && (!std::is_same_v<UnderlyingType, int16_t>) : value(static_cast<UnderlyingType>(u << FractionalBits)) {}

  // Constructor for int (only when UnderlyingType is NOT int and NOT int32_t and NOT int16_t)
  FixedPoint(int i) requires
    (!std::is_same_v<UnderlyingType, int>) && 
    (!std::is_same_v<UnderlyingType, int32_t>) && 
    (!std::is_same_v<UnderlyingType, int16_t>) &&
    (!std::is_same_v<int32_t, int>) : value(static_cast<UnderlyingType>(i << FractionalBits)) {}

  // Special constructor for int16_t underlying type to handle int literals
  FixedPoint(int i) requires std::is_same_v<UnderlyingType, int16_t> : value(static_cast<UnderlyingType>(i << FractionalBits)) {}

  static inline FixedPoint from_raw(UnderlyingType raw_value) {
    FixedPoint fp;
    fp.value = raw_value;
    return fp;
  }

  float to_float() const {
    return static_cast<float>(value) / (1 << FractionalBits);
  }

  float get_decimal() const {
    return static_cast<float>((value << IntegerBits) >> IntegerBits) / (1 << FractionalBits);
  }
  
  int8_t to_int8() const requires SignedIntegral<UnderlyingType> {
    return static_cast<int8_t>(value >> FractionalBits);
  }

  int16_t to_int16() const requires SignedIntegral<UnderlyingType> {
    return static_cast<int16_t>(value >> FractionalBits);
  }

  int32_t to_int32() const requires SignedIntegral<UnderlyingType> {
    return static_cast<int32_t>(value >> FractionalBits);
  }

  uint8_t to_uint8() const requires UnsignedIntegral<UnderlyingType> {
    return static_cast<uint8_t>(value >> FractionalBits);
  }

  uint16_t to_uint16() const requires UnsignedIntegral<UnderlyingType> {
    return static_cast<uint16_t>(value >> FractionalBits);
  }

  uint32_t to_uint32() const requires UnsignedIntegral<UnderlyingType> {
    return static_cast<uint32_t>(value >> FractionalBits);
  }

  // Casting to int
  explicit operator int32_t() const {
    return static_cast<int32_t>(value >> FractionalBits);
  }

  // Only define long operator for native builds (avoids duplicate on embedded)
#ifdef NATIVE
  explicit operator long() const {
    return static_cast<long>(value >> FractionalBits);
  }
#endif

  // Casting to int
  explicit operator float() const {
    return static_cast<float>(value) / (1 << FractionalBits);
  }

  // Casting to int
  explicit operator int64_t() const {
    return static_cast<int64_t>(value >> FractionalBits);
  }

  // Conversion operators
  explicit operator double() const {
    return static_cast<double>(value) / (1 << FractionalBits);
  }

  UnderlyingType raw() const {
    return value;
  }

  // Overloaded operators
  FixedPoint operator+(const FixedPoint& other) const {
    return FixedPoint::from_raw(value + other.raw());
  }

  FixedPoint operator-(const FixedPoint& other) const {
    return FixedPoint::from_raw(value - other.raw());
  }

  FixedPoint operator-() const {
    return FixedPoint::from_raw(((UnderlyingType) 0 ) - value);
  }

  FixedPoint operator*(const FixedPoint& other) const {
    IntermediateType_t<UnderlyingType> temp = static_cast<IntermediateType_t<UnderlyingType>>(value) * other.raw();
    return FixedPoint::from_raw(static_cast<UnderlyingType>(temp >> FractionalBits));
  }

  FixedPoint operator/(const FixedPoint& other) const {
    IntermediateType_t<UnderlyingType> temp = (static_cast<IntermediateType_t<UnderlyingType>>(value) << FractionalBits) / other.raw();
    return FixedPoint::from_raw(static_cast<UnderlyingType>(temp));
  }

  FixedPoint& operator+=(const FixedPoint& other) {
    value += other.raw();
    return *this;
  }

  FixedPoint& operator-=(const FixedPoint& other) {
    value -= other.raw();
    return *this;
  }

  FixedPoint& operator*=(const FixedPoint& other) {
    IntermediateType_t<UnderlyingType> temp = static_cast<IntermediateType_t<UnderlyingType>>(value) * other.raw();
    value = static_cast<UnderlyingType>(temp >> FractionalBits);
    return *this;
  }

  FixedPoint& operator/=(const FixedPoint& other) {
    IntermediateType_t<UnderlyingType> temp = (static_cast<IntermediateType_t<UnderlyingType>>(value) << FractionalBits) / other.raw();
    value = static_cast<UnderlyingType>(temp);
    return *this;
  }

  bool operator==(const FixedPoint& other) const {
    return value == other.raw();
  }

  bool operator!=(const FixedPoint& other) const {
    return value != other.raw();
  }

  bool operator<(const FixedPoint& other) const {
    return value < other.raw();
  }

  bool operator<=(const FixedPoint& other) const {
    return value <= other.raw();
  }

  bool operator>(const FixedPoint& other) const {
    return value > other.raw();
  }

  bool operator>=(const FixedPoint& other) const {
    return value >= other.raw();
  }

  // Stream operators for serialization
  template<typename CharT, typename Traits>
  friend std::basic_ostream<CharT, Traits>& operator<<(std::basic_ostream<CharT, Traits>& os, const FixedPoint& fp) {
    return os << static_cast<double>(fp);
  }

  template<typename CharT, typename Traits>
  friend std::basic_istream<CharT, Traits>& operator>>(std::basic_istream<CharT, Traits>& is, FixedPoint& fp) {
    double temp;
    if (is >> temp) {
      fp = FixedPoint(static_cast<float>(temp));
    }
    return is;
  }
};

// Convenience aliases for different fixed-point formats and underlying types
using Q1_7_int8 = FixedPoint<1, 7, int8_t>;
using Q1_7_uint8 = FixedPoint<1, 7, uint8_t>;
using Q1_15_int16 = FixedPoint<1, 15, int16_t>;
using Q1_15_uint16 = FixedPoint<1, 15, uint16_t>;
using Q1_31_int32 = FixedPoint<1, 31, int32_t>;
using Q1_31_uint32 = FixedPoint<1, 31, uint32_t>;

#endif // FIXED_POINT_HH

