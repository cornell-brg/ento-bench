#ifndef EIGEN_FIXED_POINT_HPP
#define EIGEN_FIXED_POINT_HPP

#include <Eigen/Core>
#include "FixedPoint.hh"

// Specialize Eigen for FixedPoint types
namespace Eigen {

template <int IntegerBits, int FractionalBits, typename UnderlyingType>
struct NumTraits<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>> : NumTraits<UnderlyingType> {
  using Real = FixedPoint<IntegerBits, FractionalBits, UnderlyingType>;
  using NonInteger = FixedPoint<IntegerBits, FractionalBits, UnderlyingType>;
  using Nested = FixedPoint<IntegerBits, FractionalBits, UnderlyingType>;

  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 0,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };

  static inline Real epsilon() {
    return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(
      static_cast<float>(1) / (1 << FractionalBits)
    );
  }
  static inline Real dummy_precision() {
    return epsilon();
  }
  static inline Real highest() {
    return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>::max();
  }
  static inline Real lowest() {
    return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>::min();
  }

  static inline int digits10() {
    return static_cast<int>(FractionalBits * 0.30103); // log10(2) ≈ 0.30103
  }
};

namespace internal {

// Specialize scalar_zero_op to avoid constructor ambiguity
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
struct scalar_zero_op<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>> {
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE scalar_zero_op() = default;
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE 
  const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> operator()() const { 
    return FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0.0); 
  }
  template <typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const PacketType packetOp() const {
    return internal::pzero<PacketType>(PacketType());
  }
};

// Specialize scalar_identity_op to avoid constructor ambiguity  
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
struct scalar_identity_op<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>> {
  template <typename IndexType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE 
  const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> operator()(IndexType row, IndexType col) const {
    return row == col ? FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(1.0) 
                      : FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0.0);
  }
};

} // namespace internal
} // namespace Eigen

#endif // EIGEN_FIXED_POINT_HPP

