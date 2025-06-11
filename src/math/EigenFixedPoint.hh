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

// COMMENTED OUT: These specializations cause compilation errors with newer Eigen versions
// The error "scalar_zero_op is not a class template" indicates these are no longer needed
// or have changed in the newer Eigen API. The basic NumTraits specialization above should
// be sufficient for most FixedPoint operations with Eigen.

/*
namespace internal {

// Check Eigen version to determine the correct specialization approach
#if EIGEN_VERSION_AT_LEAST(3,4,0)
// For Eigen 3.4+, scalar_zero_op and scalar_identity_op use different syntax

// Custom nullary functor for zero values  
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
struct scalar_constant_op<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>> {
  typedef FixedPoint<IntegerBits, FractionalBits, UnderlyingType> Scalar;
  
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE scalar_constant_op(const scalar_constant_op& other) : m_other(other.m_other) {}
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE scalar_constant_op(const Scalar& other) : m_other(other) {}
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const Scalar operator()() const { return m_other; }
  template<typename PacketType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE const PacketType packetOp() const { 
    return internal::pset1<PacketType>(m_other); 
  }
  const Scalar m_other;
};

// Functor traits for scalar_constant_op
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
struct functor_traits<scalar_constant_op<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>>> {
  enum { 
    Cost = 0,
    PacketAccess = false, 
    IsRepeatable = true 
  };
};

// Custom identity functor
template<int IntegerBits, int FractionalBits, typename UnderlyingType>
struct scalar_identity_op<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>> {
  EIGEN_EMPTY_STRUCT_CTOR(scalar_identity_op)
  template<typename IndexType>
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE 
  const FixedPoint<IntegerBits, FractionalBits, UnderlyingType> operator()(IndexType row, IndexType col) const {
    return row == col ? FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(1.0) 
                      : FixedPoint<IntegerBits, FractionalBits, UnderlyingType>(0.0);
  }
};

template<int IntegerBits, int FractionalBits, typename UnderlyingType>
struct functor_traits<scalar_identity_op<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>>> {
  enum { 
    Cost = NumTraits<FixedPoint<IntegerBits, FractionalBits, UnderlyingType>>::AddCost, 
    PacketAccess = false, 
    IsRepeatable = true 
  };
};

#else
// For Eigen < 3.4, use the original specialization approach

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

#endif // EIGEN_VERSION_AT_LEAST

} // namespace internal
*/

} // namespace Eigen

#endif // EIGEN_FIXED_POINT_HPP

