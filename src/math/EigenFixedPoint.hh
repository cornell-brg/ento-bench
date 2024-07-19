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
    IsInteger = 1,
    IsSigned = std::is_signed_v<UnderlyingType>,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1,
    DivCost = 12
  };

  static Real epsilon() {
    return Real::from_raw(1);
  }
  static Real dummy_precision() {
    return Real::from_raw(1);
  }
  static Real highest() {
    return Real::from_raw(NumTraits<UnderlyingType>::highest());
  }
  static Real lowest() {
    return Real::from_raw(NumTraits<UnderlyingType>::lowest());
  }
};

} // namespace Eigen

#endif // EIGEN_FIXED_POINT_HPP

