#ifndef ENTO_LQR_BASE_H
#define ENTO_LQR_BASE_H

#include <Eigen/Dense>

template< typename Traits >
class LQRStep
{
public:
  using Scalar = typename Traits::Scalar;
  static constexpr int N = Traits::N;
  static constexpr int M = Traits::M;

  // ----- I/O structs ----------------------------------------------------- //
  struct Input
  {
    Eigen::Matrix< Scalar, N, 1 > x;
    Eigen::Matrix< Scalar, N, 1 > x_ref;
  };
  struct Output
  {
    Eigen::Matrix< Scalar, M, 1 > u;
  };

  // ----- OptControl hooks ------------------------------------------------ //
  static constexpr Eigen::Matrix< Scalar, N, N > get_Adyn() { return Traits::Adyn; }
  static constexpr Eigen::Matrix< Scalar, N, M > get_Bdyn() { return Traits::Bdyn; }

  void set_x0   ( const Eigen::Matrix< Scalar, N, 1 >& x0   ) { x_     = x0;   }
  void set_x_ref( const Eigen::Matrix< Scalar, N, 1 >& xref ) { x_ref_ = xref; }
  void reset_duals() {}

  void solve()
  {
    u_ = -Traits::K * ( x_ - x_ref_ );
  }

  const Eigen::Matrix< Scalar, M, 1 >& get_u0() const { return u_; }

private:
  Eigen::Matrix< Scalar, N, 1 > x_ {}, x_ref_ {};
  Eigen::Matrix< Scalar, M, 1 > u_ {};
};


#endif // ENTO_LQR_BASE_H
