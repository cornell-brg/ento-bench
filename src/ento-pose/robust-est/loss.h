#ifndef ENTO_POSE_LOSS_H
#define ENTO_POSE_LOSS_H

#include <algorithm>

template <typename Scalar>
class TrivialLoss
{
public:
  TrivialLoss(Scalar) {}
  TrivialLoss() {}
  Scalar loss(Scalar r) const { return r; }
  Scalar weight(Scalar r) const { return r; }
};

template <typename Scalar>
class TruncatedLoss
{
private:
  Scalar squared_thresh_;
public:
  TruncatedLoss(Scalar t) : squared_thresh_(t) {}
  Scalar loss(Scalar r) const { return std::min(r, squared_thresh_); }
  Scalar weight(Scalar r) const
  {
    return ( r < squared_thresh_) ? static_cast<Scalar>(1) : static_cast<Scalar>(0);
  }
};

template <typename Scalar>
class HuberLoss
{
private:
  const Scalar thresh_;
public:
  HuberLoss(Scalar t) : thresh_(t) {}
  Scalar loss(Scalar r2) const
  {
    const Scalar r = std::sqrt(r2);
    if (r <= thresh_) return r;
    else return thresh_ * ( static_cast<Scalar>(2) - r - thresh_ );
  }

  Scalar weight(Scalar r2) const
  {
    const Scalar r = std::sqrt(r2);
    if (r <= thresh_) return static_cast<Scalar>(1);
    else return thresh_ / r;
  }
};

template <typename Scalar>
class CauchyLoss
{
private:
  const Scalar inv_sq_thr;
public:
  CauchyLoss(Scalar threshold) : inv_sq_thr(static_cast<Scalar>(1) / (threshold * threshold)) {}
  Scalar loss(Scalar r2) const { return std::log1p(r2 * inv_sq_thr); }
  Scalar weight(Scalar r2) const
  {
    return std::max(std::numeric_limits<Scalar>::min(), inv_sq_thr / (static_cast<Scalar>(1) + r2 * inv_sq_thr));
  }

};

template <typename Scalar>
class TruncatedLossLeZach
{
private:
  const Scalar squared_thr_;

public:
  // hyper-parameter for penalty strength
  Scalar mu_;
  // schedule for increasing mu in each iteration
  static constexpr Scalar alpha = 1.5;

  TruncatedLossLeZach(Scalar threshold) : squared_thr_(threshold * threshold), mu_(0.5) {}
  Scalar loss(Scalar r2) const { return std::min(r2, squared_thr_); }
  Scalar weight(Scalar r2) const
  {
    Scalar r2_hat = r2 / squared_thr_;
    Scalar zstar = std::min(r2_hat, 1.0);

    if (r2_hat < 1.0)
    {
      return 0.5;
    }
    else
    {
      // assumes mu > 0.5
      Scalar r2m1 = r2_hat - 1.0;
      Scalar rho = (2.0 * r2m1 + std::sqrt(4.0 * r2m1 * r2m1 * mu_ * mu_ + 2 * mu_ * r2m1)) / mu_;
      Scalar a = (r2_hat + mu_ * rho * zstar - 0.5 * rho) / (1 + mu_ * rho);
      Scalar zbar = std::max(0.0, std::min(a, 1.0));
      return (zstar - zbar) / rho;
    }
  }
};


#endif
