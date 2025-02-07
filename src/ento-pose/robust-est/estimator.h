#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <ento-util/containers.h>
#include <ento-pose/robust-est/ransac.h>

namespace EntoPose
{
using namespace EntoUtil;

// =========================
// EntoEstimator
// Scalar S, ModelType M, Derived D
template <typename S, typename M, typename D>
class EntoEstimator
{
public:
  EntoEstimator() {}
  
  S score_model();
  
  template <size_t N>
  void generate_models(EntoArray<S, N> *models);

private:
  const RansacOptions<S> &opt;

  

};
}

#endif
