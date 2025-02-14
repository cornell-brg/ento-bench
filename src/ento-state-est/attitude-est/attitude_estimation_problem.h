#ifndef ATTITUDE_PROBLEM_H
#define ATTITUDE_PROBLEM_H

#include "attitude_measurement.h"
#include <ento-bench/problem.h>

namespace EntoAttitude
{

template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter = true>
class AttitudeProblem : 
  public EntoBench::EntoProblem<AttitudeProblem<Scalar, Kernel, UseMag>>
{ 
public:
  // Expose Template Typenames to Others
  using Scalar_ = Scalar;
  using Kernel_ = Kernel;

  // Required by Problem Interface for Experiment I/O
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SavesResults_ = true;

  // @TODO: Add other member fields for input data, ground truth data
  //   and output data.
  Kernel kernel_;

  // Inputs from dataset file
  AttitudeMeasurement<Scalar, UseMag> measurement_;
  Eigen::Quaternion<Scalar> q_gt_;
  Scalar dt_;

  // Outputs from Kernel
  Eigen::Quaternion<Scalar> q_prev_;
  Eigen::Quaternion<Scalar> q_;

#ifdef NATIVE
  std::string serialize_impl() const;
  bool deserialize_impl(const std::string &line);
#else

  // @TODO: Add serialize implementation for embedded builds.
  const char* serialize_impl() const;
#endif

  // @TODO: Add deserialize implementation for embedded builds
  bool deserialize_impl(const char* line);

  // @TODO: Complete validate implementation. 
  bool validate_impl();

  // @TODO: Complete solve implementation.
  void solve_impl() { return kernel_(q_prev_, measurement_, dt_, &q_); }

  // @TODO: Complete clear implementation.
  bool clear_impl();

  static constexpr const char* header_impl()
  {
    return ""; //@TODO: Add string for header (input)
  }

  AttitudeProblem(Kernel kernel) : kernel_(std::move(kernel)) {};
};


template <typename Scalar, typename Kernel, bool UseMag, bool IsFilter>
bool AttitudeProblem<Scalar, Kernel, UseMag, IsFilter>::deserialize_impl(const char* line)
{

}


} // namespace EntoAttitude

#endif // ATTITUDE_PROBLEM_H
