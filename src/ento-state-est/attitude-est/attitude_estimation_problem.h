#ifndef ATTITUDE_PROBLEM_H
#define ATTITUDE_PROBLEM_H

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
  void solve_impl();

  // @TODO: Complete clear implementation.
  bool clear_impl();

  static constexpr const char* header_impl()
  {
    return ""; //@TODO: Add string for header (input)
  }

  AttitudeProblem(Kernel kernel) : kernel_(std::move(kernel)) {};
};

} // namespace EntoAttitude

#endif // ATTITUDE_PROBLEM_H
