#ifndef ENTO_CAPTURE_PROBLEM_H
#define ENTO_CAPTURE_PROBLEM_H

#include <cstdint>
#include <cstddef>
#include <span>

#include <ento-bench/problem.h>

namespace EntoBench
{

// Shared base for differential-testing "capture" problems.
// Subclasses provide prepare_impl() and solve_impl(), and fill the fixed-size
// `exit_` byte buffer (ResultBytes) as part of solve. This class provides the
// ProblemConcept-required stubs (deserialize/header/validate/clear) and the
// result_signature_impl() that hashes the exit_ buffer via FNV-1a.
//
// Usage:
//   class BenchFoo : public EntoBench::CaptureProblem<BenchFoo, 48>
//   {
//   public:
//     void prepare_impl() { /* populate member inputs */ }
//     void solve_impl()   { /* do op, memcpy result -> exit_ */ }
//   };
template <typename Derived, std::size_t ResultBytes>
class CaptureProblem : public EntoProblem<Derived>
{
public:
  static constexpr bool   RequiresDataset_  = false;
  static constexpr bool   SaveResults_      = false;
  static constexpr bool   RequiresPrepare_  = true;

  bool deserialize_impl(const char*) { return true; }
  static constexpr const char* header_impl() { return ""; }
  bool validate_impl() const { return true; }
  void clear_impl() {}

  ResultSig result_signature_impl() const
  {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(exit_);
    return ResultSig{ .bytes = std::span<const uint8_t>{p, ResultBytes} };
  }

protected:
  alignas(8) mutable uint8_t exit_[ResultBytes]{};
};

} // namespace EntoBench

#endif // ENTO_CAPTURE_PROBLEM_H
