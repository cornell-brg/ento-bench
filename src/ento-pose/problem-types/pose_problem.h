#ifndef POSE_PROBLEM_H
#define POSE_PROBLEM_H

#include <concepts>
#include <ento-bench/problem.h>

namespace EntoPose
{

template <typename Derived>
class PoseProblem : public
  EntoBench::EntoProblem<Derived>
{
public:
  static constexpr size_t NumPts_ = Derived::NumPts_;
protected:
  // Protected constructor prevent direct instantiation.
  PoseProblem() = default;
};

template <typename T>
concept PoseProblemConcept = EntoBench::ProblemConcept<T> && requires(T t)
{
  /*typename T::Scalar_;
  typename T::Solver_;
  typename T::NumPts_;
  requires std::is_arithmetic_v<typename T::Scalar>;
  requires std::is_same_v<decltype(T::NumPts_), const size_t>;*/

  { t.n_point_point_ };

  // Required methods for derived problems
  { t.validate_impl() };
  { t.clear_impl() };
  { t.solve_impl() };
#ifdef NATIVE
  { t.serialize_impl() } -> std::convertible_to<std::string>;
  { t.deserialize_impl(std::declval<std::string&>()) } -> std::convertible_to<bool>;
#endif

  { t.deserialize_impl(std::declval<const char*>()) } -> std::convertible_to<bool>;


  { T::header_impl() } -> std::convertible_to<const char*>;
};


} // namespace EntoPose

#endif // POSE_PROBLEM_H
