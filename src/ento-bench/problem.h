#ifndef ENTO_PROBLEM_H
#define ENTO_PROBLEM_H

#include <cstdio>
#include <concepts>
#include <ento-util/debug.h>

#ifdef NATIVE
#include <iostream>
#endif

namespace EntoBench
{
  
template <typename Derived>
class EntoProblem
{
private:
  // A solution (algorithm) to a problem. For microbenchmarks this can
  // be more considered a Callable
public:
  // Non-virtual destructor. We don't expect to use
  // Base P
  ~EntoProblem() = default;

#ifdef NATIVE
  std::string serialize() const;
  bool deserialize(const std::string& line);
#endif

  bool deserialize(const char* line);

  template <typename... Args>
  auto operator()(Args&&... args);

  bool validate();
  void solve();
  void clear();

  // Must be defined in class declaration in base classes due to static constexpr
  static constexpr const char* header() { return Derived::header_impl(); }

protected:
  // Protected constructor prevents direct instantiation
  EntoProblem() = default;
};

// Concepts

template <typename T>
concept ProblemConcept = requires(T t)
{
  requires std::derived_from<T, EntoProblem<T>>;

#ifdef NATIVE
  { t.serialize() } -> std::convertible_to<std::string>;
  { t.deserialize(std::declval<const std::string&>()) } -> std::convertible_to<bool>;
#endif

  { t.deserialize(std::declval<const char*>()) } -> std::convertible_to<bool>;

  { T::header() } -> std::convertible_to<const char*>;
};


///////////////////// IMPLEMENTATIONS /////////////////////

#ifdef NATIVE

// Serialize the problem data. Will never be called from a MCU
template <typename Derived>
std::string EntoProblem<Derived>::serialize() const
{
  return static_cast<const Derived*>(this)->serialize_impl();
}

// Deserialize the problem instance from a file stream
template <typename Derived>
bool EntoProblem<Derived>::deserialize(const std::string& line)
{
  return static_cast<Derived*>(this)->deserialize_impl(line);
}

#endif
// Deserialize the problem instance from a line of a file.
template <typename Derived>
bool EntoProblem<Derived>::deserialize(const char* line)
{
  return static_cast<Derived*>(this)->deserialize_impl(line);
}

template <typename Derived>
template <typename... Args>
auto EntoProblem<Derived>::operator()(Args&&... args)
{
  return static_cast<Derived*>(this)->run(std::forward<Args>(args)...);
}

// Evaluate the correctness after running.
template <typename Derived>
bool EntoProblem<Derived>::validate()
{
  return static_cast<const Derived*>(this)->validate_impl();
}


template <typename Derived>
void EntoProblem<Derived>::solve()
{
  static_cast<Derived*>(this)->solve_impl();
}

template <typename Derived>
void EntoProblem<Derived>::clear()
{
  return static_cast<Derived*>(this)->clear_impl();
}

template <typename Callable>
class BasicProblem : public EntoProblem<BasicProblem<Callable>>
{
public:
  static constexpr bool RequiresDataset_ = false;
  static constexpr bool SaveResults_ = false;

#ifdef NATIVE
  std::string serialize_impl() const;
  bool deserialize_impl([[maybe_unused]] const std::string &line) { return true; }
#endif

  bool deserialize_impl([[maybe_unused]] const char* line) { return true; }
  bool validate_impl() { return true; }
  void solve_impl() { callable_(); }
  bool clear_impl();

  static constexpr const char* header_impl() { return ""; }
  
  BasicProblem(Callable callable) : callable_(std::move(callable)) {};

private:
  Callable callable_;
};

template <typename T>
auto make_basic_problem(T&& callable)
{
  if constexpr (std::is_function_v<std::remove_pointer_t<std::decay_t<T>>>)
  {
    return BasicProblem([f = std::forward<T>(callable)]() { f(); });
  }
  else
  {
    return BasicProblem(std::forward<T>(callable));
  }
}

} // namespace EntoBench

#endif
