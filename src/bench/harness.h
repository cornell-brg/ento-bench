#ifndef HARNESS_HH
#define HARNESS_HH

#include <concepts>
#include <tuple>
#include <cstdio>
#include "bench/roi.h"
#include <array>
#include <initializer_list>


namespace bench{
// Define a concept for callables with no arguments
template<typename T>
concept NoArg = requires(T t) {
    t();
};

// WorkloadWrapper class
template<typename Callable>
class WorkloadWrapper {
public:
    using ReturnType = decltype(std::declval<Callable>()());

    WorkloadWrapper(Callable& callable) : callable_{callable} {}

    ReturnType operator()() const {
        if constexpr (std::is_void_v<ReturnType>) {
            callable_();
        } else {
            return callable_();
        }
    }

private:
    Callable& callable_;
};

// MultiHarness class
template <std::size_t N, NoArg... Callable>
class MultiHarness {
public:
    //MultiHarness(Callable... funcs) : functions_{WorkloadWrapper<Callable>(funcs)...} {}
    MultiHarness(Callable... funcs) : functions_{funcs...} {}

    void run() {
        run_impl(std::make_index_sequence<N>{});
    }

private:
  template<std::size_t... I>
  void run_impl(std::index_sequence<I...>) {
      (void(std::initializer_list<int>{
            (printf("Starting ROI!\n"),
             start_roi(),
             std::get<I>(functions_)(),
             end_roi(), 
             printf("Ending ROI!\n"),
             0)
            }), ...);
  }
           
  //std::tuple<WorkloadWrapper<Callable>...> functions_;
  std::tuple<Callable...> functions_;
};

template <NoArg... Callables>
MultiHarness(Callables...) -> MultiHarness<sizeof...(Callables), Callables...>;


/* (Single) Harness Class
 * Simple wrapper for a Callable encapsulating ROI calls.
 * Allows for running a function many times for cache warm up.
 *
 */
template<int Iters, typename Callable>
class Harness {
public:
  Harness(Callable callable, const char* name)
    : callable_(std::move(callable)), name_(name) {}

  template<typename... Args>
  auto run(Args&&... args) {
    printf("Running benchmark %s for %i iterations...\n", name_, Iters);

    if constexpr (!std::is_void_v<decltype(callable_(std::forward<Args>(args)...))>)
    {
      using ResultType = decltype(callable_(std::forward<Args>(args)...));
      std::array<ResultType, Iters> results;

      for (int i = 0; i < Iters; i++)
      {
        start_roi();
        auto result = callable_(std::forward<Args>(args)...);
        end_roi();
        cycles_[i] = get_cycles();
        results[i] = result;
      }
      return results;

    }
    else
    {
      for (int i = 0; i < Iters; ++i)
      {
        start_roi();
        callable_(std::forward<Args>(args)...);
        end_roi();
        cycles_[i] = get_cycles();
      }
    }

    printf("Results from running %s for %i iterations.\n", name_, Iters);
    for (int i = 0; i < Iters; ++i)
    {
      printf("Cycle count for iteration %i: %i\n", i, cycles_[i]);
    }

  }

private:
  Callable callable_;
  const char* name_;
  std::array<int, Iters> cycles_;
};

template<int Iters, typename Callable>
Harness(Callable callable, const char* name) -> Harness<Iters, Callable>;

template<int Iters, typename Callable>
Harness<Iters, Callable> make_harness(Callable callable, const char* name)
{
  return Harness<Iters, Callable>(std::move(callable), name);
}


} // namespace bench

#endif // HARNESS_HH
