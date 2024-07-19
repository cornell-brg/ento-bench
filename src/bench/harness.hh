#ifndef HARNESS_HH
#define HARNESS_HH

#include <concepts>
#include <tuple>
#include <cstdio>
#include "bench/roi.h"


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
template<typename Callable>
class Harness {
public:
  Harness(Callable callable, const char* name)
    : callable_(std::move(callable)), name_(name) {}

  template<typename... Args>
  auto run(Args&&... args) {
    printf("Running benchmark: %s", name_);
    start_roi();
    if constexpr (std::is_void_v<decltype(callable_(std::forward<Args>(args)...))>) {
      callable_(std::forward<Args>(args)...);
      end_roi();
    } else {
      auto result = callable_(std::forward<Args>(args)...);
      end_roi();
      return result;
    }
  }

private:
  Callable callable_;
  const char* name_;
};

template<typename Callable>
Harness(const char* name, Callable callable) -> Harness<Callable>;

} // namespace bench

#endif // HARNESS_HH
