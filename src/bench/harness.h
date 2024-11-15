#ifndef HARNESS_HH
#define HARNESS_HH

#include "mcu-util/timing.h"
#include <concepts>
#include <tuple>
#include <cstdio>
#include <bench/roi.h>
#include <array>
#include <limits>
#include <variant>
#include <type_traits>
#include <initializer_list>


namespace bench {
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


enum ProfileMode
{
  Full,
  Aggregate,
  Sample,
  PrintOnly
};


/* (Single) Harness Class
 * Simple wrapper for a Callable encapsulating ROI calls.
 * Allows for running a function many times for cache warm up.
 *
 */
template<int Iters, typename Callable, ProfileMode Mode = ProfileMode::Full>
class Harness {
public:
  Harness(Callable callable, const char* name, int sample_interval = 1, int iters = Iters)
    : callable_(std::move(callable)), name_(name), sample_interval_(sample_interval), iters_(iters),
      total_metrics_({0, 0, 0, 0, 0}), max_metrics_({0, 0, 0, 0, 0}),
      min_metrics_({std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max()})
  {
    init_roi_tracking();
  }

  template<typename... Args>
  auto run(Args&&... args) {
    printf("Running benchmark %s for %i iterations...\n", name_, Iters);

    if constexpr (!std::is_void_v<decltype(callable_(std::forward<Args>(args)...))>)
    {
      using ResultType = decltype(callable_(std::forward<Args>(args)...));
      std::array<ResultType, Iters> results;

      for (int i = 0; i < Iters; ++i)
      {
        start_roi();
        auto result = callable_(std::forward<Args>(args)...);
        end_roi();

        ROIMetrics metrics = get_roi_stats();
        handle_mode(metrics, i);
        results[i] = result;
      }
      print_summary();
      return results;
    } else {
      for (int i = 0; i < Iters; ++i) {
        start_roi();
        callable_(std::forward<Args>(args)...);
        end_roi();

        ROIMetrics metrics = get_roi_stats();
        handle_mode(metrics, i);

        software_delay_cycles(100000);
      }
      print_summary();
    }
  }

private:
  Callable callable_;
  const char* name_;
  int sample_interval_;
  volatile int iters_;

  // Aggregate statistics
  ROIMetrics total_metrics_, max_metrics_, min_metrics_;

  // Only define cycles array if Mode is Full or Sample
  using MetricsArray = typename std::conditional<Mode == ProfileMode::Full || Mode == ProfileMode::Sample,
                                                 std::array<ROIMetrics, Iters>, void>::type;
  std::conditional_t<Mode == ProfileMode::Full || Mode == ProfileMode::Sample, MetricsArray, std::monostate> metrics_;

  // Update aggregate stats for each metric
  void update_aggregate(const ROIMetrics& metrics)
  {
    total_metrics_.elapsed_cycles += metrics.elapsed_cycles;
    total_metrics_.delta_cpi += metrics.delta_cpi;
    total_metrics_.delta_fold += metrics.delta_fold;
    total_metrics_.delta_lsu += metrics.delta_lsu;
    total_metrics_.delta_exc += metrics.delta_exc;

    max_metrics_.elapsed_cycles = std::max(max_metrics_.elapsed_cycles, metrics.elapsed_cycles);
    min_metrics_.elapsed_cycles = std::min(min_metrics_.elapsed_cycles, metrics.elapsed_cycles);
    
    // Repeat for other metrics
  }

  // Handle different profiling modes
  void handle_mode(const ROIMetrics& metrics, int iteration) {
    if constexpr (Mode == ProfileMode::Full)
    {
      metrics_[iteration] = metrics;
    }
    else if constexpr (Mode == ProfileMode::Aggregate)
    {
      update_aggregate(metrics);
    }
    else if constexpr (Mode == ProfileMode::Sample)
    {
      if (iteration % sample_interval_ == 0)
      {
        metrics_[iteration / sample_interval_] = metrics;
      }
    }
    else if constexpr (Mode == ProfileMode::PrintOnly)
    {
      print_metrics(metrics, iteration);
    }
  }

  // Print metrics for each iteration in PrintOnly mode
  void print_metrics(const ROIMetrics& metrics, int iteration) const {
    printf("Iteration %i: Cycles=%lu, CPIEvents=%lu, Folded=%lu, LSU=%lu, Exc=%lu\n",
           iteration, metrics.elapsed_cycles, metrics.delta_cpi, metrics.delta_fold,
           metrics.delta_lsu, metrics.delta_exc);
  }

  // Print summary results
  void print_summary() const {
    printf("Results from running %s for %i iterations.\n", name_, Iters);
    if constexpr (Mode == ProfileMode::Aggregate) {
      printf("Total cycles: %lu\n", total_metrics_.elapsed_cycles);
      printf("Average cycles: %lu\n", total_metrics_.elapsed_cycles / Iters);
      printf("Max cycles: %lu\n", max_metrics_.elapsed_cycles);
      printf("Min cycles: %lu\n", min_metrics_.elapsed_cycles);
      
      // Print similar aggregate values for other metrics
    } else if constexpr (Mode == ProfileMode::Full || Mode == ProfileMode::Sample) {
      int num_entries = (Mode == ProfileMode::Sample) ? (Iters / sample_interval_) : Iters;
      for (int i = 0; i < num_entries; ++i) {
        print_metrics(metrics_[i], i);
      }
    }
  }
};

// Deduction guide without explicit Mode (defaults to ProfileMode::Full)
template<int Iters, typename Callable>
Harness(Callable, const char* name, int sample_interval = 1, int iters = Iters)
    -> Harness<Iters, Callable, ProfileMode::Full>;

// Deduction guide with explicit Mode specified
template<int Iters, ProfileMode Mode, typename Callable>
Harness(Callable, const char* name, int sample_interval = 1, int iters = Iters)
    -> Harness<Iters, Callable, Mode>;

template<int Iters, typename Callable>
Harness<Iters, Callable, ProfileMode::Full> make_harness(Callable callable, const char* name, int sample_interval = 1) {
  return Harness<Iters, Callable, ProfileMode::Full>(callable, name, sample_interval);
}

template<int Iters, ProfileMode Mode, typename Callable>
Harness<Iters, Callable, Mode> make_harness(Callable callable, const char* name, int sample_interval = 1) {
  return Harness<Iters, Callable, Mode>(callable, name, sample_interval);
}

} // namespace bench

#endif // HARNESS_HH
