#ifndef HARNESS_HH
#define HARNESS_HH

#ifndef NATIVE
#include <ento-mcu/timing.h>
#endif

#ifdef NATIVE
#include <string>
#else
#include <cstring>

#endif

#include <tuple>
#include <cstdio>
#include <ento-bench/roi.h>
#include <array>
#include <limits>
#include <variant>
#include <type_traits>
#include <initializer_list>
#include <ento-util/experiment_io.h>


namespace EntoBench {
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



// Harness Class
//
// Templated on the Problem Specification, and optionally, the ProfileMode
// and number iterations. If the Problem Spec requires a dataset to operate
// then the number of iterations is equal to the number of experiments in the
// dataset. If the Problem Spec requires results are saved to a file after
// a single experiment, then the Harness will use Problem::deserialize in
// conjunction with ExperimentIO object that Harness holds. 

template<typename Problem, bool DoWarmup = false, size_t Reps=1>
class Harness {
public:
  // Public Members
  size_t iters_;

  Harness(Problem problem, const char* name)
    : problem_(std::move(problem)), 
      name_(name),
      total_metrics_({0, 0, 0, 0, 0}), max_metrics_({0, 0, 0, 0, 0}),
      min_metrics_({std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max()})
  {
    init_roi_tracking();
  }

#ifdef NATIVE
  Harness(Problem problem, std::string& name)
    : problem_(std::move(problem)), 
      name_(name.c_str()),
      total_metrics_({0, 0, 0, 0, 0}), max_metrics_({0, 0, 0, 0, 0}),
      min_metrics_({std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max()})
  {
    init_roi_tracking();
  }

  Harness(Problem problem,
          const std::string& name,
          const std::string& input_filepath,
          const std::string& output_filepath)
    : problem_(std::move(problem)),
      name_(name.c_str()),
      experiment_io_( // Explicitly construct only if dataset is required
      [&]() -> std::conditional_t<Problem::RequiresDataset_, EntoUtil::ExperimentIO, std::monostate> {
        if constexpr (Problem::RequiresDataset_) {
          return EntoUtil::ExperimentIO(input_filepath, output_filepath);
        } else {
          return {};
        }
      }())
  {
    init_roi_tracking();

    // Initialize ExperimentIO if necessary
    static_assert(Problem::RequiresDataset_, "Problem does not require dataset. Use constructor without filepaths.");
    //if constexpr (Problem::RequiresDataset_)
    //{
    //  experiment_io_ = std::move(EntoUtil::ExperimentIO(input_filepath, output_filepath));
    //}
  }
#else
  Harness(Problem problem,
          const char* name,
          const char* input_filepath,
          const char* output_filepath)
    : problem_(std::move(problem)),
      name_(name)
  {
    init_roi_tracking();

    // Initialize ExperimentIO if necessary
    static_assert(Problem::RequiresDataset_, "Problem does not require dataset. Use constructor without filepaths.");
    if constexpr (Problem::RequiresDataset_)
    {
      strncpy(input_filepath_, input_filepath, MAX_FILEPATH_LENGTH_ - 1);
      input_filepath_[MAX_FILEPATH_LENGTH_ - 1] = '\0';
      input_filepath_set_ = true;
    }
    if constexpr (Problem::SaveResults_)
    {
      strncpy(output_filepath_, output_filepath, MAX_FILEPATH_LENGTH_ - 1);
      output_filepath_[MAX_FILEPATH_LENGTH_ - 1] = '\0';
      output_filepath_set_ = true;

    }
    experiment_io_ = EntoUtil::ExperimentIO(input_filepath_, output_filepath_);
  }

#endif

  auto run() {
    if constexpr (Problem::RequiresDataset_)
    {
      if (!experiment_io_.is_input_open()) 
      {
        ENTO_DEBUG("Problem Specification needs input data but input file path not provided.");
        return;
      }
    }
    if constexpr (Problem::SaveResults_)
    {
      if (!experiment_io_.is_output_open()) 
      {
        ENTO_DEBUG("Problem Specification requires saving results but output file path not provided.");
        return;
      }
    }
        
    ENTO_DEBUG("Running benchmark %s for %lu iterations...\n", name_, Reps);
#if defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
    trigger_pin_high();
    Delay::ms(100);

    //software_delay_cycles(1000000);
#endif // defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
    
    if constexpr (!Problem::RequiresDataset_)
    {
      if constexpr (DoWarmup)
      {
        start_roi();
        problem_.solve();
        end_roi();
        cold_metrics_ = get_roi_stats();
        update_aggregate(cold_metrics_);
      }
      // Benchmark kernel (algorithm implementation, callable) that Problem Specification holds.
      size_t i;
      for (i = 0; i < Reps; i++)
      {
        start_roi();
        problem_.solve();
        end_roi();
        // Get Stats. Update global metrics.
        metrics_ = get_roi_stats();
        update_aggregate(metrics_);
      }
      iters_ = i;

      // Save Results if necessary for Problem Spec
      if constexpr (Problem::SaveResults_)
      {
        experiment_io_.write_results(problem_);
      }
#if defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
      Delay::ms(100);
        //software_delay_cycles(100000); // delay in between experiments
#endif // defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
    }
    else
    {
      size_t i = 0;
      while (experiment_io_.read_next(problem_))
      {
#if defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
        Delay::ms(50);
#endif
        //software_delay_cycles(100000); // delay in between experiments
        if constexpr (DoWarmup)
        {
          start_roi();
          problem_.solve();
          end_roi();
          cold_metrics_ = get_roi_stats();
          update_aggregate(cold_metrics_);
        }
        // Benchmark kernel (algorithm implementation, callable) that Problem Specification holds.
        for (size_t i = 0; i < Reps; i++)
        {
          start_roi();
          problem_.solve();
          end_roi();
          // Get Stats. Update global metrics.
          metrics_ = get_roi_stats();
          update_aggregate(metrics_);
        }

        // Save Results if necessary for Problem Spec
        if constexpr (Problem::SaveResults_)
        {
          experiment_io_.write_results(problem_);
        }
        ++i;
#if defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
        Delay::ms(50);
        //software_delay_cycles(100000); // delay in between experiments
#endif // defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
      }
      iters_ = i; // Number of experiments for full dataset. We do not figure this out until after processing all.
      ENTO_DEBUG("Finished running %i experiments!", iters_);
    }

#if defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)
    //software_delay_cycles(10000);
    Delay::ms(50);
    trigger_pin_low(); // trigger pin low to signal end of all experiments
#endif // defined(STM32_BUILD) & defined(LATENCY_MEASUREMENT)

    // Print summary of global metrics. Depends on profile mode of Harness.
#ifndef NATIVE
    print_summary();
#endif
  }

private:
  
  //////// Private Class Members /////////

  Problem problem_;
  const char* name_;
  // Only define experiment_io_ if we have a Problem that requires a dataset
  std::conditional_t<Problem::RequiresDataset_,
                     EntoUtil::ExperimentIO,
                     std::monostate> experiment_io_;
#ifndef NATIVE
  static constexpr size_t MAX_FILEPATH_LENGTH_ = 128;
  char input_filepath_[MAX_FILEPATH_LENGTH_] = {0};
  bool input_filepath_set_ = false;

  char output_filepath_[MAX_FILEPATH_LENGTH_] = {0};
  bool output_filepath_set_ = false;
#endif

  // Aggregate statistics
  ROIMetrics total_metrics_, max_metrics_, min_metrics_;
  ROIMetrics metrics_;

  std::conditional_t<DoWarmup, ROIMetrics, std::monostate> cold_metrics_;


  //////// Private Class Functions /////////
  
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
    
  }

  // Print metrics for each iteration in PrintOnly mode
  void print_metrics(const ROIMetrics& metrics, int iteration) const {
    printf("Iteration %i: Cycles=%lu, CPIEvents=%lu, Folded=%lu, LSU=%lu, Exc=%lu\n",
           iteration, metrics.elapsed_cycles, metrics.delta_cpi, metrics.delta_fold,
           metrics.delta_lsu, metrics.delta_exc);
  }

  // Print summary results
  void print_summary() const
  {
    software_delay_cycles(1000000);
    printf("Results from running %s for %i iterations.\n", name_, iters_);

    if constexpr (DoWarmup)
    {
      printf("Cold cycles: %lu\n",      cold_metrics_.elapsed_cycles);
      printf("Cold cpi events: %lu\n",  cold_metrics_.delta_cpi);
      printf("Cold fold events: %lu\n", cold_metrics_.delta_fold);
      printf("Cold lsu events: %lu\n",  cold_metrics_.delta_lsu);
      printf("Cold exc events: %lu\n",  cold_metrics_.delta_exc);

      printf("Warm cycles: %lu\n",      metrics_.elapsed_cycles);
      printf("Warm cpi events: %lu\n",  metrics_.delta_cpi);
      printf("Warm fold events: %lu\n", metrics_.delta_fold);
      printf("Warm lsu events: %lu\n",  metrics_.delta_lsu);
      printf("Warm exc events: %lu\n",  metrics_.delta_exc);
    }
    else
    {
      printf("No warmup cycles: %lu\n",      metrics_.elapsed_cycles);
      printf("No warmup cpi events: %lu\n",  metrics_.delta_cpi);
      printf("No warmup fold events: %lu\n", metrics_.delta_fold);
      printf("No warmup lsu events: %lu\n",  metrics_.delta_lsu);
      printf("No warmup exc events: %lu\n",  metrics_.delta_exc);
    }
    printf("Total cycles: %lu\n", total_metrics_.elapsed_cycles);
    printf("Average cycles: %lu\n", total_metrics_.elapsed_cycles / iters_);
    printf("Max cycles: %lu\n", max_metrics_.elapsed_cycles);
    printf("Min cycles: %lu\n", min_metrics_.elapsed_cycles);
  }
};

// Deduction guide for the basic constructor (without a warmup flag):
template<typename Problem>
Harness(Problem, const char*) -> Harness<Problem, false, 1>;

#ifdef NATIVE
template<typename Problem>
Harness(Problem, const std::string&, const std::string&, const std::string&, const std::string&)
  -> Harness<Problem, false, 1>;
#else
template<typename Problem>
Harness(Problem, const char*, const char*, const char*)
  -> Harness<Problem, false, 1>;
#endif

// Deduction guide without explicit Mode (defaults to ProfileMode::Full)
//template<int Iters, typename Callable>
//Harness(Callable, const char* name, int sample_interval = 1, int iters = Iters)
//    -> Harness<Iters, Callable, ProfileMode::Full>;
//
//// Deduction guide with explicit Mode specified
//template<int Iters, ProfileMode Mode, typename Callable>
//Harness(Callable, const char* name, int sample_interval = 1, int iters = Iters)
//    -> Harness<Iters, Callable, Mode>;
//
//template<int Iters, typename Callable>
//Harness<Iters, Callable, ProfileMode::Full> make_harness(Callable callable, const char* name, int sample_interval = 1) {
//  return Harness<Iters, Callable, ProfileMode::Full>(callable, name, sample_interval);
//}
//
//template<int Iters, ProfileMode Mode, typename Callable>
//Harness<Iters, Callable, Mode> make_harness(Callable callable, const char* name, int sample_interval = 1) {
//  return Harness<Iters, Callable, Mode>(callable, name, sample_interval);
//}


} // namespace EntoBench

#endif // HARNESS_HH
