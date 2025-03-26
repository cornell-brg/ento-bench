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

template<typename Problem, bool DoWarmup = false, size_t Reps=1, int Verbosity = 1>
class Harness {
public:
  // Public Members
  size_t iters_;

  Harness(Problem problem, const char* name)
    : problem_(std::move(problem)), 
      name_(name),
      total_metrics_({0, 0, 0, 0, 0}), max_metrics_({0, 0, 0, 0, 0}),
      min_metrics_({std::numeric_limits<uint64_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max()})
  {
    init_roi_tracking();

    if constexpr (DoWarmup)
    {
      cold_metrics_ = { 0 };
    }

    if constexpr (Reps > 1)
    {
      total_rep_metrics_ = { 0 };
      min_rep_metrics_   = { std::numeric_limits<uint64_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max() };
      max_rep_metrics_ = { 0 };
    }
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

    if constexpr (DoWarmup)
    {
      cold_metrics_ = { 0 };
    }

    if constexpr (Reps > 1)
    {
      total_rep_metrics_ = { 0 };
      min_rep_metrics_   = { std::numeric_limits<uint64_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max() };
      max_rep_metrics_ = { 0 };
    }

  }

  Harness(Problem problem,
          const std::string& name,
          const std::string& input_filepath,
          const std::string& output_filepath)
    : problem_(std::move(problem)),
      name_(name.c_str()),
      total_metrics_({0, 0, 0, 0, 0}), max_metrics_({0, 0, 0, 0, 0}),
      min_metrics_({std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max()}),
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

    if constexpr (DoWarmup)
    {
      cold_metrics_ = { 0 };
    }

    if constexpr (Reps > 1)
    {
      total_rep_metrics_ = { 0 };
      min_rep_metrics_   = { std::numeric_limits<uint64_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max() };
      max_rep_metrics_ = { 0 };
    }

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
      metrics_({0, 0, 0, 0, 0}),
      total_metrics_({0, 0, 0, 0, 0}), max_metrics_({0, 0, 0, 0, 0}),
      min_metrics_({std::numeric_limits<uint64_t>::max(),
                    std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max(),
                    std::numeric_limits<uint32_t>::max()}),
      rep_metrics_({0, 0, 0, 0, 0}),
      name_(name)
  {
    init_roi_tracking();

    if constexpr (DoWarmup)
    {
      cold_metrics_ = { 0 };
    }

    if constexpr (Reps > 1)
    {
      total_rep_metrics_ = { 0 };
      min_rep_metrics_   = { std::numeric_limits<uint64_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max() };
      max_rep_metrics_ = { 0 };
    }

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
        
    ENTO_DEBUG("Running benchmark %s...\n", name_);
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
        Delay::ms(10);
        start_roi();
        problem_.solve();
        end_roi();
        // Get Stats. Update global metrics.
        Delay::ms(10);
        rep_metrics_ = get_roi_stats();

        if constexpr (Reps > 1 && Verbosity == 1)
        {
          print_rep_metrics(rep_metrics_, i); 
          Delay::ms(50);
        }
        else if constexpr (Reps > 1 && Verbosity == 2)
        {
          print_rep_metrics_verbose(rep_metrics_, i);
          Delay::ms(50);
        }

        if constexpr (Reps > 1)
        {
          update_reps_aggregate(rep_metrics_);
        }

      }
      iters_ = i;

      // Save Results if necessary for Problem Spec
      if constexpr (Problem::SaveResults_)
      {
        experiment_io_.write_results(problem_);
      }

      if constexpr (Reps == 1 && Verbosity == 1)
      {
        print_metrics(metrics_, i);
      }
      else if constexpr (Reps == 1 && Verbosity == 2)
      {
        print_metrics_verbose(metrics_, i);
      }
      else if constexpr (Reps > 1 && Verbosity > 0)
      {
        print_reps_summary();
      }

      if (Reps > 1)
      {
        //metrics_ = rep_metrics_;
        metrics_.delta_cpi  = total_rep_metrics_.delta_cpi / Reps;
        metrics_.delta_lsu  = total_rep_metrics_.delta_lsu / Reps;
        metrics_.delta_exc  = total_rep_metrics_.delta_exc / Reps;
        metrics_.delta_fold = total_rep_metrics_.delta_fold / Reps;
        metrics_.elapsed_cycles = total_rep_metrics_.elapsed_cycles / Reps;
        clear_repetition_metrics();
        update_aggregate(metrics_);
      }
      else
      {
        update_aggregate(metrics_);
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
          Delay::ms(10);
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
          Delay::ms(10);
          rep_metrics_ = get_roi_stats();

          if constexpr (Reps > 1)
          {
            update_reps_aggregate(rep_metrics_);
          }

          if constexpr (Reps > 1 && Verbosity == 1)
          {
            print_rep_metrics(rep_metrics_, i); 
            Delay::ms(50);
          }
          else if constexpr (Reps > 1 && Verbosity == 2)
          {
            print_rep_metrics_verbose(rep_metrics_, i);
            Delay::ms(50);
          }
        }

        // Save Results if necessary for Problem Spec
        if constexpr (Problem::SaveResults_)
        {
          experiment_io_.write_results(problem_);
        }
        ++i;

        // Optionally print out current metrics.
        // @TODO: Fix verbosity to be enum type maybe...
        //if constexpr (Reps == 1 && Verbosity == 1)
        //{
        //  print_metrics(metrics_, i);
        //}
        //else if constexpr (Reps == 1 && Verbosity == 2)
        //{
        //  print_metrics_verbose(metrics_, i);
        //}
        //else if constexpr (Reps > 1 && Verbosity > 0)
        //{
        //  print_reps_summary();
        //}

        if constexpr (Reps > 1)
        {
          //metrics_ = rep_metrics_;
          metrics_.delta_cpi  = total_rep_metrics_.delta_cpi / Reps;
          metrics_.delta_lsu  = total_rep_metrics_.delta_lsu / Reps;
          metrics_.delta_exc  = total_rep_metrics_.delta_exc / Reps;
          metrics_.delta_fold = total_rep_metrics_.delta_fold / Reps;
          metrics_.elapsed_cycles = total_rep_metrics_.elapsed_cycles / Reps;
          update_aggregate(metrics_);
          if constexpr (Verbosity > 0)
          {
            print_reps_summary();
          }
          clear_repetition_metrics();
        }
        else
        {
          metrics_.delta_cpi      = rep_metrics_.delta_cpi;
          metrics_.delta_lsu      = rep_metrics_.delta_lsu;
          metrics_.delta_exc      = rep_metrics_.delta_exc;
          metrics_.delta_fold     = rep_metrics_.delta_fold;
          metrics_.elapsed_cycles = rep_metrics_.elapsed_cycles;
          update_aggregate(metrics_);
          if constexpr (Verbosity == 1)
          {
            print_metrics(metrics_, i);
          }
          else if constexpr (Verbosity == 2)
          {
            print_metrics_verbose(metrics_, i);
          }
          clear_repetition_metrics();
        }

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
  ROIMetrics metrics_, rep_metrics_, total_rep_metrics_, max_rep_metrics_, min_rep_metrics_;

  std::conditional_t<DoWarmup,
                     ROIMetrics, std::monostate> cold_metrics_;

  //std::conditional_t<(Reps > 1),
  //                   ROIMetrics, std::monostate> total_rep_metrics_;

  //std::conditional_t<(Reps > 1),
  //                   ROIMetrics, std::monostate> max_rep_metrics_;

  //std::conditional_t<(Reps > 1),
  //                   ROIMetrics, std::monostate> min_rep_metrics_;



  //////// Private Class Functions /////////
  
  // Update aggregate stats for each metric
  void update_aggregate(const ROIMetrics& metrics)
  {
    total_metrics_.elapsed_cycles  += metrics.elapsed_cycles;

    total_metrics_.delta_cpi      += metrics.delta_cpi;
    total_metrics_.delta_fold     += metrics.delta_fold;
    total_metrics_.delta_lsu      += metrics.delta_lsu;
    total_metrics_.delta_exc      += metrics.delta_exc;

    max_metrics_.elapsed_cycles = std::max(max_metrics_.elapsed_cycles, metrics.elapsed_cycles);
    min_metrics_.elapsed_cycles = std::min(min_metrics_.elapsed_cycles, metrics.elapsed_cycles);
  }

  void update_reps_aggregate(const ROIMetrics& metrics)
  {
    total_rep_metrics_.elapsed_cycles  += metrics.elapsed_cycles;

    total_rep_metrics_.delta_cpi      += metrics.delta_cpi;
    total_rep_metrics_.delta_fold     += metrics.delta_fold;
    total_rep_metrics_.delta_lsu      += metrics.delta_lsu;
    total_rep_metrics_.delta_exc      += metrics.delta_exc;

    max_rep_metrics_.elapsed_cycles = std::max(max_rep_metrics_.elapsed_cycles, metrics.elapsed_cycles);
    min_rep_metrics_.elapsed_cycles = std::min(min_rep_metrics_.elapsed_cycles, metrics.elapsed_cycles);

  }

  //void update_repetition_metrics(const ROIMetrics& metrics)
  //{
  //  if constexpr ( Reps > 1 && Verbosity >= 1)
  //  {
  //    rep_metrics_.elapsed_cycles += metrics.elapsed_cycles;
  //    rep_metrics_.delta_cpi      += metrics.delta_cpi;
  //    rep_metrics_.delta_fold     += metrics.delta_fold;
  //    rep_metrics_.delta_lsu      += metrics.delta_lsu;
  //    rep_metrics_.delta_exc      += metrics.delta_exc;
  //  }
  //}

  void clear_repetition_metrics()
  {
    if constexpr ( Reps > 1 )
    {
      rep_metrics_       = { 0 };
      total_rep_metrics_ = { 0 };
      min_rep_metrics_   = { std::numeric_limits<uint64_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max(),
                             std::numeric_limits<uint32_t>::max() };
      max_rep_metrics_   = { 0 };
    }
  }

  // Print metrics for each iteration in PrintOnly mode
  void print_metrics(const ROIMetrics& metrics, int iteration) const {
    printf("Iteration %i: Cycles = %lu \n", iteration, metrics.elapsed_cycles);
  }

  void print_rep_metrics(const ROIMetrics& metrics, int iteration) const {
    printf("Rep %i: Cycles = %lu \n", iteration, metrics.elapsed_cycles);
  }

  void print_metrics_verbose(const ROIMetrics& metrics, int iteration) const {
    printf("Iteration %i: Cycles=%lu, CPIEvents=%lu, Folded=%lu, LSU=%lu, Exc=%lu\n",
           iteration, metrics.elapsed_cycles, metrics.delta_cpi, metrics.delta_fold,
           metrics.delta_lsu, metrics.delta_exc);
  }

  void print_rep_metrics_verbose(const ROIMetrics& metrics, int iteration) const {
    printf("Rep %i: Cycles=%lu, CPIEvents=%lu, Folded=%lu, LSU=%lu, Exc=%lu\n",
           iteration, metrics.elapsed_cycles, metrics.delta_cpi, metrics.delta_fold,
           metrics.delta_lsu, metrics.delta_exc);
  }

  void print_reps_summary() const
  {
    printf("Results from running %s for %i repetitions.\n", name_, Reps);
    printf("Average cycles: %.3f\n", (double) total_rep_metrics_.elapsed_cycles / Reps);
    printf("Max cycles: %u\n", (uint32_t) max_rep_metrics_.elapsed_cycles);
    printf("Min cycles: %u\n", (uint32_t) min_rep_metrics_.elapsed_cycles);
    printf("========================================\n");

  }

  // Print summary results
  void print_summary() const
  {
    printf("Results from running %s for %i iterations.\n", name_, iters_);

    if constexpr (DoWarmup)
    {
      printf("Cold cycles: %lu\n",      cold_metrics_.elapsed_cycles);
      if constexpr (Verbosity == 2)
      {
        printf("Cold cpi events: %lu\n",  cold_metrics_.delta_cpi);
        printf("Cold fold events: %lu\n", cold_metrics_.delta_fold);
        printf("Cold lsu events: %lu\n",  cold_metrics_.delta_lsu);
        printf("Cold exc events: %lu\n",  cold_metrics_.delta_exc);
      }
      printf("Final ROI cycles: %u\n",     (uint32_t) metrics_.elapsed_cycles);
      if constexpr (Verbosity == 2)
      {
        printf("Final ROI cpi events:  %u\n",  metrics_.delta_cpi);
        printf("Final ROI fold events: %u\n", metrics_.delta_fold);
        printf("Final ROI lsu events:  %u\n",  metrics_.delta_lsu);
        printf("Final ROI exc events:  %u\n",  metrics_.delta_exc);
      }
    }
    //uint32_t avg_cycles = (uint32_t) (total_metrics_.elapsed_cycles / )
    //printf("Average cycles: %lu\n", total_metrics_.elapsed_cycles / iters_);
    printf("Average cycles: %.3f\n", (double)total_metrics_.elapsed_cycles / iters_);
    printf("Max cycles: %u\n", (uint32_t) max_metrics_.elapsed_cycles);
    printf("Min cycles: %u\n", (uint32_t) min_metrics_.elapsed_cycles);
  }
};

// Deduction guide for the basic constructor (without a warmup flag):
template<typename Problem>
Harness(Problem, const char*) -> Harness<Problem, false, 1, 1>;

#ifdef NATIVE
template<typename Problem>
Harness(Problem, const std::string&, const std::string&, const std::string&, const std::string&)
  -> Harness<Problem, false, 1>;
#else
template<typename Problem>
Harness(Problem, const char*, const char*, const char*)
  -> Harness<Problem, false, 1, 1>;
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
