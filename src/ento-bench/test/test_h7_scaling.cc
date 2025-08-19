#include <ento-bench/bench_config.h>
#include <ento-util/unittest.h>
#include <ento-util/debug.h>

using namespace EntoBench;

void test_h7_multiplier_detection()
{
  ENTO_DEBUG("=== H7 Performance Multiplier Test ===");
  
  // Show the detected multiplier
  ENTO_DEBUG("H7_PERFORMANCE_MULTIPLIER: %zu", H7_PERFORMANCE_MULTIPLIER);
  
  // Show base and effective inner reps
  constexpr std::size_t test_base_inner_reps = 2;  // Typical EKF value
  constexpr std::size_t effective_inner_reps = test_base_inner_reps * H7_PERFORMANCE_MULTIPLIER;
  
  ENTO_DEBUG("Base INNER_REPS: %zu", test_base_inner_reps);
  ENTO_DEBUG("Effective INNER_REPS: %zu", effective_inner_reps);
  
  // Verify the multiplier is reasonable
  ENTO_TEST_CHECK_TRUE(H7_PERFORMANCE_MULTIPLIER >= 1);
  ENTO_TEST_CHECK_TRUE(H7_PERFORMANCE_MULTIPLIER <= 10);  // Sanity check
  
  // Show platform detection
#if defined(STM32H7) || defined(STM32H7xx) || defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx)
  ENTO_DEBUG("Platform: H7 detected - scaling enabled");
  ENTO_TEST_CHECK_TRUE(H7_PERFORMANCE_MULTIPLIER > 1);
#else
  ENTO_DEBUG("Platform: Non-H7 - no scaling");
  ENTO_TEST_CHECK_TRUE(H7_PERFORMANCE_MULTIPLIER == 1);
#endif

  // Verify CMake/C++ sync if available
#ifdef H7_CMAKE_MULTIPLIER
  ENTO_DEBUG("CMake multiplier: %d", H7_CMAKE_MULTIPLIER);
  ENTO_DEBUG("C++ multiplier: %zu", H7_PERFORMANCE_MULTIPLIER);
  ENTO_TEST_CHECK_TRUE(H7_PERFORMANCE_MULTIPLIER == H7_CMAKE_MULTIPLIER);
  ENTO_DEBUG("CMake and C++ multipliers are in sync ✓");
#else
  ENTO_DEBUG("CMake multiplier not provided (normal for non-benchmark builds)");
#endif
}

void test_default_harness_scaling()
{
  ENTO_DEBUG("=== Default Harness Scaling Test ===");
  
  // Show the default values that would be used by harness
  ENTO_DEBUG("DefaultReps: %zu", DefaultReps);
  ENTO_DEBUG("DefaultInnerReps: %zu", DefaultInnerReps);
  ENTO_DEBUG("DefaultVerbosity: %d", DefaultVerbosity);
  ENTO_DEBUG("DefaultMaxProblems: %zu", DefaultMaxProblems);
  
  // Verify that DefaultInnerReps includes H7 scaling
  constexpr std::size_t expected_base = 1;  // Default base value
  constexpr std::size_t expected_scaled = expected_base * H7_PERFORMANCE_MULTIPLIER;
  
  // If INNER_REPS is not defined, DefaultInnerReps should equal the scaled value
#ifndef INNER_REPS
  ENTO_TEST_CHECK_TRUE(DefaultInnerReps == expected_scaled);
  ENTO_DEBUG("DefaultInnerReps correctly scaled: %zu = %zu * %zu", 
             DefaultInnerReps, expected_base, H7_PERFORMANCE_MULTIPLIER);
#else
  // If INNER_REPS is defined, it should be scaled
  ENTO_DEBUG("INNER_REPS defined, DefaultInnerReps = %zu", DefaultInnerReps);
#endif
}

int main(int argc, char** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_h7_scaling_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_h7_multiplier_detection();
  if (__ento_test_num(__n, 2)) test_default_harness_scaling();
  
  ENTO_TEST_END();
} 