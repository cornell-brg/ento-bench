#include <stdlib.h>
#include <stdio.h>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <mcu-util/clk_util.h>
#include <mcu-util/flash_util.h>
#include <mcu-util/pwr_util.h>

using namespace std;

constexpr uint32_t get_target_clock_frequency()
{
#if defined(STM32F756xx)
  return 216000000;
#elif defined(STM32G474xx)
  return 170000000;
#elif defined(STM32U575xx)
  return 160000000;
#elif defined(STM32G0B1xx)
  return  64000000;
#elif defined(STM32H7A3xx)
  return 280000000;
#endif
}


extern "C" void initialise_monitor_handles(void);

void test_case_1_get_sys_clk_freq()
{
  uint32_t clk_freq = get_sys_clk_freq();

  bool non_zero_clk = clk_freq != 0;

  ENTO_DEBUG("Current clk frequency (MHz): %.2f", clk_freq / 1000000.0);

  // Makes sure SystemCoreClockUpdate is being properly called in cmsis src.
  // @TODO: Do we need something that is actually checking what this value is at 
  //  startup before configuring to max clock rate?
  ENTO_TEST_CHECK_TRUE(non_zero_clk);

}

void test_case_2_configure_max_clock_rate()
{
  uint32_t clk_freq = get_sys_clk_freq();
  uint32_t target_clk_freq = get_target_clock_frequency();

  ENTO_DEBUG("Current clk frequency (MHz): %.2f", clk_freq / 1000000.0); 

  sys_clk_cfg();

  clk_freq = get_sys_clk_freq();

  ENTO_DEBUG("Clk frequency after running sys_clk_cfg() (MHz): %.2f", clk_freq / 1000000.0);
  ENTO_TEST_CHECK_INT_EQ(clk_freq, target_clk_freq);
}

int main( int argc, char ** argv)
{
  using namespace EntoUtil;
  initialise_monitor_handles();
  int __n;
  if (argc == 2)
  {
    printf("Got a passed in arg!\n");
    __n = atoi(argv[1]);
    printf("Passed in __n: %i\n", __n);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_clock_cfg_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }
  if (__ento_test_num(__n, 1)) test_case_1_get_sys_clk_freq();
  if (__ento_test_num(__n, 2)) test_case_2_configure_max_clock_rate();
  exit(1);
}
