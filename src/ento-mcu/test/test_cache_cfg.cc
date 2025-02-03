#include <stdlib.h>
#include <stdio.h>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/pwr_util.h>
#include <ento-mcu/cache_util.h>

using namespace std;

extern "C" void initialise_monitor_handles(void);

void test_case_1_enable_instruction_cache()
{
#if defined(STM32U5) || defined(STM32F7) || defined(STM32H7)
  bool icache_en;
  icache_enable();
  icache_en = icache_is_enabled();
  ENTO_TEST_CHECK_TRUE(icache_en);

  icache_disable();
  icache_en = icache_is_enabled();
  ENTO_TEST_CHECK_FALSE(icache_en);
#else
  ENTO_TEST_CHECK_TRUE(true);
#endif
}

void test_case_2_enable_disable_data_cache()
{
#if defined(STM32U5) || defined(STM32F7) || defined(STM32H7)
  bool dcache_en;
  dcache_enable();
  dcache_en = dcache_is_enabled();
  ENTO_TEST_CHECK_TRUE(dcache_en);

  dcache_disable();
  dcache_en = dcache_is_enabled();
  ENTO_TEST_CHECK_FALSE(dcache_en);
  ENTO_TEST_CHECK_FALSE(dcache_en);
#else
  ENTO_TEST_CHECK_TRUE(true);
#endif
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

  if (__ento_test_num(__n, 1)) test_case_1_enable_instruction_cache(); 
  if (__ento_test_num(__n, 2)) test_case_2_enable_disable_data_cache();
  exit(1);
}
