#include <stdlib.h>
#include <cstdio>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <mcu-util/clk_util.h>
#include <mcu-util/flash_util.h>
#include <mcu-util/pwr_util.h>
#include <system_stm32f7xx.h>

using namespace std;

constexpr uint32_t get_target_clock_frequency()
{
#ifdef STM32F756xx
  return 216000000;
#elif
  return 170000000;
#elif
  return 160000000;
#endif
}


extern "C" void initialise_monitor_handles(void);
extern int cmdline_buffer[];

void test_case_1_get_sys_clk_freq()
{
  uint32_t clk_freq = get_sys_clk_freq();
  
  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

}

void test_case_2_configure_max_clock_rate()
{
  uint32_t clk_freq = get_sys_clk_freq();

  printf("Current clk frequency (MHz): %.2f\n", clk_freq / 1000000.0);

  sys_clk_cfg();

  clk_freq = get_sys_clk_freq();

  printf("Clk frequency after running sys_clk_cfg() (MHz): %.2f\n", clk_freq / 1000000.0);
  
}

void semihosting_call(uint32_t operation, void* arg)
{
  asm volatile (
    "mov r0, %0\n"
    "mov r1, %1\n"
    "bkpt 0xAB \n"
    :
    : "r" (operation), "r" (arg)
    : "r0", "r1"
  );
}

int main( int argc, char ** argv)
{
  struct {
    uint32_t ac;
    char *av;
  } args;
  //semihosting_call(0x15, &args);
  //printf("cmdline_buffer is located at: %p\n", (void*)&cmdline_buffer);
  printf("Argc: %i", argc);
  printf("Running test clock config!\n");
  using namespace EntoUtil;
  initialise_monitor_handles();
  int __n;
  printf("Argc: %i\n", argc);
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
