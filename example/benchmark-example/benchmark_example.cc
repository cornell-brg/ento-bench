#include <stdlib.h>
#include <stdio.h>
#include "bench/harness.hh"

extern "C" void initialise_monitor_handles(void);

void __attribute__((noinline)) hello_host_computer()
{
  printf("Hello host computer!\n");
}

int main()
{
  using namespace bench;
  initialise_monitor_handles();

  constexpr int reps = 10;
  Harness hello_host_harness(hello_host_computer,
                             "Hello Host Computer Benchmark Example", reps);
  hello_host_harness.run();
  printf("Finished running hello host computer benchmark example!\n");

  exit(1);

  return 0;
}
