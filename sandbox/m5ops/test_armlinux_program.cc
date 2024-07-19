#include <cstdio>
#include <gem5/m5ops.h>

int main()
{
  m5_dump_reset_stats(0,0);
  printf("Hello world!");
  m5_dump_reset_stats(0,0);
  
  return 0;
}
