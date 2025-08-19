#include <iostream>
#include <stdlib.h>

#include <ento-util/unittest.h>

using namespace std;

// A simple test that just prints to cout
void test_print() {
  cout << "This is a test print to cout" << endl;
  std::cerr << "This is a test print to cerr" << endl;
  fprintf(stdout, "This is a test print to stdout via fprintf\n");
  fprintf(stderr, "This is a test print to stderr via fprintf\n");
  printf("This is a test print via printf\n");
}

int main(int argc, char** argv)
{
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_print_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_print();
  
  ENTO_TEST_END();
} 