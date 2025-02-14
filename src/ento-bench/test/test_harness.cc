#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-bench/harness.h>
#include <ento-bench/test/TestProblem.h>

const char* file_path = __FILE__;
constexpr size_t FILEPATH_SIZE = 128;

char dir_path[FILEPATH_SIZE];
char test1_input_path[FILEPATH_SIZE];
char test1_output_path[FILEPATH_SIZE];
char test2_input_path[FILEPATH_SIZE];
char test2_output_path[FILEPATH_SIZE];

char* full_paths[] = { test1_input_path, test1_output_path,
                       test2_input_path, test2_output_path };
constexpr size_t num_paths = 4;



void __attribute__((noinline)) add64x8()
{
#if defined(ARMV7E_M)
  asm volatile (
    ".rept 64        \n"   // Repeat 8 times
    "add r0, r0, #1  \n"   // Add 1 to r0
    "add r1, r1, #1  \n"   // Add 1 to r1
    "add r2, r2, #1  \n"   // Add 1 to r2
    "add r3, r3, #1  \n"   // Add 1 to r3
    "add r4, r4, #1  \n"   // Add 1 to r4
    "add r5, r5, #1  \n"   // Add 1 to r5
    "add r6, r6, #1  \n"   // Add 1 to r6
    "add r7, r7, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"  // Clobbered registers
  );
#elif defined(ARM64)
  asm volatile (
    ".rept 64        \n"   // Repeat 8 times
    "add x0, x0, #1  \n"   // Add 1 to r0
    "add x1, x1, #1  \n"   // Add 1 to r1
    "add x2, x2, #1  \n"   // Add 1 to r2
    "add x3, x3, #1  \n"   // Add 1 to r3
    "add x4, x4, #1  \n"   // Add 1 to r4
    "add x5, x5, #1  \n"   // Add 1 to r5
    "add x6, x6, #1  \n"   // Add 1 to r6
    "add x7, x7, #1  \n"   // Add 1 to r7
    ".endr            \n"   // End repeat block
    : // No outputs
    : // No inputs
    : "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7"  // Clobbered registers
  );
#elif defined(X86)
  asm volatile (
    ".rept 64        \n"
    "addl $1, %%rax  \n"   // Increment eax
    "addl $1, %%rbx  \n"   // Increment ebx
    "addl $1, %%rcx  \n"   // Increment ecx
    "addl $1, %%rdx  \n"   // Increment edx
    "addl $1, %%rsi  \n"   // Increment esi
    "addl $1, %%rdi  \n"   // Increment edi
    "addl $1, %%r8  \n"   // Increment r8 (x86-64 only)
    "addl $1, %%r9  \n"   // Increment r9 (x86-64 only)
    ".endr           \n"
    :
    :
    : "eax", "ebx", "ecx", "edx", "esi", "edi", "r8d", "r9d"  // Clobbered registers
  );
#endif

}

void test_harness_basic_problem_single()
{
  std::string name = "Add 64x8 Microbenchmark";
  auto basic_problem = EntoBench::make_basic_problem(add64x8);
  EntoBench::Harness harness(basic_problem, name);

  harness.run();
  
  ENTO_TEST_CHECK_TRUE(harness.iters_ == 1);
}

void test_harness_basic_problem_multi()
{
  std::string name = "Add 64x8 Microbenchmark";
  auto basic_problem = EntoBench::make_basic_problem(add64x8);
  using Harness = EntoBench::Harness<decltype(basic_problem), false, 10>;
  Harness harness(basic_problem, name);

  harness.run();
  
  ENTO_TEST_CHECK_TRUE(harness.iters_ == 10);
}

void test_harness_test_problem_single_native()
{
  auto vvadd = [](auto& a, auto& b, auto& c) {
      // Assume a.size() equals b.size() and equals c.size()
      for (size_t i = 0; i < a.size(); ++i) {
          c[0] += a[i] * b[i];
      }
  };
   
  std::string name = "Test Harness Basic Native";
  EntoBench::TestProblem<decltype(vvadd), int, 0, 0> test_problem(vvadd);
  EntoBench::Harness harness(test_problem, name,
      test1_input_path, test1_output_path);

  harness.run();

  ENTO_TEST_CHECK_TRUE(harness.iters_ == 1);
}

void test_harness_test_problem_multi_native()
{
  auto vvadd = [](auto& a, auto& b, auto& c) {
      // Assume a.size() equals b.size() and equals c.size()
      for (size_t i = 0; i < a.size(); ++i) {
          c[0] += a[i] * b[i];
      }
  };
   
  std::string name = "Test Harness Multi Native";
  EntoBench::TestProblem<decltype(vvadd), int, 0, 0> test_problem(vvadd);
  EntoBench::Harness harness(test_problem, name,
      test2_input_path, test2_output_path);

  harness.run();

  ENTO_TEST_CHECK_TRUE(harness.iters_ == 5);
}

int main ( int argc, char ** argv )
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
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  // Setup Directory Path and Test Data Paths
  get_file_directory(file_path, sizeof(dir_path), dir_path);
  const char* file_names[] = { "test_harness_dot_input_1.txt" , "test_harness_dot_output_1.txt",
                               "test_harness_dot_input_2.txt" , "test_harness_dot_output_2.txt" };
  build_file_paths(dir_path, file_names, full_paths, FILEPATH_SIZE, num_paths);
  
  printf("Generated Paths:\n");
  for (size_t i = 0; i < num_paths; ++i) {
    printf("  %s\n", full_paths[i]);
  }
  // Run Tests

  if (__ento_test_num(__n, 1)) test_harness_basic_problem_single();
  if (__ento_test_num(__n, 2)) test_harness_basic_problem_multi();
  if (__ento_test_num(__n, 3)) test_harness_test_problem_single_native();
  if (__ento_test_num(__n, 4)) test_harness_test_problem_multi_native();
}

