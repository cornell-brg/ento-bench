//========================================================================
// unittest.cc
//========================================================================
// Utility functions for unittesting. Modified from
// ECE2400/4750 testing framework.
//
//
// Author: Yanghui Ou, Peitian Pan, Derin Ozturk
//   Date: Oct 13, 2020

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <sys/time.h>
#include "unittest.h"


namespace EntoUtil
{
//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

/* ENTO_TEST_CHECK_* global variables */

// Initial value of __n should be > 0 so that debug info is not suppressed
// in *-adhoc.c files.
int   __n                 = 1;
int   __failed            = 0;
int   __failure_condition = 0;
int   __int_expr0         = 0;
int   __int_expr1         = 0;
float __float_expr0      = 0.0;
float __float_expr1      = 0.0;


//------------------------------------------------------------------------
// __ento_debug_get_file_name
//------------------------------------------------------------------------
// Return file name extracted from a __FILE__ string.

const char* __ento_debug_get_file_name( const char* full_path )
{
  using namespace EntoUtil;
  int len = strlen( full_path ), start_pos = 0;

  for ( int i = len-1; i >= 0; i-- )
    if ( full_path[i] == '/' ) {
      start_pos = i+1;
      break;
    }

  return full_path + start_pos;
}

//------------------------------------------------------------------------
// __ento_test_start
//------------------------------------------------------------------------
void __ento_test_start( const char* file )
{
  using namespace EntoUtil;
  file = __ento_debug_get_file_name( file );
  if ( __n < 0 ) std::printf( "%s" , file );
  if ( __n > 0 ) std::printf( "Running test cases in %s.\n", file);
}


//------------------------------------------------------------------------
// __ento_test_end
//------------------------------------------------------------------------
void __ento_test_end( const char* file )
{
  using namespace EntoUtil;
  file = __ento_debug_get_file_name( file );
  if ( __n < 0 ) std::printf( "\n" );
  if ( __n > 0 )
  {
    if ( __failed ) std::printf( __RED "Finished running test cases in %s.\n", file);
    else std::printf( __GREEN "Finished!\n");
  }
}

//------------------------------------------------------------------------
// __ento_test_fail
//------------------------------------------------------------------------

void __ento_test_fail( const char* file, int lineno, char *expr )
{
  using namespace EntoUtil;
  file = __ento_debug_get_file_name( file );
  if ( __n < 0 ) std::printf( __RED "." __RESET );
  std::printf(" - [ " __RED "FAILED" __RESET " ] File %s:%d:  %s\n", file, lineno, expr );
  __failed = 1;
}

//------------------------------------------------------------------------
// __ento_test_check_and_print_uniop
//------------------------------------------------------------------------

void __ento_test_check_and_print_uniop( const char* file, int lineno, const char* expr )
{
  using namespace EntoUtil;
  file = __ento_debug_get_file_name( file );
  if ( __failure_condition ) {
    if ( __n < 0 ) std::printf( "\n" );
    std::printf(" - [ " __RED "FAILED" __RESET " ] File %s:%d:  %s (%d)\n", file, lineno, expr, __int_expr0 );
    __failed = 1;
  } else if ( __n > 0 ) {
    std::printf(" - [ " __GREEN "passed" __RESET " ] File %s:%d:  %s (%d)\n", file, lineno, expr, __int_expr0 );
  } else if ( __n < 0 ) {
    std::printf( __GREEN "." __RESET );
  }
}

//------------------------------------------------------------------------
// __ento_test_check_and_print_int_binop
//------------------------------------------------------------------------

void __ento_test_check_and_print_int_binop( const char* file, int lineno, const char* expr1, const char* expr2 )
{
  using namespace EntoUtil;
  file = __ento_debug_get_file_name( file );
  if ( __failure_condition ) {
    if ( __n < 0 ) std::printf( "\n" );
    std::printf(" - [ " __RED "FAILED" __RESET " ] File %s:%d:  %s != %s (%d != %d)\n",
           file, lineno, expr1, expr2, __int_expr0, __int_expr1 );
    __failed = 1;
  } else if ( __n > 0 ) {
    std::printf(" - [ " __GREEN "passed" __RESET " ] File %s:%d:  %s == %s (%d == %d)\n",
           file, lineno, expr1, expr2, __int_expr0, __int_expr1 );
  } else if ( __n < 0 ) {
    std::printf( __GREEN "." __RESET );
  }
}

//------------------------------------------------------------------------
// __ento_test_check_and_print_matrix_binop
//------------------------------------------------------------------------
void __ento_test_check_and_print_matrix_binop( const char* file, int lineno, const char* expr1, const char* expr2 );

//------------------------------------------------------------------------
// __ento_test_check_and_print_float_binop
//------------------------------------------------------------------------

void __ento_test_check_and_print_float_binop( const char* file, int lineno, const char* expr1, const char* expr2 )
{
  using namespace EntoUtil;
  file = __ento_debug_get_file_name( file );
  if ( __failure_condition ) {
    if ( __n < 0 ) std::printf( "\n" );
    std::printf(" - [ " __RED "FAILED" __RESET " ] File %s:%d:  %s != %s (%.10e != %.10e)\n",
           file, lineno, expr1, expr2, __float_expr0, __float_expr1 );
    __failed = 1;
  } else if ( __n > 0 ) {
    std::printf(" - [ " __GREEN "passed" __RESET " ] File %s:%d:  %s == %s (%.10e == %.10e)\n",
           file, lineno, expr1, expr2, __float_expr0, __float_expr1 );
  } else if ( __n < 0 ) {
    std::printf( __GREEN ".");
  }
}

//------------------------------------------------------------------------
// __ento_test_num
//------------------------------------------------------------------------
bool __ento_test_num( int _n, int _id )
{
  using namespace EntoUtil;
  return ( ( _n <= 0 || _n == _id) );
}


//------------------------------------------------------------------------
// __ento_test_num
//------------------------------------------------------------------------
int __ento_get_test_num_from_file(const char* filename)
{
  FILE *config_file = fopen(filename, "r");
  int test_num = 0;
  if (config_file)
  {
    fscanf(config_file, "%d", &test_num);
    fclose(config_file);
  }
  return test_num;
}

//------------------------------------------------------------------------
// __ento_test_num
//------------------------------------------------------------------------
bool __ento_is_mcu_test()
{
#ifdef STM32_BUILD
  return true;
#endif
  return false;
}


void __ento_replace_file_suffix(const char* path, const char* new_suffix) {
  const char* last_slash = path;
  for (const char* p = path; *p != '\0'; ++p) {
    if (*p == '/' || *p == '\\') {
      last_slash = p + 1;
    }
  }

  int i = 0;
  for (const char* p = path; p != last_slash; ++p, ++i) {
    __ento_cmdline_args_path_buffer[i] = *p;
  }

  for (int j = 0; new_suffix[j] != '\0'; ++j, ++i) {
    __ento_cmdline_args_path_buffer[i] = new_suffix[j];
  }

  __ento_cmdline_args_path_buffer[i] = '\0';  // Null-terminate
}


}
