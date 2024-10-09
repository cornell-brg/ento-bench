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
int    __n                 = 1;
int    __failed            = 0;
int    __failure_condition = 0;
int    __int_expr0         = 0;
int    __int_expr1         = 0;
double __double_expr0      = 0.0;
double __double_expr1      = 0.0;


//************************************************************************
// DO NOT DIRECTLY CALL THE FUNCTIONS BELOW!
//************************************************************************
// The functions implemented below are helper functions that should only
// be called in ENTO_TEST_CHECK_* macros.

//------------------------------------------------------------------------
// __ento_test_get_file_name
//------------------------------------------------------------------------
// Return file name extracted from a __FILE__ string.

const char* __ento_test_get_file_name( const char* full_path )
{
  int len = strlen( full_path ), start_pos = 0;

  for ( int i = len-1; i >= 0; i-- )
    if ( full_path[i] == '/' ) {
      start_pos = i+1;
      break;
    }

  return full_path + start_pos;
}

//------------------------------------------------------------------------
// __ento_test_fail
//------------------------------------------------------------------------

void __ento_test_fail( const char* file, int lineno, char *expr )
{
  file = __ento_test_get_file_name( file );
  if ( __n < 0 ) std::printf( "\n" );
  std::printf(" - [ " RED "FAILED" RESET " ] File %s:%d:  %s\n", file, lineno, expr );
  __failed = 1;
}

//------------------------------------------------------------------------
// __ento_test_check_and_print_uniop
//------------------------------------------------------------------------

void __ento_test_check_and_print_uniop( const char* file, int lineno, const char* expr )
{
  file = __ento_test_get_file_name( file );
  if ( __failure_condition ) {
    if ( __n < 0 ) std::printf( "\n" );
    std::printf(" - [ " RED "FAILED" RESET " ] File %s:%d:  %s (%d)\n", file, lineno, expr, __int_expr0 );
    __failed = 1;
  } else if ( __n > 0 ) {
    std::printf(" - [ " GREEN "passed" RESET " ] File %s:%d:  %s (%d)\n", file, lineno, expr, __int_expr0 );
  } else if ( __n < 0 ) {
    std::printf( GREEN "." RESET );
  }
}

//------------------------------------------------------------------------
// __ento_test_check_and_print_int_binop
//------------------------------------------------------------------------

void __ento_test_check_and_print_int_binop( const char* file, int lineno, const char* expr1, const char* expr2 )
{
  file = __ento_test_get_file_name( file );
  if ( __failure_condition ) {
    if ( __n < 0 ) std::printf( "\n" );
    std::printf(" - [ " RED "FAILED" RESET " ] File %s:%d:  %s != %s (%d != %d)\n",
           file, lineno, expr1, expr2, __int_expr0, __int_expr1 );
    __failed = 1;
  } else if ( __n > 0 ) {
    std::printf(" - [ " GREEN "passed" RESET " ] File %s:%d:  %s == %s (%d == %d)\n",
           file, lineno, expr1, expr2, __int_expr0, __int_expr1 );
  } else if ( __n < 0 ) {
    std::printf( GREEN "." RESET );
  }
}

//------------------------------------------------------------------------
// __ento_test_check_and_print_double_binop
//------------------------------------------------------------------------

void __ento_test_check_and_print_double_binop( const char* file, int lineno, const char* expr1, const char* expr2 )
{
  file = __ento_test_get_file_name( file );
  if ( __failure_condition ) {
    if ( __n < 0 ) std::printf( "\n" );
    std::printf(" - [ " RED "FAILED" RESET " ] File %s:%d:  %s != %s (%.10e != %.10e)\n",
           file, lineno, expr1, expr2, __double_expr0, __double_expr1 );
    __failed = 1;
  } else if ( __n > 0 ) {
    std::printf(" - [ " GREEN "passed" RESET " ] File %s:%d:  %s == %s (%.10e == %.10e)\n",
           file, lineno, expr1, expr2, __double_expr0, __double_expr1 );
  } else if ( __n < 0 ) {
    std::printf( GREEN "." RESET );
  }
}

}
