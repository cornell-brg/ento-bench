//========================================================================
// unittest.h
//========================================================================
// Unit testing framework for EntoBench. Modified from
// ECE2400/4750 testing framework.
//
// Author: Yanghui Ou, Peitian Pan, Derin Ozturk
//   Date: Oct 13, 2020

#ifndef ENTO_UNITTEST_H
#define ENTO_UNITTEST_H

#include <cstdio>
#include <cmath>

namespace EntoUtil 
{

#define RED    "\033[31m"
#define GREEN  "\033[32m"
#define YELLOW "\033[33m"
#define RESET  "\033[0m"

constexpr float __rel_tol_pct = 0.01;

typedef unsigned int uint_t;

//========================================================================
// ENTO_TEST_CHECK_* macros
//========================================================================

// __n  > 0: display full [ passed ] line
// __n == 0: do not display anything for passed case
// __n  < 0: display a dot for passed case
extern int __n;

// The status of the current test file. Any failed check sets
// this variable to 1.
extern int __failed;

// Temporary variable to save the condition so that we don't
// evaluate the given expressions multiple times.
extern int   __failure_condition;
extern int   __int_expr0;
extern int   __int_expr1;
extern float __float_expr0;
extern float __float_expr1;

// Check macro helper functions
const char* __ento_debug_get_file_name(const char*);
void  __ento_test_fail(const char*, int, const char*);
void  __ento_test_check_and_print_uniop(const char*, int, const char*);
void  __ento_test_check_and_print_int_binop(const char*, int, const char*, const char*);
void  __ento_test_check_and_print_float_binop(const char*, int, const char*, const char*);

// Other helper functions
bool  __ento_test_num(int, int);

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_FAIL()
//------------------------------------------------------------------------
// Unconditionally fail a test case.

#define ENTO_TEST_CHECK_FAIL() \
  __ento_test_fail( __FILE__, __LINE__, "ENTO_TEST_CHECK_FAIL" ); \
  return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_TRUE( expr_ )
//------------------------------------------------------------------------
// Checks to see if the expression is true.

#define ENTO_TEST_CHECK_TRUE(expr_) \
  EntoUtil::__int_expr0 = expr_; \
  EntoUtil::__failure_condition = !EntoUtil::__int_expr0; \
  EntoUtil::__ento_test_check_and_print_uniop( __FILE__, __LINE__, #expr_ ); \
  if ( EntoUtil::__failure_condition ) return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_FALSE( expr_ )
//------------------------------------------------------------------------
// Checks to see if the expression is false.

#define ENTO_TEST_CHECK_FALSE(expr_) \
  EntoUtil::__int_expr0 = expr_; \
  EntoUtil::__failure_condition = __int_expr0; \
  EntoUtil::__ento_test_check_and_print_uniop( __FILE__, __LINE__, #expr_ ); \
  if ( EntoUtil::__failure_condition ) return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_INT_EQ( expr0_, expr1_ )
//------------------------------------------------------------------------
// Checks to see if the two expressions are equal using the != operator.

#define ENTO_TEST_CHECK_INT_EQ(expr0_, expr1_) {\
  EntoUtil::__int_expr0 = (int)(expr0_); \
  EntoUtil::__int_expr1 = (int)(expr1_); \
  EntoUtil::__failure_condition = EntoUtil::__int_expr0 != EntoUtil::__int_expr1; \
  EntoUtil::__ento_test_check_and_print_int_binop( __FILE__, __LINE__, #expr0_, #expr1_ ); \
  if ( EntoUtil::__failure_condition ) return;}

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_APPROX_EQ( expr0_, expr1_, pct_ )
//------------------------------------------------------------------------
// Checks to see if the two expressions are within percent of each other.

#define ENTO_TEST_CHECK_APPROX_EQ(expr0_, expr1_, pct_) \
  EntoUtil::__float_expr0 = expr0_; \
  EntoUtil::__float_expr1 = expr1_; \
  EntoUtil::__failure_condition = fabsf( EntoUtil::__float_expr0 - EntoUtil::__float_expr1 ) > fabs( (float)(pct_) * EntoUtil::__float_expr1 ); \
  EntoUtil::__ento_test_check_and_print_float_binop( __FILE__, __LINE__, #expr0_, #expr1_ ); \
  if ( EntoUtil::__failure_condition ) return;

//------------------------------------------------------------------------
// ENTO_TEST_CHECK_FLOAT_EQ( expr0_, expr1_ )
//------------------------------------------------------------------------
// Checks to see if two float experssions are within the predefined rel.
// tolerance of each other.

#define ENTO_TEST_CHECK_FLOAT_EQ(expr0_, expr1_)                      \
  ENTO_TEST_CHECK_APPROX_EQ(expr0_, expr1_, EntoUtil::__rel_tol_pct)  


//------------------------------------------------------------------------
// ENTO_TEST_CHECK_ARRAY_INT_EQ( expr0_, expr1_, size_ )
//------------------------------------------------------------------------
// Checks to see if the two arrays of integers are equal using the
// != operator.

#define ENTO_TEST_CHECK_ARRAY_INT_EQ(expr0_, expr1_, size_)             \
  for ( size_t i = 0; i < size_; i++ ) {                                \
    ENTO_TEST_CHECK_INT_EQ( expr0_[i], expr1_[i] );                     \
  }



}

#endif  // ENTO_UNITTEST_H
