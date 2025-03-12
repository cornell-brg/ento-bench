#include <cstdlib>
#include <cstdio>
#include <ento-math/gemv.h>
#include <Eigen/Dense>
#include <ento-bench/harness.h>
#include <ento-bench/roi.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-mcu/flash_util.h>

#ifdef SEMIHOSTING
extern "C" void initialise_monitor_handles( void );
#endif

template< typename T, int vlen >
void __attribute__((noinline)) vvadd_handwritten( std::array< T, vlen >& a,
                                                  std::array< T, vlen >& b,
                                                  std::array< T, vlen >& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    for ( int i = 0; i < vlen; i++ ) {
        c[i] = a[i] + b[i];
    }
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int vlen >
void __attribute__((noinline)) vvadd_eigen( Eigen::Matrix< T, vlen, 1 >& a,
                                            Eigen::Matrix< T, vlen, 1 >& b,
                                            Eigen::Matrix< T, vlen, 1 >& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    c = a + b;
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int vlen >
void __attribute__((noinline)) vdot_handwritten( std::array< T, vlen >& a, std::array< T, vlen >& b, T& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    T tmp = 0;
    for ( int i = 0; i < vlen; i++ ) {
       tmp += a[i] * b[i];
    }
    c = tmp;
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int vlen >
void __attribute__((noinline)) vdot_eigen( Eigen::Matrix< T, vlen, 1 >& a, Eigen::Matrix< T, vlen, 1 >& b, T& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    c = a.dot(b);
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int rows, int cols, int vlen >
void __attribute__((noinline)) gemv_handwritten( std::array< T, ( rows * cols ) >& a,
                                                 std::array< T, vlen >& b,
                                                 std::array< T, vlen >& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
  for ( int i = 0; i < rows; i++ ) {
    T sum = 0;
    int row_idx = i * cols;
    for ( int j = 0; j < cols; j++ ) {
      sum += a[row_idx + j] * b[j];
    }
    c[i] = sum;
  }
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int rows, int cols, int vlen >
void __attribute__((noinline)) gemv_eigen( Eigen::Matrix< T, rows, cols >& a,
                                           Eigen::Matrix< T, vlen, 1 >& b,
                                           Eigen::Matrix< T, vlen, 1 >& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    c = a * b;
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int rows_a, int cols_a, int rows_b, int cols_b >
void __attribute__((noinline)) gemm_handwritten( std::array< T, ( rows_a * cols_a ) >& a,
                                                 std::array< T, ( rows_b * cols_b ) >& b,
                                                 std::array< T, ( rows_a * cols_b ) >& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    for ( int i = 0; i < rows_a; i++ ) {
        for ( int j = 0; j < cols_b; j++ ) {
            T sum = 0;
            for ( int k = 0; k < cols_a; k++ ) {
                sum += a[( i * cols_a ) + k] * b[( k * cols_b ) + j];
            }
            c[( i * cols_b ) + j] = sum;
        }
    }
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

template< typename T, int rows_a, int cols_a, int rows_b, int cols_b >
void __attribute__((noinline)) gemm_eigen( Eigen::Matrix< T, rows_a, cols_a >& a,
                                           Eigen::Matrix< T, rows_b, cols_b >& b,
                                           Eigen::Matrix< T, rows_a, cols_b >& c )
{
//    start_roi();
//    asm volatile( "" ::: "memory" );
    c.noalias() = a * b;
//    asm volatile( "" ::: "memory" );
//    end_roi();
}

int main()
{
    using namespace bench;
    #ifdef SEMIHOSTING
    //initialise_monitor_handles();
    #endif

    bool is_systick_enabled = ( SysTick->CTRL & SysTick_CTRL_ENABLE_Msk ) != 0;

    printf( "Is systick enabled: %i\n", is_systick_enabled );
    constexpr int reps = 1;

    // Configure max clock rate and set flash latency
    sys_clk_cfg();

    // Turn on caches if applicable
    enable_instruction_cache();
    enable_instruction_cache_prefetch();
    icache_enable();
    // dcache_disable();
    branch_predictor_disable();

    printf( "============================================================\n" );
    printf( "Running blas comparison microbenchmarks.\n" );
    printf( "============================================================\n\n" );

    uint32_t clk_freq = get_sys_clk_freq();

    printf( "Current clk frequency (MHz): %.2f\n", ( clk_freq / 1000000.0 ) );

    uint32_t flash_latency = get_flash_latency();

    printf( "Current flash latency: %i\n", flash_latency );
    printf( "ICache is enabled!\n\n" );
    printf( "============================================================\n\n" );

    const int vlen = 16;

    //auto vvadd_handwritten_harness = make_harness<reps>( vvadd_handwritten< int, vlen >,
    //                                                     "vvadd handwritten benchmark");
    //auto vvadd_eigen_harness       = make_harness<reps>( vvadd_eigen< int, vlen >,
    //                                                     "vvadd eigen benchmark");
    //auto vdot_handwritten_harness  = make_harness<reps>( vdot_handwritten< int, vlen >,
    //                                                     "vdot handwritten benchmark");
    //auto vdot_eigen_harness        = make_harness<reps>( vdot_eigen< int, vlen >,
    //                                                     "vdot eigen benchmark");
    auto gemv_handwritten_harness  = make_harness<reps>( gemv_handwritten< int, vlen, vlen, vlen >,
                                                         "gemv handwritten benchmark");
    auto gemv_eigen_harness        = make_harness<reps>( gemv_eigen< int, vlen, vlen, vlen >,
                                                         "gemv eigen benchmark");
    //auto gemm_handwritten_harness  = make_harness<reps>( gemm_handwritten< int, vlen, vlen, vlen, vlen >,
    //                                                     "gemm handwritten benchmark");
    //auto gemm_eigen_harness        = make_harness<reps, ProfileMode::Aggregate>( gemm_eigen< int, vlen, vlen, vlen, vlen >,
    //                                                     "gemm eigen benchmark");

    std::array< int, vlen > a_vec_arr;
    std::array< int, vlen > b_vec_arr;
    std::array< int, vlen > c_vec_arr;
    Eigen::Matrix< int, vlen, 1 > a_vec_eig;
    Eigen::Matrix< int, vlen, 1 > b_vec_eig;
    Eigen::Matrix< int, vlen, 1 > c_vec_eig;
    c_vec_eig.setZero();
    int c_scalar;
    for ( int i = 0; i < vlen; i++ ) {
        a_vec_arr[i] = i;
        b_vec_arr[i] = ( 2 * i ) + 1;
        a_vec_eig[i] = i;
        b_vec_eig[i] = ( 2 * i ) + 1;
    }

    std::array< int, ( vlen * vlen ) > a_mat_arr;
    std::array< int, ( vlen * vlen ) > b_mat_arr;
    std::array< int, ( vlen * vlen ) > c_mat_arr;
    Eigen::Matrix< int, vlen, vlen > a_mat_eig;
    Eigen::Matrix< int, vlen, vlen > b_mat_eig;
    Eigen::Matrix< int, vlen, vlen > c_mat_eig;
    //c_mat_eig.setZero();
    for ( int i = 0; i < vlen; i++ ) {
        for ( int j = 0; j < vlen; j++ ) {
            a_mat_arr[( i * vlen ) + j] = ( i * j ) + j;
            a_mat_arr[( i * vlen ) + j] = ( i * j ) + ( 2 * j ) + 1;
            a_mat_eig( i, j ) = ( i * j ) + j;
            b_mat_eig( i, j ) = ( i * j ) + ( 2 * j ) + 1;
        }
    }

    //vvadd_handwritten_harness.run( a_vec_arr, b_vec_arr, c_vec_arr );
    printf( "Finished running vvadd handwritten benchmark.\n\n" );

    //vdot_handwritten_harness.run( a_vec_arr, b_vec_arr, c_scalar );
    //printf( "Finished running vdot handwritten benchmark.\n\n" );

    //vvadd_eigen_harness.run( a_vec_eig, b_vec_eig, c_vec_eig );
    //printf( "Finished running vvadd eigen benchmark.\n\n" );

    //vdot_eigen_harness.run( a_vec_eig, b_vec_eig, c_scalar );
    //printf( "Finished running vdot eigen benchmark.\n\n" );

    gemv_handwritten_harness.run( a_mat_arr, b_vec_arr, c_vec_arr );
    printf( "Finished running gemv handwritten benchmark.\n\n" );

    gemv_eigen_harness.run( a_mat_eig, b_vec_eig, c_vec_eig );
    printf( "Finished running gemv eigen benchmark.\n\n" );

    //gemm_handwritten_harness.run( a_mat_arr, b_mat_arr, c_mat_arr );
    //printf( "Finished running gemm handwritten benchmark.\n\n" );

    //gemm_eigen_harness.run( a_mat_eig, b_mat_eig, c_mat_eig );
    //printf( "Finished running gemm eigen benchmark.\n\n" );

    printf( "============================================================\n\n" );

    exit( 1 );

    return 0;
}

