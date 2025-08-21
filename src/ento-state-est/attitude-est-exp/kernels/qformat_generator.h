#ifndef QFORMAT_GENERATOR_H
#define QFORMAT_GENERATOR_H

#include <Eigen/Dense>
#include <array>
#include <string>
#include <iostream>
#include <iomanip>
#include <ento-math/core.h>
#include <ento-state-est/attitude-est/attitude_measurement.h>
#include <math/FixedPoint.hh>
#include <math/FixedPointMath.hh>
#include <math/EigenFixedPoint.hh>

// Include our checked filter templates
#include <ento-state-est/attitude-est-exp/kernels/madgwick_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/madgwick_fixed_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/mahoney_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/mahoney_fixed_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/fourati_checked.h>
#include <ento-state-est/attitude-est-exp/kernels/fourati_fixed_checked.h>
#include <ento-state-est/attitude-est-exp/core/failure_flags.h>

namespace EntoAttitudeExp {

//------------------------------------------------------------------------------
// 1. Runtime format specification for tracking
//------------------------------------------------------------------------------
struct QFormatSpec {
    int integer_bits;
    int fractional_bits;
    int total_bits;
    bool is_float;
    const char* underlying_type;  // Use const char* instead of std::string for constexpr
    std::string mode;
    std::string name() const {
        if (is_float) {
            return (total_bits == 32) ? "Float" : "Double";
        }
        return "Q" + std::to_string(integer_bits) + "." + std::to_string(fractional_bits);
    }
    
    std::string full_name() const {
        if (is_float) {
            return name() + " (" + std::string(underlying_type) + ")";
        }
        return name() + " (" + std::to_string(total_bits) + "-bit: " + 
               std::to_string(integer_bits) + "I." + std::to_string(fractional_bits) + "F)";
    }
};

//------------------------------------------------------------------------------
// 2. Failure statistics structure (was missing from original)
//------------------------------------------------------------------------------
struct FailureStats {
    size_t total_samples = 0;
    size_t overflow_count = 0;
    size_t bad_norm_count = 0;
    size_t near_zero_div_count = 0;
    size_t excessive_err_count = 0;
    size_t any_failure_count = 0;
    
    void add_failure(const FailureFlags& flags) {
        if (flags[FailureReason::Overflow]) overflow_count++;
        if (flags[FailureReason::BadNorm]) bad_norm_count++;
        if (flags[FailureReason::NearZeroDiv]) near_zero_div_count++;
        if (flags[FailureReason::ExcessiveErr]) excessive_err_count++;
        if (flags.any()) any_failure_count++;
        total_samples++;
    }
    
    void print_summary() const {
        std::cout << "\n=== FAILURE ANALYSIS SUMMARY ===\n";
        std::cout << "Total samples processed: " << total_samples << "\n";
        std::cout << "Samples with any failure: " << any_failure_count 
                  << " (" << std::fixed << std::setprecision(2) 
                  << (100.0 * any_failure_count / total_samples) << "%)\n";
        std::cout << "\nFailure breakdown:\n";
        std::cout << "  Overflow/saturation: " << overflow_count 
                  << " (" << (100.0 * overflow_count / total_samples) << "%)\n";
        std::cout << "  Bad quaternion norm: " << bad_norm_count 
                  << " (" << (100.0 * bad_norm_count / total_samples) << "%)\n";
        std::cout << "  Near-zero divisors: " << near_zero_div_count 
                  << " (" << (100.0 * near_zero_div_count / total_samples) << "%)\n";
        std::cout << "  Excessive error: " << excessive_err_count 
                  << " (" << (100.0 * excessive_err_count / total_samples) << "%)\n";
    }
};

//------------------------------------------------------------------------------
// 3. Kernel type aliases for all Q-formats (template specializations)
//------------------------------------------------------------------------------

// Float and Double (baseline)
template<bool UseMag> using FilterFloat = FilterMadgwickChecked<float, UseMag>;
template<bool UseMag> using FilterDouble = FilterMadgwickChecked<double, UseMag>;

// 16-bit Q-formats (15 combinations)
template<bool UseMag> using FilterQ1_15 = FilterMadgwickFixedChecked<FixedPoint<1,15,int16_t>, UseMag>;
template<bool UseMag> using FilterQ2_14 = FilterMadgwickFixedChecked<FixedPoint<2,14,int16_t>, UseMag>;
template<bool UseMag> using FilterQ3_13 = FilterMadgwickFixedChecked<FixedPoint<3,13,int16_t>, UseMag>;
template<bool UseMag> using FilterQ4_12 = FilterMadgwickFixedChecked<FixedPoint<4,12,int16_t>, UseMag>;
template<bool UseMag> using FilterQ5_11 = FilterMadgwickFixedChecked<FixedPoint<5,11,int16_t>, UseMag>;
template<bool UseMag> using FilterQ6_10 = FilterMadgwickFixedChecked<FixedPoint<6,10,int16_t>, UseMag>;
template<bool UseMag> using FilterQ7_9 = FilterMadgwickFixedChecked<FixedPoint<7,9,int16_t>, UseMag>;
template<bool UseMag> using FilterQ8_8 = FilterMadgwickFixedChecked<FixedPoint<8,8,int16_t>, UseMag>;
template<bool UseMag> using FilterQ9_7 = FilterMadgwickFixedChecked<FixedPoint<9,7,int16_t>, UseMag>;
template<bool UseMag> using FilterQ10_6 = FilterMadgwickFixedChecked<FixedPoint<10,6,int16_t>, UseMag>;
template<bool UseMag> using FilterQ11_5 = FilterMadgwickFixedChecked<FixedPoint<11,5,int16_t>, UseMag>;
template<bool UseMag> using FilterQ12_4 = FilterMadgwickFixedChecked<FixedPoint<12,4,int16_t>, UseMag>;
template<bool UseMag> using FilterQ13_3 = FilterMadgwickFixedChecked<FixedPoint<13,3,int16_t>, UseMag>;
template<bool UseMag> using FilterQ14_2 = FilterMadgwickFixedChecked<FixedPoint<14,2,int16_t>, UseMag>;
template<bool UseMag> using FilterQ15_1 = FilterMadgwickFixedChecked<FixedPoint<15,1,int16_t>, UseMag>;

// 32-bit Q-formats (31 combinations)
template<bool UseMag> using FilterQ1_31 = FilterMadgwickFixedChecked<FixedPoint<1,31,int32_t>, UseMag>;
template<bool UseMag> using FilterQ2_30 = FilterMadgwickFixedChecked<FixedPoint<2,30,int32_t>, UseMag>;
template<bool UseMag> using FilterQ3_29 = FilterMadgwickFixedChecked<FixedPoint<3,29,int32_t>, UseMag>;
template<bool UseMag> using FilterQ4_28 = FilterMadgwickFixedChecked<FixedPoint<4,28,int32_t>, UseMag>;
template<bool UseMag> using FilterQ5_27 = FilterMadgwickFixedChecked<FixedPoint<5,27,int32_t>, UseMag>;
template<bool UseMag> using FilterQ6_26 = FilterMadgwickFixedChecked<FixedPoint<6,26,int32_t>, UseMag>;
template<bool UseMag> using FilterQ7_25 = FilterMadgwickFixedChecked<FixedPoint<7,25,int32_t>, UseMag>;
template<bool UseMag> using FilterQ8_24 = FilterMadgwickFixedChecked<FixedPoint<8,24,int32_t>, UseMag>;
template<bool UseMag> using FilterQ9_23 = FilterMadgwickFixedChecked<FixedPoint<9,23,int32_t>, UseMag>;
template<bool UseMag> using FilterQ10_22 = FilterMadgwickFixedChecked<FixedPoint<10,22,int32_t>, UseMag>;
template<bool UseMag> using FilterQ11_21 = FilterMadgwickFixedChecked<FixedPoint<11,21,int32_t>, UseMag>;
template<bool UseMag> using FilterQ12_20 = FilterMadgwickFixedChecked<FixedPoint<12,20,int32_t>, UseMag>;
template<bool UseMag> using FilterQ13_19 = FilterMadgwickFixedChecked<FixedPoint<13,19,int32_t>, UseMag>;
template<bool UseMag> using FilterQ14_18 = FilterMadgwickFixedChecked<FixedPoint<14,18,int32_t>, UseMag>;
template<bool UseMag> using FilterQ15_17 = FilterMadgwickFixedChecked<FixedPoint<15,17,int32_t>, UseMag>;
template<bool UseMag> using FilterQ16_16 = FilterMadgwickFixedChecked<FixedPoint<16,16,int32_t>, UseMag>;
template<bool UseMag> using FilterQ17_15 = FilterMadgwickFixedChecked<FixedPoint<17,15,int32_t>, UseMag>;
template<bool UseMag> using FilterQ18_14 = FilterMadgwickFixedChecked<FixedPoint<18,14,int32_t>, UseMag>;
template<bool UseMag> using FilterQ19_13 = FilterMadgwickFixedChecked<FixedPoint<19,13,int32_t>, UseMag>;
template<bool UseMag> using FilterQ20_12 = FilterMadgwickFixedChecked<FixedPoint<20,12,int32_t>, UseMag>;
template<bool UseMag> using FilterQ21_11 = FilterMadgwickFixedChecked<FixedPoint<21,11,int32_t>, UseMag>;
template<bool UseMag> using FilterQ22_10 = FilterMadgwickFixedChecked<FixedPoint<22,10,int32_t>, UseMag>;
template<bool UseMag> using FilterQ23_9 = FilterMadgwickFixedChecked<FixedPoint<23,9,int32_t>, UseMag>;
template<bool UseMag> using FilterQ24_8 = FilterMadgwickFixedChecked<FixedPoint<24,8,int32_t>, UseMag>;
template<bool UseMag> using FilterQ25_7 = FilterMadgwickFixedChecked<FixedPoint<25,7,int32_t>, UseMag>;
template<bool UseMag> using FilterQ26_6 = FilterMadgwickFixedChecked<FixedPoint<26,6,int32_t>, UseMag>;
template<bool UseMag> using FilterQ27_5 = FilterMadgwickFixedChecked<FixedPoint<27,5,int32_t>, UseMag>;
template<bool UseMag> using FilterQ28_4 = FilterMadgwickFixedChecked<FixedPoint<28,4,int32_t>, UseMag>;
template<bool UseMag> using FilterQ29_3 = FilterMadgwickFixedChecked<FixedPoint<29,3,int32_t>, UseMag>;
template<bool UseMag> using FilterQ30_2 = FilterMadgwickFixedChecked<FixedPoint<30,2,int32_t>, UseMag>;
template<bool UseMag> using FilterQ31_1 = FilterMadgwickFixedChecked<FixedPoint<31,1,int32_t>, UseMag>;

//------------------------------------------------------------------------------
// Madgwick Float/Double convenience aliases  
//------------------------------------------------------------------------------
template<bool UseMag> using FilterMadgwickFloatChecked = FilterMadgwickChecked<float, UseMag>;
template<bool UseMag> using FilterMadgwickDoubleChecked = FilterMadgwickChecked<double, UseMag>;

// Additional Q-format type aliases for compatibility 
using Q5_26 = FixedPoint<5,26,int32_t>;

//------------------------------------------------------------------------------
// MAHONEY KERNEL ALIASES (48 formats total)
//------------------------------------------------------------------------------

// Float and Double (baseline) - Mahoney
template<bool UseMag> using FilterMahoneyFloat = FilterMahoneyChecked<float, UseMag>;
template<bool UseMag> using FilterMahoneyDouble = FilterMahoneyChecked<double, UseMag>;

// 16-bit Q-formats (15 combinations) - Mahoney
template<bool UseMag> using FilterMahoneyQ1_15 = FilterMahoneyFixedChecked<FixedPoint<1,15,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ2_14 = FilterMahoneyFixedChecked<FixedPoint<2,14,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ3_13 = FilterMahoneyFixedChecked<FixedPoint<3,13,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ4_12 = FilterMahoneyFixedChecked<FixedPoint<4,12,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ5_11 = FilterMahoneyFixedChecked<FixedPoint<5,11,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ6_10 = FilterMahoneyFixedChecked<FixedPoint<6,10,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ7_9 = FilterMahoneyFixedChecked<FixedPoint<7,9,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ8_8 = FilterMahoneyFixedChecked<FixedPoint<8,8,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ9_7 = FilterMahoneyFixedChecked<FixedPoint<9,7,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ10_6 = FilterMahoneyFixedChecked<FixedPoint<10,6,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ11_5 = FilterMahoneyFixedChecked<FixedPoint<11,5,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ12_4 = FilterMahoneyFixedChecked<FixedPoint<12,4,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ13_3 = FilterMahoneyFixedChecked<FixedPoint<13,3,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ14_2 = FilterMahoneyFixedChecked<FixedPoint<14,2,int16_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ15_1 = FilterMahoneyFixedChecked<FixedPoint<15,1,int16_t>, UseMag>;

// 32-bit Q-formats (31 combinations) - Mahoney
template<bool UseMag> using FilterMahoneyQ1_31 = FilterMahoneyFixedChecked<FixedPoint<1,31,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ2_30 = FilterMahoneyFixedChecked<FixedPoint<2,30,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ3_29 = FilterMahoneyFixedChecked<FixedPoint<3,29,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ4_28 = FilterMahoneyFixedChecked<FixedPoint<4,28,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ5_27 = FilterMahoneyFixedChecked<FixedPoint<5,27,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ6_26 = FilterMahoneyFixedChecked<FixedPoint<6,26,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ7_25 = FilterMahoneyFixedChecked<FixedPoint<7,25,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ8_24 = FilterMahoneyFixedChecked<FixedPoint<8,24,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ9_23 = FilterMahoneyFixedChecked<FixedPoint<9,23,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ10_22 = FilterMahoneyFixedChecked<FixedPoint<10,22,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ11_21 = FilterMahoneyFixedChecked<FixedPoint<11,21,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ12_20 = FilterMahoneyFixedChecked<FixedPoint<12,20,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ13_19 = FilterMahoneyFixedChecked<FixedPoint<13,19,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ14_18 = FilterMahoneyFixedChecked<FixedPoint<14,18,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ15_17 = FilterMahoneyFixedChecked<FixedPoint<15,17,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ16_16 = FilterMahoneyFixedChecked<FixedPoint<16,16,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ17_15 = FilterMahoneyFixedChecked<FixedPoint<17,15,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ18_14 = FilterMahoneyFixedChecked<FixedPoint<18,14,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ19_13 = FilterMahoneyFixedChecked<FixedPoint<19,13,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ20_12 = FilterMahoneyFixedChecked<FixedPoint<20,12,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ21_11 = FilterMahoneyFixedChecked<FixedPoint<21,11,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ22_10 = FilterMahoneyFixedChecked<FixedPoint<22,10,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ23_9 = FilterMahoneyFixedChecked<FixedPoint<23,9,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ24_8 = FilterMahoneyFixedChecked<FixedPoint<24,8,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ25_7 = FilterMahoneyFixedChecked<FixedPoint<25,7,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ26_6 = FilterMahoneyFixedChecked<FixedPoint<26,6,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ27_5 = FilterMahoneyFixedChecked<FixedPoint<27,5,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ28_4 = FilterMahoneyFixedChecked<FixedPoint<28,4,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ29_3 = FilterMahoneyFixedChecked<FixedPoint<29,3,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ30_2 = FilterMahoneyFixedChecked<FixedPoint<30,2,int32_t>, UseMag>;
template<bool UseMag> using FilterMahoneyQ31_1 = FilterMahoneyFixedChecked<FixedPoint<31,1,int32_t>, UseMag>;

//------------------------------------------------------------------------------
// Mahoney Float/Double convenience aliases
//------------------------------------------------------------------------------
template<bool UseMag> using FilterMahoneyFloatChecked = FilterMahoneyChecked<float, UseMag>;
template<bool UseMag> using FilterMahoneyDoubleChecked = FilterMahoneyChecked<double, UseMag>;

//------------------------------------------------------------------------------
// FOURATI KERNEL ALIASES (48 formats total) - MARG ONLY
//------------------------------------------------------------------------------

// Float and Double (baseline) - Fourati
using FilterFouratiFloat = FilterFouratiChecked<float>;
using FilterFouratiDouble = FilterFouratiChecked<double>;

// 16-bit Q-formats (15 combinations) - Fourati
using FilterFouratiQ1_15 = FilterFouratiFixedChecked<FixedPoint<1,15,int16_t>>;
using FilterFouratiQ2_14 = FilterFouratiFixedChecked<FixedPoint<2,14,int16_t>>;
using FilterFouratiQ3_13 = FilterFouratiFixedChecked<FixedPoint<3,13,int16_t>>;
using FilterFouratiQ4_12 = FilterFouratiFixedChecked<FixedPoint<4,12,int16_t>>;
using FilterFouratiQ5_11 = FilterFouratiFixedChecked<FixedPoint<5,11,int16_t>>;
using FilterFouratiQ6_10 = FilterFouratiFixedChecked<FixedPoint<6,10,int16_t>>;
using FilterFouratiQ7_9 = FilterFouratiFixedChecked<FixedPoint<7,9,int16_t>>;
using FilterFouratiQ8_8 = FilterFouratiFixedChecked<FixedPoint<8,8,int16_t>>;
using FilterFouratiQ9_7 = FilterFouratiFixedChecked<FixedPoint<9,7,int16_t>>;
using FilterFouratiQ10_6 = FilterFouratiFixedChecked<FixedPoint<10,6,int16_t>>;
using FilterFouratiQ11_5 = FilterFouratiFixedChecked<FixedPoint<11,5,int16_t>>;
using FilterFouratiQ12_4 = FilterFouratiFixedChecked<FixedPoint<12,4,int16_t>>;
using FilterFouratiQ13_3 = FilterFouratiFixedChecked<FixedPoint<13,3,int16_t>>;
using FilterFouratiQ14_2 = FilterFouratiFixedChecked<FixedPoint<14,2,int16_t>>;
using FilterFouratiQ15_1 = FilterFouratiFixedChecked<FixedPoint<15,1,int16_t>>;

// 32-bit Q-formats (31 combinations) - Fourati
using FilterFouratiQ1_31 = FilterFouratiFixedChecked<FixedPoint<1,31,int32_t>>;
using FilterFouratiQ2_30 = FilterFouratiFixedChecked<FixedPoint<2,30,int32_t>>;
using FilterFouratiQ3_29 = FilterFouratiFixedChecked<FixedPoint<3,29,int32_t>>;
using FilterFouratiQ4_28 = FilterFouratiFixedChecked<FixedPoint<4,28,int32_t>>;
using FilterFouratiQ5_27 = FilterFouratiFixedChecked<FixedPoint<5,27,int32_t>>;
using FilterFouratiQ6_26 = FilterFouratiFixedChecked<FixedPoint<6,26,int32_t>>;
using FilterFouratiQ7_25 = FilterFouratiFixedChecked<FixedPoint<7,25,int32_t>>;
using FilterFouratiQ8_24 = FilterFouratiFixedChecked<FixedPoint<8,24,int32_t>>;
using FilterFouratiQ9_23 = FilterFouratiFixedChecked<FixedPoint<9,23,int32_t>>;
using FilterFouratiQ10_22 = FilterFouratiFixedChecked<FixedPoint<10,22,int32_t>>;
using FilterFouratiQ11_21 = FilterFouratiFixedChecked<FixedPoint<11,21,int32_t>>;
using FilterFouratiQ12_20 = FilterFouratiFixedChecked<FixedPoint<12,20,int32_t>>;
using FilterFouratiQ13_19 = FilterFouratiFixedChecked<FixedPoint<13,19,int32_t>>;
using FilterFouratiQ14_18 = FilterFouratiFixedChecked<FixedPoint<14,18,int32_t>>;
using FilterFouratiQ15_17 = FilterFouratiFixedChecked<FixedPoint<15,17,int32_t>>;
using FilterFouratiQ16_16 = FilterFouratiFixedChecked<FixedPoint<16,16,int32_t>>;
using FilterFouratiQ17_15 = FilterFouratiFixedChecked<FixedPoint<17,15,int32_t>>;
using FilterFouratiQ18_14 = FilterFouratiFixedChecked<FixedPoint<18,14,int32_t>>;
using FilterFouratiQ19_13 = FilterFouratiFixedChecked<FixedPoint<19,13,int32_t>>;
using FilterFouratiQ20_12 = FilterFouratiFixedChecked<FixedPoint<20,12,int32_t>>;
using FilterFouratiQ21_11 = FilterFouratiFixedChecked<FixedPoint<21,11,int32_t>>;
using FilterFouratiQ22_10 = FilterFouratiFixedChecked<FixedPoint<22,10,int32_t>>;
using FilterFouratiQ23_9 = FilterFouratiFixedChecked<FixedPoint<23,9,int32_t>>;
using FilterFouratiQ24_8 = FilterFouratiFixedChecked<FixedPoint<24,8,int32_t>>;
using FilterFouratiQ25_7 = FilterFouratiFixedChecked<FixedPoint<25,7,int32_t>>;
using FilterFouratiQ26_6 = FilterFouratiFixedChecked<FixedPoint<26,6,int32_t>>;
using FilterFouratiQ27_5 = FilterFouratiFixedChecked<FixedPoint<27,5,int32_t>>;
using FilterFouratiQ28_4 = FilterFouratiFixedChecked<FixedPoint<28,4,int32_t>>;
using FilterFouratiQ29_3 = FilterFouratiFixedChecked<FixedPoint<29,3,int32_t>>;
using FilterFouratiQ30_2 = FilterFouratiFixedChecked<FixedPoint<30,2,int32_t>>;
using FilterFouratiQ31_1 = FilterFouratiFixedChecked<FixedPoint<31,1,int32_t>>;

//------------------------------------------------------------------------------
// 4. Runtime helper functions
//------------------------------------------------------------------------------
inline size_t total_format_count() { return 144; }          // 48 per kernel × 3 kernels
inline size_t total_kernel_count() { return 3; }            // Madgwick, Mahoney, Fourati
inline size_t madgwick_format_count() { return 48; }        // Madgwick: 48 formats
inline size_t mahoney_format_count() { return 48; }         // Mahoney: 48 formats  
inline size_t fourati_format_count() { return 48; }         // Fourati: 48 formats
inline size_t floating_format_count() { return 2; }         // Float, Double per kernel
inline size_t fixed_16bit_format_count() { return 15; }     // 16-bit per kernel
inline size_t fixed_32bit_format_count() { return 31; }     // 32-bit per kernel

} // namespace EntoAttitudeExp

#endif // QFORMAT_GENERATOR_H 