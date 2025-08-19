#include <cstdio>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <image_io/Image.h>
#include <image_io/image_util.h>
#include <array>
#include <iostream>
#include <iomanip>
#include <cmath>

// Include the Python-generated golden reference data
#include "../../../gaussian_test_data.h"

#include "ento-feature2d/sift.h"

using namespace EntoUtil;

using DoGImageT = Image<32, 32, float>;
using SmallImageT = Image<5, 5, float>;
using TinyImageT = Image<3, 3, float>;

template<typename ImageT>
void copy_from_array(ImageT& img, const float src[ImageT::rows_][ImageT::cols_]) {
    for (int i = 0; i < ImageT::rows_; i++) {
        for (int j = 0; j < ImageT::cols_; j++) {
            img(i, j) = src[i][j];
        }
    }
}

template<typename ImageT>
void create_expected_image(ImageT& expected, const float src[ImageT::rows_][ImageT::cols_]) {
    for (int i = 0; i < ImageT::rows_; i++) {
        for (int j = 0; j < ImageT::cols_; j++) {
            expected(i, j) = src[i][j];
        }
    }
}

// Test data generated from Python SciPy with mode='constant' (matches SIFT++)
float test_input_5x5[5][5] = {
  {0.0, 0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 1.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0, 0.0}
};

float expected_5x5[5][5] = {
  {0.00291504, 0.01306431, 0.02153941, 0.01306431, 0.00291504},
  {0.01306431, 0.05855018, 0.09653293, 0.05855018, 0.01306431},
  {0.02153941, 0.09653293, 0.15915589, 0.09653293, 0.02153941},
  {0.01306431, 0.05855018, 0.09653293, 0.05855018, 0.01306431},
  {0.00291504, 0.01306431, 0.02153941, 0.01306431, 0.00291504}
};

float test_input_3x3[3][3] = {
  {0.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 0.0}
};

float expected_3x3[3][3] = {
  {0.01133177, 0.08373106, 0.01133177},
  {0.08373106, 0.61869351, 0.08373106},
  {0.01133177, 0.08373106, 0.01133177}
};

template<int Rows, int Cols>
void print_matrix(const char* name, float matrix[Rows][Cols]) {
    std::cout << name << ":\n";
    for (int i = 0; i < Rows; ++i) {
        std::cout << "  ";
        for (int j = 0; j < Cols; ++j) {
            std::cout << std::fixed << std::setprecision(8) << matrix[i][j];
            if (j < Cols - 1) std::cout << ", ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

template<int Rows, int Cols>
double compare_matrices(float actual[Rows][Cols], float expected[Rows][Cols]) {
    double total_diff = 0.0;
    double max_diff = 0.0;
    
    for (int i = 0; i < Rows; ++i) {
        for (int j = 0; j < Cols; ++j) {
            double diff = std::abs(actual[i][j] - expected[i][j]);
            total_diff += diff;
            max_diff = std::max(max_diff, diff);
        }
    }
    
    std::cout << "Total absolute difference: " << total_diff << "\n";
    std::cout << "Max absolute difference: " << max_diff << "\n";
    
    return total_diff;
}

bool test_sift_smooth_5x5() {
    std::cout << "=== Test 1: 5x5 impulse with sigma=1.0 ===\n";
    
    // Create input image
    Image<5, 5, float> input;
    Image<5, 5, float> output;
    
    // Copy test data
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            input(i, j) = test_input_5x5[i][j];
        }
    }
    
    // Apply SIFT smooth
    sift_smooth(input, output, 1.0f);
    
    // Extract results for comparison
    float actual[5][5];
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            actual[i][j] = output(i, j);
        }
    }
    
    // Print results
    print_matrix<5, 5>("Input", test_input_5x5);
    print_matrix<5, 5>("Expected (SciPy constant)", expected_5x5);
    print_matrix<5, 5>("Actual (C++ sift_smooth)", actual);
    
    // Compare
    double total_diff = compare_matrices<5, 5>(actual, expected_5x5);
    
    // Tolerance check
    const double tolerance = 1e-6;
    bool passed = total_diff < tolerance;
    
    std::cout << "Test 1 " << (passed ? "PASSED" : "FAILED") 
              << " (tolerance: " << tolerance << ")\n\n";
    
    return passed;
}

bool test_sift_smooth_3x3() {
    std::cout << "=== Test 2: 3x3 impulse with sigma=0.5 ===\n";
    
    // Create input image
    Image<3, 3, float> input;
    Image<3, 3, float> output;
    
    // Copy test data
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            input(i, j) = test_input_3x3[i][j];
        }
    }
    
    // Apply SIFT smooth
    sift_smooth(input, output, 0.5f);
    
    // Extract results for comparison
    float actual[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            actual[i][j] = output(i, j);
        }
    }
    
    // Print results
    print_matrix<3, 3>("Input", test_input_3x3);
    print_matrix<3, 3>("Expected (SciPy constant)", expected_3x3);
    print_matrix<3, 3>("Actual (C++ sift_smooth)", actual);
    
    // Compare
    double total_diff = compare_matrices<3, 3>(actual, expected_3x3);
    
    // Tolerance check
    const double tolerance = 1e-6;
    bool passed = total_diff < tolerance;
    
    std::cout << "Test 2 " << (passed ? "PASSED" : "FAILED") 
              << " (tolerance: " << tolerance << ")\n\n";
    
    return passed;
}

bool test_sift_smooth_efficient_vs_original() {
    std::cout << "=== Test 3: Memory-efficient vs Original (5x5, sigma=1.0) ===\n";
    
    // Create input images
    Image<5, 5, float> input;
    Image<5, 5, float> output_original;
    Image<5, 5, float> output_efficient;
    
    // Copy test data
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            input(i, j) = test_input_5x5[i][j];
        }
    }
    
    // Apply both versions
    sift_smooth(input, output_original, 1.0f);
    sift_smooth_efficient(input, output_efficient, 1.0f);
    
    // Extract results for comparison
    float original[5][5], efficient[5][5];
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            original[i][j] = output_original(i, j);
            efficient[i][j] = output_efficient(i, j);
        }
    }
    
    // Print results
    print_matrix<5, 5>("Original sift_smooth", original);
    print_matrix<5, 5>("Efficient sift_smooth_efficient", efficient);
    
    // Compare the two implementations
    double total_diff = compare_matrices<5, 5>(efficient, original);
    
    // Tolerance check (should be nearly identical)
    const double tolerance = 1e-6;
    bool passed = total_diff < tolerance;
    
    std::cout << "Test 3 " << (passed ? "PASSED" : "FAILED") 
              << " (tolerance: " << tolerance << ")\n";
    
    if (passed) {
        std::cout << "✅ Memory-efficient version produces identical results!\n";
    } else {
        std::cout << "❌ Memory-efficient version differs from original\n";
    }
    std::cout << "\n";
    
    return passed;
}

int main() {
    std::cout << "Testing C++ sift_smooth against SciPy constant mode (SIFT++ equivalent)\n";
    std::cout << "====================================================================\n\n";
    
    bool test1_passed = test_sift_smooth_5x5();
    bool test2_passed = test_sift_smooth_3x3();
    bool test3_passed = test_sift_smooth_efficient_vs_original();
    
    std::cout << "=== Summary ===\n";
    std::cout << "Test 1 (5x5): " << (test1_passed ? "PASSED" : "FAILED") << "\n";
    std::cout << "Test 2 (3x3): " << (test2_passed ? "PASSED" : "FAILED") << "\n";
    std::cout << "Test 3 (Memory-efficient vs Original): " << (test3_passed ? "PASSED" : "FAILED") << "\n";
    
    bool all_passed = test1_passed && test2_passed && test3_passed;
    std::cout << "Overall: " << (all_passed ? "PASSED" : "FAILED") << "\n";
    
    return all_passed ? 0 : 1;
} 