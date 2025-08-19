#include <iostream>
#include <Eigen/Dense>
#include <ento-state-est/attitude-est-exp/kernels/qformat_generator.h>
#include <ento-state-est/attitude-est/madgwick_fixed.h>
#include <ento-state-est/attitude-est/mahoney_fixed.h>
#include <ento-state-est/attitude-est/madgwick.h>
#include <ento-state-est/attitude-est/mahoney.h>

using namespace EntoAttitudeExp;

// Helper: run the fixed-point test for a given Scalar type
template<typename Scalar>
void run_fixedpoint_test(const std::string& label) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "FIXED-POINT TEST: " << label << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Pre-normalize accelerometer and magnetometer data in float before converting to fixed-point
    float acc_x = -0.021139f;
    float acc_y = 0.037897f;
    float acc_z = 8.326411f;
    float acc_norm = std::sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    float acc_x_norm = acc_x / acc_norm;
    float acc_y_norm = acc_y / acc_norm;
    float acc_z_norm = acc_z / acc_norm;
    
    // For magnetometer, use the same normalized values (since we don't have mag data in this test)
    float mag_x_norm = acc_x_norm;
    float mag_y_norm = acc_y_norm;
    float mag_z_norm = acc_z_norm;
    
    std::cout << "Pre-normalized accel: [" << acc_x_norm << ", " << acc_y_norm << ", " << acc_z_norm << "]" << std::endl;
    
    Scalar qw = Scalar(1.0);
    Scalar qx = Scalar(0.000031);
    Scalar qy = Scalar(0.000003);
    Scalar qz = Scalar(-0.000006);
    Eigen::Quaternion<Scalar> q(qw, qx, qy, qz);
    
    EntoAttitude::AttitudeMeasurement<Scalar, false> meas;
    // Use pre-normalized accelerometer data
    meas.acc = EntoMath::Vec3<Scalar>(Scalar(acc_x_norm), Scalar(acc_y_norm), Scalar(acc_z_norm));
    // Keep gyroscope data as-is (angular velocity shouldn't be normalized)
    meas.gyr = EntoMath::Vec3<Scalar>(Scalar(-0.000050), Scalar(-0.000042), Scalar(0.000172));
    Scalar dt = Scalar(0.001);
    
    std::vector<std::pair<std::string, double>> mahoney_params = {
        {"low", 0.1}, {"medium", 0.5}, {"high", 0.9}, {"very_high", 10.0}, {"extreme", 100.0}
    };
    std::vector<std::pair<std::string, double>> madgwick_params = {
        {"low", 0.001}, {"medium", 0.01}, {"high", 0.1}, {"very_high", 1.0}, {"extreme", 10.0}
    };
    
    std::vector<Eigen::Quaternion<Scalar>> mahoney_results;
    for (const auto& param : mahoney_params) {
        EntoAttitude::FilterMahonyFixed<Scalar, false> mahoney;
        EntoMath::Vec3<Scalar> bias(Scalar(0.0), Scalar(0.0), Scalar(0.0));
        Eigen::Quaternion<Scalar> result = mahoney(q, meas, dt, Scalar(param.second), Scalar(0.01), bias);
        mahoney_results.push_back(result);
        std::cout << "Mahoney (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    std::vector<Eigen::Quaternion<Scalar>> madgwick_results;
    for (const auto& param : madgwick_params) {
        EntoAttitude::FilterMadgwickFixed<Scalar, false> madgwick;
        Eigen::Quaternion<Scalar> result = madgwick(q, meas, dt, Scalar(param.second));
        madgwick_results.push_back(result);
        std::cout << "Madgwick (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    // Check if results are different
    bool mahoney_different = false;
    for (size_t i = 1; i < mahoney_results.size(); ++i) {
        if (mahoney_results[i].w() != mahoney_results[0].w() || 
            mahoney_results[i].x() != mahoney_results[0].x() || 
            mahoney_results[i].y() != mahoney_results[0].y() || 
            mahoney_results[i].z() != mahoney_results[0].z()) {
            mahoney_different = true;
            break;
        }
    }
    
    bool madgwick_different = false;
    for (size_t i = 1; i < madgwick_results.size(); ++i) {
        if (madgwick_results[i].w() != madgwick_results[0].w() || 
            madgwick_results[i].x() != madgwick_results[0].x() || 
            madgwick_results[i].y() != madgwick_results[0].y() || 
            madgwick_results[i].z() != madgwick_results[0].z()) {
            madgwick_different = true;
            break;
        }
    }
    
    bool algorithms_different = (mahoney_results[0].w() != madgwick_results[0].w() || 
                                mahoney_results[0].x() != madgwick_results[0].x() || 
                                mahoney_results[0].y() != madgwick_results[0].y() || 
                                mahoney_results[0].z() != madgwick_results[0].z());
    
    std::cout << "Mahoney with different parameters: " << (mahoney_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Madgwick with different gains: " << (madgwick_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Mahoney vs Madgwick: " << (algorithms_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
}

int main() {
    std::cout << "=== Testing 32-bit Mahoney vs Madgwick with real ICM data (line 50) ===" << std::endl;
    
    // Test Q2.30 format
    using Scalar = FixedPoint<2,30,int32_t>;
    
    // Line 50 from IMU dataset
    // -0.021139 0.037897 8.326411 -0.000050 -0.000042 0.000172 1.000000 0.000031 0.000003 -0.000006 0.001000
    Scalar qw = Scalar(1.0);
    Scalar qx = Scalar(0.000031);
    Scalar qy = Scalar(0.000003);
    Scalar qz = Scalar(-0.000006);
    Eigen::Quaternion<Scalar> q(qw, qx, qy, qz);
    
    EntoAttitude::AttitudeMeasurement<Scalar, false> meas;
    meas.acc = EntoMath::Vec3<Scalar>(Scalar(-0.021139), Scalar(0.037897), Scalar(8.326411));
    meas.gyr = EntoMath::Vec3<Scalar>(Scalar(-0.000050), Scalar(-0.000042), Scalar(0.000172));
    Scalar dt = Scalar(0.001);
    
    std::cout << "\n--- Using line 50 from IMU dataset ---" << std::endl;
    std::cout << "Initial quaternion: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]" << std::endl;
    std::cout << "Acc: [" << meas.acc.x() << ", " << meas.acc.y() << ", " << meas.acc.z() << "]" << std::endl;
    std::cout << "Gyr: [" << meas.gyr.x() << ", " << meas.gyr.y() << ", " << meas.gyr.z() << "]" << std::endl;
    std::cout << "dt: " << dt << std::endl;
    
    // Test Mahoney with different parameters
    std::cout << "\n--- Testing Mahoney Filter ---" << std::endl;
    
    std::vector<std::pair<std::string, double>> mahoney_params = {
        {"low", 0.1},
        {"medium", 0.5}, 
        {"high", 0.9},
        {"very_high", 10.0},  // Much larger value to test fixed-point precision
        {"extreme", 100.0}    // Extreme value to see if any effect
    };
    
    std::vector<Eigen::Quaternion<Scalar>> mahoney_results;
    for (const auto& param : mahoney_params) {
        EntoAttitude::FilterMahonyFixed<Scalar, false> mahoney;
        EntoMath::Vec3<Scalar> bias(Scalar(0.0), Scalar(0.0), Scalar(0.0));
        Eigen::Quaternion<Scalar> result = mahoney(q, meas, dt, Scalar(param.second), Scalar(0.01), bias);
        mahoney_results.push_back(result);
        std::cout << "Mahoney (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    // Test Madgwick with different gains
    std::cout << "\n--- Testing Madgwick Filter ---" << std::endl;
    
    std::vector<std::pair<std::string, double>> madgwick_params = {
        {"low", 0.001},
        {"medium", 0.01}, 
        {"high", 0.1},
        {"very_high", 1.0},   // Much larger value to test fixed-point precision
        {"extreme", 10.0}     // Extreme value to see if any effect
    };
    
    std::vector<Eigen::Quaternion<Scalar>> madgwick_results;
    for (const auto& param : madgwick_params) {
        EntoAttitude::FilterMadgwickFixed<Scalar, false> madgwick;
        Eigen::Quaternion<Scalar> result = madgwick(q, meas, dt, Scalar(param.second));
        madgwick_results.push_back(result);
        std::cout << "Madgwick (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    // Check if results are different
    std::cout << "\n--- Analysis ---" << std::endl;
    
    // Check if Mahoney results are different
    bool mahoney_different = false;
    for (size_t i = 1; i < mahoney_results.size(); ++i) {
        if (mahoney_results[i].w() != mahoney_results[0].w() || 
            mahoney_results[i].x() != mahoney_results[0].x() || 
            mahoney_results[i].y() != mahoney_results[0].y() || 
            mahoney_results[i].z() != mahoney_results[0].z()) {
            mahoney_different = true;
            break;
        }
    }
    
    // Check if Madgwick results are different
    bool madgwick_different = false;
    for (size_t i = 1; i < madgwick_results.size(); ++i) {
        if (madgwick_results[i].w() != madgwick_results[0].w() || 
            madgwick_results[i].x() != madgwick_results[0].x() || 
            madgwick_results[i].y() != madgwick_results[0].y() || 
            madgwick_results[i].z() != madgwick_results[0].z()) {
            madgwick_different = true;
            break;
        }
    }
    
    // Check if algorithms are different
    bool algorithms_different = (mahoney_results[0].w() != madgwick_results[0].w() || 
                                mahoney_results[0].x() != madgwick_results[0].x() || 
                                mahoney_results[0].y() != madgwick_results[0].y() || 
                                mahoney_results[0].z() != madgwick_results[0].z());
    
    std::cout << "Mahoney with different parameters: " << (mahoney_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Madgwick with different gains: " << (madgwick_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Mahoney vs Madgwick: " << (algorithms_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    
    // =============================================================================
    // FLOATING-POINT CONTROLS
    // =============================================================================
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "FLOATING-POINT CONTROLS (Float)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Convert data to float
    Eigen::Quaternion<float> q_float(1.0f, 0.000031f, 0.000003f, -0.000006f);
    EntoAttitude::AttitudeMeasurement<float, false> meas_float;
    meas_float.acc = EntoMath::Vec3<float>(-0.021139f, 0.037897f, 8.326411f);
    meas_float.gyr = EntoMath::Vec3<float>(-0.000050f, -0.000042f, 0.000172f);
    float dt_float = 0.001f;
    
    std::cout << "\n--- Testing Float Mahoney Filter ---" << std::endl;
    std::vector<Eigen::Quaternion<float>> mahoney_float_results;
    for (const auto& param : mahoney_params) {
        EntoAttitude::FilterMahoney<float, false> mahoney;
        EntoMath::Vec3<float> bias(0.0f, 0.0f, 0.0f);
        Eigen::Quaternion<float> result = mahoney(q_float, meas_float, dt_float, param.second, 0.01f, bias);
        mahoney_float_results.push_back(result);
        std::cout << "Float Mahoney (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    std::cout << "\n--- Testing Float Madgwick Filter ---" << std::endl;
    std::vector<Eigen::Quaternion<float>> madgwick_float_results;
    for (const auto& param : madgwick_params) {
        EntoAttitude::FilterMadgwick<float, false> madgwick;
        Eigen::Quaternion<float> result = madgwick(q_float, meas_float, dt_float, param.second);
        madgwick_float_results.push_back(result);
        std::cout << "Float Madgwick (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    // Check float results
    bool mahoney_float_different = false;
    for (size_t i = 1; i < mahoney_float_results.size(); ++i) {
        if (mahoney_float_results[i].w() != mahoney_float_results[0].w() || 
            mahoney_float_results[i].x() != mahoney_float_results[0].x() || 
            mahoney_float_results[i].y() != mahoney_float_results[0].y() || 
            mahoney_float_results[i].z() != mahoney_float_results[0].z()) {
            mahoney_float_different = true;
            break;
        }
    }
    
    bool madgwick_float_different = false;
    for (size_t i = 1; i < madgwick_float_results.size(); ++i) {
        if (madgwick_float_results[i].w() != madgwick_float_results[0].w() || 
            madgwick_float_results[i].x() != madgwick_float_results[0].x() || 
            madgwick_float_results[i].y() != madgwick_float_results[0].y() || 
            madgwick_float_results[i].z() != madgwick_float_results[0].z()) {
            madgwick_float_different = true;
            break;
        }
    }
    
    bool algorithms_float_different = (mahoney_float_results[0].w() != madgwick_float_results[0].w() || 
                                      mahoney_float_results[0].x() != madgwick_float_results[0].x() || 
                                      mahoney_float_results[0].y() != madgwick_float_results[0].y() || 
                                      mahoney_float_results[0].z() != madgwick_float_results[0].z());
    
    std::cout << "\n--- Float Analysis ---" << std::endl;
    std::cout << "Float Mahoney with different parameters: " << (mahoney_float_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Float Madgwick with different gains: " << (madgwick_float_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Float Mahoney vs Madgwick: " << (algorithms_float_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    
    // =============================================================================
    // DOUBLE-PRECISION CONTROLS
    // =============================================================================
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "DOUBLE-PRECISION CONTROLS" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    // Convert data to double
    Eigen::Quaternion<double> q_double(1.0, 0.000031, 0.000003, -0.000006);
    EntoAttitude::AttitudeMeasurement<double, false> meas_double;
    meas_double.acc = EntoMath::Vec3<double>(-0.021139, 0.037897, 8.326411);
    meas_double.gyr = EntoMath::Vec3<double>(-0.000050, -0.000042, 0.000172);
    double dt_double = 0.001;
    
    std::cout << "\n--- Testing Double Mahoney Filter ---" << std::endl;
    std::vector<Eigen::Quaternion<double>> mahoney_double_results;
    for (const auto& param : mahoney_params) {
        EntoAttitude::FilterMahoney<double, false> mahoney;
        EntoMath::Vec3<double> bias(0.0, 0.0, 0.0);
        Eigen::Quaternion<double> result = mahoney(q_double, meas_double, dt_double, param.second, 0.01, bias);
        mahoney_double_results.push_back(result);
        std::cout << "Double Mahoney (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    std::cout << "\n--- Testing Double Madgwick Filter ---" << std::endl;
    std::vector<Eigen::Quaternion<double>> madgwick_double_results;
    for (const auto& param : madgwick_params) {
        EntoAttitude::FilterMadgwick<double, false> madgwick;
        Eigen::Quaternion<double> result = madgwick(q_double, meas_double, dt_double, param.second);
        madgwick_double_results.push_back(result);
        std::cout << "Double Madgwick (" << param.first << "): [" << result.w() << ", " << result.x() << ", " << result.y() << ", " << result.z() << "]" << std::endl;
    }
    
    // Check double results
    bool mahoney_double_different = false;
    for (size_t i = 1; i < mahoney_double_results.size(); ++i) {
        if (mahoney_double_results[i].w() != mahoney_double_results[0].w() || 
            mahoney_double_results[i].x() != mahoney_double_results[0].x() || 
            mahoney_double_results[i].y() != mahoney_double_results[0].y() || 
            mahoney_double_results[i].z() != mahoney_double_results[0].z()) {
            mahoney_double_different = true;
            break;
        }
    }
    
    bool madgwick_double_different = false;
    for (size_t i = 1; i < madgwick_double_results.size(); ++i) {
        if (madgwick_double_results[i].w() != madgwick_double_results[0].w() || 
            madgwick_double_results[i].x() != madgwick_double_results[0].x() || 
            madgwick_double_results[i].y() != madgwick_double_results[0].y() || 
            madgwick_double_results[i].z() != madgwick_double_results[0].z()) {
            madgwick_double_different = true;
            break;
        }
    }
    
    bool algorithms_double_different = (mahoney_double_results[0].w() != madgwick_double_results[0].w() || 
                                       mahoney_double_results[0].x() != madgwick_double_results[0].x() || 
                                       mahoney_double_results[0].y() != madgwick_double_results[0].y() || 
                                       mahoney_double_results[0].z() != madgwick_double_results[0].z());
    
    std::cout << "\n--- Double Analysis ---" << std::endl;
    std::cout << "Double Mahoney with different parameters: " << (mahoney_double_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Double Madgwick with different gains: " << (madgwick_double_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    std::cout << "Double Mahoney vs Madgwick: " << (algorithms_double_different ? "DIFFERENT" : "IDENTICAL") << std::endl;
    
    // =============================================================================
    // SUMMARY
    // =============================================================================
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "SUMMARY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Fixed-point (Q2.30):" << std::endl;
    std::cout << "  - Mahoney parameter sensitivity: " << (mahoney_different ? "YES" : "NO") << std::endl;
    std::cout << "  - Madgwick gain sensitivity: " << (madgwick_different ? "YES" : "NO") << std::endl;
    std::cout << "  - Algorithm differences: " << (algorithms_different ? "YES" : "NO") << std::endl;
    std::cout << "Float:" << std::endl;
    std::cout << "  - Mahoney parameter sensitivity: " << (mahoney_float_different ? "YES" : "NO") << std::endl;
    std::cout << "  - Madgwick gain sensitivity: " << (madgwick_float_different ? "YES" : "NO") << std::endl;
    std::cout << "  - Algorithm differences: " << (algorithms_float_different ? "YES" : "NO") << std::endl;
    std::cout << "Double:" << std::endl;
    std::cout << "  - Mahoney parameter sensitivity: " << (mahoney_double_different ? "YES" : "NO") << std::endl;
    std::cout << "  - Madgwick gain sensitivity: " << (madgwick_double_different ? "YES" : "NO") << std::endl;
    std::cout << "  - Algorithm differences: " << (algorithms_double_different ? "YES" : "NO") << std::endl;
    
    // List of Q-formats to test
    struct QFormatDesc {
        std::string name;
        int int_bits, frac_bits;
        using ScalarType = void;
    };
    // We'll use type aliases for each format
    #define TEST_QFORMAT(NAME, INTB, FRACB, TYPE) \
        { #NAME, INTB, FRACB }
    std::vector<std::tuple<std::string, std::function<void()>>> qformats = {
        {"Q7.24", [&](){
            using Scalar = FixedPoint<7,24,int32_t>;
            run_fixedpoint_test<Scalar>("Q7.24");
        }},
        {"Q5.26", [&](){
            using Scalar = FixedPoint<5,26,int32_t>;
            run_fixedpoint_test<Scalar>("Q5.26");
        }},
        {"Q8.23", [&](){
            using Scalar = FixedPoint<8,23,int32_t>;
            run_fixedpoint_test<Scalar>("Q8.23");
        }},
        {"Q4.27", [&](){
            using Scalar = FixedPoint<4,27,int32_t>;
            run_fixedpoint_test<Scalar>("Q4.27");
        }},
        {"Q2.29", [&](){
            using Scalar = FixedPoint<2,29,int32_t>;
            run_fixedpoint_test<Scalar>("Q2.29");
        }},
        {"Q2.13", [&](){
            using Scalar = FixedPoint<2,13,int16_t>;
            run_fixedpoint_test<Scalar>("Q2.13");
        }},
        {"Q4.11", [&](){
            using Scalar = FixedPoint<4,11,int16_t>;
            run_fixedpoint_test<Scalar>("Q4.11");
        }},
    };

    // Run all Q-format tests
    for (const auto& qf : qformats) {
        std::get<1>(qf)();
    }
    
    return 0;
} 