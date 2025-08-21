#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/madgwick.h>

#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif

using namespace EntoAttitude;

// Test data: ax ay az gx gy gz mx my mz qw qx qy qz dt (MARG format)
const char* test_data_marg = "0.1 0.2 9.8 0.01 0.02 0.03 25.0 5.0 -40.0 1.0 0.0 0.0 0.0 0.01";

// Test data: ax ay az gx gy gz qw qx qy qz dt (IMU format)  
const char* test_data_imu = "0.1 0.2 9.8 0.01 0.02 0.03 1.0 0.0 0.0 0.0 0.01";

template<typename Scalar>
void test_deserialization_precision() {
    // Test MARG (magnetometer) version
    {
        FilterMahoney<Scalar, true> mahony_filter;
        AttitudeProblem<Scalar, FilterMahoney<Scalar, true>, true> problem(mahony_filter, Scalar(1.0), Scalar(0.1));
        
        bool success = problem.deserialize_impl(test_data_marg);
        
#ifdef NATIVE
        std::cout << "=== MARG Deserialization Test (" << typeid(Scalar).name() << ") ===" << std::endl;
        std::cout << "Success: " << (success ? "YES" : "NO") << std::endl;
        
        if (success) {
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "Accelerometer: (" << problem.measurement_.acc[0] << ", " 
                      << problem.measurement_.acc[1] << ", " << problem.measurement_.acc[2] << ")" << std::endl;
            std::cout << "Gyroscope: (" << problem.measurement_.gyr[0] << ", " 
                      << problem.measurement_.gyr[1] << ", " << problem.measurement_.gyr[2] << ")" << std::endl;
            std::cout << "Magnetometer: (" << problem.measurement_.mag[0] << ", " 
                      << problem.measurement_.mag[1] << ", " << problem.measurement_.mag[2] << ")" << std::endl;
            std::cout << "Ground Truth Quaternion: (" << problem.q_gt_.w() << ", " 
                      << problem.q_gt_.x() << ", " << problem.q_gt_.y() << ", " << problem.q_gt_.z() << ")" << std::endl;
            std::cout << "Delta Time: " << problem.dt_ << std::endl;
        }
        std::cout << std::endl;
#else
        // For embedded builds, use debug output
        ENTO_DEBUG("MARG Deserialization (%s): %s", typeid(Scalar).name(), success ? "SUCCESS" : "FAILED");
        if (success) {
            ENTO_DEBUG("Acc: (%.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.measurement_.acc[0]),
                      static_cast<float>(problem.measurement_.acc[1]), 
                      static_cast<float>(problem.measurement_.acc[2]));
            ENTO_DEBUG("Gyr: (%.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.measurement_.gyr[0]),
                      static_cast<float>(problem.measurement_.gyr[1]), 
                      static_cast<float>(problem.measurement_.gyr[2]));
            ENTO_DEBUG("Mag: (%.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.measurement_.mag[0]),
                      static_cast<float>(problem.measurement_.mag[1]), 
                      static_cast<float>(problem.measurement_.mag[2]));
            ENTO_DEBUG("GT Quat: (%.6f, %.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.q_gt_.w()),
                      static_cast<float>(problem.q_gt_.x()),
                      static_cast<float>(problem.q_gt_.y()),
                      static_cast<float>(problem.q_gt_.z()));
            ENTO_DEBUG("DT: %.6f", static_cast<float>(problem.dt_));
        }
#endif
    }
    
    // Test IMU (no magnetometer) version
    {
        FilterMadgwick<Scalar, false> madgwick_filter;
        AttitudeProblem<Scalar, FilterMadgwick<Scalar, false>, false> problem(madgwick_filter, Scalar(0.1));
        
        bool success = problem.deserialize_impl(test_data_imu);
        
#ifdef NATIVE
        std::cout << "=== IMU Deserialization Test (" << typeid(Scalar).name() << ") ===" << std::endl;
        std::cout << "Success: " << (success ? "YES" : "NO") << std::endl;
        
        if (success) {
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "Accelerometer: (" << problem.measurement_.acc[0] << ", " 
                      << problem.measurement_.acc[1] << ", " << problem.measurement_.acc[2] << ")" << std::endl;
            std::cout << "Gyroscope: (" << problem.measurement_.gyr[0] << ", " 
                      << problem.measurement_.gyr[1] << ", " << problem.measurement_.gyr[2] << ")" << std::endl;
            std::cout << "Ground Truth Quaternion: (" << problem.q_gt_.w() << ", " 
                      << problem.q_gt_.x() << ", " << problem.q_gt_.y() << ", " << problem.q_gt_.z() << ")" << std::endl;
            std::cout << "Delta Time: " << problem.dt_ << std::endl;
        }
        std::cout << std::endl;
#else
        // For embedded builds, use debug output
        ENTO_DEBUG("IMU Deserialization (%s): %s", typeid(Scalar).name(), success ? "SUCCESS" : "FAILED");
        if (success) {
            ENTO_DEBUG("Acc: (%.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.measurement_.acc[0]),
                      static_cast<float>(problem.measurement_.acc[1]), 
                      static_cast<float>(problem.measurement_.acc[2]));
            ENTO_DEBUG("Gyr: (%.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.measurement_.gyr[0]),
                      static_cast<float>(problem.measurement_.gyr[1]), 
                      static_cast<float>(problem.measurement_.gyr[2]));
            ENTO_DEBUG("GT Quat: (%.6f, %.6f, %.6f, %.6f)", 
                      static_cast<float>(problem.q_gt_.w()),
                      static_cast<float>(problem.q_gt_.x()),
                      static_cast<float>(problem.q_gt_.y()),
                      static_cast<float>(problem.q_gt_.z()));
            ENTO_DEBUG("DT: %.6f", static_cast<float>(problem.dt_));
        }
#endif
    }
}

int main() {
#ifdef NATIVE
    std::cout << "=== FPU Deserialization Test ===" << std::endl;
    std::cout << "Testing float and double precision deserialization..." << std::endl;
    std::cout << std::endl;
#else
    ENTO_DEBUG("=== FPU Deserialization Test ===");
    
    // Print FPU configuration info
#ifdef HARDWARE_FPU_DOUBLE
    ENTO_DEBUG("Hardware double-precision FPU enabled");
#endif
#ifdef __FPU_PRESENT
    ENTO_DEBUG("FPU present: %d", __FPU_PRESENT);
#endif
#ifdef __FPU_USED  
    ENTO_DEBUG("FPU used: %d", __FPU_USED);
#endif
#endif

    // Test float precision
    test_deserialization_precision<float>();
    
    // Test double precision
    test_deserialization_precision<double>();
    
#ifdef NATIVE
    std::cout << "=== Test Complete ===" << std::endl;
#else
    ENTO_DEBUG("=== Test Complete ===");
#endif
    
    return 0;
} 