#include <ento-util/debug.h>
#include <ento-state-est/attitude-est/mahoney.h>
#include <ento-state-est/attitude-est/mahoney_fixed.h>
#include <ento-state-est/attitude-est/madgwick.h>
#include <Eigen/Dense>
#include <cmath>

void debug_marg_step_by_step() {
    ENTO_DEBUG("=== MARG Debug: Step-by-Step Comparison ===");
    
    // Use the same test data from our benchmark
    Eigen::Quaternion<float> q_init(1.0f, 0.0f, 0.0f, 0.0f);
    
    // Sample data from benchmark - using realistic µT values
    Eigen::Matrix<float, 3, 1> gyr(-0.002930f, -0.001221f, -0.000488f);
    Eigen::Matrix<float, 3, 1> acc(-0.072158f, 0.030966f, 8.317409f);
    Eigen::Matrix<float, 3, 1> mag(17.0f, 8.5f, 34.0f);  // µT values (realistic)
    
    float dt = 0.004f;
    float kp = 1.0f, ki = 0.1f;
    EntoMath::Vec3<float> bias(0.0f, 0.0f, 0.0f);
    
    ENTO_DEBUG("Input data:");
    ENTO_DEBUG("  q_init: [%.6f, %.6f, %.6f, %.6f]", q_init.w(), q_init.x(), q_init.y(), q_init.z());
    ENTO_DEBUG("  gyr: [%.6f, %.6f, %.6f]", gyr.x(), gyr.y(), gyr.z());
    ENTO_DEBUG("  acc: [%.6f, %.6f, %.6f]", acc.x(), acc.y(), acc.z());
    ENTO_DEBUG("  mag: [%.1f, %.1f, %.1f]", mag.x(), mag.y(), mag.z());
    
    // Test floating-point Mahony MARG
    auto q_float = EntoAttitude::mahoney_update_marg(q_init, gyr, acc, mag, dt, kp, ki, bias);
    ENTO_DEBUG("Float Mahony MARG result: [%.6f, %.6f, %.6f, %.6f]", 
               q_float.w(), q_float.x(), q_float.y(), q_float.z());
    
    // Test fixed-point Mahony MARG
    using Q7_24 = EntoAttitude::Q7_24;
    Eigen::Quaternion<Q7_24> q_init_fp(Q7_24(1.0f), Q7_24(0.0f), Q7_24(0.0f), Q7_24(0.0f));
    Eigen::Matrix<Q7_24, 3, 1> gyr_fp, acc_fp, mag_fp;
    gyr_fp << Q7_24(gyr.x()), Q7_24(gyr.y()), Q7_24(gyr.z());
    acc_fp << Q7_24(acc.x()), Q7_24(acc.y()), Q7_24(acc.z());
    mag_fp << Q7_24(mag.x()), Q7_24(mag.y()), Q7_24(mag.z());
    Eigen::Matrix<Q7_24, 3, 1> bias_fp(Q7_24(0.0f), Q7_24(0.0f), Q7_24(0.0f));
    
    auto q_fixed = EntoAttitude::mahony_update_marg(q_init_fp, gyr_fp, acc_fp, mag_fp, 
                                                    Q7_24(dt), Q7_24(kp), Q7_24(ki), bias_fp);
    ENTO_DEBUG("Fixed Mahony MARG result: [%.6f, %.6f, %.6f, %.6f]", 
               q_fixed.w().to_float(), q_fixed.x().to_float(), q_fixed.y().to_float(), q_fixed.z().to_float());
    
    // Now let's manually step through the floating-point algorithm to see where it diverges
    ENTO_DEBUG("\n=== Manual Step-by-Step Analysis ===");
    
    // Step 1: Normalize accelerometer and magnetometer
    float a_norm = acc.norm();
    float m_norm = mag.norm();
    Eigen::Matrix<float, 3, 1> a_normalized = acc / a_norm;
    Eigen::Matrix<float, 3, 1> m_normalized = mag / m_norm;
    
    ENTO_DEBUG("Step 1 - Normalization:");
    ENTO_DEBUG("  a_norm: %.6f", a_norm);
    ENTO_DEBUG("  m_norm: %.6f", m_norm);
    ENTO_DEBUG("  a_normalized: [%.6f, %.6f, %.6f]", a_normalized.x(), a_normalized.y(), a_normalized.z());
    ENTO_DEBUG("  m_normalized: [%.6f, %.6f, %.6f]", m_normalized.x(), m_normalized.y(), m_normalized.z());
    
    // Step 2: Rotation matrix and gravity vector
    Eigen::Matrix<float, 3, 3> R = q_init.toRotationMatrix();
    Eigen::Matrix<float, 3, 1> v_a = R.transpose() * Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 1.0f);
    
    ENTO_DEBUG("Step 2 - Gravity vector:");
    ENTO_DEBUG("  v_a: [%.6f, %.6f, %.6f]", v_a.x(), v_a.y(), v_a.z());
    
    // Step 3: Magnetometer processing (the critical part!)
    Eigen::Matrix<float, 3, 1> h = R * m_normalized;
    float h_xy2 = h.x() * h.x() + h.y() * h.y();
    float h_xy_norm = (h_xy2 == 0.0f) ? 0.0f : std::sqrt(h_xy2);
    
    ENTO_DEBUG("Step 3 - Magnetometer processing:");
    ENTO_DEBUG("  h: [%.6f, %.6f, %.6f]", h.x(), h.y(), h.z());
    ENTO_DEBUG("  h_xy2: %.6f", h_xy2);
    ENTO_DEBUG("  h_xy_norm: %.6f", h_xy_norm);
    
    // Step 4: Construct v_m
    Eigen::Matrix<float, 3, 1> v_m = R.transpose() * Eigen::Matrix<float, 3, 1>(0.0f, h_xy_norm, h.z());
    float v_m_norm = v_m.norm();
    if (v_m_norm > 0.0f) {
        v_m /= v_m_norm;
    }
    
    ENTO_DEBUG("Step 4 - v_m construction:");
    ENTO_DEBUG("  v_m (before norm): [%.6f, %.6f, %.6f]", v_m.x(), v_m.y(), v_m.z());
    ENTO_DEBUG("  v_m_norm: %.6f", v_m_norm);
    ENTO_DEBUG("  v_m (after norm): [%.6f, %.6f, %.6f]", v_m.x(), v_m.y(), v_m.z());
    
    // Step 5: Cross products
    Eigen::Matrix<float, 3, 1> cross_a = a_normalized.cross(v_a);
    Eigen::Matrix<float, 3, 1> cross_m = m_normalized.cross(v_m);
    Eigen::Matrix<float, 3, 1> omega_mes = cross_a + cross_m;
    
    ENTO_DEBUG("Step 5 - Cross products:");
    ENTO_DEBUG("  cross_a: [%.6f, %.6f, %.6f]", cross_a.x(), cross_a.y(), cross_a.z());
    ENTO_DEBUG("  cross_m: [%.6f, %.6f, %.6f]", cross_m.x(), cross_m.y(), cross_m.z());
    ENTO_DEBUG("  omega_mes: [%.6f, %.6f, %.6f]", omega_mes.x(), omega_mes.y(), omega_mes.z());
    
    // Check if omega_mes is reasonable
    float omega_mes_norm = omega_mes.norm();
    ENTO_DEBUG("  omega_mes_norm: %.6f", omega_mes_norm);
    
    if (omega_mes_norm > 100.0f) {
        ENTO_DEBUG("*** WARNING: omega_mes is extremely large! This suggests a bug. ***");
    }
    
    // Let's compare with what AHRS reference would do
    ENTO_DEBUG("\n=== AHRS Reference Comparison ===");
    
    // AHRS: v_m = R.T @ [0.0, h_xy_norm, h.z()]
    // This is exactly what we're doing, so the issue must be elsewhere
    
    // Let's check the individual components more carefully
    ENTO_DEBUG("Detailed component analysis:");
    ENTO_DEBUG("  h.x(): %.6f, h.y(): %.6f, h.z(): %.6f", h.x(), h.y(), h.z());
    ENTO_DEBUG("  h_xy_norm: %.6f", h_xy_norm);
    ENTO_DEBUG("  Reference vector [0, h_xy_norm, h.z()]: [0.000000, %.6f, %.6f]", h_xy_norm, h.z());
    
    // Check if the issue is in the cross product computation
    ENTO_DEBUG("Cross product analysis:");
    ENTO_DEBUG("  m_normalized: [%.6f, %.6f, %.6f]", m_normalized.x(), m_normalized.y(), m_normalized.z());
    ENTO_DEBUG("  v_m: [%.6f, %.6f, %.6f]", v_m.x(), v_m.y(), v_m.z());
    
    // Manual cross product calculation: m_normalized × v_m
    float cross_x = m_normalized.y() * v_m.z() - m_normalized.z() * v_m.y();
    float cross_y = m_normalized.z() * v_m.x() - m_normalized.x() * v_m.z();
    float cross_z = m_normalized.x() * v_m.y() - m_normalized.y() * v_m.x();
    
    ENTO_DEBUG("  Manual cross product: [%.6f, %.6f, %.6f]", cross_x, cross_y, cross_z);
    ENTO_DEBUG("  Eigen cross product:  [%.6f, %.6f, %.6f]", cross_m.x(), cross_m.y(), cross_m.z());
    
    // The issue might be that our magnetometer values are too large
    // Let's check what happens if we scale them down
    ENTO_DEBUG("\n=== Magnetometer Scaling Test ===");
    Eigen::Matrix<float, 3, 1> mag_scaled = mag / 1000.0f;  // Scale down by 1000
    float m_scaled_norm = mag_scaled.norm();
    Eigen::Matrix<float, 3, 1> m_scaled_normalized = mag_scaled / m_scaled_norm;
    
    ENTO_DEBUG("Scaled magnetometer:");
    ENTO_DEBUG("  mag_scaled: [%.6f, %.6f, %.6f]", mag_scaled.x(), mag_scaled.y(), mag_scaled.z());
    ENTO_DEBUG("  m_scaled_normalized: [%.6f, %.6f, %.6f]", m_scaled_normalized.x(), m_scaled_normalized.y(), m_scaled_normalized.z());
    
    // This should be identical to m_normalized since normalization removes scaling
    float diff_norm = (m_scaled_normalized - m_normalized).norm();
    ENTO_DEBUG("  Difference after normalization: %.6f", diff_norm);
    
    if (diff_norm > 1e-6f) {
        ENTO_DEBUG("*** ERROR: Normalization should make scaling irrelevant! ***");
    }
}

int main() {
    debug_marg_step_by_step();
    return 0;
} 