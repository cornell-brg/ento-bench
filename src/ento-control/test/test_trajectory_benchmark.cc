#include <ento-bench/problem.h>
#include <ento-control/control_problem.h>
#include <ento-control/geometric_controller_solver.h>
#include <ento-control/adaptive_controller.h>
#include <ento-math/core.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <cassert>
#include <cmath>

using namespace EntoControl;
using namespace EntoMath;
using namespace std;

// Type aliases for different controller configurations
using GeometricSolver = GeometricControllerSolver<float, QuadrotorTraits<float>, false, false>;
using AdaptiveControlProblemType = AdaptiveControlProblem<float, 10, 3>;  // 10 states, 3 controls
using GeometricControlProblemType = GeometricControlProblem<float>;       // 13 states, 4 controls

class TrajectoryBenchmark {
private:
    std::vector<std::string> trajectory_data_;
    std::string trajectory_name_;
    
public:
    TrajectoryBenchmark(const std::string& trajectory_name) 
        : trajectory_name_(trajectory_name) {}
    
    // Generate synthetic trajectories directly in C++
    void generate_hover_trajectory(float duration = 5.0f, float dt = 0.01f) {
        trajectory_name_ = "Hover";
        trajectory_data_.clear();
        
        int num_points = static_cast<int>(duration / dt);
        
        for (int i = 0; i < num_points; i++) {
            // Hover at (0, 0, 1)
            std::ostringstream oss;
            oss << "0,0,1,0,0,0,0,0,0,0,0,0,0";
            trajectory_data_.push_back(oss.str());
        }
        
        std::cout << "Generated hover trajectory: " << num_points << " points" << std::endl;
    }
    
    void generate_circle_trajectory(float duration = 10.0f, float radius = 1.0f, 
                                   float angular_velocity = 0.5f, float dt = 0.01f) {
        trajectory_name_ = "Circle";
        trajectory_data_.clear();
        
        int num_points = static_cast<int>(duration / dt);
        
        for (int i = 0; i < num_points; i++) {
            float t = i * dt;
            float angle = angular_velocity * t;
            
            // Position
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            float z = 1.0f;
            
            // Velocity
            float vx = -radius * angular_velocity * sin(angle);
            float vy = radius * angular_velocity * cos(angle);
            float vz = 0.0f;
            
            // Orientation
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = angle + M_PI/2;  // Tangent direction
            float yaw_rate = angular_velocity;
            
            // Angular velocities
            float wx = 0.0f;
            float wy = 0.0f;
            float wz = yaw_rate;
            
            std::ostringstream oss;
            oss << x << "," << y << "," << z << ","
                << vx << "," << vy << "," << vz << ","
                << roll << "," << pitch << "," << yaw << "," << yaw_rate << ","
                << wx << "," << wy << "," << wz;
            trajectory_data_.push_back(oss.str());
        }
        
        std::cout << "Generated circle trajectory: " << num_points << " points, r=" << radius 
                  << "m, ω=" << angular_velocity << "rad/s" << std::endl;
    }
    
    void generate_helix_trajectory(float duration = 15.0f, float radius = 1.0f,
                                  float climb_rate = 0.2f, float frequency = 0.2f, float dt = 0.01f) {
        trajectory_name_ = "Helix";
        trajectory_data_.clear();
        
        int num_points = static_cast<int>(duration / dt);
        float omega = 2.0f * M_PI * frequency;
        
        for (int i = 0; i < num_points; i++) {
            float t = i * dt;
            float angle = omega * t;
            
            // Position
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            float z = 1.0f + climb_rate * t;
            
            // Velocity
            float vx = -radius * omega * sin(angle);
            float vy = radius * omega * cos(angle);
            float vz = climb_rate;
            
            // Orientation
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = angle + M_PI/2;
            float yaw_rate = omega;
            
            // Angular velocities
            float wx = 0.0f;
            float wy = 0.0f;
            float wz = yaw_rate;
            
            std::ostringstream oss;
            oss << x << "," << y << "," << z << ","
                << vx << "," << vy << "," << vz << ","
                << roll << "," << pitch << "," << yaw << "," << yaw_rate << ","
                << wx << "," << wy << "," << wz;
            trajectory_data_.push_back(oss.str());
        }
        
        std::cout << "Generated helix trajectory: " << num_points << " points, r=" << radius 
                  << "m, climb=" << climb_rate << "m/s, f=" << frequency << "Hz" << std::endl;
    }
    
    void generate_step_response(float step_x = 2.0f, float step_y = 0.0f, float step_z = 0.0f,
                               float duration = 5.0f, float settle_time = 1.0f, float dt = 0.01f) {
        trajectory_name_ = "Step Response";
        trajectory_data_.clear();
        
        int num_points = static_cast<int>(duration / dt);
        
        for (int i = 0; i < num_points; i++) {
            float t = i * dt;
            
            // Smooth step using sigmoid
            float progress = (t < settle_time) ? 
                1.0f / (1.0f + exp(-10.0f * (t / settle_time - 0.5f))) : 1.0f;
            
            // Position
            float x = step_x * progress;
            float y = step_y * progress;
            float z = 1.0f + step_z * progress;
            
            // Velocity (derivative of smooth step)
            float vx = 0.0f, vy = 0.0f, vz = 0.0f;
            if (t < settle_time) {
                float sigmoid_val = 1.0f / (1.0f + exp(-10.0f * (t / settle_time - 0.5f)));
                float sigmoid_deriv = 10.0f * sigmoid_val * (1.0f - sigmoid_val) / settle_time;
                vx = step_x * sigmoid_deriv;
                vy = step_y * sigmoid_deriv;
                vz = step_z * sigmoid_deriv;
            }
            
            // Orientation (face direction of motion)
            float yaw = (abs(vx) > 1e-6f || abs(vy) > 1e-6f) ? atan2(vy, vx) : 0.0f;
            
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw_rate = 0.0f;
            float wx = 0.0f, wy = 0.0f, wz = 0.0f;
            
            std::ostringstream oss;
            oss << x << "," << y << "," << z << ","
                << vx << "," << vy << "," << vz << ","
                << roll << "," << pitch << "," << yaw << "," << yaw_rate << ","
                << wx << "," << wy << "," << wz;
            trajectory_data_.push_back(oss.str());
        }
        
        std::cout << "Generated step response: " << num_points << " points, step=(" 
                  << step_x << "," << step_y << "," << step_z << ")" << std::endl;
    }
    
    bool load_trajectory_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "Error: Could not open trajectory file: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        trajectory_data_.clear();
        
        while (std::getline(file, line)) {
            // Skip comment lines
            if (line.empty() || line[0] == '#') {
                continue;
            }
            trajectory_data_.push_back(line);
        }
        
        file.close();
        std::cout << "Loaded " << trajectory_data_.size() << " trajectory points from " << filename << std::endl;
        return !trajectory_data_.empty();
    }
    
    template<typename ProblemType>
    void benchmark_controller(const std::string& controller_name) {
        std::cout << "\n=== Benchmarking " << controller_name << " on " << trajectory_name_ << " ===" << std::endl;
        
        // Create problem instance
        ProblemType problem;
        
        // Load trajectory data
        std::cout << "Loading trajectory data..." << std::endl;
        for (const auto& line : trajectory_data_) {
            bool success = problem.deserialize_impl(line.c_str());
            if (!success) {
                std::cout << "Error: Failed to deserialize trajectory point" << std::endl;
                return;
            }
        }
        
        // Benchmark metrics
        std::vector<double> solve_times;
        std::vector<bool> validation_results;
        
        auto start_total = std::chrono::high_resolution_clock::now();
        
        // Run benchmark
        std::cout << "Running benchmark..." << std::endl;
        for (size_t i = 0; i < trajectory_data_.size(); i++) {
            // Time the solve step
            auto start_solve = std::chrono::high_resolution_clock::now();
            problem.solve_impl();
            auto end_solve = std::chrono::high_resolution_clock::now();
            
            double solve_time = std::chrono::duration<double, std::micro>(end_solve - start_solve).count();
            solve_times.push_back(solve_time);
            
            // Validate the solution
            bool valid = problem.validate_impl();
            validation_results.push_back(valid);
            
            if (i % 100 == 0) {
                std::cout << "  Step " << i << "/" << trajectory_data_.size() 
                         << " - Solve time: " << solve_time << " μs, Valid: " << (valid ? "✓" : "✗") << std::endl;
            }
        }
        
        auto end_total = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double, std::milli>(end_total - start_total).count();
        
        // Calculate statistics
        double avg_solve_time = 0.0;
        double min_solve_time = solve_times[0];
        double max_solve_time = solve_times[0];
        
        for (double time : solve_times) {
            avg_solve_time += time;
            min_solve_time = std::min(min_solve_time, time);
            max_solve_time = std::max(max_solve_time, time);
        }
        avg_solve_time /= solve_times.size();
        
        size_t valid_count = 0;
        for (bool valid : validation_results) {
            if (valid) valid_count++;
        }
        
        // Print results
        std::cout << "\n📊 Benchmark Results for " << controller_name << ":" << std::endl;
        std::cout << "  Trajectory: " << trajectory_name_ << std::endl;
        std::cout << "  Trajectory points: " << trajectory_data_.size() << std::endl;
        std::cout << "  Total time: " << total_time << " ms" << std::endl;
        std::cout << "  Average solve time: " << avg_solve_time << " μs" << std::endl;
        std::cout << "  Min solve time: " << min_solve_time << " μs" << std::endl;
        std::cout << "  Max solve time: " << max_solve_time << " μs" << std::endl;
        std::cout << "  Validation success rate: " << (100.0 * valid_count / validation_results.size()) << "%" << std::endl;
        std::cout << "  Frequency: " << (1000000.0 / avg_solve_time) << " Hz" << std::endl;
        
        // Save detailed results
        save_benchmark_results(controller_name, solve_times, validation_results);
    }
    
    void save_benchmark_results(const std::string& controller_name, 
                               const std::vector<double>& solve_times,
                               const std::vector<bool>& validation_results) {
        std::string filename = controller_name + "_" + trajectory_name_ + "_benchmark.csv";
        // Replace spaces with underscores
        std::replace(filename.begin(), filename.end(), ' ', '_');
        
        std::ofstream file(filename);
        
        if (file.is_open()) {
            file << "step,solve_time_us,valid" << std::endl;
            for (size_t i = 0; i < solve_times.size(); i++) {
                file << i << "," << solve_times[i] << "," << (validation_results[i] ? 1 : 0) << std::endl;
            }
            file.close();
            std::cout << "  Detailed results saved to: " << filename << std::endl;
        }
    }
    
    void run_comparative_benchmark() {
        std::cout << "\n🏁 Running Comparative Benchmark" << std::endl;
        std::cout << "Trajectory: " << trajectory_name_ << std::endl;
        std::cout << "Trajectory points: " << trajectory_data_.size() << std::endl;
        
        // Benchmark different controllers
        benchmark_controller<GeometricControlProblemType>("GeometricController");
        benchmark_controller<AdaptiveControlProblemType>("AdaptiveController");
        
        std::cout << "\n✅ Comparative benchmark complete!" << std::endl;
    }
};

void test_trajectory_benchmarks() {
    std::cout << "🚁 Testing Trajectory-Based Benchmarks" << std::endl;
    std::cout << "======================================" << std::endl;
    
    // Test different trajectory types
    std::vector<std::function<void()>> trajectory_tests = {
        []() {
            std::cout << "\n🔵 Testing Hover Trajectory" << std::endl;
            TrajectoryBenchmark benchmark("Hover");
            benchmark.generate_hover_trajectory(3.0f);  // Short duration for testing
            benchmark.run_comparative_benchmark();
        },
        
        []() {
            std::cout << "\n🔴 Testing Circle Trajectory" << std::endl;
            TrajectoryBenchmark benchmark("Circle");
            benchmark.generate_circle_trajectory(5.0f, 1.0f, 0.5f);  // 5s, 1m radius, 0.5 rad/s
            benchmark.run_comparative_benchmark();
        },
        
        []() {
            std::cout << "\n🟡 Testing Helix Trajectory" << std::endl;
            TrajectoryBenchmark benchmark("Helix");
            benchmark.generate_helix_trajectory(8.0f, 1.0f, 0.1f, 0.2f);  // 8s, gentle helix
            benchmark.run_comparative_benchmark();
        },
        
        []() {
            std::cout << "\n🟢 Testing Step Response" << std::endl;
            TrajectoryBenchmark benchmark("Step");
            benchmark.generate_step_response(1.0f, 0.0f, 0.0f, 4.0f);  // 1m step in X
            benchmark.run_comparative_benchmark();
        }
    };
    
    // Run all trajectory tests
    for (auto& test : trajectory_tests) {
        test();
    }
}

void test_performance_scaling() {
    std::cout << "\n📈 Testing Performance Scaling" << std::endl;
    std::cout << "==============================" << std::endl;
    
    // Test how performance scales with trajectory complexity
    std::vector<std::tuple<std::string, float, float>> complexity_tests = {
        {"Simple Circle", 0.3f, 1.0f},    // Low angular velocity, small radius
        {"Medium Circle", 0.6f, 1.5f},    // Medium angular velocity, medium radius
        {"Fast Circle", 1.0f, 2.0f},      // High angular velocity, large radius
    };
    
    for (const auto& [name, angular_vel, radius] : complexity_tests) {
        std::cout << "\n--- " << name << " ---" << std::endl;
        TrajectoryBenchmark benchmark(name);
        benchmark.generate_circle_trajectory(6.0f, radius, angular_vel);
        
        // Only test geometric controller for scaling analysis
        benchmark.benchmark_controller<GeometricControlProblemType>("GeometricController");
    }
}

void test_mpc_trajectory_dependency() {
    std::cout << "\n🎯 Testing MPC Trajectory Dependency" << std::endl;
    std::cout << "====================================" << std::endl;
    
    // Test the same trajectory on MPC vs non-MPC controllers
    // This shows whether trajectory complexity affects MPC solve times
    
    TrajectoryBenchmark simple_benchmark("Simple");
    simple_benchmark.generate_hover_trajectory(3.0f);
    
    TrajectoryBenchmark complex_benchmark("Complex");
    complex_benchmark.generate_helix_trajectory(6.0f, 1.5f, 0.3f, 0.4f);  // More aggressive
    
    std::cout << "\n--- Simple Trajectory (Hover) ---" << std::endl;
    simple_benchmark.benchmark_controller<AdaptiveControlProblemType>("AdaptiveController");
    
    std::cout << "\n--- Complex Trajectory (Aggressive Helix) ---" << std::endl;
    complex_benchmark.benchmark_controller<AdaptiveControlProblemType>("AdaptiveController");
    
    std::cout << "\n💡 Note: Compare solve times between simple and complex trajectories" << std::endl;
    std::cout << "     MPC controllers may show trajectory-dependent performance" << std::endl;
}

int main() {
    std::cout << "🚁 Trajectory-Based Controller Benchmark" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        // Run comprehensive trajectory benchmarks
        test_trajectory_benchmarks();
        
        // Test performance scaling
        test_performance_scaling();
        
        // Test MPC trajectory dependency
        test_mpc_trajectory_dependency();
        
        std::cout << "\n🎯 Trajectory benchmark testing complete!" << std::endl;
        std::cout << "\nKey Insights:" << std::endl;
        std::cout << "• Geometric/LQR controllers: Solve time independent of trajectory" << std::endl;
        std::cout << "• MPC controllers: Solve time may depend on trajectory complexity" << std::endl;
        std::cout << "• Use standardized trajectories for fair controller comparison" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ Benchmark failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Benchmark failed with unknown exception" << std::endl;
        return 1;
    }
} 