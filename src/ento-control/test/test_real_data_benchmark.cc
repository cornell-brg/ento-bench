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

using namespace EntoControl;
using namespace EntoMath;
using namespace std;

// Type aliases for different controller configurations
using GeometricSolver = GeometricControllerSolver<float, QuadrotorTraits<float>, false, false>;
using AdaptiveControlProblem = ControlProblem<float, 10, 3, AdaptiveController>;
using GeometricControlProblem = ControlProblem<float, 10, 3, GeometricSolver>;

class RealDataBenchmark {
private:
    std::vector<std::string> trajectory_data_;
    std::string dataset_name_;
    
public:
    RealDataBenchmark(const std::string& data_file, const std::string& dataset_name) 
        : dataset_name_(dataset_name) {
        load_trajectory_data(data_file);
    }
    
    bool load_trajectory_data(const std::string& filename) {
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
        std::cout << "\n=== Benchmarking " << controller_name << " on " << dataset_name_ << " ===" << std::endl;
        
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
        std::cout << "  Dataset: " << dataset_name_ << std::endl;
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
        std::string filename = controller_name + "_" + dataset_name_ + "_benchmark.csv";
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
        std::cout << "\n🏁 Running Comparative Benchmark on Real Data" << std::endl;
        std::cout << "Dataset: " << dataset_name_ << std::endl;
        std::cout << "Trajectory points: " << trajectory_data_.size() << std::endl;
        
        // Benchmark different controllers
        benchmark_controller<GeometricControlProblem>("GeometricController");
        benchmark_controller<AdaptiveControlProblem>("AdaptiveController");
        
        std::cout << "\n✅ Comparative benchmark complete!" << std::endl;
    }
};

void test_real_data_benchmark() {
    std::cout << "Testing Real Data Benchmark..." << std::endl;
    
    // Test with different data sources
    std::vector<std::pair<std::string, std::string>> test_datasets = {
        {"data/real_flight_trajectory.csv", "RealFlight"},
        {"data/euroc_trajectory.csv", "EuRoC"},
        {"data/synthetic_helix.csv", "SyntheticHelix"}
    };
    
    for (const auto& [filename, dataset_name] : test_datasets) {
        std::ifstream test_file(filename);
        if (test_file.good()) {
            test_file.close();
            
            std::cout << "\n🔍 Found dataset: " << filename << std::endl;
            RealDataBenchmark benchmark(filename, dataset_name);
            benchmark.run_comparative_benchmark();
        } else {
            std::cout << "⚠️  Dataset not found: " << filename << " (skipping)" << std::endl;
        }
    }
}

void test_synthetic_data_generation() {
    std::cout << "\n🔧 Generating synthetic test data..." << std::endl;
    
    // Generate a simple synthetic trajectory for testing
    std::ofstream file("data/synthetic_helix.csv");
    if (file.is_open()) {
        file << "# Synthetic helix trajectory for testing" << std::endl;
        file << "# State vector: x,y,z,vx,vy,vz,roll,pitch,yaw,yaw_rate,wx,wy,wz" << std::endl;
        
        const int num_points = 100;
        const float dt = 0.01f;
        const float radius = 1.0f;
        const float frequency = 0.1f;
        const float climb_rate = 0.1f;
        
        for (int i = 0; i < num_points; i++) {
            float t = i * dt;
            float angle = 2.0f * M_PI * frequency * t;
            
            // Helix trajectory
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            float z = 1.0f + climb_rate * t;
            
            // Velocities
            float vx = -radius * 2.0f * M_PI * frequency * sin(angle);
            float vy = radius * 2.0f * M_PI * frequency * cos(angle);
            float vz = climb_rate;
            
            // Orientation (simple)
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = angle;
            float yaw_rate = 2.0f * M_PI * frequency;
            
            // Angular velocities
            float wx = 0.0f;
            float wy = 0.0f;
            float wz = yaw_rate;
            
            file << x << "," << y << "," << z << "," 
                 << vx << "," << vy << "," << vz << ","
                 << roll << "," << pitch << "," << yaw << "," << yaw_rate << ","
                 << wx << "," << wy << "," << wz << std::endl;
        }
        
        file.close();
        std::cout << "Generated synthetic helix trajectory: data/synthetic_helix.csv" << std::endl;
    }
}

int main() {
    std::cout << "🚁 Real Data Benchmark for ento-bench" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    // Create data directory
    system("mkdir -p data");
    
    // Generate synthetic data for testing
    test_synthetic_data_generation();
    
    // Run benchmark tests
    test_real_data_benchmark();
    
    std::cout << "\n🎯 Real data benchmark testing complete!" << std::endl;
    std::cout << "\nTo use with your own data:" << std::endl;
    std::cout << "1. Convert your data using: python3 tools/real_data_converter.py" << std::endl;
    std::cout << "2. Place CSV files in the data/ directory" << std::endl;
    std::cout << "3. Run this benchmark to compare controller performance" << std::endl;
    
    return 0;
} 