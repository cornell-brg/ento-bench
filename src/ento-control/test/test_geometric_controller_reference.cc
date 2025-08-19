#include <stdlib.h>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cmath>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-control/geometric_controller_problem.h>
#include <ento-math/core.h>

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoControl;

// Test basic initialization of the GeometricControllerProblem with ReferenceControlProblem
void test_geometric_controller_problem_init() {
  ENTO_DEBUG("Testing GeometricControllerProblem initialization...");
  
  using Scalar = float;
  using Problem = GeometricControllerProblem<Scalar>;
  
  // Create problem instance with 10ms timestep
  Problem problem(0.01f);
  
  // Check that the time step is set correctly
  ENTO_TEST_CHECK_FLOAT_EQ(problem.get_dt(), 0.01f);
  
  ENTO_DEBUG("GeometricControllerProblem initialized successfully");
}

// Test loading FDCL reference trajectory data
void test_fdcl_trajectory_loading() {
  ENTO_DEBUG("Testing FDCL trajectory loading...");
  
  using Scalar = float;
  using Problem = GeometricControllerProblem<Scalar>;
  
  Problem problem(0.01f);
  
  // Try to load FDCL helix trajectory
  const string trajectory_file = "../../datasets/geometric-control/fdcl_helix.csv";
  ifstream file(trajectory_file);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Warning: Could not open %s, using simple hover data", trajectory_file.c_str());
    
    // Fallback to simple hover reference
    const char* ref_line = "0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0";
    for (int i = 0; i < 3; i++) {
      bool success = problem.deserialize_impl(ref_line);
      ENTO_TEST_CHECK_TRUE(success);
    }
  } else {
    ENTO_DEBUG("Loading FDCL trajectory from %s", trajectory_file.c_str());
    
    string line;
    int loaded_points = 0;
    
    // Skip header lines
    while (getline(file, line) && line[0] == '#') {
      // Skip comments
    }
    
    // Load trajectory data
    do {
      if (!line.empty() && line[0] != '#') {
        bool success = problem.deserialize_impl(line.c_str());
        if (success) {
          loaded_points++;
        } else {
          ENTO_DEBUG("Failed to parse line: %s", line.c_str());
        }
      }
    } while (getline(file, line));
    
    file.close();
    ENTO_TEST_CHECK_TRUE(loaded_points > 0);
    ENTO_DEBUG("Successfully loaded %d FDCL trajectory points", loaded_points);
  }
}

// Test controller execution with FDCL reference trajectory
void test_controller_execution_fdcl() {
  ENTO_DEBUG("Testing controller execution with FDCL trajectory...");
  
  using Scalar = float;
  using Problem = GeometricControllerProblem<Scalar>;
  
  Problem problem(0.01f);
  
  // Load FDCL trajectory
  const string trajectory_file = "../../datasets/geometric-control/fdcl_helix.csv";
  ifstream file(trajectory_file);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Warning: Could not open %s, using simple hover data", trajectory_file.c_str());
    
    // Fallback to simple hover reference
    const char* ref_line = "0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0";
    for (int i = 0; i < 3; i++) {
      bool success = problem.deserialize_impl(ref_line);
      ENTO_TEST_CHECK_TRUE(success);
    }
  } else {
    string line;
    int loaded_points = 0;
    
    // Skip header lines
    while (getline(file, line) && line[0] == '#') {
      // Skip comments
    }
    
    // Load trajectory data
    do {
      if (!line.empty() && line[0] != '#') {
        bool success = problem.deserialize_impl(line.c_str());
        if (success) {
          loaded_points++;
        }
      }
    } while (getline(file, line));
    
    file.close();
    ENTO_DEBUG("Loaded %d FDCL trajectory points", loaded_points);
  }
  
  // Run the controller for available steps
  int max_steps = 3;
  for (int i = 0; i < max_steps; i++) {
    ENTO_DEBUG("Running controller step %d", i);
    problem.solve_impl();
    
    // Serialize the control output to verify it's working
    string control_output = problem.serialize_impl();
    ENTO_DEBUG("Control output step %d: %s", i, control_output.c_str());
    
    // Check that we got some control output (not empty)
    ENTO_TEST_CHECK_TRUE(!control_output.empty());
    
    // Parse and check control values are reasonable
    // Format: thrust,Mx,My,Mz
    size_t comma1 = control_output.find(',');
    if (comma1 != string::npos) {
      float thrust = stof(control_output.substr(0, comma1));
      ENTO_DEBUG("  Thrust: %.3f N", thrust);
      
      // Check thrust is positive and reasonable (should be around mg = 19.13N for hover)
      ENTO_TEST_CHECK_TRUE(thrust > 0.0f);
      ENTO_TEST_CHECK_TRUE(thrust < 100.0f);  // Sanity check
    }
  }
  
  ENTO_DEBUG("Controller execution with FDCL trajectory completed successfully");
}

// Test validation functionality
void test_validation() {
  ENTO_DEBUG("Testing validation functionality...");
  
  using Scalar = float;
  using Problem = GeometricControllerProblem<Scalar>;
  
  Problem problem(0.01f);
  
  // Load reference trajectory (fallback to hover if FDCL data not available)
  const char* ref_line = "0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0";
  
  for (int i = 0; i < 3; i++) {
    bool success = problem.deserialize_impl(ref_line);
    ENTO_TEST_CHECK_TRUE(success);
  }
  
  // Run controller for 2 steps
  for (int i = 0; i < 2; i++) {
    problem.solve_impl();
  }
  
  // Validate results
  bool valid = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(valid);
  
  ENTO_DEBUG("Validation completed successfully");
}

// Test different gain configurations
void test_gain_configurations() {
  ENTO_DEBUG("Testing different gain configurations...");
  
  using Scalar = float;
  using Problem = GeometricControllerProblem<Scalar>;
  
  Problem problem(0.01f);
  
  // Test conservative gains
  problem.set_conservative_gains();
  ENTO_DEBUG("Set conservative gains");
  
  // Test aggressive gains
  problem.set_aggressive_gains();
  ENTO_DEBUG("Set aggressive gains");
  
  // Load a reference point and run one step to verify gains work
  const char* ref_line = "0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0";
  bool success = problem.deserialize_impl(ref_line);
  ENTO_TEST_CHECK_TRUE(success);
  
  problem.solve_impl();
  
  // Check that we got control output
  string control_output = problem.serialize_impl();
  ENTO_TEST_CHECK_TRUE(!control_output.empty());
  
  ENTO_DEBUG("Gain configuration test completed successfully");
}

// Test FDCL single step data
void test_fdcl_single_step() {
  ENTO_DEBUG("Testing FDCL single step data...");
  
  using Scalar = float;
  using Problem = GeometricControllerProblem<Scalar>;
  
  Problem problem(0.01f);
  
  // Try to load FDCL single step trajectory
  const string trajectory_file = "../../datasets/geometric-control/fdcl_single_step.csv";
  ifstream file(trajectory_file);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Warning: Could not open %s, skipping test", trajectory_file.c_str());
    return;
  }
  
  string line;
  
  // Skip header lines
  while (getline(file, line) && line[0] == '#') {
    // Skip comments
  }
  
  // Load the single trajectory point
  if (!line.empty() && line[0] != '#') {
    bool success = problem.deserialize_impl(line.c_str());
    ENTO_TEST_CHECK_TRUE(success);
    ENTO_DEBUG("Loaded FDCL single step data: %s", line.c_str());
    
    // Run controller once
    problem.solve_impl();
    
    // Get control output
    string control_output = problem.serialize_impl();
    ENTO_DEBUG("FDCL single step control output: %s", control_output.c_str());
    ENTO_TEST_CHECK_TRUE(!control_output.empty());
  }
  
  file.close();
  ENTO_DEBUG("FDCL single step test completed successfully");
}

// FDCL reference trajectory data point
struct FDCLReferencePoint {
    double time;
    Vector3f position;
    Vector3f velocity;
    Vector4f quaternion;  // qw, qx, qy, qz
    Vector3f angular_velocity;
    float thrust;
    Vector3f moment;
};

// Load FDCL reference trajectory from CSV
vector<FDCLReferencePoint> load_fdcl_reference(const string& filename) {
    vector<FDCLReferencePoint> trajectory;
    ifstream file(filename);
    
    if (!file.is_open()) {
        ENTO_DEBUG("Cannot open reference trajectory file: %s", filename.c_str());
        return trajectory;
    }
    
    string line;
    // Skip header line
    getline(file, line);
    
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<double> values;
        
        // Parse CSV values
        while (getline(ss, cell, ',')) {
            values.push_back(stod(cell));
        }
        
        // Parse CSV line (expecting 18 columns: time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,qw,qx,qy,qz,wx,wy,wz,thrust,moment_x,moment_y,moment_z)
        if (values.size() != 18) {
            ENTO_DEBUG("Invalid CSV format: expected 18 columns, got %zu", values.size());
            continue;
        }
        
        FDCLReferencePoint point;
        point.time = values[0];
        point.position = Vector3f(values[1], values[2], values[3]);
        point.velocity = Vector3f(values[4], values[5], values[6]);
        point.quaternion = Vector4f(values[7], values[8], values[9], values[10]);  // qw, qx, qy, qz
        point.angular_velocity = Vector3f(values[11], values[12], values[13]);
        point.thrust = values[14];
        point.moment = Vector3f(values[15], values[16], values[17]);
        
        trajectory.push_back(point);
    }
    
    ENTO_DEBUG("Loaded %zu reference points from %s", trajectory.size(), filename.c_str());
    return trajectory;
}

// Test comparison with FDCL reference trajectory
void test_fdcl_comparison() {
    ENTO_DEBUG("Testing comparison with FDCL reference trajectory...");
    
    using Scalar = float;
    using Problem = GeometricControllerProblem<Scalar>;
    
    // Load FDCL reference trajectory
    const string trajectory_file = "../../datasets/geometric-control/fdcl_helix_reference.csv";
    vector<FDCLReferencePoint> reference = load_fdcl_reference(trajectory_file);
    
    if (reference.empty()) {
        ENTO_DEBUG("Warning: Could not load FDCL reference trajectory, skipping comparison test");
        return;
    }
    
    ENTO_DEBUG("Loaded FDCL reference with %zu points", reference.size());
    
    // Create problem with FDCL parameters
    Problem problem(0.01f);  // 10ms timestep to match FDCL
    
    // The gains are now set by default in ControlGains constructor to match Python simulation
    ENTO_DEBUG("Using default FDCL-compatible gains from QuadrotorTraits");
    
    // Convert first few reference points to our format and test
    int max_test_points = min(10, (int)reference.size());
    double total_position_error = 0.0;
    double max_position_error = 0.0;
    double total_thrust_error = 0.0;
    double max_thrust_error = 0.0;
    int comparison_points = 0;
    
    for (int i = 0; i < max_test_points; i++) {
        const auto& ref_point = reference[i];
        
        // Convert FDCL reference point to our format
        // Format: pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,qw,qx,qy,qz,wx,wy,wz
        stringstream ss;
        ss << fixed << setprecision(6);
        ss << ref_point.position[0] << "," << ref_point.position[1] << "," << ref_point.position[2] << ",";
        ss << ref_point.velocity[0] << "," << ref_point.velocity[1] << "," << ref_point.velocity[2] << ",";
        ss << ref_point.quaternion[0] << "," << ref_point.quaternion[1] << "," << ref_point.quaternion[2] << "," << ref_point.quaternion[3] << ",";
        ss << ref_point.angular_velocity[0] << "," << ref_point.angular_velocity[1] << "," << ref_point.angular_velocity[2];
        
        string ref_line = ss.str();
        
        // Load reference point into problem
        bool success = problem.deserialize_impl(ref_line.c_str());
        ENTO_TEST_CHECK_TRUE(success);
        
        // Run controller
        problem.solve_impl();
        
        // Get our control output
        string control_output = problem.serialize_impl();
        
        // Parse our control output (format: thrust,Mx,My,Mz)
        stringstream control_ss(control_output);
        string thrust_str;
        getline(control_ss, thrust_str, ',');
        float our_thrust = stof(thrust_str);
        
        // Compare with FDCL reference
        float thrust_error = abs(our_thrust - ref_point.thrust);
        total_thrust_error += thrust_error;
        max_thrust_error = max(max_thrust_error, (double)thrust_error);
        
        comparison_points++;
        
        // Print detailed comparison for first few points
        if (i < 3) {
            ENTO_DEBUG("t=%.3f: pos=[%.3f,%.3f,%.3f]", ref_point.time, 
                      ref_point.position[0], ref_point.position[1], ref_point.position[2]);
            ENTO_DEBUG("  Thrust - Ours: %.3f N, FDCL: %.3f N, Error: %.3f N", 
                      our_thrust, ref_point.thrust, thrust_error);
        }
    }
    
    if (comparison_points > 0) {
        double avg_thrust_error = total_thrust_error / comparison_points;
        
        ENTO_DEBUG("=== FDCL Comparison Results ===");
        ENTO_DEBUG("Comparison points: %d", comparison_points);
        ENTO_DEBUG("Average thrust error: %.3f N", avg_thrust_error);
        ENTO_DEBUG("Maximum thrust error: %.3f N", max_thrust_error);
        
        // Reasonable error thresholds for comparison
        ENTO_TEST_CHECK_TRUE(avg_thrust_error < 2.0f);  // Average error < 2N
        ENTO_TEST_CHECK_TRUE(max_thrust_error < 10.0f); // Max error < 10N
        
        ENTO_DEBUG("FDCL comparison test completed successfully");
    } else {
        ENTO_DEBUG("No comparison points processed");
    }
}

int main(int argc, char** argv)
{
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_geometric_controller_reference_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_geometric_controller_problem_init();
  if (__ento_test_num(__n, 2)) test_fdcl_trajectory_loading();
  if (__ento_test_num(__n, 3)) test_controller_execution_fdcl();
  if (__ento_test_num(__n, 4)) test_validation();
  if (__ento_test_num(__n, 5)) test_gain_configurations();
  if (__ento_test_num(__n, 6)) test_fdcl_single_step();
  if (__ento_test_num(__n, 7)) test_fdcl_comparison();
  
  ENTO_TEST_END();
} 