#include <stdlib.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-control/adaptive_controller.h>
#include <ento-control/adaptive_control_problem.h>
#include "trajectory_data.h"

using namespace std;
using namespace Eigen;
using namespace EntoUtil;
using namespace EntoControl;

// Test the controller with real trajectory data
void test_adaptive_controller_real_trajectory() {
  ENTO_DEBUG("Testing AdaptiveController with real trajectory data...");
  
  // Create controller with default time step
  AdaptiveController controller;
  
  // Print controller parameters
  ENTO_DEBUG("Controller time step: %f", controller.get_dt());
  
  // Set up data collection for analysis
  vector<Matrix<float, 10, 1>> states;
  vector<Matrix<float, 10, 1>> next_states;
  vector<Matrix<float, 3, 1>> controls;
  
  // Initialize state for sequential simulation
  Matrix<float, 10, 1> current_state = Matrix<float, 10, 1>::Zero();
  
  // Position (x, y, z) from first data point
  current_state[0] = TrajectoryData::data[0][0]; // x
  current_state[1] = TrajectoryData::data[0][1]; // y
  current_state[2] = TrajectoryData::data[0][2]; // z
  
  // Orientation (roll, pitch, yaw = alpha, beta, gamma) from first data point
  current_state[6] = TrajectoryData::data[0][3]; // roll (alpha)
  current_state[7] = TrajectoryData::data[0][4]; // pitch (beta)
  current_state[8] = TrajectoryData::data[0][5]; // yaw (gamma)
  
  ENTO_DEBUG("\n----- Initial State -----");
  ENTO_DEBUG("Position: [%f, %f, %f]", current_state[0], current_state[1], current_state[2]);
  ENTO_DEBUG("Orientation: [%f, %f, %f]", current_state[6], current_state[7], current_state[8]);
  
  // Run through the trajectory
  for (size_t i = 0; i < TrajectoryData::data.size(); i++) {
    ENTO_DEBUG("\n===== STEP %zu =====", i);
    
    // Create reference state vector for this step
    Matrix<float, 10, 1> ref_state = Matrix<float, 10, 1>::Zero();
    
    // Position (x, y, z)
    ref_state[0] = TrajectoryData::data[i][0]; // x
    ref_state[1] = TrajectoryData::data[i][1]; // y
    ref_state[2] = TrajectoryData::data[i][2]; // z
    
    // Orientation (roll, pitch, yaw = alpha, beta, gamma)
    ref_state[6] = TrajectoryData::data[i][3]; // roll (alpha)
    ref_state[7] = TrajectoryData::data[i][4]; // pitch (beta)
    ref_state[8] = TrajectoryData::data[i][5]; // yaw (gamma)
    
    // Set the current state in the controller
    controller.set_x0(current_state);
    
    // Set reference state (where we want to go)
    controller.set_x_ref(ref_state);
    
    ENTO_DEBUG("Current state:");
    ENTO_DEBUG("  Position: [%f, %f, %f]", current_state[0], current_state[1], current_state[2]);
    ENTO_DEBUG("  Velocity: [%f, %f, %f]", current_state[3], current_state[4], current_state[5]);
    ENTO_DEBUG("  Orientation: [%f, %f, %f]", current_state[6], current_state[7], current_state[8]);
    ENTO_DEBUG("  Yaw rate: %f", current_state[9]);
    
    ENTO_DEBUG("Reference state:");
    ENTO_DEBUG("  Position: [%f, %f, %f]", ref_state[0], ref_state[1], ref_state[2]);
    ENTO_DEBUG("  Velocity: [%f, %f, %f]", ref_state[3], ref_state[4], ref_state[5]);
    ENTO_DEBUG("  Orientation: [%f, %f, %f]", ref_state[6], ref_state[7], ref_state[8]);
    ENTO_DEBUG("  Yaw rate: %f", ref_state[9]);
    
    // Solve for control inputs
    controller.solve();
    
    // Get the control inputs and next state
    auto u = controller.get_u0();
    auto next_state = controller.get_next_state();
    
    ENTO_DEBUG("Control outputs:");
    ENTO_DEBUG("  Thrust: %f", u[0]);
    ENTO_DEBUG("  Roll torque: %f", u[1]);
    ENTO_DEBUG("  Pitch torque: %f", u[2]);
    
    ENTO_DEBUG("Next state:");
    ENTO_DEBUG("  Position: [%f, %f, %f]", next_state[0], next_state[1], next_state[2]);
    ENTO_DEBUG("  Velocity: [%f, %f, %f]", next_state[3], next_state[4], next_state[5]);
    ENTO_DEBUG("  Orientation: [%f, %f, %f]", next_state[6], next_state[7], next_state[8]);
    ENTO_DEBUG("  Yaw rate: %f", next_state[9]);
    
    // Calculate error between reference and next state
    Vector3f pos_error;
    pos_error << ref_state[0] - next_state[0],
                 ref_state[1] - next_state[1],
                 ref_state[2] - next_state[2];
    
    Vector3f orient_error;
    orient_error << ref_state[6] - next_state[6],
                    ref_state[7] - next_state[7],
                    ref_state[8] - next_state[8];
    
    ENTO_DEBUG("Errors:");
    ENTO_DEBUG("  Position error: %f", pos_error.norm());
    ENTO_DEBUG("  Orientation error: %f", orient_error.norm());
    
    // Store results for analysis
    states.push_back(current_state);
    next_states.push_back(next_state);
    controls.push_back(u);
    
    // Verify we get finite values for next state
    for (int j = 0; j < 10; j++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(next_state[j]));
    }
    
    // Update current state for next iteration
    current_state = next_state;
  }
  
  // Optional: Write results to a CSV file for external analysis
  ofstream outfile("trajectory_results.csv");
  if (outfile.is_open()) {
    // Write header
    outfile << "step,x,y,z,vx,vy,vz,roll,pitch,yaw,yaw_rate,"
            << "ref_x,ref_y,ref_z,ref_roll,ref_pitch,ref_yaw,"
            << "next_x,next_y,next_z,next_vx,next_vy,next_vz,next_roll,next_pitch,next_yaw,next_yaw_rate,"
            << "thrust,roll_torque,pitch_torque,position_error,orientation_error" << endl;
    
    // Write data
    for (size_t i = 0; i < states.size(); i++) {
      outfile << i;
      
      // Current state
      for (int j = 0; j < 10; j++) {
        outfile << "," << states[i][j];
      }
      
      // Reference state (for this step)
      outfile << "," << TrajectoryData::data[i][0]  // x
              << "," << TrajectoryData::data[i][1]  // y
              << "," << TrajectoryData::data[i][2]  // z
              << "," << TrajectoryData::data[i][3]  // roll
              << "," << TrajectoryData::data[i][4]  // pitch
              << "," << TrajectoryData::data[i][5]; // yaw
      
      // Next state
      for (int j = 0; j < 10; j++) {
        outfile << "," << next_states[i][j];
      }
      
      // Control inputs
      for (int j = 0; j < 3; j++) {
        outfile << "," << controls[i][j];
      }
      
      // Calculate errors
      Vector3f pos_error;
      pos_error << TrajectoryData::data[i][0] - next_states[i][0],
                   TrajectoryData::data[i][1] - next_states[i][1],
                   TrajectoryData::data[i][2] - next_states[i][2];
      
      Vector3f orient_error;
      orient_error << TrajectoryData::data[i][3] - next_states[i][6],
                      TrajectoryData::data[i][4] - next_states[i][7],
                      TrajectoryData::data[i][5] - next_states[i][8];
      
      outfile << "," << pos_error.norm() << "," << orient_error.norm();
      
      outfile << endl;
    }
    
    outfile.close();
    ENTO_DEBUG("\nWrote results to trajectory_results.csv");
  }
  
  // Calculate tracking error
  float total_pos_error = 0.0f;
  float total_orient_error = 0.0f;
  
  for (size_t i = 0; i < states.size(); i++) {
    // Calculate position error
    Vector3f pos_error;
    pos_error << TrajectoryData::data[i][0] - next_states[i][0],
                 TrajectoryData::data[i][1] - next_states[i][1],
                 TrajectoryData::data[i][2] - next_states[i][2];
    
    // Calculate orientation error
    Vector3f orient_error;
    orient_error << TrajectoryData::data[i][3] - next_states[i][6],
                    TrajectoryData::data[i][4] - next_states[i][7],
                    TrajectoryData::data[i][5] - next_states[i][8];
    
    total_pos_error += pos_error.norm();
    total_orient_error += orient_error.norm();
  }
  
  float avg_pos_error = total_pos_error / states.size();
  float avg_orient_error = total_orient_error / states.size();
  
  ENTO_DEBUG("\nSummary Statistics:");
  ENTO_DEBUG("  Average position error: %f", avg_pos_error);
  ENTO_DEBUG("  Average orientation error: %f", avg_orient_error);
  ENTO_DEBUG("  Total iterations: %zu", states.size());
  
  // Test passes if tracking error is reasonable
  ENTO_TEST_CHECK_TRUE(avg_pos_error < 0.1f);
  ENTO_TEST_CHECK_TRUE(avg_orient_error < 0.1f);
}

// Test with multi-step trajectory tracking - Modified to avoid accessing private members
void test_adaptive_control_problem_trajectory() {
  ENTO_DEBUG("\n\nTesting AdaptiveControlProblem with real trajectory...");
  
  // Create problem instance with default time step
  AdaptiveControlProblem<float, 10, 3, 20, 100> problem;
  
  ENTO_DEBUG("Problem time step: %f", problem.get_dt());
  
  // Add reference trajectory points
  for (const auto& point : TrajectoryData::data) {
    // Convert to full state
    Matrix<float, 10, 1> state = Matrix<float, 10, 1>::Zero();
    
    // Position (x, y, z)
    state[0] = point[0]; // x
    state[1] = point[1]; // y
    state[2] = point[2]; // z
    
    // Orientation (roll, pitch, yaw = alpha, beta, gamma)
    state[6] = point[3]; // roll (alpha)
    state[7] = point[4]; // pitch (beta)
    state[8] = point[5]; // yaw (gamma)
    
    // Serialize the state for the problem
    string serialized;
    for (int j = 0; j < 10; j++) {
      serialized += to_string(state[j]) + ",";
    }
    
    // Add to the problem
    bool result = problem.deserialize_impl(serialized.c_str());
    ENTO_TEST_CHECK_TRUE(result);
    
    ENTO_DEBUG("Added reference point: [%f, %f, %f, ..., %f, %f, %f]",
              state[0], state[1], state[2], state[6], state[7], state[8]);
  }
  
  // Solve the problem for each step in a loop
  for (size_t i = 0; i < TrajectoryData::data.size(); i++) {
    ENTO_DEBUG("\nSolving step %zu", i);
    problem.solve_impl();
    
    // Validate after each step to check incremental progress
    ENTO_DEBUG("Validating step %zu", i);
    bool valid = problem.validate_impl();
    ENTO_TEST_CHECK_TRUE(valid);
    
    // Since we can't access private members directly, we should check
    // if the problem stays valid across multiple solve calls
  }
  
  // Final validation
  ENTO_DEBUG("\nFinal validation of the complete trajectory...");
  ENTO_TEST_CHECK_TRUE(problem.validate_impl());
}

// Test a complete sequence tracking all reference trajectory points
void test_complete_trajectory_tracking() {
  cout << "\n\nTesting complete trajectory tracking sequence with extended data...\n";
  
  // Create controller with default time step
  AdaptiveController controller;
  float dt = controller.get_dt();
  cout << "Controller time step: " << dt << endl;
  
  // Initialize state with zeros (starting from origin)
  Matrix<float, 10, 1> current_state = Matrix<float, 10, 1>::Zero();
  
  // Set initial position to first trajectory point
  current_state[0] = TrajectoryData::extended_data[0][0]; // x
  current_state[1] = TrajectoryData::extended_data[0][1]; // y
  current_state[2] = TrajectoryData::extended_data[0][2]; // z
  current_state[6] = TrajectoryData::extended_data[0][3]; // roll
  current_state[7] = TrajectoryData::extended_data[0][4]; // pitch
  current_state[8] = TrajectoryData::extended_data[0][5]; // yaw
  
  cout << "Initial state: [" 
       << current_state[0] << ", " << current_state[1] << ", " << current_state[2] << ", ..., "
       << current_state[6] << ", " << current_state[7] << ", " << current_state[8] << "]\n";
  
  // Data collection
  vector<Matrix<float, 10, 1>> states;
  vector<Matrix<float, 10, 1>> references;
  vector<Matrix<float, 10, 1>> next_states;
  vector<Matrix<float, 3, 1>> controls;
  vector<float> position_errors;
  vector<float> orientation_errors;
  
  // Run the entire sequence
  for (size_t i = 0; i < TrajectoryData::extended_data.size(); i++) {
    cout << "\n----- Step " << i << " -----\n";
    
    // Create reference state
    Matrix<float, 10, 1> ref_state = Matrix<float, 10, 1>::Zero();
    ref_state[0] = TrajectoryData::extended_data[i][0]; // x
    ref_state[1] = TrajectoryData::extended_data[i][1]; // y
    ref_state[2] = TrajectoryData::extended_data[i][2]; // z
    ref_state[6] = TrajectoryData::extended_data[i][3]; // roll
    ref_state[7] = TrajectoryData::extended_data[i][4]; // pitch
    ref_state[8] = TrajectoryData::extended_data[i][5]; // yaw
    
    // Configure controller
    controller.set_x0(current_state);
    controller.set_x_ref(ref_state);
    
    // Run the controller
    controller.solve();
    
    // Get outputs
    auto u = controller.get_u0();
    auto next_state = controller.get_next_state();
    
    // Calculate errors
    Vector3f pos_error;
    pos_error << ref_state[0] - next_state[0],
                 ref_state[1] - next_state[1],
                 ref_state[2] - next_state[2];
    
    Vector3f orient_error;
    orient_error << ref_state[6] - next_state[6],
                    ref_state[7] - next_state[7],
                    ref_state[8] - next_state[8];
    
    float pos_error_norm = pos_error.norm();
    float orient_error_norm = orient_error.norm();
    
    // Store results
    states.push_back(current_state);
    references.push_back(ref_state);
    next_states.push_back(next_state);
    controls.push_back(u);
    position_errors.push_back(pos_error_norm);
    orientation_errors.push_back(orient_error_norm);
    
    cout << "Current: ["
         << current_state[0] << ", " << current_state[1] << ", " << current_state[2] << ", ..., "
         << current_state[6] << ", " << current_state[7] << ", " << current_state[8] << "]\n";
    
    cout << "Reference: ["
         << ref_state[0] << ", " << ref_state[1] << ", " << ref_state[2] << ", ..., "
         << ref_state[6] << ", " << ref_state[7] << ", " << ref_state[8] << "]\n";
    
    cout << "Next: ["
         << next_state[0] << ", " << next_state[1] << ", " << next_state[2] << ", ..., "
         << next_state[6] << ", " << next_state[7] << ", " << next_state[8] << "]\n";
    
    cout << "Control: [" << u[0] << ", " << u[1] << ", " << u[2] << "]\n";
    cout << "Errors: position=" << pos_error_norm << ", orientation=" << orient_error_norm << "\n";
    
    // Ensure we get valid outputs
    for (int j = 0; j < 10; j++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(next_state[j]));
    }
    
    // Update state for next iteration
    current_state = next_state;
  }
  
  // Write detailed results to CSV
  ofstream outfile("complete_trajectory_results.csv");
  if (outfile.is_open()) {
    // Write header
    outfile << "step,current_x,current_y,current_z,current_vx,current_vy,current_vz,current_roll,current_pitch,current_yaw,current_yaw_rate,"
            << "ref_x,ref_y,ref_z,ref_roll,ref_pitch,ref_yaw,"
            << "next_x,next_y,next_z,next_vx,next_vy,next_vz,next_roll,next_pitch,next_yaw,next_yaw_rate,"
            << "thrust,roll_torque,pitch_torque,position_error,orientation_error" << endl;
    
    // Write data
    for (size_t i = 0; i < states.size(); i++) {
      outfile << i;
      
      // Current state
      for (int j = 0; j < 10; j++) {
        outfile << "," << states[i][j];
      }
      
      // Reference state
      outfile << "," << references[i][0] << "," << references[i][1] << "," << references[i][2]
              << "," << references[i][6] << "," << references[i][7] << "," << references[i][8];
      
      // Next state
      for (int j = 0; j < 10; j++) {
        outfile << "," << next_states[i][j];
      }
      
      // Control inputs
      outfile << "," << controls[i][0] << "," << controls[i][1] << "," << controls[i][2];
      
      // Errors
      outfile << "," << position_errors[i] << "," << orientation_errors[i];
      
      outfile << endl;
    }
    
    outfile.close();
    cout << "\nWrote results to complete_trajectory_results.csv\n";
  }
  
  // Calculate overall statistics
  float total_pos_error = 0.0f;
  float total_orient_error = 0.0f;
  float max_pos_error = 0.0f;
  float max_orient_error = 0.0f;
  
  for (size_t i = 0; i < position_errors.size(); i++) {
    total_pos_error += position_errors[i];
    total_orient_error += orientation_errors[i];
    max_pos_error = std::max(max_pos_error, position_errors[i]);
    max_orient_error = std::max(max_orient_error, orientation_errors[i]);
  }
  
  float avg_pos_error = total_pos_error / position_errors.size();
  float avg_orient_error = total_orient_error / orientation_errors.size();
  
  cout << "\nComplete Trajectory Summary:\n";
  cout << "  Tracking points: " << TrajectoryData::extended_data.size() << "\n";
  cout << "  Average position error: " << avg_pos_error << "\n";
  cout << "  Maximum position error: " << max_pos_error << "\n";
  cout << "  Average orientation error: " << avg_orient_error << "\n";
  cout << "  Maximum orientation error: " << max_orient_error << "\n";
  
  // Test passes if errors are within reasonable bounds
  ENTO_TEST_CHECK_TRUE(avg_pos_error < 0.1f);
  ENTO_TEST_CHECK_TRUE(avg_orient_error < 0.1f);
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
    __ento_replace_file_suffix(__FILE__, "test_adaptive_controller_trajectory_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_TEST_START();
  
  if (__ento_test_num(__n, 1)) test_adaptive_controller_real_trajectory();
  if (__ento_test_num(__n, 2)) test_adaptive_control_problem_trajectory();
  if (__ento_test_num(__n, 3)) test_complete_trajectory_tracking();
  
  ENTO_TEST_END();
} 