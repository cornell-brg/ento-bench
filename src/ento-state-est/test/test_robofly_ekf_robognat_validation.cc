#include <ento-util/debug.h>
#include <ento-util/unittest.h>
#include <ento-state-est/EKFProblem.h>
#include <ento-state-est/ekf_kernels.h>
#include <ento-util/file_path_util.h>
#include <fstream>
#include <sstream>
#include <cstring>

using Scalar = double;
constexpr Scalar TOLERANCE = 1e-6;

using namespace EntoUtil;
using namespace EntoStateEst;

//------------------------------------------------------------------------------
// Test RoboFly EKF with Robognat Validation Dataset
//------------------------------------------------------------------------------
void test_robofly_ekf_robognat_validation()
{
  ENTO_DEBUG("Running test_robofly_ekf_robognat_validation...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Initialize EKF with realistic noise matrices for RoboFly
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity();
  Q(0,0) = 0.001; // theta process noise
  Q(1,1) = 0.001; // vx process noise  
  Q(2,2) = 0.001; // z process noise
  Q(3,3) = 0.001; // vz process noise
  
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001*0.001; // range measurement noise (1mm std)
  R(1,1) = 0.05*0.05;   // optical flow noise (0.05 rad/s std)
  R(2,2) = 0.1*0.1;     // accel x noise (0.1 m/s^2 std)
  R(3,3) = 0.1*0.1;     // accel z noise (0.1 m/s^2 std)
  
  ENTO_DEBUG("=== NOISE MATRICES ===");
  ENTO_DEBUG("Process noise Q diagonal: [%.6f, %.6f, %.6f, %.6f]", Q(0,0), Q(1,1), Q(2,2), Q(3,3));
  ENTO_DEBUG("Measurement noise R diagonal: [%.6f, %.6f, %.6f, %.6f]", R(0,0), R(1,1), R(2,2), R(3,3));
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 2500> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Set initial conditions for RoboFly EKF
  Eigen::Matrix<Scalar, 4, 1> x0;
  x0 << 0.0,   // theta = 0 (level attitude)
        0.0,   // vx = 0 (no initial velocity)
        0.12,  // z = 0.12m (reasonable hover height from our data)
        0.0;   // vz = 0 (no initial vertical velocity)
  
  Eigen::Matrix<Scalar, 4, 4> P0 = Eigen::Matrix<Scalar, 4, 4>::Identity();
  P0(0,0) = 0.1*0.1;   // theta uncertainty (0.1 rad std)
  P0(1,1) = 0.1*0.1;   // vx uncertainty (0.1 m/s std)  
  P0(2,2) = 0.01*0.01; // z uncertainty (1cm std)
  P0(3,3) = 0.1*0.1;   // vz uncertainty (0.1 m/s std)
  
  ENTO_DEBUG("=== INITIAL CONDITIONS ===");
  ENTO_DEBUG("Initial state x0: [%.6f, %.6f, %.6f, %.6f]", x0(0), x0(1), x0(2), x0(3));
  ENTO_DEBUG("Initial covariance P0 diagonal: [%.6f, %.6f, %.6f, %.6f]", P0(0,0), P0(1,1), P0(2,2), P0(3,3));
  
  // Check if initial conditions are finite
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(x0(i))) {
      ENTO_DEBUG("ERROR: Initial state x0(%d) = %.6f is not finite!", i, x0(i));
    }
    if (!std::isfinite(P0(i,i))) {
      ENTO_DEBUG("ERROR: Initial covariance P0(%d,%d) = %.6f is not finite!", i, i, P0(i,i));
    }
  }
  
  problem.setInitialConditions(x0, P0);
  
  // Load the robognat validation dataset
  std::string dataset_path = "../../datasets/state-est/robofly_robognat_validation.csv";
  std::ifstream file(dataset_path);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Could not open dataset file: %s", dataset_path.c_str());
    ENTO_TEST_CHECK_TRUE(false);
    return;
  }
  
  ENTO_DEBUG("Loading RoboFly robognat validation dataset from: %s", dataset_path.c_str());
  
  std::string line;
  int line_count = 0;
  int parsed_count = 0;
  int max_lines_to_process = 10; // Process only first 10 lines for detailed debugging
  
  // Skip header lines - both # comments and quoted comments
  while (std::getline(file, line) && 
         (line.length() > 0 && (line[0] == '#' || (line.length() > 1 && line[0] == '"' && line[1] == '#')))) {
    ENTO_DEBUG("Skipping header: %s", line.c_str());
  }
  
  while (std::getline(file, line) && line_count < max_lines_to_process) {
    line_count++;
    
    if (line.empty()) continue;
    
    ENTO_DEBUG("=== PROCESSING LINE %d ===", line_count);
    ENTO_DEBUG("CSV line: %s", line.c_str());
    
    // Parse the line manually to check data values
    std::istringstream ss(line);
    std::string token;
    std::vector<double> values;
    
    while (std::getline(ss, token, ',')) {
      values.push_back(std::stod(token));
    }
    
    if (values.size() >= 11) {
      ENTO_DEBUG("Parsed values: timestamp=%.6f, dt=%.6f", values[0], values[1]);
      ENTO_DEBUG("  measurements: [%.6f, %.6f, %.6f, %.6f]", values[2], values[3], values[4], values[5]);
      ENTO_DEBUG("  control: %.6f", values[6]);
      ENTO_DEBUG("  sensor_mask: [%d, %d, %d, %d]", (int)values[7], (int)values[8], (int)values[9], (int)values[10]);
      
      // Check for any non-finite input values
      for (size_t i = 0; i < values.size(); i++) {
        if (!std::isfinite(values[i])) {
          ENTO_DEBUG("ERROR: Non-finite input value at index %zu: %.6f", i, values[i]);
        }
      }
    }
    
    // Get trajectory state before processing this line
    auto& trajectory = problem.getStateTrajectory();
    if (!trajectory.empty()) {
      auto current_state = trajectory[trajectory.size() - 1];
      ENTO_DEBUG("State BEFORE processing line %d: [%.6f, %.6f, %.6f, %.6f]", 
                 line_count, current_state(0), current_state(1), current_state(2), current_state(3));
      
      // Check if current state has any NaN values
      for (int i = 0; i < 4; i++) {
        if (!std::isfinite(current_state(i))) {
          ENTO_DEBUG("ERROR: State already has NaN at index %d before processing line %d!", i, line_count);
        }
      }
    }
    
    bool result = problem.deserialize_impl(line.c_str());
    if (result) {
      parsed_count++;
      ENTO_DEBUG("✓ Line %d parsed successfully", line_count);
      
      // Get trajectory state after processing this line
      auto& trajectory_after = problem.getStateTrajectory();
      if (!trajectory_after.empty()) {
        auto new_state = trajectory_after[trajectory_after.size() - 1];
        ENTO_DEBUG("State AFTER processing line %d: [%.6f, %.6f, %.6f, %.6f]", 
                   line_count, new_state(0), new_state(1), new_state(2), new_state(3));
        
        // Check if processing this line introduced NaN values
        for (int i = 0; i < 4; i++) {
          if (!std::isfinite(new_state(i))) {
            ENTO_DEBUG("ERROR: Line %d processing introduced NaN at state index %d!", line_count, i);
            ENTO_DEBUG("This is the FIRST occurrence of NaN - stopping detailed analysis here.");
            break;
          }
        }
      }
    } else {
      ENTO_DEBUG("✗ Failed to parse line %d: %s", line_count, line.c_str());
    }
    
    ENTO_DEBUG(""); // Empty line for readability
  }
  
  file.close();
  
  ENTO_DEBUG("Dataset loaded: %d lines processed, %d successfully parsed", line_count, parsed_count);
  
  // Validate the problem has trajectory data
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);
  
  // Check we have a reasonable number of data points
  auto& trajectory = problem.getStateTrajectory();
  ENTO_DEBUG("Final trajectory size: %zu", trajectory.size());
  ENTO_TEST_CHECK_TRUE(trajectory.size() > 0);
  ENTO_TEST_CHECK_TRUE(trajectory.size() <= max_lines_to_process);
  
  // Check basic properties of final state
  if (!trajectory.empty()) {
    auto final_state = trajectory[trajectory.size() - 1];
    ENTO_DEBUG("Final state: [%.6f, %.6f, %.6f, %.6f]", 
               final_state(0), final_state(1), final_state(2), final_state(3));
    
    // Check states are reasonable (not NaN or inf)
    for (int i = 0; i < 4; i++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(final_state(i)));
    }
    
    // Check z position is positive (robot above ground)
    if (std::isfinite(final_state(2))) {
      ENTO_TEST_CHECK_TRUE(final_state(2) > 0.0);
    }
  }

  ENTO_DEBUG("test_robofly_ekf_robognat_validation COMPLETED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF Estimation Quality with Robognat Data
//------------------------------------------------------------------------------
void test_robofly_ekf_estimation_quality()
{
  ENTO_DEBUG("Running test_robofly_ekf_estimation_quality...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Use more conservative noise parameters for better tracking
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.0001;
  
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001*0.001; // range measurement noise
  R(1,1) = 0.05*0.05;   // optical flow noise  
  R(2,2) = 0.1*0.1;     // accel x noise
  R(3,3) = 0.1*0.1;     // accel z noise
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 500> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement));
  
  // Load subset of validation data
  std::string dataset_path = "../../datasets/state-est/robofly_robognat_validation.csv";
  std::ifstream file(dataset_path);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Could not open dataset file: %s", dataset_path.c_str());
    ENTO_TEST_CHECK_TRUE(false);
    return;
  }
  
  std::string line;
  int line_count = 0;
  
  // Skip header lines - both # comments and quoted comments
  while (std::getline(file, line) && 
         (line.length() > 0 && (line[0] == '#' || (line.length() > 1 && line[0] == '"' && line[1] == '#')))) {
    ENTO_DEBUG("Skipping header: %s", line.c_str());
  }
  
  // Process first 400 lines for stability analysis
  while (std::getline(file, line) && line_count < 400) {
    line_count++;
    
    if (line.empty()) continue;
    
    bool result = problem.deserialize_impl(line.c_str());
    if (!result) {
      ENTO_DEBUG("Failed to parse line %d", line_count);
    }
  }
  
  file.close();
  
  // Validate estimation quality
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);
  
  auto& trajectory = problem.getStateTrajectory();
  ENTO_DEBUG("Processed %d lines, trajectory size: %zu", line_count, trajectory.size());
  
  if (!trajectory.empty()) {
    // Check estimation stability (no divergence)
    auto initial_state = trajectory[0]; // Use index 0 instead of front()
    auto final_state = trajectory[trajectory.size() - 1]; // Use back() equivalent
    
    ENTO_DEBUG("Initial state: [%.4f, %.4f, %.4f, %.4f]", 
               initial_state(0), initial_state(1), initial_state(2), initial_state(3));
    ENTO_DEBUG("Final state: [%.4f, %.4f, %.4f, %.4f]", 
               final_state(0), final_state(1), final_state(2), final_state(3));
    
    // Check for reasonable state bounds (no extreme divergence)
    ENTO_TEST_CHECK_TRUE(std::abs(final_state(0)) < 1.0);  // |theta| < 1 rad
    ENTO_TEST_CHECK_TRUE(std::abs(final_state(1)) < 2.0);  // |vx| < 2 m/s  
    ENTO_TEST_CHECK_TRUE(final_state(2) > 0.05 && final_state(2) < 0.3); // z in [0.05, 0.3] m
    ENTO_TEST_CHECK_TRUE(std::abs(final_state(3)) < 2.0);  // |vz| < 2 m/s
  }

  ENTO_DEBUG("test_robofly_ekf_estimation_quality PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF with Terrain Crossing Synchronous Data
//------------------------------------------------------------------------------
void test_robofly_ekf_terrain_crossing_sync()
{
  ENTO_DEBUG("Running test_robofly_ekf_terrain_crossing_sync...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Initialize EKF with realistic noise matrices for RoboFly
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity();
  Q(0,0) = 0.0001; // theta process noise
  Q(1,1) = 0.0001; // vx process noise  
  Q(2,2) = 0.0001; // z process noise
  Q(3,3) = 0.0001; // vz process noise
  
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001*0.001; // range measurement noise (1mm std)
  R(1,1) = 0.05*0.05;   // optical flow noise (0.05 rad/s std)
  R(2,2) = 0.1*0.1;     // accel x noise (0.1 m/s^2 std)
  R(3,3) = 0.1*0.1;     // accel z noise (0.1 m/s^2 std)
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  // Use SYNCHRONOUS update method for this test
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 1000> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement), 
    EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 1000>::UpdateMethod::SYNCHRONOUS);
  
  // Set initial conditions for terrain crossing trajectory
  Eigen::Matrix<Scalar, 4, 1> x0;
  x0 << 0.0,   // theta = 0 (level attitude)
        0.0,   // vx = 0 (no initial velocity)
        0.15,  // z = 0.15m (hover height for terrain crossing)
        0.0;   // vz = 0 (no initial vertical velocity)
  
  Eigen::Matrix<Scalar, 4, 4> P0 = Eigen::Matrix<Scalar, 4, 4>::Identity();
  P0(0,0) = 0.05*0.05; // theta uncertainty (0.05 rad std)
  P0(1,1) = 0.1*0.1;   // vx uncertainty (0.1 m/s std)  
  P0(2,2) = 0.01*0.01; // z uncertainty (1cm std)
  P0(3,3) = 0.1*0.1;   // vz uncertainty (0.1 m/s std)
  
  problem.setInitialConditions(x0, P0);
  
  // Load the terrain crossing synchronous dataset
  std::string dataset_path = "../../datasets/state-est/terrain_crossing_flat_sync.csv";
  std::ifstream file(dataset_path);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Could not open terrain crossing sync dataset: %s", dataset_path.c_str());
    ENTO_TEST_CHECK_TRUE(false);
    return;
  }
  
  ENTO_DEBUG("Loading terrain crossing synchronous dataset from: %s", dataset_path.c_str());
  
  std::string line;
  int line_count = 0;
  int parsed_count = 0;
  
  // Skip header lines - both # comments and quoted comments
  while (std::getline(file, line) && 
         (line.length() > 0 && (line[0] == '#' || (line.length() > 1 && line[0] == '"' && line[1] == '#')))) {
    ENTO_DEBUG("Skipping header: %s", line.c_str());
  }
  
  // Process data lines (limit to 200 for manageable testing)
  do {
    if (line.empty()) continue;
    line_count++;
    
    bool result = problem.deserialize_impl(line.c_str());
    if (result) {
      parsed_count++;
    } else {
      ENTO_DEBUG("Failed to parse line %d: %s", line_count, line.c_str());
    }
    
    if (line_count >= 200) break; // Limit for testing
    
  } while (std::getline(file, line));
  
  file.close();
  
  ENTO_DEBUG("Terrain crossing sync dataset: %d lines processed, %d successfully parsed", line_count, parsed_count);
  
  // Validate the problem
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);
  
  // Check trajectory properties
  auto& trajectory = problem.getStateTrajectory();
  ENTO_DEBUG("Terrain crossing sync trajectory size: %zu", trajectory.size());
  ENTO_TEST_CHECK_TRUE(trajectory.size() > 50); // Should have processed significant data
  
  if (!trajectory.empty()) {
    auto final_state = trajectory[trajectory.size() - 1];
    ENTO_DEBUG("Final state: [%.6f, %.6f, %.6f, %.6f]", 
               final_state(0), final_state(1), final_state(2), final_state(3));
    
    // Check states are finite and reasonable
    for (int i = 0; i < 4; i++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(final_state(i)));
    }
    
    // For terrain crossing: height should be positive and reasonable
    ENTO_TEST_CHECK_TRUE(final_state(2) > 0.05 && final_state(2) < 0.3);
    
    // Attitude should be reasonable (not crazy diverged)
    ENTO_TEST_CHECK_TRUE(std::abs(final_state(0)) < 0.5); // |theta| < 0.5 rad
  }

  ENTO_DEBUG("test_robofly_ekf_terrain_crossing_sync PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF with Asynchronous Sensors - Sequential Update Method
//------------------------------------------------------------------------------
void test_robofly_ekf_async_sequential()
{
  ENTO_DEBUG("Running test_robofly_ekf_async_sequential...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Initialize EKF with slightly more conservative noise for async sensors
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.0005;
  
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001*0.001; // range measurement noise
  R(1,1) = 0.05*0.05;   // optical flow noise
  R(2,2) = 0.1*0.1;     // accel x noise
  R(3,3) = 0.1*0.1;     // accel z noise
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  // Use SEQUENTIAL update method for asynchronous sensors
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 800> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement), 
    EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 800>::UpdateMethod::SEQUENTIAL);
  
  // Set initial conditions
  Eigen::Matrix<Scalar, 4, 1> x0;
  x0 << 0.0, 0.0, 0.15, 0.0;
  
  Eigen::Matrix<Scalar, 4, 4> P0 = Eigen::Matrix<Scalar, 4, 4>::Identity();
  P0(0,0) = 0.05*0.05;
  P0(1,1) = 0.1*0.1;
  P0(2,2) = 0.01*0.01;
  P0(3,3) = 0.1*0.1;
  
  problem.setInitialConditions(x0, P0);
  
  // Load the asynchronous dataset
  std::string dataset_path = "../../datasets/state-est/terrain_crossing_flat_async.csv";
  std::ifstream file(dataset_path);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Could not open async dataset: %s", dataset_path.c_str());
    ENTO_TEST_CHECK_TRUE(false);
    return;
  }
  
  ENTO_DEBUG("Loading asynchronous dataset for SEQUENTIAL updates from: %s", dataset_path.c_str());
  
  std::string line;
  int line_count = 0;
  int parsed_count = 0;
  int sequential_updates = 0;
  
  // Skip header lines - both # comments and quoted comments
  while (std::getline(file, line) && 
         (line.length() > 0 && (line[0] == '#' || (line.length() > 1 && line[0] == '"' && line[1] == '#')))) {
    // Skip headers
  }
  
  // Process async data lines
  do {
    if (line.empty()) continue;
    line_count++;
    
    // Parse line manually to check sensor masks
    std::istringstream ss(line);
    std::string token;
    std::vector<double> values;
    
    while (std::getline(ss, token, ',')) {
      values.push_back(std::stod(token));
    }
    
    if (values.size() >= 11) {
      // Check if this line has partial sensor data (async behavior)
      bool has_partial_sensors = false;
      for (int i = 7; i < 11; i++) { // mask indices
        if (values[i] == 0) {
          has_partial_sensors = true;
          break;
        }
      }
      
      if (has_partial_sensors) {
        sequential_updates++;
        if (sequential_updates <= 5) {
          ENTO_DEBUG("Sequential update %d: masks=[%d,%d,%d,%d] at t=%.3f", 
                     sequential_updates, (int)values[7], (int)values[8], (int)values[9], (int)values[10], values[0]);
        }
      }
    }
    
    bool result = problem.deserialize_impl(line.c_str());
    if (result) {
      parsed_count++;
    }
    
    if (line_count >= 300) break; // Limit for testing
    
  } while (std::getline(file, line));
  
  file.close();
  
  ENTO_DEBUG("Async SEQUENTIAL: %d lines processed, %d parsed, %d sequential updates detected", 
             line_count, parsed_count, sequential_updates);
  
  // Validate estimation worked with sequential updates
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);
  
  auto& trajectory = problem.getStateTrajectory();
  ENTO_DEBUG("Sequential trajectory size: %zu", trajectory.size());
  ENTO_TEST_CHECK_TRUE(trajectory.size() > 50);
  ENTO_TEST_CHECK_TRUE(sequential_updates > 10); // Should have detected async behavior
  
  if (!trajectory.empty()) {
    auto final_state = trajectory[trajectory.size() - 1];
    ENTO_DEBUG("Sequential final state: [%.6f, %.6f, %.6f, %.6f]", 
               final_state(0), final_state(1), final_state(2), final_state(3));
    
    // Check convergence despite async sensors
    for (int i = 0; i < 4; i++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(final_state(i)));
    }
    
    ENTO_TEST_CHECK_TRUE(final_state(2) > 0.05 && final_state(2) < 0.3);
    ENTO_TEST_CHECK_TRUE(std::abs(final_state(0)) < 0.5);
  }

  ENTO_DEBUG("test_robofly_ekf_async_sequential PASSED!");
}

//------------------------------------------------------------------------------
// Test RoboFly EKF with Asynchronous Sensors - Truncated Update Method
//------------------------------------------------------------------------------
void test_robofly_ekf_async_truncated()
{
  ENTO_DEBUG("Running test_robofly_ekf_async_truncated...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Initialize EKF 
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.0005;
  
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001*0.001;
  R(1,1) = 0.05*0.05;
  R(2,2) = 0.1*0.1;
  R(3,3) = 0.1*0.1;
  
  RoboFlyKernel kernel(Q, R);
  DynamicsModel dynamics;
  MeasurementModel measurement;
  
  // Use TRUNCATED update method for asynchronous sensors
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 800> problem(
    std::move(kernel), std::move(dynamics), std::move(measurement), 
    EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 800>::UpdateMethod::TRUNCATED);
  
  // Set initial conditions
  Eigen::Matrix<Scalar, 4, 1> x0;
  x0 << 0.0, 0.0, 0.15, 0.0;
  
  Eigen::Matrix<Scalar, 4, 4> P0 = Eigen::Matrix<Scalar, 4, 4>::Identity();
  P0(0,0) = 0.05*0.05;
  P0(1,1) = 0.1*0.1;
  P0(2,2) = 0.01*0.01;
  P0(3,3) = 0.1*0.1;
  
  problem.setInitialConditions(x0, P0);
  
  // Load the same asynchronous dataset
  std::string dataset_path = "../../datasets/state-est/terrain_crossing_flat_async.csv";
  std::ifstream file(dataset_path);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Could not open async dataset for truncated test: %s", dataset_path.c_str());
    ENTO_TEST_CHECK_TRUE(false);
    return;
  }
  
  ENTO_DEBUG("Loading asynchronous dataset for TRUNCATED updates from: %s", dataset_path.c_str());
  
  std::string line;
  int line_count = 0;
  int parsed_count = 0;
  int truncated_updates = 0;
  
  // Skip header lines - both # comments and quoted comments
  while (std::getline(file, line) && 
         (line.length() > 0 && (line[0] == '#' || (line.length() > 1 && line[0] == '"' && line[1] == '#')))) {
    // Skip headers
  }
  
  // Process async data lines
  do {
    if (line.empty()) continue;
    line_count++;
    
    // Parse line manually to check sensor masks
    std::istringstream ss(line);
    std::string token;
    std::vector<double> values;
    
    while (std::getline(ss, token, ',')) {
      values.push_back(std::stod(token));
    }
    
    if (values.size() >= 11) {
      // Check if this line has partial sensor data (truncated behavior)
      bool has_partial_sensors = false;
      for (int i = 7; i < 11; i++) {
        if (values[i] == 0) {
          has_partial_sensors = true;
          break;
        }
      }
      
      if (has_partial_sensors) {
        truncated_updates++;
        if (truncated_updates <= 5) {
          ENTO_DEBUG("Truncated update %d: masks=[%d,%d,%d,%d] at t=%.3f", 
                     truncated_updates, (int)values[7], (int)values[8], (int)values[9], (int)values[10], values[0]);
        }
      }
    }
    
    bool result = problem.deserialize_impl(line.c_str());
    if (result) {
      parsed_count++;
    }
    
    if (line_count >= 300) break; // Limit for testing
    
  } while (std::getline(file, line));
  
  file.close();
  
  ENTO_DEBUG("Async TRUNCATED: %d lines processed, %d parsed, %d truncated updates detected", 
             line_count, parsed_count, truncated_updates);
  
  // Validate estimation worked with truncated updates
  bool validation_result = problem.validate_impl();
  ENTO_TEST_CHECK_TRUE(validation_result);
  
  auto& trajectory = problem.getStateTrajectory();
  ENTO_DEBUG("Truncated trajectory size: %zu", trajectory.size());
  ENTO_TEST_CHECK_TRUE(trajectory.size() > 50);
  ENTO_TEST_CHECK_TRUE(truncated_updates > 10); // Should have detected async behavior
  
  if (!trajectory.empty()) {
    auto final_state = trajectory[trajectory.size() - 1];
    ENTO_DEBUG("Truncated final state: [%.6f, %.6f, %.6f, %.6f]", 
               final_state(0), final_state(1), final_state(2), final_state(3));
    
    // Check convergence despite async sensors and truncation
    for (int i = 0; i < 4; i++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(final_state(i)));
    }
    
    ENTO_TEST_CHECK_TRUE(final_state(2) > 0.05 && final_state(2) < 0.3);
    ENTO_TEST_CHECK_TRUE(std::abs(final_state(0)) < 0.5);
  }

  ENTO_DEBUG("test_robofly_ekf_async_truncated PASSED!");
}

//------------------------------------------------------------------------------
// Test Comparison: Sequential vs Truncated Update Methods
//------------------------------------------------------------------------------
void test_robofly_ekf_sequential_vs_truncated_comparison()
{
  ENTO_DEBUG("Running test_robofly_ekf_sequential_vs_truncated_comparison...");

  using RoboFlyKernel = RoboFlyEKFKernel<Scalar>;
  using DynamicsModel = RoboflyDynamicsModel<Scalar>;
  using MeasurementModel = RoboflyMeasurementModel<Scalar>;
  
  // Same noise matrices for fair comparison
  Eigen::Matrix<Scalar, 4, 4> Q = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.0001;
  Eigen::Matrix<Scalar, 4, 4> R = Eigen::Matrix<Scalar, 4, 4>::Identity();
  R(0,0) = 0.001*0.001;
  R(1,1) = 0.05*0.05;
  R(2,2) = 0.1*0.1;
  R(3,3) = 0.1*0.1;
  
  // Sequential problem
  RoboFlyKernel kernel_seq(Q, R);
  DynamicsModel dynamics_seq;
  MeasurementModel measurement_seq;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 500> problem_seq(
    std::move(kernel_seq), std::move(dynamics_seq), std::move(measurement_seq), 
    EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 500>::UpdateMethod::SEQUENTIAL);
  
  // Truncated problem  
  RoboFlyKernel kernel_trunc(Q, R);
  DynamicsModel dynamics_trunc;
  MeasurementModel measurement_trunc;
  
  EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 500> problem_trunc(
    std::move(kernel_trunc), std::move(dynamics_trunc), std::move(measurement_trunc), 
    EKFProblem<RoboFlyKernel, DynamicsModel, MeasurementModel, 500>::UpdateMethod::TRUNCATED);
  
  // Same initial conditions for both
  Eigen::Matrix<Scalar, 4, 1> x0;
  x0 << 0.0, 0.0, 0.15, 0.0;
  Eigen::Matrix<Scalar, 4, 4> P0 = Eigen::Matrix<Scalar, 4, 4>::Identity() * 0.01;
  
  problem_seq.setInitialConditions(x0, P0);
  problem_trunc.setInitialConditions(x0, P0);
  
  // Load async dataset once and process with both methods
  std::string dataset_path = "../../datasets/state-est/terrain_crossing_flat_async.csv";
  std::ifstream file(dataset_path);
  
  if (!file.is_open()) {
    ENTO_DEBUG("Could not open async dataset for comparison: %s", dataset_path.c_str());
    ENTO_TEST_CHECK_TRUE(false);
    return;
  }
  
  std::vector<std::string> data_lines;
  std::string line;
  
  // Skip header lines - both # comments and quoted comments
  while (std::getline(file, line) && 
         (line.length() > 0 && (line[0] == '#' || (line.length() > 1 && line[0] == '"' && line[1] == '#')))) {
    ENTO_DEBUG("Skipping header: %s", line.c_str());
  }
  
  int line_count = 0;
  do {
    if (!line.empty()) {
      data_lines.push_back(line);
      line_count++;
    }
    if (line_count >= 150) break; // Limit for comparison
  } while (std::getline(file, line));
  
  file.close();
  
  ENTO_DEBUG("Comparison: processing %d lines with both methods", (int)data_lines.size());
  
  // Process with sequential method
  int seq_parsed = 0;
  for (const auto& data_line : data_lines) {
    if (problem_seq.deserialize_impl(data_line.c_str())) {
      seq_parsed++;
    }
  }
  
  // Process with truncated method
  int trunc_parsed = 0;
  for (const auto& data_line : data_lines) {
    if (problem_trunc.deserialize_impl(data_line.c_str())) {
      trunc_parsed++;
    }
  }
  
  ENTO_DEBUG("Sequential parsed: %d, Truncated parsed: %d", seq_parsed, trunc_parsed);
  
  // Both should parse successfully
  ENTO_TEST_CHECK_TRUE(seq_parsed > 100);
  ENTO_TEST_CHECK_TRUE(trunc_parsed > 100);
  
  // Compare final states
  auto& seq_trajectory = problem_seq.getStateTrajectory();
  auto& trunc_trajectory = problem_trunc.getStateTrajectory();
  
  ENTO_TEST_CHECK_TRUE(seq_trajectory.size() > 0);
  ENTO_TEST_CHECK_TRUE(trunc_trajectory.size() > 0);
  
  if (!seq_trajectory.empty() && !trunc_trajectory.empty()) {
    auto seq_final = seq_trajectory[seq_trajectory.size() - 1];
    auto trunc_final = trunc_trajectory[trunc_trajectory.size() - 1];
    
    ENTO_DEBUG("Sequential final: [%.4f, %.4f, %.4f, %.4f]", 
               seq_final(0), seq_final(1), seq_final(2), seq_final(3));
    ENTO_DEBUG("Truncated final: [%.4f, %.4f, %.4f, %.4f]", 
               trunc_final(0), trunc_final(1), trunc_final(2), trunc_final(3));
    
    // Both methods should converge to reasonable states (may differ slightly)
    for (int i = 0; i < 4; i++) {
      ENTO_TEST_CHECK_TRUE(std::isfinite(seq_final(i)));
      ENTO_TEST_CHECK_TRUE(std::isfinite(trunc_final(i)));
    }
    
    // Height estimates should be reasonably close (within 5cm)
    double height_diff = std::abs(seq_final(2) - trunc_final(2));
    ENTO_DEBUG("Height difference between methods: %.4f m", height_diff);
    ENTO_TEST_CHECK_TRUE(height_diff < 0.05); // Less than 5cm difference
  }

  ENTO_DEBUG("test_robofly_ekf_sequential_vs_truncated_comparison PASSED!");
}

//------------------------------------------------------------------------------
// Main Test Runner  
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  using namespace EntoUtil;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    __ento_replace_file_suffix(__FILE__, "test_harness_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  ENTO_DEBUG("N: %i", __n);
  ENTO_TEST_START();

  if (__ento_test_num(__n, 1)) test_robofly_ekf_robognat_validation();
  if (__ento_test_num(__n, 2)) test_robofly_ekf_estimation_quality();
  if (__ento_test_num(__n, 3)) test_robofly_ekf_terrain_crossing_sync();
  if (__ento_test_num(__n, 4)) test_robofly_ekf_async_sequential();
  if (__ento_test_num(__n, 5)) test_robofly_ekf_async_truncated();
  if (__ento_test_num(__n, 6)) test_robofly_ekf_sequential_vs_truncated_comparison();

  ENTO_TEST_END();
} 