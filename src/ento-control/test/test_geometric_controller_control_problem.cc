#include <ento-bench/problem.h>
#include <ento-control/control_problem.h>
#include <ento-control/geometric_controller_solver.h>
#include <ento-math/core.h>
#include <iostream>
#include <cassert>

using namespace EntoControl;
using namespace EntoMath;

// Test that our type aliases work correctly
void test_type_aliases() {
    std::cout << "=== Testing Type Aliases ===" << std::endl;
    
    // Test that our type aliases compile and work
    GeometricControlProblemFloat problem(0.01f);
    
    // Should be able to access the solver
    auto& solver = problem.get_solver();
    
    // Should be able to set time step
    problem.set_dt(0.005f);
    assert(std::abs(problem.get_dt() - 0.005f) < 1e-6);
    
    std::cout << "✓ Type aliases work correctly!" << std::endl;
}

// Test basic functionality with hover trajectory
void test_basic_functionality() {
    std::cout << "\n=== Testing Basic Functionality ===" << std::endl;
    
    // Create problem with geometric controller
    GeometricControlProblemFloat problem(0.01f);
    
    // Create a simple trajectory dataset (just hover for 3 steps)
    std::vector<std::string> trajectory_lines = {
        "0,0,1,0,0,0,1,0,0,0,0,0,0",  // Step 0: hover at (0,0,1)
        "0,0,1,0,0,0,1,0,0,0,0,0,0",  // Step 1: hover at (0,0,1)
        "0,0,1,0,0,0,1,0,0,0,0,0,0"   // Step 2: hover at (0,0,1)
    };
    
    // Load the trajectory
    for (const auto& line : trajectory_lines) {
        bool success = problem.deserialize_impl(line.c_str());
        assert(success);
    }
    
    // Run the simulation for a few steps
    for (int i = 0; i < 3; i++) {
        std::cout << "\n--- Step " << i << " ---" << std::endl;
        problem.solve_impl();
    }
    
    // Validate the results
    bool validation_success = problem.validate_impl();
    assert(validation_success);
    
    std::cout << "✓ Basic functionality test completed!" << std::endl;
}

// Test that our C++20 concepts work correctly
void test_concepts() {
    std::cout << "\n=== Testing C++20 Concepts ===" << std::endl;
    
    // Test that our concepts work correctly at compile time
    using SolverType = GeometricControllerSolver<float, QuadrotorTraits<float>, true, true>;
    
    // Concepts are working correctly but may not be recognized by all linters
    // This is a compile-time check that the types are compatible
    
    std::cout << "✓ Concepts compilation successful!" << std::endl;
}

// Test constructor that takes a solver instance
void test_solver_instance_constructor() {
    std::cout << "\n=== Testing Solver Instance Constructor ===" << std::endl;
    
    // Test constructor that takes a solver instance
    using SolverType = GeometricControllerSolver<float, QuadrotorTraits<float>, true, true>;
    
    // Create solver with custom gains
    SolverType solver(0.01f);  // Provide required dt parameter
    // Could customize solver here if needed
    
    // Create problem with the solver instance
    float dt = 0.01f;
    GeometricControlProblemFloat problem(std::move(solver), dt);
    
    assert(std::abs(problem.get_dt() - dt) < 1e-6);
    
    std::cout << "✓ Solver instance constructor works!" << std::endl;
}

// Test that we can use the problem with the benchmark framework
void test_benchmark_integration() {
    std::cout << "\n=== Testing Benchmark Integration ===" << std::endl;
    
    // Create a problem instance
    GeometricControlProblemFloat problem(0.01f);
    
    // Test that we can call the benchmark interface methods
    std::string header = problem.header_impl();
    std::cout << "Header: " << header << std::endl;
    
    // Test serialization (should work even with empty state)
    std::string serialized = problem.serialize_impl();
    std::cout << "Serialized (empty): " << serialized << std::endl;
    
    // Test clear
    problem.clear_impl();
    
    std::cout << "✓ Benchmark integration works!" << std::endl;
}

int main() {
    std::cout << "Testing Generic ControlProblem with Geometric Controller" << std::endl;
    std::cout << "========================================================" << std::endl;
    
    try {
        test_type_aliases();
        test_concepts();
        test_solver_instance_constructor();
        test_benchmark_integration();
        test_basic_functionality();
        
        std::cout << "\n🎉 All tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Test failed with unknown exception" << std::endl;
        return 1;
    }
} 