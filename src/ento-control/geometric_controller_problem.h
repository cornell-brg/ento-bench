#ifndef GEOMETRIC_CONTROLLER_PROBLEM_H
#define GEOMETRIC_CONTROLLER_PROBLEM_H

#include "control_problem.h"
#include "geometric_controller_solver.h"
#include <ento-util/debug.h>

namespace EntoControl {

/**
 * @brief Geometric Controller Problem for benchmarking
 * 
 * This class provides a complete benchmarking problem for the geometric controller
 * that can be used with the ento-bench framework. It uses ReferenceControlProblem as
 * the base since geometric controllers don't have reliable forward dynamics models.
 * Instead, it uses reference trajectories and benchmarks computational performance.
 * 
 * Template parameters:
 * - Scalar: float or double precision
 * - VehicleTraits: QuadrotorTraits or RoboBeeTraits
 * - UseDecoupledYaw: Enable decoupled yaw control
 * - UseIntegralControl: Enable integral control terms
 * - HorizonSize: Planning horizon (set to 1 for geometric controller)
 * - PathLen: Maximum trajectory length
 */
template <
  typename Scalar = float,
  typename VehicleTraits = QuadrotorTraits<Scalar>,
  bool UseDecoupledYaw = true,
  bool UseIntegralControl = true,
  int HorizonSize = 1,  // Geometric controller only needs current state
  int PathLen = 2000
>
class GeometricControllerProblem : 
  public ReferenceControlProblem<
    Scalar, 
    GeometricControllerSolver<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>,
    13,  // StateSize: [x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz]
    4,   // ControlSize: [thrust, Mx, My, Mz]
    HorizonSize,
    PathLen
  >
{
public:
  using Base = ReferenceControlProblem<
    Scalar, 
    GeometricControllerSolver<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>,
    13, 4, HorizonSize, PathLen
  >;
  
  using Solver = GeometricControllerSolver<Scalar, VehicleTraits, UseDecoupledYaw, UseIntegralControl>;
  using Controller = typename Solver::Controller;
  using Gains = typename Controller::Gains;
  
  static constexpr int StateSize = 13;
  static constexpr int ControlSize = 4;
  
  /**
   * @brief Constructor with timestep
   */
  explicit GeometricControllerProblem(Scalar dt = Scalar(0.01)) 
    : Base(dt)
  {
    // Set default FDCL gains
    set_default_gains();
    
#ifdef NATIVE
    ENTO_DEBUG("GeometricControllerProblem initialized:");
    ENTO_DEBUG("  Vehicle: %s", 
               std::is_same_v<VehicleTraits, QuadrotorTraits<Scalar>> ? "Quadrotor" : "RoboBee");
    ENTO_DEBUG("  Precision: %s", std::is_same_v<Scalar, float> ? "float" : "double");
    ENTO_DEBUG("  Decoupled Yaw: %s", UseDecoupledYaw ? "true" : "false");
    ENTO_DEBUG("  Integral Control: %s", UseIntegralControl ? "true" : "false");
    ENTO_DEBUG("  Time step: %.4f s", dt);
    ENTO_DEBUG("  State size: %d, Control size: %d", StateSize, ControlSize);
    ENTO_DEBUG("  Using ReferenceControlProblem (no forward dynamics)");
#endif
  }
  
  /**
   * @brief Set control gains
   */
  void set_gains(const Gains& gains) {
    this->get_solver().set_gains(gains);
    
#ifdef NATIVE
    ENTO_DEBUG("Updated geometric controller gains:");
    ENTO_DEBUG("  kX = [%.2f, %.2f, %.2f]", gains.kX(0,0), gains.kX(1,1), gains.kX(2,2));
    ENTO_DEBUG("  kV = [%.2f, %.2f, %.2f]", gains.kV(0,0), gains.kV(1,1), gains.kV(2,2));
    ENTO_DEBUG("  kR = [%.3f, %.3f, %.3f]", gains.kR(0,0), gains.kR(1,1), gains.kR(2,2));
    ENTO_DEBUG("  kW = [%.3f, %.3f, %.3f]", gains.kW(0,0), gains.kW(1,1), gains.kW(2,2));
    if constexpr (UseIntegralControl) {
      ENTO_DEBUG("  kIX = %.3f, kI = %.3f, kyI = %.3f", gains.kIX, gains.kI, gains.kyI);
    }
#endif
  }
  
  /**
   * @brief Get current control gains
   */
  const Gains& get_gains() const {
    return this->get_solver().get_gains();
  }
  
  /**
   * @brief Reset integral errors (useful between runs)
   */
  void reset_integral_errors() {
    this->get_solver().reset_integral_errors();
    
#ifdef NATIVE
    ENTO_DEBUG("Reset geometric controller integral errors");
#endif
  }
  
  /**
   * @brief Set conservative gains for stable operation
   */
  void set_conservative_gains() {
    Gains gains;
    gains.set_kX(Scalar(8.0), Scalar(8.0), Scalar(8.0));
    gains.set_kV(Scalar(6.0), Scalar(6.0), Scalar(6.0));
    gains.set_kR(Scalar(0.8), Scalar(0.8), Scalar(0.3));
    gains.set_kW(Scalar(0.2), Scalar(0.2), Scalar(0.05));
    
    if constexpr (UseIntegralControl) {
      gains.kIX = Scalar(2.0);
      gains.ki = Scalar(0.005);
      gains.kIR = Scalar(0.01);
      gains.kI = Scalar(0.005);
      gains.kyI = Scalar(0.01);
      gains.c1 = Scalar(1.0);
      gains.c2 = Scalar(1.0);
      gains.c3 = Scalar(1.0);
    }
    
    set_gains(gains);
    
#ifdef NATIVE
    ENTO_DEBUG("Set conservative gains for stable operation");
#endif
  }
  
  /**
   * @brief Set aggressive gains for high performance
   */
  void set_aggressive_gains() {
    Gains gains;
    gains.set_kX(Scalar(25.0), Scalar(25.0), Scalar(25.0));
    gains.set_kV(Scalar(20.0), Scalar(20.0), Scalar(20.0));
    gains.set_kR(Scalar(2.5), Scalar(2.5), Scalar(1.0));
    gains.set_kW(Scalar(0.6), Scalar(0.6), Scalar(0.15));
    
    if constexpr (UseIntegralControl) {
      gains.kIX = Scalar(6.0);
      gains.ki = Scalar(0.02);
      gains.kIR = Scalar(0.025);
      gains.kI = Scalar(0.02);
      gains.kyI = Scalar(0.03);
      gains.c1 = Scalar(1.0);
      gains.c2 = Scalar(1.0);
      gains.c3 = Scalar(1.0);
    }
    
    set_gains(gains);
    
#ifdef NATIVE
    ENTO_DEBUG("Set aggressive gains for high performance");
#endif
  }
  
private:
  /**
   * @brief Set default FDCL gains
   */
  void set_default_gains() {
    Gains gains;
    
    // Python simulation gains (for 1.95kg quadrotor)
    gains.set_kX(Scalar(8.0), Scalar(8.0), Scalar(8.0));
    gains.set_kV(Scalar(4.0), Scalar(4.0), Scalar(4.0));
    gains.set_kR(Scalar(0.5), Scalar(0.5), Scalar(0.5));
    gains.set_kW(Scalar(0.1), Scalar(0.1), Scalar(0.1));
    
    if constexpr (UseIntegralControl) {
      gains.kIX = Scalar(4.0);
      gains.ki = Scalar(0.01);
      gains.kIR = Scalar(0.015);
      gains.kI = Scalar(0.01);
      gains.kyI = Scalar(0.02);
      gains.c1 = Scalar(1.0);
      gains.c2 = Scalar(1.0);
      gains.c3 = Scalar(1.0);
    }
    
    set_gains(gains);
  }
};

// ============================================================================
// COMMON TYPE ALIASES FOR BENCHMARKING
// ============================================================================

// Float precision variants
using GeometricControllerProblemFloat = GeometricControllerProblem<float, QuadrotorTraits<float>, true, true>;
using GeometricControllerProblemFloatNoIntegral = GeometricControllerProblem<float, QuadrotorTraits<float>, true, false>;
using GeometricControllerProblemFloatCoupled = GeometricControllerProblem<float, QuadrotorTraits<float>, false, true>;

// Double precision variants  
using GeometricControllerProblemDouble = GeometricControllerProblem<double, QuadrotorTraits<double>, true, true>;
using GeometricControllerProblemDoubleNoIntegral = GeometricControllerProblem<double, QuadrotorTraits<double>, true, false>;
using GeometricControllerProblemDoubleCoupled = GeometricControllerProblem<double, QuadrotorTraits<double>, false, true>;

// RoboBee variants
using GeometricControllerProblemRoboBee = GeometricControllerProblem<float, RoboBeeTraits<float>, true, true>;
using GeometricControllerProblemRoboBeeDouble = GeometricControllerProblem<double, RoboBeeTraits<double>, true, true>;

// Large trajectory variants (for long simulations)
template<typename Scalar = float>
using GeometricControllerProblemLarge = GeometricControllerProblem<Scalar, QuadrotorTraits<Scalar>, true, true, 10, 5000>;

} // namespace EntoControl

#endif // GEOMETRIC_CONTROLLER_PROBLEM_H 