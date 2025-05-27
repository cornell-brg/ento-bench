#ifndef ENTO_CONTROL_OSQP_H
#define ENTO_CONTROL_OSQP_H

#include <ento-control/osqp_generic_wrapper.h>

// Include the generated API headers
#include "generated/robofly_mpc/robofly_api.h"
#include "generated/robobee_mpc/robobee_api.h"

namespace EntoControl {

// RoboFly MPC Solver using generated OSQP code (no traits needed)
using RoboFlyMPCSolver = OSQPSolver<RoboFlyAPI>;
using RoboFlyMPCSolverWarmStart = OSQPSolver<RoboFlyAPI, EmptyTraits, true>;

// RoboBee MPC Solver using generated OSQP code (no traits needed)
using RoboBeeMPCSolver = OSQPSolver<RoboBeeAPI>;
using RoboBeeMPCSolverWarmStart = OSQPSolver<RoboBeeAPI, EmptyTraits, true>;

// Default MPC solver (for backward compatibility with tests)
using MpcSolver = RoboFlyMPCSolver;

} // namespace EntoControl

#endif // ENTO_CONTROL_OSQP_H
