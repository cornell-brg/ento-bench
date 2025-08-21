#include <ento-bench/harness.h>
#include <ento-bench/bench_config.h>
#include <Eigen/Dense>
#include <ento-state-est/EKFProblem.h>
#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>

namespace DummyEKF {

struct DynamicsModel { };
struct MeasurementModel { };

template <typename ScalarT>
struct Kernel {
  using Scalar_ = ScalarT;
  static constexpr size_t StateDim_ = 4;
  static constexpr size_t MeasurementDim_ = 4;
  static constexpr size_t ControlDim_ = 4;

  using StateVec   = Eigen::Matrix<ScalarT, StateDim_, 1>;
  using CovMatrix  = Eigen::Matrix<ScalarT, StateDim_, StateDim_>;

  Kernel() = default;
  Kernel(const Eigen::Matrix<ScalarT, StateDim_, StateDim_>&,
         const Eigen::Matrix<ScalarT, MeasurementDim_, MeasurementDim_>&) {}

  void setState(const StateVec&) {}
  void setCovariance(const CovMatrix&) {}
  const StateVec& getState() const { static StateVec s = StateVec::Zero(); return s; }
  const CovMatrix& getCovariance() const { static CovMatrix P = CovMatrix::Identity(); return P; }

  template<typename DynModel, typename ControlVec>
  void predict(const DynModel&, const ControlVec&) { __asm__ volatile("" ::: "memory"); }

  template<typename MeasModel, typename MeasVec>
  void update(const MeasModel&, const MeasVec&) { __asm__ volatile("" ::: "memory"); }

  template<typename MeasModel, typename MeasVec, typename ControlVec, typename MaskArray>
  void update_sequential(const MeasModel&, const MeasVec&, const ControlVec&, const MaskArray&) { __asm__ volatile("" ::: "memory"); }

  template<typename MeasModel, typename MeasVec, typename ControlVec, typename MaskArray>
  void update_truncated(const MeasModel&, const MeasVec&, const ControlVec&, const MaskArray&) { __asm__ volatile("" ::: "memory"); }
};

} // namespace DummyEKF

int main()
{
  using Scalar = float;
  using EKFKernel = DummyEKF::Kernel<Scalar>;
  using Dynamics = DummyEKF::DynamicsModel;
  using Measurement = DummyEKF::MeasurementModel;
  using Problem = EntoStateEst::EKFProblem<EKFKernel, Dynamics, Measurement, 1>;

  // Configure clock
  sys_clk_cfg();
  SysTick_Setup();
  __enable_irq();

  // Generic cache setup via config macro
  ENTO_BENCH_SETUP();
  // Print benchmark configuration
  ENTO_BENCH_PRINT_CONFIG();
  ENTO_BENCH_HARNESS_TYPE(Problem);
  Problem problem(EKFKernel{}, Dynamics{}, Measurement{}, Problem::UpdateMethod::SYNCHRONOUS);
  BenchHarness harness(problem, "Empty EKF Baseline", "", "");
  harness.run();
  return 0;
} 