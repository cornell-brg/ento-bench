#ifndef EKF_PROBLEM_H
#define EKF_PROBLEM_H

#include <Eigen/Dense>
#include <ento-bench/problem.h>

namespace EntoStateEst
{
  
template <typename Kernel>
class KalmanFilterProblem
  : EntoBench::EntoProblem<KalmanFilterProblem<Kernel>>
{
private:
  Kernel kernel_;

public:
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SavesResults_ = true;
  static constexpr size_t StateDim_ = Kernel::StateDim_;
  static constexpr size_t MeasurementDim_ = Kernel::MeasurementDim_;
  static constexpr size_t ControlDim_ = Kernel::ControlDim_;
  using Scalar_ = Kernel::Scalar_;

  //////// Class Members /////////
  
  // Inputs
  Eigen::Matrix<Scalar_, StateDim_, 1> x0_; // Initial state
  Eigen::Matrix<Scalar_, ControlDim_, 1> u_; // Control vector
  Eigen::Matrix<Scalar_, MeasurementDim_, 1> z_; // Measurement

  // Outputs
  Eigen::Matrix<Scalar_, StateDim_, 1> x_; // State vector


  //////// Constructor /////////
  KalmanFilterProblem(Kernel kernel) : kernel_(std::move(kernel)) {}

  //////// Problem Interface /////////
#ifdef NATIVE
  std::string serialize_impl() const;
  bool        deserialize_impl(const std::string& line);
#else
  const char* serialize_impl() const;
#endif 
  bool        deserialize_impl(const char* line);
    
  bool validate_impl() const;
  void solve_impl();
  void clear_impl();

  static constexpr const char* header_impl() { return ""; }
  static constexpr const char* output_header_impl() { return ""; }
};

#ifdef NATIVE
template <typename Kernel>
bool KalmanFilterProblem<Kernel>::deserialize_impl(const std::string& line)
{
  std::istringstream iss(line);


  size_t state_dim = 0, meas_dim = 0, control_dim = 0;
  // Header includes the initial state, so it should be:
  // {COLUMN_INFO}\n
  // Initial State (x0)
  // Data comes in the form: dt, measurement, control input
  

}


template <typename Kernel>
std::string KalmanFilterProblem<Kernel>::serialize_impl() const
{

}
#else
template <typename Kernel>
const char* KalmanFilterProblem<Kernel>::serialize_impl()
{

}
#endif
template <typename Kernel>
bool KalmanFilterProblem<Kernel>::deserialize_impl(const char* line)
{
  
}

template <typename Kernel>
void KalmanFilterProblem<Kernel>::solve_impl()
{
}


template <typename Kernel>
bool KalmanFilterProblem<Kernel>::validate_impl() const
{
}


template <typename Kernel>
void KalmanFilterProblem<Kernel>::clear_impl()
{
}

} // EntoStateEst

#endif // EKF_PROBLEM_H
