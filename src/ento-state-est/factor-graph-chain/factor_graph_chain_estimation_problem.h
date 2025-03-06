#ifndef FACTOR_GRAPH_CHAIN_EST_PROBLEM_H
#define FACTOR_GRAPH_CHAIN_EST_PROBLEM_H

#include <ento-bench/problem.h>
#include <ento-util/containers.h>
#include <ento-util/debug.h>

namespace EntoFgChainEst
{

template <typename Scalar, typename Kernel>
class FactorGraphChainEstimationProblem :
  public EntoBench::EntoProblem<FactorGraphChainEstimationProblem<Scalar, Kernel>>
{
  public:
    // Expose Template Typenames to Others
    using Scalar_t = Scalar;
    using Kernel_t = Kernel;

    // Required by Problem Interface for Experiment I/O
    static constexpr bool RequiresDataset_ = true;
    static constexpr bool SavesResults_ = true;

    // @TODO: Add other member fields for input data, ground truth data
    //   and output data.
    Kernel_t m_kernel;
    Eigen::Matrix< Scalar_t, 2, 1 > m_observation;
    EntoUtil::EntoContainer< Eigen::Matrix< Scalar_t, 4, 1 >, 0 > m_gt_states;
    const double m_tol = 0.1;
    int m_states_count = 0;
  
  #ifdef NATIVE
    std::string serialize_impl() const;
    bool deserialize_impl(const std::string &line)
    {
      std::stringstream ss( line );
      std::string gt_x_str, gt_y_str, gt_v_str, gt_theta_str, x_str, y_str;
      std::getline( ss, gt_x_str, ',' );
      std::getline( ss, gt_y_str, ',' );
      std::getline( ss, gt_v_str, ',' );
      std::getline( ss, gt_theta_str, ',' );
      std::getline( ss, x_str, ',' );
      std::getline( ss, y_str, ',' );
      Scalar_t gt_x, gt_y, gt_v, gt_theta, x, y;
      gt_x     = (Scalar_t) std::stold( gt_x_str );
      gt_y     = (Scalar_t) std::stold( gt_y_str );
      gt_v     = (Scalar_t) std::stold( gt_v_str );
      gt_theta = (Scalar_t) std::stold( gt_theta_str );
      x        = (Scalar_t) std::stold( x_str );
      y        = (Scalar_t) std::stold( y_str );
      m_observation << x, y;
      Eigen::Matrix< Scalar_t, 4, 1 > gt_state;
      gt_state << gt_x, gt_y, gt_v, gt_theta;
      m_gt_states.push_back( gt_state );
      m_states_count++;
      return true;
    }
  #else
    // @TODO: Add serialize implementation for embedded builds.
    const char* serialize_impl() const;
    // @TODO: Add deserialize implementation for embedded builds
    bool deserialize_impl(const char* line);
  #endif

    // @TODO: Complete validate implementation. 
    bool validate_impl() const
    {
      for ( int i = 0; i < m_states_count; i++ ) {
        // diff m_gt_states[i] with calculated state, compare to m_tol
      }
      return true;
    }

    // @TODO: Complete solve implementation.
    void solve_impl()
    {
      m_kernel( m_observation, 1 );
    }

    // @TODO: Complete clear implementation.
    void clear_impl()
    {
      // m_gt_states.clear(); // clear gets run after every line of input...
    }

    static constexpr const char* header_impl()
    {
      return "Factor Graph Chain Path Observations ( gt_x, gt_y, gt_v, gt_theta, obs_x, obs_y )";
    }

    FactorGraphChainEstimationProblem(Kernel kernel) : 
      m_kernel(std::move(kernel))
    {}
};

}

#endif // FACTOR_GRAPH_CHAIN_EST_PROBLEM_H