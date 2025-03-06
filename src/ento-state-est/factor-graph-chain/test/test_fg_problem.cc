#include <Eigen/Dense>
#include <ento-state-est/factor-graph-chain/LinearBuffer.h>
#include <ento-state-est/factor-graph-chain/FgChainUnicycleStateEst.h>
#include <ento-state-est/factor-graph-chain/factor_graph_chain_estimation_problem.h>
#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-util/dataset_reader.h>
#include <ento-util/file_path_util.h>

const double PI = 3.14159265358979323846;

void test_fg_chain()
{
  using Scalar = float;
  class FgKernel{
    using StatesType = LinearBuffer< Eigen::Matrix< float, 4, 1 >, 10, 40 >;
    StatesType states;
    Eigen::Matrix< float, 4, 1 > init_state;
    FgChainUnicycleStateEst< float, 10, StatesType, 2 > estimator;
    public:
      void operator() ( Eigen::Matrix< float, 2, 1 > observation, int dt )
      {
        estimator.update( observation, dt );
      }
      FgKernel() :
        init_state( { 0, 0, .01, PI / 4 } ),
        estimator( states, init_state )
      {}
  };
  using Problem = EntoFgChainEst::FactorGraphChainEstimationProblem<float, FgKernel>;
  constexpr Scalar tol = 1e-4;

  const char* base_path = DATASET_PATH;
  const char* rel_path = "factor-graph-chain/test/path1.csv";
  char full_path[256];
  ENTO_DEBUG("================\n");
  ENTO_DEBUG("Running test_fg_chain...");
  if (!EntoUtil::build_file_path(base_path, rel_path, full_path, sizeof(full_path)))
  {
    ENTO_DEBUG("ERROR! Could not build file path for test_p3p_single!");
  }
  Problem problem(FgKernel{});

  ENTO_DEBUG("File path: %s", full_path);
  EntoUtil::DatasetReader reader(full_path);

  int num_experiments = 0;
  while (reader.read_next(problem))
  {
    num_experiments++;
  }
  ENTO_DEBUG( "experiments run: %d", num_experiments );
//   ENTO_TEST_CHECK_INT_EQ(num_experiments, 1);

  if ( num_experiments > 0 ) {
    problem.validate();
  }
}

int main ( int argc, char ** argv)
{
  using namespace EntoUtil;
  int __n;
  if (argc > 1)
  {
    __n = atoi(argv[1]);
  }
  else
  {
    // For the case we are running on the MCU and we can't pass in args
    // the same way args are passed for a native build.
    __ento_replace_file_suffix(__FILE__, "test_p3p_cmdline.txt");
    __n = __ento_get_test_num_from_file(__ento_cmdline_args_path_buffer);
  }

  if (__ento_test_num(__n, 1)) test_fg_chain();
}
