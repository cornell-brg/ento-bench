#include <ento-util/unittest.h>
#include <ento-util/debug.h>
#include <ento-pose/problem-types/robust_pose_problem.h>
#include <ento-pose/synthetic_relpose.h>

using namespace EntoPose;
using namespace EntoUtil;

// Dummy solver for now; replace with a real solver when ready
struct DummySolver {};

void test_robust_relative_pose_problem_basic()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_basic...");
    using Scalar = float;
    constexpr size_t N = 8;

    // Generate synthetic data
    EntoContainer<Vec2<Scalar>, N> x1, x2;
    CameraPose<Scalar> true_pose;
    generate_synthetic_relpose_general<Scalar, N>(x1, x2, true_pose, N, 0.0);

    // Instantiate the robust problem (replace DummySolver with a real solver when ready)
    RobustRelativePoseProblem<Scalar, DummySolver, N> problem(DummySolver{});
    problem.initialize_inliers();

    // Set/check inlier mask
    problem.set_inlier(3, true);
    ENTO_TEST_CHECK_TRUE(problem.is_inlier(3));
    problem.set_inlier(3, false);
    ENTO_TEST_CHECK_TRUE(!problem.is_inlier(3));

    ENTO_DEBUG("test_robust_relative_pose_problem_basic passed.");
}

void test_robust_relative_pose_problem_deserialize_inlier_mask()
{
    ENTO_DEBUG("Running test_robust_relative_pose_problem_deserialize_inlier_mask...");
    using Scalar = float;
    constexpr size_t N = 8;
    RobustRelativePoseProblem<Scalar, DummySolver, N> problem(DummySolver{});
    // Compose a CSV line for inlier mask only (no trailing comma, as would be in a real file)
    // Only 1 byte for N=8
    char csv_line[64] = "1";
    char* pos = csv_line;
    bool ok = problem.deserialize_inlier_mask(pos);

    // Print the inlier mask bytes and bits (debugging already fixed by user)
    ENTO_DEBUG_ARRAY(problem.inlier_flags());
    for (size_t i = 0; i < problem.inlier_flags().size(); ++i) {
        std::string bits;
        for (int b = 7; b >= 0; --b) {
            bits += ((problem.inlier_flags()[i] >> b) & 1) ? '1' : '0';
        }
        ENTO_DEBUG("inlier_flags_[%zu] bits: %s", i, bits.c_str());
    }

    ENTO_TEST_CHECK_TRUE(ok);
    ENTO_TEST_CHECK_TRUE(problem.is_inlier(0)); // 1st bit of 1 is set
    ENTO_TEST_CHECK_TRUE(!problem.is_inlier(1)); // 2nd bit of 1 is not set
    // No check for is_inlier(16) since N=8 (static-size)
    ENTO_DEBUG("test_robust_relative_pose_problem_deserialize_inlier_mask passed.");
}

int main(int argc, char** argv)
{
    int __n = (argc == 1) ? 0 : atoi(argv[1]);
    if (__ento_test_num(__n, 1)) test_robust_relative_pose_problem_basic();
    if (__ento_test_num(__n, 2)) test_robust_relative_pose_problem_deserialize_inlier_mask();
    // Add more tests as you expand coverage
    return 0;
} 