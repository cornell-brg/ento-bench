#ifndef LEVENBERG_MARQUADT_H
#define LEVENBERG_MARQUADT_H

#include <Eigen/Dense>
#include <ento-pose/pose_util.h>

namespace EntoPose
{

/*
 Templated implementation of Levenberg-Marquadt.

 The Problem class must provide
    Problem::num_params - number of parameters to optimize over
    Problem::params_t - type for the parameters which optimize over
    Problem::accumulate(param, JtJ, Jtr) - compute jacobians
    Problem::residual(param) - compute the current residuals
    Problem::step(delta_params, param) - take a step in parameter space

    Check jacobian_impl.h for examples
*/

template <typename Scalar>
using IterationCallback = std::function<void(const BundleStats<Scalar> &stats)>;

template <typename Scalar, typename Problem, typename Param = typename Problem::param_t>
BundleStats<Scalar> lm_impl(Problem &problem,
                            Param *parameters,
                            const BundleOptions<Scalar> &opt,
                            std::function<void(const BundleStats<Scalar> &stats)> callback = nullptr) {
    constexpr int n_params = Problem::num_params;
    Eigen::Matrix<Scalar, n_params, n_params> JtJ;
    Eigen::Matrix<Scalar, n_params, 1> Jtr;

    // Initialize
    BundleStats<Scalar> stats;
    stats.cost = problem.residual(*parameters);
    stats.initial_cost = stats.cost;
    stats.grad_norm = -1;
    stats.step_norm = -1;
    stats.invalid_steps = 0;
    stats.lambda = opt.initial_lambda;

    bool recompute_jac = true;
    for (stats.iterations = 0; stats.iterations < opt.max_iterations; ++stats.iterations) {
        // We only recompute jacobian and residual vector if last step was successful
        if (recompute_jac) {
            JtJ.setZero();
            Jtr.setZero();
            // Add debug prints here
            // ENTO_DEBUG("lm_impl: parameters = (%f, %f, %f, %f, %f, %f), JtJ = (%f, %f, %f, %f, %f, %f), Jtr = (%f, %f, %f, %f, %f, %f)",
            //            parameters->q.x(), parameters->q.y(), parameters->q.z(), parameters->q.w(),
            //            parameters->t.x(), parameters->t.y(),
            //            JtJ.coeff(0, 0), JtJ.coeff(1, 1), JtJ.coeff(2, 2), JtJ.coeff(3, 3), JtJ.coeff(4, 4), JtJ.coeff(5, 5),
            //            Jtr.coeff(0), Jtr.coeff(1), Jtr.coeff(2), Jtr.coeff(3), Jtr.coeff(4), Jtr.coeff(5));
            problem.accumulate(*parameters, JtJ, Jtr);
            stats.grad_norm = Jtr.norm();
            if (stats.grad_norm < opt.gradient_tol) {
                break;
            }
        }

        // Add dampening
        for (size_t k = 0; k < n_params; ++k) {
            JtJ(k, k) += stats.lambda;
        }

        Eigen::Matrix<Scalar, n_params, 1> sol = -JtJ.template selfadjointView<Eigen::Lower>().llt().solve(Jtr);

        stats.step_norm = sol.norm();
        if (stats.step_norm < opt.step_tol) {
            break;
        }

        Param parameters_new = problem.step(sol, *parameters);

        Scalar cost_new = problem.residual(parameters_new);

        if (cost_new < stats.cost) {
            *parameters = parameters_new;
            stats.lambda = std::max(opt.min_lambda, stats.lambda / 10);
            stats.cost = cost_new;
            recompute_jac = true;
        } else {
            stats.invalid_steps++;
            // Remove dampening
            for (size_t k = 0; k < n_params; ++k) {
                JtJ(k, k) -= stats.lambda;
            }
            stats.lambda = std::min(opt.max_lambda, stats.lambda * 10);
            recompute_jac = false;
        }
        if (callback != nullptr) {
            callback(stats);
        }
    }
    return stats;
}

} // namespace EntoPose
  
#endif // LEVENBERG_MARQUADT_H
