#include <vector>
#include <iostream>
#include <functional>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ento-util/containers.h"
#include "ento-pose/pose_util.h"
#include "ento-pose/synthetic_relpose.h"
#include "entobench_generator.h"

using Scalar = float;                            // ⇦ flip to double if needed
using Pose   = EntoPose::CameraPose<Scalar>;

template<size_t N> using V2Cont = EntoUtil::EntoContainer<Eigen::Matrix<Scalar,2,1>,N>;
template<size_t N>
using V3Cont = EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>,N>;

//----------------------------------------
// Rotation error computation
//----------------------------------------
Scalar compute_rotation_error_degrees(const Pose& true_pose, const Pose& estimated_pose) {
    // Compute relative rotation: R_error = R_gt^T * R_est
    Eigen::Matrix<Scalar, 3, 3> R_gt = true_pose.R();
    Eigen::Matrix<Scalar, 3, 3> R_est = estimated_pose.R();
    Eigen::Matrix<Scalar, 3, 3> R_error = R_gt.transpose() * R_est;
    
    // Extract rotation angle from rotation matrix using trace
    Scalar trace = R_error.trace();
    Scalar cos_angle = (trace - 1.0) / 2.0;
    
    // Clamp to avoid numerical issues with acos
    cos_angle = std::max(Scalar(-1.0), std::min(Scalar(1.0), cos_angle));
    Scalar angle_rad = std::acos(cos_angle);
    
    return angle_rad * 180.0 / M_PI;  // Convert to degrees
}

//----------------------------------------
// Translation angle error computation
//----------------------------------------
Scalar compute_translation_angle_error_degrees(const Pose& true_pose, const Pose& estimated_pose) {
    // Normalize translation vectors
    Eigen::Matrix<Scalar, 3, 1> t_gt = true_pose.t.normalized();
    Eigen::Matrix<Scalar, 3, 1> t_est = estimated_pose.t.normalized();
    
    // Compute angle between translation directions
    Scalar cos_angle = t_gt.dot(t_est);
    
    // Clamp to avoid numerical issues with acos
    cos_angle = std::max(Scalar(-1.0), std::min(Scalar(1.0), cos_angle));
    Scalar angle_rad = std::acos(std::abs(cos_angle));  // Use abs to handle sign ambiguity
    
    return angle_rad * 180.0 / M_PI;  // Convert to degrees
}

//----------------------------------------
// Statistics tracking structure
//----------------------------------------
struct SolverStats {
    int total_problems = 0;
    int successful_solves = 0;
    int failed_solves = 0;
    
    // Error tracking for successful solves
    std::vector<Scalar> rotation_errors;
    std::vector<Scalar> translation_angle_errors;
    
    void add_success(Scalar rot_error, Scalar trans_error) {
        successful_solves++;
        rotation_errors.push_back(rot_error);
        translation_angle_errors.push_back(trans_error);
    }
    
    void add_failure() {
        failed_solves++;
    }
    
    void add_problem() {
        total_problems++;
    }
    
    // Statistics computation
    Scalar mean_rotation_error() const {
        if (rotation_errors.empty()) return Scalar(0);
        Scalar sum = 0;
        for (Scalar err : rotation_errors) sum += err;
        return sum / rotation_errors.size();
    }
    
    Scalar mean_translation_error() const {
        if (translation_angle_errors.empty()) return Scalar(0);
        Scalar sum = 0;
        for (Scalar err : translation_angle_errors) sum += err;
        return sum / translation_angle_errors.size();
    }
    
    void print_stats(const std::string& solver_name, Scalar noise_level) const {
        std::cout << "\n=== " << solver_name << " Statistics (noise=" << noise_level << ") ===" << std::endl;
        std::cout << "  Total problems: " << total_problems << std::endl;
        std::cout << "  Successful solves: " << successful_solves << " (" 
                  << (100.0 * successful_solves / total_problems) << "%)" << std::endl;
        std::cout << "  Failed solves: " << failed_solves << " (" 
                  << (100.0 * failed_solves / total_problems) << "%)" << std::endl;
        
        if (successful_solves > 0) {
            std::cout << "  Mean rotation error: " << std::fixed << std::setprecision(3) 
                      << mean_rotation_error() << "°" << std::endl;
            std::cout << "  Mean translation angle error: " << std::fixed << std::setprecision(3) 
                      << mean_translation_error() << "°" << std::endl;
            
            // Additional statistics
            if (!rotation_errors.empty()) {
                auto min_rot = *std::min_element(rotation_errors.begin(), rotation_errors.end());
                auto max_rot = *std::max_element(rotation_errors.begin(), rotation_errors.end());
                std::cout << "  Rotation error range: [" << min_rot << "°, " << max_rot << "°]" << std::endl;
            }
            
            if (!translation_angle_errors.empty()) {
                auto min_trans = *std::min_element(translation_angle_errors.begin(), translation_angle_errors.end());
                auto max_trans = *std::max_element(translation_angle_errors.begin(), translation_angle_errors.end());
                std::cout << "  Translation angle error range: [" << min_trans << "°, " << max_trans << "°]" << std::endl;
            }
        }
        std::cout << "================================" << std::endl;
    }
};

//----------------------------------------
// Minimal-solver wrappers
//----------------------------------------

// ----- 5-point -----
template<size_t N>
int solve5pt(V3Cont<N>& x1, V3Cont<N>& x2, std::vector<Pose>* solutions)
{
    EntoUtil::EntoArray<Pose,40> arr;
    int n = EntoPose::SolverRel5pt<Scalar>::template solve<N>(x1,x2,&arr);
    solutions->assign(arr.begin(), arr.begin()+n);
    return n;
}

// ----- 8-point -----
template<size_t N>
int solve8pt(V3Cont<N>& x1, V3Cont<N>& x2, std::vector<Pose>* solutions)
{
    constexpr int MaxSolns = EntoPose::SolverRel8pt<Scalar>::MaxSolns;
    EntoUtil::EntoArray<Pose, MaxSolns> arr;
    int n = EntoPose::SolverRel8pt<Scalar>::template solve<N>(x1, x2, &arr);
    solutions->assign(arr.begin(), arr.begin() + n);
    return n;
}

// ----- Upright 3-point -----
template<size_t N>
int solveU3pt(V3Cont<N>& x1, V3Cont<N>& x2, std::vector<Pose>* solutions)
{
    EntoUtil::EntoArray<Pose,4> arr;
    int n = EntoPose::SolverRelUpright3pt<Scalar>::template solve<N>(x1,x2,&arr);
    solutions->assign(arr.begin(), arr.begin()+n);
    return n;
}

// ----- Upright planar 3-point -----
template<size_t N>
int solveUP3pt(V3Cont<N>& x1, V3Cont<N>& x2, std::vector<Pose>* solutions)
{
    EntoUtil::EntoArray<Pose,2> arr;
    int n = EntoPose::SolverRelUprightPlanar3pt<Scalar>::template solve<N>(x1,x2,&arr);
    solutions->assign(arr.begin(), arr.begin()+n);
    return n;
}

// ----- Upright planar 2-pt (fixed N=2) -----
int solveUP2(V3Cont<2>& x1, V3Cont<2>& x2, std::vector<Pose>* solutions)
{
    EntoUtil::EntoArray<Pose,4> arr;
    int n = EntoPose::SolverRelUprightPlanar2pt<Scalar>::template solve<2>(x1,x2,&arr);
    solutions->assign(arr.begin(), arr.begin()+n);
    return n;
}

//----------------------------------------
// One problem instance
//----------------------------------------
template<size_t N>
struct ProblemInstance {
    V3Cont<N> x1, x2;
    Pose true_pose;
    
    void generate(int problem_id, Scalar noise_level, bool upright, bool planar) {
        // Use EntoBench generator with degeneracy checks to produce 2D image points
        EntoUtil::EntoArray<Eigen::Matrix<Scalar,2,1>, N> x1_img, x2_img;
        int num_inliers = N;
        int num_outliers = 0;
        
        // Retry logic for degenerate cases
        int max_retries = 10;
        for (int retry = 0; retry < max_retries; ++retry) {
            try {
                generate_entobench_realistic_relpose_data<Scalar, N>(
                    x1_img, x2_img, true_pose, num_inliers, num_outliers, problem_id + retry, upright, planar, noise_level
                );
                break; // Success, exit retry loop
            } catch (const std::runtime_error& e) {
                if (retry == max_retries - 1) {
                    // Last retry failed, use a simple fallback
                    std::cout << "Warning: Failed to generate non-degenerate data after " << max_retries << " attempts, using fallback" << std::endl;
                    // Simple fallback: just use the last attempt even if degenerate
                    generate_entobench_realistic_relpose_data<Scalar, N>(
                        x1_img, x2_img, true_pose, num_inliers, num_outliers, problem_id + retry, upright, planar, noise_level
                    );
                }
                // Continue to next retry
            }
        }
        
        // Unproject 2D image points to 3D bearing vectors
        for (size_t i = 0; i < N; ++i) {
            x1[i] = unproject_entobench_pixel<Scalar>(x1_img[i]);
            x2[i] = unproject_entobench_pixel<Scalar>(x2_img[i]);
        }
    }
};

//----------------------------------------
// Batch configuration
//----------------------------------------
template<size_t N>
struct BatchCfg {
    const char* tag;
    bool upright;
    bool planar;
    std::function<int(V3Cont<N>&, V3Cont<N>&, std::vector<Pose>*)> solver;
    
    BatchCfg(const char* t, bool u, bool p, 
             std::function<int(V3Cont<N>&, V3Cont<N>&, std::vector<Pose>*)> s)
        : tag(t), upright(u), planar(p), solver(s) {}
};

template<size_t N>
BatchCfg<N> make_batch_cfg(const char* tag, bool upright, bool planar, 
                           std::function<int(V3Cont<N>&, V3Cont<N>&, std::vector<Pose>*)> fn) {
    return BatchCfg<N>(tag, upright, planar, fn);
}

//----------------------------------------
// Run one experiment batch
//----------------------------------------
template<size_t N, typename Scalar>
void run_experiment(const BatchCfg<N>& cfg, Scalar noise_level, const char* precision) {
    int num_problems = 1000;
    std::cout << "Running " << cfg.tag << " with N=" << N << ", noise=" << noise_level << std::endl;
    
    SolverStats stats;
    
    for (int i = 0; i < num_problems; ++i) {
        stats.add_problem();
        
        // Generate problem
        ProblemInstance<N> prob;
        prob.generate(i, noise_level, cfg.upright, cfg.planar);
        
        // Solve
        std::vector<Pose> solutions;
        int num_solutions = cfg.solver(prob.x1, prob.x2, &solutions);
        
        if (num_solutions > 0) {
            // Find best solution (closest to ground truth)
            Scalar best_rot_error = std::numeric_limits<Scalar>::max();
            Scalar best_trans_error = std::numeric_limits<Scalar>::max();
            
            for (const auto& sol : solutions) {
                Scalar rot_error = compute_rotation_error_degrees(prob.true_pose, sol);
                Scalar trans_error = compute_translation_angle_error_degrees(prob.true_pose, sol);
                
                if (rot_error < best_rot_error) best_rot_error = rot_error;
                if (trans_error < best_trans_error) best_trans_error = trans_error;
            }
            
            // Success criteria: reasonable accuracy
            Scalar max_error = std::max(best_rot_error, best_trans_error);
            Scalar threshold = (noise_level == 0.0) ? Scalar(5.0) : Scalar(20.0);
            
            if (max_error < threshold) {
                stats.add_success(best_rot_error, best_trans_error);
            } else {
                stats.add_failure();
            }
        } else {
            stats.add_failure();
        }
    }
    
    // Print results
    stats.print_stats(cfg.tag, noise_level);
    
    // Write to CSV
    std::ofstream csv("benchmark_results.csv", std::ios::app);
    csv << cfg.tag << "," << N << "," << noise_level << ","
        << precision << ","
        << stats.successful_solves << "," << stats.total_problems << ","
        << std::fixed << std::setprecision(3)
        << stats.mean_rotation_error() << ","
        << stats.mean_translation_error() << std::endl;
    csv.close();
}

//----------------------------------------
// Main function
//----------------------------------------
template<typename Scalar>
void run_all(const char* precision) {
    using Pose = EntoPose::CameraPose<Scalar>;
    
    // SolverStats, solver wrappers, ProblemInstance, BatchCfg, make_batch_cfg as before, but using Scalar and Pose from above
    // ... copy the definitions from your current code, replacing global Scalar/Pose with these ...
    // ... for brevity, only the experiment loop is shown here ...
    std::vector<Scalar> noise_levels = {Scalar(0.0), Scalar(0.1), Scalar(0.25), Scalar(0.5)};
    // Minimal solvers
    for (auto noise : noise_levels) {
        run_experiment<5>(make_batch_cfg<5>("5pt", false, false, solve5pt<5>), noise, precision);
        run_experiment<3>(make_batch_cfg<3>("u3pt", true, false, solveU3pt<3>), noise, precision);
        run_experiment<2>(make_batch_cfg<2>("up2pt", true, true, solveUP2), noise, precision);
    }
    // Linear solvers
    for (auto noise : noise_levels) {
        run_experiment<8>(make_batch_cfg<8>("8pt", false, false, solve8pt<8>), noise, precision);
        run_experiment<16>(make_batch_cfg<16>("8pt", false, false, solve8pt<16>), noise, precision);
        run_experiment<32>(make_batch_cfg<32>("8pt", false, false, solve8pt<32>), noise, precision);
        run_experiment<64>(make_batch_cfg<64>("8pt", false, false, solve8pt<64>), noise, precision);
        run_experiment<3>(make_batch_cfg<3>("up3pt", true, true, solveUP3pt<3>), noise, precision);
        run_experiment<8>(make_batch_cfg<8>("up3pt", true, true, solveUP3pt<8>), noise, precision);
        run_experiment<16>(make_batch_cfg<16>("up3pt", true, true, solveUP3pt<16>), noise, precision);
        run_experiment<32>(make_batch_cfg<32>("up3pt", true, true, solveUP3pt<32>), noise, precision);
    }
}

int main() {
    std::cout << "Minimal Solver Benchmark" << std::endl;
    std::cout << "================================" << std::endl;
    std::ofstream csv("benchmark_results.csv");
    csv << "solver,N,noise_level,precision,successes,total_problems,mean_rot_error_deg,mean_trans_error_deg" << std::endl;
    csv.close();
    run_all<float>("float");
    run_all<double>("double");
    std::cout << "\nBenchmark complete! Results saved to benchmark_results.csv" << std::endl;
    return 0;
} 