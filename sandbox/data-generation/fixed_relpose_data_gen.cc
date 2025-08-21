// FIXED RELATIVE POSE DATA GENERATION
// 
// Key fixes applied:
// 1. PROPER IMAGE POINT NOISE: Uses realistic pinhole camera projection + pixel noise
//    instead of tangent space bearing vector noise (matches abs-pose methodology)
// 2. CONTROLLED POSE GENERATION: Uses realistic ±45° rotations and fixed baseline magnitude
//    instead of pathological random poses that create ill-conditioned problems
// 3. NESTED RETRY LOGIC: Proper pose/point generation retry like abs-pose
// 4. COMPREHENSIVE DEGENERACY CHECKS: Multiple geometric tests for well-conditioned data
//
// This should make traditional and realistic modes perform similarly well,
// with differences only coming from noise model (pixel vs tangent space).

template<typename Scalar, size_t N>
void generate_unified_relpose_data(
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x1_bear,
    EntoUtil::EntoContainer<Eigen::Matrix<Scalar,3,1>, N>& x2_bear,
    CameraPose<Scalar>& true_pose,
    size_t num_points,
    Scalar noise_level = Scalar(0.0),
    int problem_id = 0,
    bool upright_only = false,
    bool planar_only = false)
{
    using Vec2 = Eigen::Matrix<Scalar,2,1>;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    
    // Use different seed for each problem
    std::default_random_engine rng(42 + problem_id);
    std::uniform_real_distribution<Scalar> coord_gen(-1.0, 1.0);
    std::uniform_real_distribution<Scalar> depth_gen(1.0, 5.0);
    std::normal_distribution<Scalar> pixel_noise_gen(0.0, noise_level);  // PIXEL noise, not radians!
    
    // Add proper nested retry logic like abs-pose
    int max_pose_attempts = 100;  // Try different poses
    int max_point_attempts = 1000; // Try generating points for each pose
    
    for (int pose_attempt = 0; pose_attempt < max_pose_attempts; ++pose_attempt) {
        // FIXED: Generate controlled realistic poses (not pathological random poses)
        if (upright_only) {
            // Controlled yaw rotation like realistic mode (not random 0-360°)
            Scalar yaw_deg = 45.0 * coord_gen(rng);  // ±16° controlled rotation
            Scalar yaw_rad = yaw_deg * M_PI / 180.0;
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(yaw_rad, Vec3::UnitY()));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
            
            if (planar_only) {
                // Controlled baseline in XZ plane with fixed magnitude
                Scalar tx = coord_gen(rng);  // ±1 normalized direction
                Scalar tz = coord_gen(rng);
                Vec3 t_dir = Vec3(tx, 0, tz).normalized();
                true_pose.t = t_dir;  // Fixed baseline magnitude = 1.0
            } else {
                // Controlled 3D translation with fixed magnitude
                Vec3 t_dir = Vec3::Random().normalized();
                true_pose.t = t_dir;  // Fixed baseline magnitude = 1.0
            }
        } else {
            // FIXED: Controlled 6-DoF pose (not completely random)
            // Rotation: controlled magnitude around random axis (like realistic mode)
            Vec3 rot_axis = Vec3::Random().normalized();
            Scalar rot_angle_deg = 16.0 * coord_gen(rng);  // ±16° controlled rotation
            Scalar rot_angle_rad = rot_angle_deg * M_PI / 180.0;
            Quaternion q = Quaternion(Eigen::AngleAxis<Scalar>(rot_angle_rad, rot_axis));
            true_pose.q(0) = q.w();
            true_pose.q(1) = q.x();
            true_pose.q(2) = q.y();
            true_pose.q(3) = q.z();
            
            // Translation: fixed baseline magnitude (not random [0-4])
            Vec3 t_dir = Vec3::Random().normalized();
            true_pose.t = t_dir;  // Fixed baseline magnitude = 1.0
        }
        
        // Try to generate points with this pose using degeneracy checks
        for (int point_attempt = 0; point_attempt < max_point_attempts; ++point_attempt) {
            // Clear containers for this attempt
            if constexpr (N == 0) {
                x1_bear.clear();
                x2_bear.clear();
                x1_bear.reserve(num_points);
                x2_bear.reserve(num_points);
            } else {
                x1_bear.clear();
                x2_bear.clear();
            }
            
            // Temporary storage for degeneracy checks
            std::vector<Vec3> points3D_temp;
            std::vector<Vec2> points2D_cam1_temp, points2D_cam2_temp;
            points3D_temp.reserve(num_points);
            points2D_cam1_temp.reserve(num_points);
            points2D_cam2_temp.reserve(num_points);
            
            // Generate points
            size_t valid_points = 0;
            int safety_counter = 0;
            const int max_safety = 10000;
            
            while (valid_points < num_points && safety_counter < max_safety) {
                Vec3 X = Vec3(coord_gen(rng), coord_gen(rng), depth_gen(rng));
                Vec3 x1_cam = X;
                Vec3 x2_cam = true_pose.R() * X + true_pose.t;
                
                if (x1_cam(2) > Scalar(0.1) && x2_cam(2) > Scalar(0.1)) {
                    points3D_temp.push_back(X);
                    points2D_cam1_temp.push_back(Vec2(x1_cam(0) / x1_cam(2), x1_cam(1) / x1_cam(2)));
                    points2D_cam2_temp.push_back(Vec2(x2_cam(0) / x2_cam(2), x2_cam(1) / x2_cam(2)));
                    
                    // FIXED: Use proper image point projection and pixel noise (like abs-pose)
                    const Scalar focal_length = Scalar(500.0);
                    
                    // Project to image coordinates (pinhole camera model)
                    Vec2 x1_img(focal_length * x1_cam(0) / x1_cam(2), focal_length * x1_cam(1) / x1_cam(2));
                    Vec2 x2_img(focal_length * x2_cam(0) / x2_cam(2), focal_length * x2_cam(1) / x2_cam(2));
                    
                    // Add pixel noise (NOT tangent space noise!)
                    if (noise_level > Scalar(0.0)) {
                        std::normal_distribution<Scalar> pixel_noise_gen(0.0, noise_level * focal_length);
                        x1_img(0) += pixel_noise_gen(rng);
                        x1_img(1) += pixel_noise_gen(rng);
                        x2_img(0) += pixel_noise_gen(rng);
                        x2_img(1) += pixel_noise_gen(rng);
                    }
                    
                    // Convert back to bearing vectors
                    Vec3 f1 = Vec3(x1_img(0) / focal_length, x1_img(1) / focal_length, Scalar(1.0)).normalized();
                    Vec3 f2 = Vec3(x2_img(0) / focal_length, x2_img(1) / focal_length, Scalar(1.0)).normalized();
                    
                    // Degeneracy checks (like abs-pose)
                    Scalar dot_product = std::abs(f1.dot(f2));
                    if (dot_product > Scalar(0.999)) {  // Skip near-parallel rays
                        continue;
                    }
                    
                    x1_bear.push_back(f1);
                    x2_bear.push_back(f2);
                    ++valid_points;
                }
                safety_counter++;
            }
            
            if (safety_counter >= max_safety) continue; // Try again with same pose
            
            // DEGENERACY CHECKS
            bool is_degenerate = false;
            
            // Check 1: Collinearity test for 3D points
            if (num_points >= 3) {
                Vec3 v1 = points3D_temp[1] - points3D_temp[0];
                Vec3 v2 = points3D_temp[2] - points3D_temp[0];
                Vec3 cross = v1.cross(v2);
                if (cross.norm() < Scalar(1e-6)) {
                    is_degenerate = true;
                }
            }
            
            // Check 2: Coplanarity test for 4+ points
            if (num_points >= 4 && !is_degenerate) {
                Vec3 v1 = points3D_temp[1] - points3D_temp[0];
                Vec3 v2 = points3D_temp[2] - points3D_temp[0];
                Vec3 v3 = points3D_temp[3] - points3D_temp[0];
                Vec3 normal = v1.cross(v2);
                if (normal.norm() > Scalar(1e-8)) {
                    Scalar distance = std::abs(normal.dot(v3)) / normal.norm();
                    if (distance < Scalar(1e-4)) {
                        is_degenerate = true;
                    }
                }
            }
            
            // Check 3: Depth variation test
            if (!is_degenerate) {
                Scalar min_depth1 = std::numeric_limits<Scalar>::max();
                Scalar max_depth1 = std::numeric_limits<Scalar>::lowest();
                Scalar min_depth2 = std::numeric_limits<Scalar>::max();
                Scalar max_depth2 = std::numeric_limits<Scalar>::lowest();
                
                for (size_t i = 0; i < num_points; ++i) {
                    Vec3 x1_cam = points3D_temp[i];
                    Vec3 x2_cam = true_pose.R() * points3D_temp[i] + true_pose.t;
                    
                    min_depth1 = std::min(min_depth1, x1_cam(2));
                    max_depth1 = std::max(max_depth1, x1_cam(2));
                    min_depth2 = std::min(min_depth2, x2_cam(2));
                    max_depth2 = std::max(max_depth2, x2_cam(2));
                }
                
                Scalar depth_ratio1 = max_depth1 / min_depth1;
                Scalar depth_ratio2 = max_depth2 / min_depth2;
                if (depth_ratio1 < Scalar(1.5) || depth_ratio2 < Scalar(1.5)) {
                    is_degenerate = true;
                }
            }
            
            // Check 4: 2D point spread test
            if (!is_degenerate) {
                Scalar min_x1 = std::numeric_limits<Scalar>::max(), max_x1 = std::numeric_limits<Scalar>::lowest();
                Scalar min_y1 = std::numeric_limits<Scalar>::max(), max_y1 = std::numeric_limits<Scalar>::lowest();
                Scalar min_x2 = std::numeric_limits<Scalar>::max(), max_x2 = std::numeric_limits<Scalar>::lowest();
                Scalar min_y2 = std::numeric_limits<Scalar>::max(), max_y2 = std::numeric_limits<Scalar>::lowest();
                
                for (size_t i = 0; i < num_points; ++i) {
                    min_x1 = std::min(min_x1, points2D_cam1_temp[i](0));
                    max_x1 = std::max(max_x1, points2D_cam1_temp[i](0));
                    min_y1 = std::min(min_y1, points2D_cam1_temp[i](1));
                    max_y1 = std::max(max_y1, points2D_cam1_temp[i](1));
                    
                    min_x2 = std::min(min_x2, points2D_cam2_temp[i](0));
                    max_x2 = std::max(max_x2, points2D_cam2_temp[i](0));
                    min_y2 = std::min(min_y2, points2D_cam2_temp[i](1));
                    max_y2 = std::max(max_y2, points2D_cam2_temp[i](1));
                }
                
                Scalar x_spread1 = max_x1 - min_x1, y_spread1 = max_y1 - min_y1;
                Scalar x_spread2 = max_x2 - min_x2, y_spread2 = max_y2 - min_y2;
                
                if (x_spread1 < Scalar(0.5) || y_spread1 < Scalar(0.5) || 
                    x_spread2 < Scalar(0.5) || y_spread2 < Scalar(0.5)) {
                    is_degenerate = true;
                }
            }
            
            // Check 5: Baseline and rotation magnitude
            if (!is_degenerate) {
                Scalar baseline_length = true_pose.t.norm();
                if (baseline_length < Scalar(0.1)) {
                    is_degenerate = true;
                }
                
                Scalar trace_val = true_pose.R().trace();
                Scalar angle_rad = std::acos(std::clamp((trace_val - 1.0) / 2.0, -1.0, 1.0));
                Scalar angle_deg = angle_rad * 180.0 / M_PI;
                if (angle_deg < Scalar(2.0)) {
                    is_degenerate = true;
                }
            }
            
            if (!is_degenerate) {
                return; // SUCCESS! We found good data
            }
        } // end point_attempts loop
    } // end pose_attempts loop
    
    // If we get here, we couldn't generate a good configuration - REJECT this experiment
    std::cerr << "ERROR: Could not generate well-conditioned relative pose data after " << max_pose_attempts 
              << " pose attempts for problem " << problem_id << ". This experiment should be discarded!" << std::endl;
    throw std::runtime_error("Failed to generate valid relative pose data - experiment should be discarded");
}