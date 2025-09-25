#include "rizon_kinematics.hpp"
#include <iostream>
#include <torch/torch.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tensor_utils.hpp"  


RizonKinematics::RizonKinematics() {

    // Initialize variables 
    //===========================================================================================================================
    // base-joint1, joint1-joint2, joint2-joint3, joint3-joint4, joint4-joint5, joint5-joint6, joint6-joint7, joint7-flange
    T_adjacent = torch::zeros({8, 4, 4}, tensor_options); 
    // joint1-7 and flange
    T_joint = torch::zeros({8, 4, 4}, tensor_options);
    // end-effector jacobian matrix: row(6): pos(3) + rot(3); col(7): 7 DOF
    jaco_eff = torch::zeros({6, 7}, tensor_options);
    // Empty obstacles, to be updated later 
    obstacle_points_cpu = torch::empty({0, 3}, tensor_options); 
    obstacle_points_gpu = torch::empty({0, 3}, gpu_options); 

    initialize_meshes(); 
    
}

// Update functions 
//=========================================================================================
/**
 * @brief Update the joint positions, refresh the homogenous transformations between adjacent joints, 
 * refresh the joint transformation relative to robot base, and refresh the end-effector (flange) jacobian matrix 
 * @param q {7} tensor, joint positions   
 */
void RizonKinematics::update(const torch::Tensor& q) {
    current_q_ptr = std::make_shared<torch::Tensor>(tensor_utils::normalize_angles(q));
    update_adjacent_transformation(); 
    update_joint_transformation(); 
    update_jaco_eff();
    if (is_using_meshes) {
        update_meshes();
    }
}

/**
 * @brief Update the joint positions, call another overload update function 
 * @param q Size 7 std::vector<double> or std::array<double, 7>
 */
template <typename Container>
void RizonKinematics::update(const Container& q) {
    update(tensor_utils::container_to_tensor(q));
}
// Explicit template instantiation for std::vector and std::array
template void RizonKinematics::update<std::vector<double>>(const std::vector<double>&);
template void RizonKinematics::update<std::array<double, 7>>(const std::array<double, 7>&);


/**
 * @brief Update the homogenous transformation matrices between adjacent joints based on new joint positions 
 */
void RizonKinematics::update_adjacent_transformation() {

    // Extract parameters
    auto theta = DH_PARAMS.index({torch::indexing::Slice(), 3}).clone(); // Theta    
    theta.index({torch::indexing::Slice(0, 7)}) += *current_q_ptr;
    auto d = DH_PARAMS.index({torch::indexing::Slice(), 1});     // d
    auto a = DH_PARAMS.index({torch::indexing::Slice(), 0});     // a
    auto alpha = DH_PARAMS.index({torch::indexing::Slice(), 2}); // Alpha

    // Precompute cosines and sines
    auto cos_theta = torch::cos(theta);
    auto sin_theta = torch::sin(theta);
    auto cos_alpha = torch::cos(alpha);
    auto sin_alpha = torch::sin(alpha);

    // Efficiently fill the transformation matrices using tensor operations
    T_adjacent.index_put_({torch::indexing::Slice(), 0, 0}, cos_theta);
    T_adjacent.index_put_({torch::indexing::Slice(), 0, 1}, -sin_theta);
    T_adjacent.index_put_({torch::indexing::Slice(), 0, 3}, a);

    T_adjacent.index_put_({torch::indexing::Slice(), 1, 0}, sin_theta * cos_alpha);
    T_adjacent.index_put_({torch::indexing::Slice(), 1, 1}, cos_theta * cos_alpha);
    T_adjacent.index_put_({torch::indexing::Slice(), 1, 2}, -sin_alpha);
    T_adjacent.index_put_({torch::indexing::Slice(), 1, 3}, -sin_alpha * d);

    T_adjacent.index_put_({torch::indexing::Slice(), 2, 0}, sin_theta * sin_alpha);
    T_adjacent.index_put_({torch::indexing::Slice(), 2, 1}, cos_theta * sin_alpha);
    T_adjacent.index_put_({torch::indexing::Slice(), 2, 2}, cos_alpha);
    T_adjacent.index_put_({torch::indexing::Slice(), 2, 3}, cos_alpha * d);

    T_adjacent.index_put_({torch::indexing::Slice(), 3, 3}, 1.0);
}


/**
 * @brief Update homogenous transformation matrices of joints relative to robot base 
 */
void RizonKinematics::update_joint_transformation() {
    T_joint[0] = T_adjacent[0];
    for (int i = 1; i < T_adjacent.size(0); i++) { 
        T_joint[i] = T_joint[i-1].matmul(T_adjacent[i]);
    }
}


/**
 * @brief Update end-effector (flange) jacobian matrix
 */
void RizonKinematics::update_jaco_eff() {
    torch::Tensor z = T_joint.slice(0, 0, -1).slice(1, 0, 3).slice(2, 2, 3).squeeze(-1); 
    torch::Tensor t = T_joint[-1].slice(0, 0, 3).slice(1, 3, 4).squeeze(-1) 
                      - T_joint.slice(0, 0, -1).slice(1, 0, 3).slice(2, 3, 4).squeeze(-1); 
    jaco_eff.slice(0, 0, 3) = z.cross(t, 1).transpose(0, 1); 
    jaco_eff.slice(0, 3, 6) = z.transpose(0, 1);
}

void RizonKinematics::initialize_meshes() { 
    Assimp::Importer importer; 
    std::string modelPath; 
    const aiScene* scene; 

    for (int link_id = 0; link_id < 8; ++link_id) { 
        std::string link_name = "link" + std::to_string(link_id); 
        if (link_id == 7) {
            link_name = "link_7_with_tool"; 
        }
        modelPath = "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/flexiv_rdk/resources/meshes/rizon4/visual/" + link_name + ".obj"; 
        scene = importer.ReadFile(modelPath, 
            aiProcess_Triangulate | 
            aiProcess_JoinIdenticalVertices | 
            aiProcess_GenNormals 
        ); 

        link_meshes[link_id] = torch::empty({0, 4}, tensor_options); 

        for(unsigned int j = 0; j < scene->mNumMeshes; j++){ 

            aiMesh* mesh = scene->mMeshes[j]; 

            auto vertices = torch::empty({mesh->mNumVertices, 4}, tensor_options); 

            for(unsigned int v = 0; v < mesh->mNumVertices; v++){
                aiVector3D vertex = mesh->mVertices[v];

                vertices[v][0] = vertex.x;
                vertices[v][1] = vertex.y;
                vertices[v][2] = vertex.z; 
                vertices[v][3] = 1.0; 

                if (link_id == 7) {
                    vertices[v][0] = vertex.x / 100.0;
                    vertices[v][1] = vertex.y / 100.0;
                    vertices[v][2] = vertex.z / 100.0; 
                }
            } 
            link_meshes[link_id] = torch::cat({link_meshes[link_id], vertices}, 0); 
        } 
        link_meshes[link_id] = torch::matmul(link_meshes[link_id], torch::transpose(mesh_offsets[link_id], 0, 1)).to(torch::kCUDA); 
    } 
} 

void RizonKinematics::update_meshes() { 
    for (int link_id = 1; link_id < T_adjacent.size(0); link_id++) { 

        torch::Tensor trans_gpu = T_joint[link_id - 1].to(torch::kCUDA); 

        auto start_index = link_mesh_index[link_id]; 
        auto end_index = link_mesh_index[link_id + 1]; 
        mesh_points.index_put_({torch::indexing::Slice(start_index, end_index), torch::indexing::Slice(torch::indexing::None)}, 
            torch::matmul(link_meshes[link_id], torch::transpose(trans_gpu, 0, 1)).index({torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(0, 3)})); 
    }
} 

/**
 * @brief Update pose difference (dpose) given the target pose of the end-effector (flange)
 * @param desired_pose Size 7 std::vector<double> or std::array<double, 7> in [px, py, pz, rx, ry, rz, rw] 
 */
template <typename Container>
void RizonKinematics::update_dpose(const Container& desired_pose) {
    //=================slower=======================
    // torch::Tensor target_pose = tensor_utils::container_to_tensor(desired_pose); 
    // torch::Tensor current_pose = tensor_utils::tensor_matrix_to_tensor_vector(T_joint[-1]); 

    // torch::Tensor quat_1 = target_pose.slice(0, 3, 7); 
    // torch::Tensor quat_2 = tensor_utils::quaternion_conjugate(current_pose.slice(0, 3, 7)); 
    // torch::Tensor rot_diff = tensor_utils::quaternion_multiply(quat_1, quat_2, false); 
    // torch::Tensor rot_diff_tensor = rot_diff.slice(0, 0, 3) * torch::sign(rot_diff[3]); 

    // torch::Tensor trans_diff_tensor = target_pose.slice(0, 0, 3) - current_pose.slice(0, 0, 3); 
    
    // dpose = torch::cat({trans_diff_tensor, rot_diff_tensor}, 0); 
    //==============================================

    Eigen::Quaterniond quat_1(desired_pose[6], desired_pose[3],
                              desired_pose[4], desired_pose[5]);
    auto rot_tensor = T_joint[-1].slice(0, 0, 3).slice(1, 0, 3).clone();  
    // one transpose of rot_tensor: from torch::Tensor to Eigen::Matrix3d
    // one transpose of rot_tensor: conjugate of quaternion == transpose of matrix 
    float* data_ptr = rot_tensor.to(torch::kCPU).data_ptr<float>(); 
    std::vector<double> vec(data_ptr, data_ptr+9); 
    Eigen::Map<Eigen::Matrix3d> eigen_matix(vec.data()); 
    Eigen::Quaterniond quat_2(eigen_matix); 
    Eigen::Quaterniond rot_diff = quat_1 * quat_2; 
    double sign = std::copysign(1.0, rot_diff.w());
	auto rot_diff_tensor = torch::tensor({rot_diff.vec()[0] * sign, 
										  rot_diff.vec()[1] * sign, 
										  rot_diff.vec()[2] * sign}, tensor_options); 
    
	auto trans_diff_tensor = torch::tensor({desired_pose[0], desired_pose[1], desired_pose[2]}, tensor_options) 
							 - T_joint[-1].slice(0, 0, 3).slice(1, 3, 4).squeeze(-1);
	dpose = torch::cat({trans_diff_tensor, rot_diff_tensor}, 0);

}
// Explicit template instantiation for std::vector and std::array
template void RizonKinematics::update_dpose<std::vector<double>>(const std::vector<double>&);
template void RizonKinematics::update_dpose<std::array<double, 7>>(const std::array<double, 7>&);


// IK solvers 
//================================================================================================ 
/**
 * @brief Determine the increment in joint position resulting from the displacement relative to the desired pose and the closest obstacle 
 * @param desired_pose Size 7 std::vector<double> or std::array<double, 7> in [px, py, pz, rx, ry, rz, rw] 
 * @return Size 7 joint position increment 
 */
template <typename Container>
std::vector<double> RizonKinematics::control_ik(const Container& desired_pose) { 
    // update dpose 
    update_dpose(desired_pose); 

    // Calculate the increment due to the target pose 
    torch::Tensor jaco_eff_T = torch::transpose(jaco_eff, 0, 1); 
    torch::Tensor lmbda = torch::eye({6}, tensor_options) * (std::pow(0.05, 2)); 
    torch::Tensor lmbda_2 = torch::eye({7}, tensor_options) * (std::pow(0.05, 2)); 
    torch::Tensor u = jaco_eff_T.matmul((jaco_eff.matmul(jaco_eff_T) + lmbda).inverse().matmul(dpose)); 

    // Calculate the increment due to the closest obstacle 
    auto u3 = torch::zeros({7}, tensor_options); 
    std::pair<at::Tensor, at::Tensor> close_info; 
    if (is_using_meshes) {
        close_info = closest_to_links_based_on_meshes(); 
    } 
    else {
        close_info = closest_to_links(); 
    }
    u3 = (jaco_eff_T.matmul(jaco_eff) + lmbda_2).inverse().matmul(torch::transpose(close_info.first, 0, 1)).matmul(close_info.second); 

    std::vector<double> stream_u = tensor_utils::tensor_to_vector(u - u3); 

    return tensor_utils::tensor_to_vector(u - u3); 
}
// Explicit template instantiation for std::vector and std::array
template std::vector<double> RizonKinematics::control_ik<std::vector<double>>(const std::vector<double>&);
template std::vector<double> RizonKinematics::control_ik<std::array<double, 7>>(const std::array<double, 7>&);


/**
 * @brief Determine the increment in joint position resulting from the displacement relative to the desired pose 
 * @param desired_pose Size 7 std::vector<double> or std::array<double, 7> in [px, py, pz, rx, ry, rz, rw] 
 * @return Size 7 joint position increment 
 */
template <typename Container>
std::vector<double> RizonKinematics::direct_ik(const Container& desired_pose) { 
    // Update dpose 
    update_dpose(desired_pose); 

    // Calculate the increment due to the target pose 
    torch::Tensor jaco_eff_T = torch::transpose(jaco_eff, 0, 1); 
    torch::Tensor lmbda = torch::eye({6}, tensor_options) * (std::pow(0.05, 2)); 
    torch::Tensor u = jaco_eff_T.matmul((jaco_eff.matmul(jaco_eff_T) + lmbda).inverse().matmul(dpose)); 

    return tensor_utils::tensor_to_vector(u);
}
// Explicit template instantiation for std::vector and std::array
template std::vector<double> RizonKinematics::direct_ik<std::vector<double>>(const std::vector<double>&);
template std::vector<double> RizonKinematics::direct_ik<std::array<double, 7>>(const std::array<double, 7>&);


/**
 * @brief Iteratively determine the final joint positions to reach the desired pose. 
 * If no iteration_num or tolerance is provided, default values will be used.  
 * @param desired_pose Size 7 std::vector<double> or std::array<double, 7>  in [px, py, pz, rx, ry, rz, rw] 
 * @param iteration_num Maxmium iteration time. Defaults to 100. 
 * @param tolerance Maxmium error in each dimension of dpose: 3 pos + 3 rot. Defaults to 1e-5 
 * @warning This function will overwrite the kinematics 
 * That is, change the configurations (joint positions) of this kinematics 
 */
template <typename Container>
void RizonKinematics::solve_ik(const Container& desired_pose, uint iteration_num, double tolerance) {
    // Run interation_num times 
    uint i = 0;
    while (i < iteration_num) {
        ++i; 
        // Update dpose 
        update_dpose(desired_pose); 

        // Calculate the joint position increment 
        torch::Tensor jaco_eff_T = torch::transpose(jaco_eff, 0, 1); 
        torch::Tensor lmbda = torch::eye({6}, tensor_options) * (std::pow(0.05, 2)); 
        torch::Tensor u = jaco_eff_T.matmul((jaco_eff.matmul(jaco_eff_T) + lmbda).inverse().matmul(dpose)); 

        // Update the kinematics 
        update(*current_q_ptr+u); 

        // Within the tolerance, stop loop 
        if (torch::max(torch::abs(dpose)).item<float>() < 1e-5) { break; };
    }
}
// Explicit template instantiation for std::vector and std::array
// Correct explicit instantiation syntax
template void RizonKinematics::solve_ik<std::vector<double>>(const std::vector<double>&, uint, double);
template void RizonKinematics::solve_ik<std::array<double, 7>>(const std::array<double, 7>&, uint, double);


// Obstacle methods 
//===================================================================================================
/**
 * @brief Update the obstacles 
 * @param obstacle_data 2D vectors, each row is a size 3 std::vector<double>, showing [px, py, pz] of a point obstacle 
 */
void RizonKinematics::update_obstacles(const std::vector<std::vector<double>>& obstacle_data) { 
    obstacle_points_gpu = tensor_utils::vector2d_to_tensor(obstacle_data).to(torch::kCUDA); 
    obstacle_points_cpu = tensor_utils::vector2d_to_tensor(obstacle_data); 
    
}

void RizonKinematics::min_distance_between_obstacle_points_and_point_meshes() { 

    int64_t N = obstacle_points_gpu.size(0); 
    int64_t M = mesh_points.size(0); 

    auto diff = obstacle_points_gpu.unsqueeze(1) - mesh_points.unsqueeze(0);  // [N, M, 3]
    auto dist_sq = diff.pow(2).sum(-1);          // shape [N, M] 
    auto dist = dist_sq.sqrt();   // [N, M]

    auto flat_dist = dist.view({-1});    // [N*M] 

    auto result = flat_dist.min(0);    // returns {values, indices} 

    float min_distance = std::get<0>(result).item<float>(); 
    int64_t flat_idx = std::get<1>(result).item<int64_t>(); 

    int64_t i = flat_idx / M; 
    int64_t j = flat_idx % M; 

    closest_point.first = obstacle_points_gpu[i].to(torch::kCPU); 
    closest_point.second = mesh_points[j].to(torch::kCPU);  
} 

std::pair<torch::Tensor, torch::Tensor> RizonKinematics::closest_to_links_based_on_meshes() {
    // Return zeros if no obstacle 
    if (obstacle_points_gpu.size(0) == 0) { 
        return std::make_pair(torch::zeros({3, 7}, tensor_options), torch::zeros({3}, tensor_options)); 
    }
    min_distance_between_obstacle_points_and_point_meshes(); 

    torch::Tensor closest_obstacle_point = closest_point.first.view({1, 1, 3}); 

    torch::Tensor joint_points = T_joint.slice(1, 0, 3).slice(2, 3, 4).squeeze(-1); 
    torch::Tensor vector_1 = closest_obstacle_point - joint_points.slice(0, 0, -1); 
    torch::Tensor vector_2 = closest_obstacle_point - joint_points.slice(0, 1, joint_points.size(0)); 

    // Find the projection of each obstacle onto the link segmentations 
    torch::Tensor diff = vector_1 - vector_2;
    torch::Tensor norm_diff = torch::norm(diff, 2, 2);
    torch::Tensor l = torch::sum(vector_1 * diff, 2) / (norm_diff * norm_diff); 
    l = torch::clamp(l, 0, 1);
    torch::Tensor v_min = (1 - l).unsqueeze(-1) * vector_1 + l.unsqueeze(-1) * vector_2; 

    torch::Tensor norm_v = torch::norm(v_min, 2, -1);
    torch::Tensor min_index = torch::argmin(norm_v);
    int obstcle_index = min_index.item<int32_t>() / 7;
    int joint_index = min_index.item<int32_t>() % 7; 

    // Initialize the return pair 
    std::pair<at::Tensor, at::Tensor> res = 
        std::make_pair(torch::zeros({3, 7}, tensor_options), torch::zeros({3}, tensor_options));

    // Check if the closest obstacle is within the threshold 

    double distance = norm_v[obstcle_index][joint_index].item<double>(); 
    min_obstacle_distance = distance; 

    double threshold = 0.1; 
    if (distance > threshold) {
        // Not within the threshold ==> not dangerous, do nothing 
        ;
    }
    else { 
        // Within the threshold 
        // Get the corresponding vector 
        torch::Tensor l_min = l[obstcle_index][joint_index]; 
        v_min = v_min[obstcle_index][joint_index]; 

        // Calculate the corresponding matrix and assign to the return pair
        torch::Tensor z = T_joint.slice(0, 0, joint_index+1).slice(1, 0, 3).slice(2, 2, 3).squeeze(-1);
        torch::Tensor point = v_min + closest_obstacle_point[obstcle_index]; 
        torch::Tensor t = point - T_joint.slice(0, 0, joint_index + 1).slice(1, 0, 3).slice(2, 3, 4).squeeze(-1); 
        torch::Tensor j = torch::zeros({3, 7}, tensor_options);
        j.slice(0, 0, j.size(0)).slice(1, 0, joint_index + 1) = torch::transpose(torch::cross(z, t, 1), 0, 1);
        res.first = j;

        // Calculate the coefficient and scale the vector pointing to the cloeset obstacle 
        double lmd = 10.0; 
        double coeff = lmd * (threshold - distance) / std::pow(distance, 2);
        res.second = (coeff * v_min);
    }

    return res;
}

/**
 * @brief Find the closest obstacle, return the corresponding jaccbian matrix on the robot and the scaled vector to the obstacle. 
 * @return a pair, first: {3, 7} position jacobian matrix; second: scaled position vector 
 */
std::pair<torch::Tensor, torch::Tensor> RizonKinematics::closest_to_links() {

    // Return zeros if no obstacle 
    if (obstacle_points_cpu.size(0) == 0) { 
        return std::make_pair(torch::zeros({3, 7}, tensor_options), torch::zeros({3}, tensor_options)); 
    }

    // Get the vectors from the joint points to the obstacles 
    torch::Tensor joint_points = T_joint.slice(1, 0, 3).slice(2, 3, 4).squeeze(-1);
    torch::Tensor vector_1 = obstacle_points_cpu.unsqueeze(1) - joint_points.slice(0, 0, -1); 
    torch::Tensor vector_2 = obstacle_points_cpu.unsqueeze(1) - joint_points.slice(0, 1, joint_points.size(0));

    // Find the projection of each obstacle onto the link segmentations 
    torch::Tensor diff = vector_1 - vector_2;
    torch::Tensor norm_diff = torch::norm(diff, 2, 2);
    torch::Tensor l = torch::sum(vector_1 * diff, 2) / (norm_diff * norm_diff); 
    l = torch::clamp(l, 0, 1);
    torch::Tensor v_min = (1 - l).unsqueeze(-1) * vector_1 + l.unsqueeze(-1) * vector_2; 

    // Find the shortest vector (most dangerous)
    torch::Tensor norm_v = torch::norm(v_min, 2, -1);
    torch::Tensor min_index = torch::argmin(norm_v);
    int obstcle_index = min_index.item<int32_t>() / 7;
    int joint_index = min_index.item<int32_t>() % 7;

    // Initialize the return pair 
    std::pair<at::Tensor, at::Tensor> res = 
        std::make_pair(torch::zeros({3, 7}, tensor_options), torch::zeros({3}, tensor_options));

    // Check if the closest obstacle is within the threshold 
    double distance = norm_v[obstcle_index][joint_index].item<double>(); 
    min_obstacle_distance = distance; 
    double threshold = 0.1; 
    if (distance > threshold) {
        // Not within the threshold ==> not dangerous, do nothing 
        ;
    }
    else {
        // Within the threshold 
        // Get the corresponding vector 
        torch::Tensor l_min = l[obstcle_index][joint_index]; 
        v_min = v_min[obstcle_index][joint_index]; 

        // Calculate the corresponding matrix and assign to the return pair
        torch::Tensor z = T_joint.slice(0, 0, joint_index+1).slice(1, 0, 3).slice(2, 2, 3).squeeze(-1);
        torch::Tensor point = v_min + obstacle_points_cpu[obstcle_index]; 
        torch::Tensor t = point - T_joint.slice(0, 0, joint_index + 1).slice(1, 0, 3).slice(2, 3, 4).squeeze(-1); 
        torch::Tensor j = torch::zeros({3, 7}, tensor_options);
        j.slice(0, 0, j.size(0)).slice(1, 0, joint_index + 1) = torch::transpose(torch::cross(z, t, 1), 0, 1);
        res.first = j;

        // Calculate the coefficient and scale the vector pointing to the cloeset obstacle 
        double lmd = 10.0; 
        double coeff = lmd * (threshold - distance) / std::pow(distance, 2);
        res.second = coeff * v_min;
    }

    return res;
}

// Get functions 
//=============================================================================================================
/** 
 * @brief Get the current flange pose 
 * @return: vector in [px, py, pz, rx, ry, rz, rw] 
 */
std::vector<double> RizonKinematics::get_flange_pose() {
    return tensor_utils::tensor_to_vector(tensor_utils::tensor_matrix_to_tensor_vector(T_joint[-1])); 
}

/**
 * @brief Get the current joint positions 
 * @return Size 7 joint position vector 
 */
std::vector<double> RizonKinematics::get_current_q_vector() {
    return tensor_utils::tensor_to_vector(*current_q_ptr); 
}

// std::vector<std::vector<double>> RizonKinematics::get_current_qva_vector() {
//     auto data = history_time_q; 
//     std::vector<double> current_q = data.back().second; 
//     std::vector<double> current_v = std::vector<double>(7, 0.0); 
//     std::vector<double> current_a = std::vector<double>(7, 0.0); 

//     static double largest_v = 0.0; 
//     static double largest_a = 0.0; 
//     // std::cout << largest_v << " " << largest_a << std::endl; 


//     if (data.size() == 3) {
//         double duration_now = std::chrono::duration_cast<std::chrono::microseconds>(
//             data.back().first - data[1].first).count() / 1000000.0; 
//         double duration_last = std::chrono::duration_cast<std::chrono::microseconds>(
//             data[1].first - data.front().first).count() / 1000000.0; 
//         std::vector<double> last_v(7, 0.0); 
//         std::vector<double> last_q = data[1].second; 
//         std::vector<double> last_last_q = data.front().second; 

//         for (int i = 0; i < 7; ++i) {
//             current_v[i] = (current_q[i] - last_q[i]) / duration_now; 
//             last_v[i] = (last_q[i] - last_last_q[i]) / duration_last; 

//             if (largest_v < std::abs(current_v[i])) { largest_v = std::abs(current_v[i]); }

//             current_a[i] = (current_v[i] - last_v[i]) / duration_now; 

//             if (largest_a < std::abs(current_a[i])) { largest_a = std::abs(current_a[i]); }

//             if (current_v[i] > 10.0) { current_v[i] = 10.0; } 
//             if (current_a[i] > 10.0) { current_a[i] = 10.0; } 


//         }
//     }

//     std::vector<std::vector<double>> res;
//     res.push_back(current_q); 
//     res.push_back(current_v); 
//     res.push_back(current_a); 
//     return res;
// }


double RizonKinematics::get_manipulability() {
    return torch::linalg_det(jaco_eff.matmul(torch::transpose(jaco_eff, 0, 1))).item<double>();
}
