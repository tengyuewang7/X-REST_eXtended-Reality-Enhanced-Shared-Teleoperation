#ifndef RIZON_KINEMATICS_HPP
#define RIZON_KINEMATICS_HPP

#include <torch/torch.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex> 
#include <deque>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

class RizonKinematics {
public:
    RizonKinematics(); 


    // Update functions 
    template <typename Container> 
    void update(const Container&); 

    void update(const torch::Tensor&); 
    void update_adjacent_transformation();
    void update_joint_transformation();
    void update_jaco_eff();

    template <typename Container>
    void update_dpose(const Container&); 

    // IK solvers 
    template <typename Container>
    std::vector<double> control_ik(const Container&); 

    template <typename Container>
    std::vector<double> direct_ik(const Container&); 

    template <typename Container>
    void solve_ik(const Container& desired_pose, uint interation_num = 100, double tolerance = 1e-5); 

    // Obstacle methods 
    void update_obstacles(const std::vector<std::vector<double>>&);
    std::pair<torch::Tensor, torch::Tensor> closest_to_links();

    // Get functions 
    std::vector<double> get_current_q_vector(); 
    std::vector<std::vector<double>> get_current_qva_vector(); 
    std::vector<double> get_flange_pose(); 
    double get_manipulability(); 

    void initialize_meshes(); 
    void update_meshes(); 
    void min_distance_between_obstacle_points_and_point_meshes(); 
    std::pair<torch::Tensor, torch::Tensor> closest_to_links_based_on_meshes(); 


    // Variables 
    const torch::Device device = torch::kCPU;
    const torch::TensorOptions tensor_options = torch::TensorOptions().dtype(torch::kFloat).device(device); 
    const torch::TensorOptions gpu_options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA); 

    std::shared_ptr<torch::Tensor> current_q_ptr; 
    torch::Tensor T_adjacent;
    torch::Tensor T_joint;
    torch::Tensor jaco_eff;
    torch::Tensor dpose;
    torch::Tensor obstacle_points_cpu;
    torch::Tensor obstacle_points_gpu;

    double min_obstacle_distance = 10.0; 

    bool is_using_meshes = false; 
    std::map<int, torch::Tensor> link_meshes; 
    std::vector<int64_t> link_mesh_index = std::vector<int64_t>({0, 4292, 7494, 10506, 13596, 16687, 
        19691, 24615, 29309}); 
    torch::Tensor mesh_points = torch::empty({29309, 3}, gpu_options);  
    std::pair<torch::Tensor, torch::Tensor> closest_point; 

    // // manual adjusted
    // torch::Tensor DH_PARAMS = torch::tensor({ 
    //     {  0.0,      0.365,    0.0,           M_PI       },
    //     {  0.0,      0.065,   -M_PI / 2.0,    0.0        },
    //     {  0.0,      0.395,    M_PI / 2.0,    0.0        },
    //     { -0.02,     0.055,    M_PI / 2.0,    M_PI       },
    //     { -0.02,     0.385,    M_PI / 2.0,    M_PI       },
    //     {  0.0,      0.1,     -M_PI / 2.0,   -M_PI / 2.0 },
    //     {  0.1118,   0.0,      M_PI / 2.0,   -M_PI / 2.0 },
    //     {  0.0,      0.132,    0.0,          -M_PI / 2.0 } 
    // }, torch::dtype(torch::kDouble));  

    // from urdf
    torch::Tensor DH_PARAMS = torch::tensor({ 
        {  0.0,      0.365,    0.0,           M_PI       },
        {  0.0,      0.065,   -M_PI / 2.0,    0.0        },
        {  0.0,      0.395,    M_PI / 2.0,    0.0        },
        { -0.02,     0.055,    M_PI / 2.0,    M_PI       },
        { -0.02,     0.385,    M_PI / 2.0,    M_PI       },
        {  0.0,      0.1,     -M_PI / 2.0,   -M_PI / 2.0 },
        {  0.11,     0.0,      M_PI / 2.0,   -M_PI / 2.0 },
        {  0.0,      0.136,    0.0,          -M_PI / 2.0 } 
    }, tensor_options);  

    // // from original urdf
    // // a d alpha theta
    // torch::Tensor DH_PARAMS = torch::tensor({ 
    //     {  0.0,    0.155,    0.0,           -M_PI / 2.0  },
    //     {  -0.03,    0.21,    0.0,     0.0  },
    //     {  0.0,    0.065,   -M_PI / 2.0,     0.0  },
    //     {  0.02,   0.395,   -M_PI / 2.0,     0.0  },
    //     {  0.02,   0.055,   -M_PI / 2.0,     M_PI  },
    //     {  0.0,    0.385,   -M_PI / 2.0,     0.0  },
    //     {  0.11,   0.1,      M_PI / 2.0,    -M_PI / 2.0  },
    //     {  0.0,    0.136,    M_PI / 2.0,    -M_PI / 2.0  } 
    // }, torch::dtype(torch::kDouble));  
    
    // // optimized
    // torch::Tensor DH_PARAMS = torch::tensor({ 
    //     { 1.6440e-06,  3.6507e-01,  2.7903e-05,  3.1411e+00 },
    //     { 5.7283e-04,  6.5129e-02, -1.5716e+00,  3.2217e-03 },
    //     { 1.9495e-03,  3.9470e-01,  1.5695e+00, -1.6832e-03 },
    //     { -2.1464e-02,  5.5076e-02,  1.5713e+00,  3.1407e+00 },
    //     { -2.0711e-02,  3.8469e-01,  1.5712e+00,  3.1421e+00 },
    //     { 1.6478e-04,  9.9910e-02, -1.5710e+00, -1.5717e+00 },
    //     { 1.1177e-01,  1.1657e-03,  1.5709e+00, -1.5715e+00 },
    //     { 9.0638e-05,  1.3317e-01,  3.4331e-05, -1.5708e+00 } 
    //     }, torch::dtype(torch::kDouble));


    std::map<int, torch::Tensor> mesh_offsets = {
        {0, torch::eye(4, tensor_options)},
        {1, torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                           {0.0, 1.0, 0.0, 0.0},
                           {0.0,  0.0, 1.0, -0.21}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)}, 
        {2, torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                           {0.0, 0.0, -1.0, 0.0},
                           {0.0, 1.0, 0.0, -0.035}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)}, 
        {3, torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                           {0.0, 1.0, 0.0, 0.0},
                           {0.0, 0.0, 1.0, -0.19}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)},
        {4, torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                           {0.0, 0.0, -1.0, 0.0},
                           {0.0, 1.0, 0.0, -0.025}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)}, 
        {5, torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                           {0.0, 1.0, 0.0, 0.0},
                           {0.0, 0.0, 1.0, -0.19}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)}, 
        {6, torch::tensor({{0.0, 0.0, 1.0, 0.0}, 
                           {1.0, 0.0, 0.0, 0.0},
                           {0.0, 1.0, 0.0, -0.07}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)}, 
        {7, torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                           {0.0, 1.0, 0.0, 0.0},
                           {0.0, 0.0, 1.0, 0.055}, 
                           {0.0, 0.0, 0.0, 1.0}}, tensor_options)} 
    };




private:

};

#endif 