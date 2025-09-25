#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <torch/torch.h>


#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <flexiv_msgs/msg/robot_states.hpp>

#include "/home/rvc/colcon_ws/utils/rizon_kinematics/src/rizon_kinematics.hpp"


using namespace std::chrono_literals;

class FlexivMesh : public rclcpp::Node
{
public:
    FlexivMesh()
    : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("/mesh", 10);
        publisher_2 = this->create_publisher<flexiv_msgs::msg::RobotStates>("/robot_states", 10);
        timer_ = this->create_wall_timer(
            2s, std::bind(&FlexivMesh::timer_callback, this)); 

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

            meshes[link_id] = torch::empty({0, 4}, rk.tensor_options); 

            for(unsigned int j = 0; j < scene->mNumMeshes; j++){ 

                aiMesh* mesh = scene->mMeshes[j];

                auto vertices = torch::empty({mesh->mNumVertices, 4}, rk.tensor_options); 

                std::cout << link_id << ": " << mesh->mNumVertices << std::endl; 

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
                meshes[link_id] = torch::cat({meshes[link_id], vertices}, 0);
            } 

        }
        
        torch::Tensor C1 = torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                                          {0.0, 1.0, 0.0, 0.0},
                                          {0.0,  0.0, 1.0, -0.21}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[1] = torch::matmul(meshes[1], torch::transpose(C1, 0, 1)); 

        torch::Tensor C2 = torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                                          {0.0, 0.0, -1.0, 0.0},
                                          {0.0, 1.0, 0.0, -0.035}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[2] = torch::matmul(meshes[2], torch::transpose(C2, 0, 1)); 

        torch::Tensor C3 = torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                                          {0.0, 1.0, 0.0, 0.0},
                                          {0.0, 0.0, 1.0, -0.19}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[3] = torch::matmul(meshes[3], torch::transpose(C3, 0, 1)); 

        torch::Tensor C4 = torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                                          {0.0, 0.0, -1.0, 0.0},
                                          {0.0, 1.0, 0.0, -0.025}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[4] = torch::matmul(meshes[4], torch::transpose(C4, 0, 1)); 

        torch::Tensor C5 = torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                                          {0.0, 1.0, 0.0, 0.0},
                                          {0.0, 0.0, 1.0, -0.19}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[5] = torch::matmul(meshes[5], torch::transpose(C5, 0, 1)); 

        torch::Tensor C6 = torch::tensor({{0.0, 0.0, 1.0, 0.0}, 
                                          {1.0, 0.0, 0.0, 0.0},
                                          {0.0, 1.0, 0.0, -0.07}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[6] = torch::matmul(meshes[6], torch::transpose(C6, 0, 1)); 

        torch::Tensor C7 = torch::tensor({{1.0, 0.0, 0.0, 0.0}, 
                                          {0.0, 1.0, 0.0, 0.0},
                                          {0.0, 0.0, 1.0, 0.055}, 
                                          {0.0, 0.0, 0.0, 1.0}}, rk.tensor_options); 
        meshes[7] = torch::matmul(meshes[7], torch::transpose(C7, 0, 1));  
    }

private:
    void timer_callback()
    {

        sensor_msgs::msg::PointCloud message = sensor_msgs::msg::PointCloud(); 

        flexiv_msgs::msg::RobotStates states = flexiv_msgs::msg::RobotStates(); 
        states.q = {0.5, -0.67, 1.0, 0.4, -1.1, 2.1, 0.6}; 

        publisher_2->publish(states); 

        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "mesh"; 

        std::vector<double> pos = {0.5, -0.67, 1.0, 0.4, -1.1, 2.1, 0.6}; 
        pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        rk.update(pos); 

        // Link 0 
        torch::Tensor link_mesh = meshes[0]; 
        auto accessor = link_mesh.accessor<float, 2>();
        int64_t num_points = link_mesh.size(0);
        std::cout << "link 0: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link0_points;
        link0_points.reserve(num_points); 


        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link0_points.emplace_back(point);
        }

        // Link 1 
        torch::Tensor T1 = rk.T_adjacent[0]; 
        // link_mesh = torch::transpose(torch::matmul(T1, torch::transpose(meshes[1], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[1], torch::transpose(T1, 0, 1)); 
        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 1: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link1_points;
        link1_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link1_points.emplace_back(point);
        }

        // Link 2 
        torch::Tensor T2 = torch::matmul(T1, rk.T_adjacent[1]); 
        // link_mesh = torch::transpose(torch::matmul(T2, torch::transpose(meshes[2], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[2], torch::transpose(T2, 0, 1)); 

        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 2: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link2_points;
        link2_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link2_points.emplace_back(point);
        }

        // Link 3 
        torch::Tensor T3 = torch::matmul(T2, rk.T_adjacent[2]); 
        // link_mesh = torch::transpose(torch::matmul(T3, torch::transpose(meshes[3], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[3], torch::transpose(T3, 0, 1)); 

        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 3: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link3_points;
        link3_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link3_points.emplace_back(point);
        } 

        // Link 4 
        torch::Tensor T4 = torch::matmul(T3, rk.T_adjacent[3]); 
        // link_mesh = torch::transpose(torch::matmul(T4, torch::transpose(meshes[4], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[4], torch::transpose(T4, 0, 1)); 

        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 4: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link4_points;
        link4_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link4_points.emplace_back(point);
        }

        // Link 5 
        torch::Tensor T5 = torch::matmul(T4, rk.T_adjacent[4]); 
        // link_mesh = torch::transpose(torch::matmul(T5, torch::transpose(meshes[5], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[5], torch::transpose(T5, 0, 1)); 

        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 5: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link5_points;
        link5_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link5_points.emplace_back(point);
        }

        // Link 6 
        torch::Tensor T6 = torch::matmul(T5, rk.T_adjacent[5]); 
        // link_mesh = torch::transpose(torch::matmul(T6, torch::transpose(meshes[6], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[6], torch::transpose(T6, 0, 1)); 

        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 6: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link6_points;
        link6_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link6_points.emplace_back(point);
        }

        // Link 7 
        torch::Tensor T7 = torch::matmul(T6, rk.T_adjacent[6]); 
        // link_mesh = torch::transpose(torch::matmul(T7, torch::transpose(meshes[7], 0, 1)), 0, 1); 
        link_mesh = torch::matmul(meshes[7], torch::transpose(T7, 0, 1)); 

        accessor = link_mesh.accessor<float, 2>();
        num_points = link_mesh.size(0);
        std::cout << "link 7: " << num_points << std::endl; 

        std::vector<geometry_msgs::msg::Point32> link7_points;
        link7_points.reserve(num_points); 

        for (int64_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Point32 point;
            point.x = accessor[i][0];
            point.y = accessor[i][1];
            point.z = accessor[i][2];
            link7_points.emplace_back(point);
        }

        std::vector<geometry_msgs::msg::Point32> all_link_points;
        all_link_points.reserve(link0_points.size() + link1_points.size() + link2_points.size() 
            + link3_points.size() + link4_points.size() + link5_points.size() 
            + link6_points.size() + link7_points.size()); 

        all_link_points.insert(all_link_points.end(), link0_points.begin(), link0_points.end()); 
        all_link_points.insert(all_link_points.end(), link1_points.begin(), link1_points.end());
        all_link_points.insert(all_link_points.end(), link2_points.begin(), link2_points.end());
        all_link_points.insert(all_link_points.end(), link3_points.begin(), link3_points.end());
        all_link_points.insert(all_link_points.end(), link4_points.begin(), link4_points.end());
        all_link_points.insert(all_link_points.end(), link5_points.begin(), link5_points.end());
        all_link_points.insert(all_link_points.end(), link6_points.begin(), link6_points.end());
        all_link_points.insert(all_link_points.end(), link7_points.begin(), link7_points.end());

        message.points = link7_points; 

        publisher_->publish(message);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_; 
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr publisher_2; 

    std::map<int, torch::Tensor> meshes; 
    RizonKinematics rk = RizonKinematics(); 
};

int main(int argc, char * argv[])
{
    // RizonKinematics rk = RizonKinematics(); 

    // std::vector<double> pos = std::vector<double>(7, 0.0); 
    
    // rk.update(pos); 

    // std::cout <<  rk.get_flange_pose() << std::endl; 

    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<FlexivMesh>()); 
    // rclcpp::shutdown();

    torch::Device device = torch::kCUDA;

    std::vector<double> spend_time; 

    double loop_num = 1000;

    for (int i = 0; i < 100; ++i) {

        torch::Tensor mesh_points = (torch::rand({30000, 3}, torch::TensorOptions().device(device).dtype(torch::kFloat32))) * 4.0 + 2.0; 
        torch::Tensor obstacle_points = (torch::rand({2000, 3}, torch::TensorOptions().device(device).dtype(torch::kFloat32))) * 4.0 + 2.0; 


        std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now(); 
        for (int j = 0; j < loop_num; j++) {
            int64_t N = obstacle_points.size(0); 
            int64_t M = mesh_points.size(0); 

            auto diff = obstacle_points.unsqueeze(1) - mesh_points.unsqueeze(0);  // [N, M, 3]
            auto dist_sq = diff.pow(2).sum(-1);          // shape [N, M] 

            auto flat_dist_sq = dist_sq.view({-1});    // [N*M] 

            auto result = flat_dist_sq.min(0);    // returns {values, indices} 

            float min_distance = std::get<0>(result).item<float>(); 
            int64_t flat_idx = std::get<1>(result).item<int64_t>(); 

            int64_t index_1 = flat_idx / M; 
            int64_t index_2 = flat_idx % M; 

        }

        std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now(); 
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / loop_num; 
        
        std::cout << duration/loop_num << std::endl; 

        spend_time.push_back(duration/loop_num); 

    }

    double sum = std::accumulate(spend_time.begin(), spend_time.end(), 0.0); 
    double avg = sum / spend_time.size();
    std::cout << "average: " << avg << std::endl; 
    double maxVal = *std::max_element(spend_time.begin(), spend_time.end()); 
    double minVal = *std::min_element(spend_time.begin(), spend_time.end());
    std::cout << "max: " << maxVal << std::endl; 
    std::cout << "min: " << minVal << std::endl; 
    std::cout <<  avg - minVal << std::endl; 
    std::cout <<  maxVal - avg << std::endl; 


    return 0;
}