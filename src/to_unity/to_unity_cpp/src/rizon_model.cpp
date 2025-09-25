#include <iomanip>
#include <iostream>

#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <cmath> 
#include <algorithm>
#include <thread> 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "flexiv_msgs/msg/robot_states.hpp" 
#include "flexiv_msgs/msg/hanoi_grab.hpp"
#include "flexiv_msgs/msg/hanoi_pose.hpp" 
#include "flexiv_msgs/msg/robot_states.hpp"
#include "flexiv_msgs/msg/mode_controller.hpp"
#include "flexiv_msgs/srv/set_mode.hpp" 

#include "trac_ik/trac_ik.hpp" 

using namespace std::chrono_literals;

class RizonIKForward : public rclcpp::Node 
{
    public: 
        RizonIKForward() : Node("rizon_ik_forward") 
        {
            this->declare_parameter<double>("timeout", 0.005);
            this->declare_parameters<std::string>(
                std::string(),
                std::map<std::string, std::string> {
                    {"chain_start", std::string()},
                    {"chain_end", std::string()},
                    {"robot_description", std::string()},
                }
            );

            std::string chain_start;
            std::string chain_end;
            std::string urdf_xml;
            double timeout;

            this->get_parameter("timeout", timeout);
            this->get_parameter("chain_start", chain_start);
            this->get_parameter("chain_end", chain_end);
            this->get_parameter("robot_description", urdf_xml);

            if (chain_start.empty() || chain_end.empty()) {
                RCLCPP_FATAL(this->get_logger(), "Missing chain info in launch file");
                exit(-1);
            }
            double eps = 1e-5; // error
            tracik_solver_ptr = std::make_shared<TRAC_IK::TRAC_IK>(chain_start, chain_end, urdf_xml, timeout, eps);

            KDL::JntArray ll, ul; // lower joint limits, upper joint limits
            KDL::Chain chain;


            // Check if KDL chain, lower joint limits and upper joint limits are valid. 
            if (!(tracik_solver_ptr->getKDLChain(chain)) || !(tracik_solver_ptr->getKDLLimits(ll, ul))) {
                RCLCPP_ERROR(this->get_logger(), "There are no valid KDL chain or ll/ul found");
            }

            // Set default initial joint state at the home position [0, -40, 0, 90, 0, 40, 0] in degree. 
            nominal = std::make_shared<KDL::JntArray>(chain.getNrOfJoints()); 

            fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
            jntToJacSolver = std::make_shared<KDL::ChainJntToJacSolver>(chain);

            // KDL::Frame link_pose;
            // KDL::Jacobian jac(chain.getNrOfJoints());
            // for (uint i=0; i <= chain.getNrOfJoints()+1; ++i) {
            //     fk_solver->JntToCart(*nominal, link_pose, i);
            //     RCLCPP_INFO(this->get_logger(), "Using %f, %f, %f ", link_pose.p[0], link_pose.p[1], link_pose.p[2]); 
            // }

            // for (uint i=0; i <= chain.getNrOfJoints()+1; ++i) {
            //     RCLCPP_INFO(this->get_logger(),"\n"); 
            //     jntToJacSolver->JntToJac(*nominal, jac, i);
            //     std::stringstream ss;
            //     ss << std::fixed << std::setprecision(3) << jac.data;
            //     RCLCPP_INFO(this->get_logger(), ss.str().c_str()); 
            //     }
        } 

        void update_joint_frames() {
            KDL::Frame temp_frame;
            for (uint i=1; i < 9; ++i) {
                fk_solver->JntToCart(*current_q, temp_frame, i); 
                (*joint_point_ptr)[i] = temp_frame.p;
            }
        }
            


    private: 
        std::shared_ptr<KDL::JntArray> current_q = std::make_shared<KDL::JntArray>(7);
        std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver_ptr;

        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver; 
        std::shared_ptr<KDL::ChainJntToJacSolver> jntToJacSolver;

        std::shared_ptr<KDL::JntArray> nominal;
        std::shared_ptr<std::array<KDL::Vector, 8>> joint_point_ptr;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RizonIKForward>());
    rclcpp::shutdown();
    return 0;
}

