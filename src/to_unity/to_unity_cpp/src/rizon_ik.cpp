#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "flexiv_msgs/msg/robot_states.hpp" 

#include "trac_ik/trac_ik.hpp" 
#include <urdf/model.h> 
#include <kdl_parser/kdl_parser.hpp>

using namespace std::chrono_literals;

class RizonIK : public rclcpp::Node 
{
    public: 
        RizonIK() : Node("rizon_ik") 
        {
            robot_states_pub = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);

            target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "target_pose", 10, std::bind(&RizonIK::target_pose_callback, this, std::placeholders::_1));
            
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

            fk_solver_ptr = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain); // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

            // Set default initial joint state at the home position [0, -40, 0, 90, 0, 40, 0] in degree. 
            nominal = std::make_shared<KDL::JntArray>(chain.getNrOfJoints());
            (*nominal)(0) = 0;
            (*nominal)(1) = -40.0 / 180.0 * 3.1415926;
            (*nominal)(2) = 0.0;
            (*nominal)(3) = 90.0 / 180.0 * 3.1415926;
            (*nominal)(4) = 0.0;
            (*nominal)(5) = 40.0 / 180.0 * 3.1415926;
            (*nominal)(6) = 0.0;

            result = std::make_shared<KDL::JntArray>(chain.getNrOfJoints()); 

            RCLCPP_INFO(this->get_logger(), "Using %d joints", chain.getNrOfJoints());  
            RCLCPP_INFO(this->get_logger(), "Initial joint state: \n[%lf, %lf, %lf, %lf, %lf, %lf, %lf]", 
                (*nominal)(0), (*nominal)(1), (*nominal)(2), (*nominal)(3), (*nominal)(4), (*nominal)(5), (*nominal)(6)); 

            auto msg_pub = flexiv_msgs::msg::RobotStates();
            robot_states_pub->publish(msg_pub);
            robot_states_pub->publish(msg_pub);
            robot_states_pub->publish(msg_pub);


        }

    private: 
        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_sub)
        {
            // double l = 0.04;
            // KDL::Frame end_effector_pose = this->coord_unity2ros(msg_sub, l); 

            KDL::Vector pos(msg_sub->pose.position.x, msg_sub->pose.position.y, msg_sub->pose.position.z); 
            KDL::Rotation rot = KDL::Rotation::Quaternion(
                msg_sub->pose.orientation.x, msg_sub->pose.orientation.y, 
                msg_sub->pose.orientation.z, msg_sub->pose.orientation.w);  
            KDL::Frame end_effector_pose(rot, pos); 

            int rc = tracik_solver_ptr->CartToJnt(*nominal, end_effector_pose, *result); 

            if (rc >= 0) {
                std::cout << "solved" << std::endl;
                auto msg_pub = flexiv_msgs::msg::RobotStates();
                for (unsigned int i=0; i < result->data.size(); i++) {
                    msg_pub.q[i] = (*result)(i);       
                    std::cout << (*result)(i) << " ";          
                }
                std::cout << std::endl;
                KDL::Frame temp_pose = KDL::Frame(); 
                fk_solver_ptr->JntToCart(*result, temp_pose); 
                std::cout << temp_pose.p.data[0] << " " << temp_pose.p.data[1] 
                << " " << temp_pose.p.data[2] << std::endl;
                msg_pub.flange_pose.position.x = msg_sub->pose.position.x; 
                msg_pub.flange_pose.position.y = msg_sub->pose.position.y; 
                msg_pub.flange_pose.position.z = msg_sub->pose.position.z; 

                robot_states_pub->publish(msg_pub);
                *nominal = *result; 
            }
            else{
                RCLCPP_INFO(this->get_logger(), "No solution!");
            }
        }

        
        inline KDL::Frame coord_unity2ros(const geometry_msgs::msg::PoseStamped::SharedPtr &msg, const double l = 0.0) {
            KDL::Rotation unity_rot = KDL::Rotation::Quaternion(
                msg->pose.orientation.x, msg->pose.orientation.y, 
                msg->pose.orientation.z, msg->pose.orientation.w);   
            KDL::Rotation T(0, 0, 1, -1, 0, 0, 0, 1, 0); 
            KDL::Rotation ros_rot = (T * unity_rot) * T;

            KDL::Vector ros_pos(msg->pose.position.z + l * unity_rot(2, 0), 
                -(msg->pose.position.x + l * unity_rot(0, 0)), msg->pose.position.y + l * unity_rot(1, 0)); 

            return KDL::Frame(ros_rot, ros_pos);
        }

        rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr robot_states_pub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub; 
        std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver_ptr; 
        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_ptr; 
        std::shared_ptr<KDL::JntArray> nominal;
        std::shared_ptr<KDL::JntArray> result;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RizonIK>());
    rclcpp::shutdown();
    return 0;
}

