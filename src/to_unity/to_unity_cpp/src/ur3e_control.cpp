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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "trac_ik/trac_ik.hpp" 

using namespace std::chrono_literals;

class UR3eControl : public rclcpp::Node 
{
    public: 
        UR3eControl() : Node("ur3e_control") 
        {
            joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("ik_joint_state", 10);
            pendant_robot_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
            pendant_robot_timer = this->create_wall_timer(1s, std::bind(&UR3eControl::pendant_robot_timer_callback, this));

            target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "target_pose", 10, 
                std::bind(&UR3eControl::target_pose_callback, this, std::placeholders::_1));

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

            nominal = std::make_shared<KDL::JntArray>(chain.getNrOfJoints());
            (*nominal)(0) = -91.71 / 180 * 3.1415926;
            (*nominal)(1) = -98.96 / 180 * 3.1415926;
            (*nominal)(2) = -126.22 / 180 * 3.1415926;
            (*nominal)(3) = -46.29 / 180 * 3.1415926;
            (*nominal)(4) = 91.39 / 180 * 3.1415926;
            (*nominal)(5) = 358.22 / 180 * 3.1415926;

            result = std::make_shared<KDL::JntArray>(chain.getNrOfJoints()); 
            // (*result) = (*nominal);

            RCLCPP_INFO(this->get_logger(), "Using %d joints", chain.getNrOfJoints());  
            RCLCPP_INFO(this->get_logger(), "Initial joint state: \n[%lf, %lf, %lf, %lf, %lf, %lf]", 
                (*nominal)(0), (*nominal)(1), (*nominal)(2), (*nominal)(3), (*nominal)(4), (*nominal)(5)); 
        
        }

    private: 

        void pendant_robot_timer_callback()
        {
            auto message = trajectory_msgs::msg::JointTrajectory();
            message.joint_names = {"elbow_joint", "shoulder_lift_joint", 
                "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto new_point = trajectory_msgs::msg::JointTrajectoryPoint();

            new_point.positions = {(*nominal)(2), (*nominal)(1), (*nominal)(0), 
                (*nominal)(3), (*nominal)(4), (*nominal)(5)};;
            new_point.time_from_start.sec = 1;
            
            message.points = {new_point}; 
            
            pendant_robot_pub->publish(message);
        }

        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_sub)
        {
            KDL::Frame end_effector_pose;
            end_effector_pose = this->ROS2KDLWithShift(msg_sub, 0.0);

            int rc = tracik_solver_ptr->CartToJnt(*nominal, end_effector_pose, *result); 

            if (rc >= 0) {
                *nominal = *result;          
                auto msg_pub = sensor_msgs::msg::JointState();
                
                msg_pub.position = {(*result)(0), (*result)(1), (*result)(2), (*result)(3), (*result)(4), (*result)(5)};

                joint_states_pub->publish(msg_pub);

            }
            else{
                RCLCPP_INFO(this->get_logger(), "No solution!");
            }
        }

        inline KDL::Frame ROS2KDLWithShift(const geometry_msgs::msg::PoseStamped::SharedPtr &msg, double L) {
            KDL::Rotation rot = KDL::Rotation::Quaternion(
                msg->pose.orientation.x, msg->pose.orientation.y, 
                msg->pose.orientation.z, msg->pose.orientation.w);

            KDL::Vector pos = KDL::Vector(msg->pose.position.x + L * rot(1, 2), 
            msg->pose.position.y + L * rot(2, 2), msg->pose.position.z + L * rot(0, 2)); 
            return KDL::Frame(rot, pos);
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub; 
        std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver_ptr;
        std::shared_ptr<KDL::JntArray> nominal;
        std::shared_ptr<KDL::JntArray> result;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub; 
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pendant_robot_pub; 
        rclcpp::TimerBase::SharedPtr pendant_robot_timer;



};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR3eControl>());
    rclcpp::shutdown();
    return 0;
}

