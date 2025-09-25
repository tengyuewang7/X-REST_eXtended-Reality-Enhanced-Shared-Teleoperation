#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <torch/torch.h>

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>
#include <flexiv/Model.hpp>
#include <flexiv/Scheduler.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "flexiv_msgs/msg/robot_states.hpp" 
#include "flexiv_msgs/msg/joint_pos_vel.hpp" 
#include "flexiv_msgs/msg/mode.hpp" 
#include "flexiv_msgs/msg/mode_controller.hpp" 
#include "flexiv_msgs/msg/rigid_body.hpp"
#include "flexiv_msgs/msg/digital_state.hpp"
#include "flexiv_msgs/msg/digital_output.hpp"
#include "flexiv_msgs/msg/digital_input.hpp"
#include "flexiv_msgs/msg/marker.hpp"
#include "flexiv_msgs/srv/set_mode.hpp" 

#include "/home/rvc/colcon_ws/utils/rizon_ik_solver/src/rizon_ik_solver.hpp"


using namespace std::chrono_literals;

class UnityDecisionMaking : public rclcpp::Node 
{
    public: 
        UnityDecisionMaking(std::string nodeName = "unity_decision_making"): Node(nodeName)
        {
            robot_states_sub = this->create_subscription<flexiv_msgs::msg::RobotStates>(
                                    "robot_states", 
                                    10, 
                                    std::bind(&UnityDecisionMaking::robot_states_callback, this, std::placeholders::_1));

            update_timer = this->create_wall_timer(
                10ms, std::bind(&UnityDecisionMaking::update, this));

            target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        } 

    private: 

        void robot_states_callback(const flexiv_msgs::msg::RobotStates::SharedPtr msg) 
        {
            robot_states = msg; 
        } 

        void update() {
            static std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
            std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000000.0;

            auto target_pose = geometry_msgs::msg::PoseStamped(); 

            // // Linear
            // if (duration < 20) {
            //     target_pose.pose.position.x = 0.4; 
            //     target_pose.pose.position.y = -0.4; 
            //     target_pose.pose.position.z = 0.3; 
            //     target_pose.pose.orientation.x = 0.0; 
            //     target_pose.pose.orientation.y = 1.0; 
            //     target_pose.pose.orientation.z = 0.0; 
            //     target_pose.pose.orientation.w = 0.0; 
            // }
            // else {
            //     target_pose.pose.position.x = 0.65; 
            //     double y = -0.4 + 0.10 * (duration - 20); 
            //     if (y > 0.4) { y = 0.4; }
            //     target_pose.pose.position.y = y; 
            //     target_pose.pose.position.z = 0.3; 
            //     target_pose.pose.orientation.x = 0.0; 
            //     target_pose.pose.orientation.y = 1.0; 
            //     target_pose.pose.orientation.z = 0.0; 
            //     target_pose.pose.orientation.w = 0.0; 
            // }

            // Circular
            double center_x = 0.6, center_y = 0.0; 
            double start_time = 6 * M_PI; 
            if (duration < start_time) {
                target_pose.pose.position.x = center_x; 
                target_pose.pose.position.y = center_y; 
                target_pose.pose.position.z = 0.3; 
                target_pose.pose.orientation.x = 0.0; 
                target_pose.pose.orientation.y = 1.0; 
                target_pose.pose.orientation.z = 0.0; 
                target_pose.pose.orientation.w = 0.0; 
            }
            else {
                target_pose.pose.position.x = center_x + 0.15 * cos(2 * M_PI / (7) * (duration - start_time)); 
                target_pose.pose.position.y = center_y - 0.15 * sin(2 * M_PI / (7) * (duration - start_time)); 
                target_pose.pose.position.z = 0.3; 
                target_pose.pose.orientation.x = 0.0; 
                target_pose.pose.orientation.y = 1.0; 
                target_pose.pose.orientation.z = 0.0; 
                target_pose.pose.orientation.w = 0.0; 
            }
            
            std::cout << target_pose.pose.position.x << " " <<
                target_pose.pose.position.y << " " << std::endl; 

            target_pose.header.frame_id = "false+shared"; // Do not suck.
            target_pose.header.stamp.sec = 10; 
            target_pose_pub->publish(target_pose);

        }

        flexiv_msgs::msg::RobotStates::SharedPtr robot_states;
        rclcpp::Subscription<flexiv_msgs::msg::RobotStates>::SharedPtr robot_states_sub;
        rclcpp::TimerBase::SharedPtr update_timer;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub;


};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv); 

    rclcpp::executors::MultiThreadedExecutor executor; 
    auto unity_decision_making = std::make_shared<UnityDecisionMaking>("unity_decision_making"); 
    executor.add_node(unity_decision_making); 
    executor.spin();
    rclcpp::shutdown();


    return 0;

}

