#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <cmath> 
#include <algorithm>
#include <thread> 

#include "rclcpp/rclcpp.hpp"
#include "flexiv_msgs/msg/hanoi_states.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "std_msgs/msg/string.hpp" 


using namespace std::chrono_literals;

class UnityDecisionMaking : public rclcpp::Node 
{
    public: 
        UnityDecisionMaking(std::string nodeName = "unity_decision_making"): Node(nodeName)
        {
            hanoi.push_back(Hanoi()); 
            hanoi.push_back(Hanoi()); 
            hanoi.push_back(Hanoi()); 

            hanoi_states_sub = this->create_subscription<flexiv_msgs::msg::HanoiStates>(
                                            "hanoi_states", 
                                            10, 
                                            std::bind(&UnityDecisionMaking::hanoi_states_callback, this, std::placeholders::_1));

            robot_states_sub = this->create_subscription<flexiv_msgs::msg::RobotStates>(
                                    "robot_states", 
                                    10, 
                                    std::bind(&UnityDecisionMaking::robot_states_callback, this, std::placeholders::_1));

            is_manual_sub = this->create_subscription<std_msgs::msg::String>(
                                    "is_manual", 
                                    10, 
                                    std::bind(&UnityDecisionMaking::is_manual_callback, this, std::placeholders::_1));

            endeffector_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                                    "endeffector_pose", 
                                    10, 
                                    std::bind(&UnityDecisionMaking::endeffector_pose_callback, this, std::placeholders::_1));

            update_timer = this->create_wall_timer(
                10ms, std::bind(&UnityDecisionMaking::update, this));

            target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        } 

    private: 
        void hanoi_states_callback(const flexiv_msgs::msg::HanoiStates::SharedPtr msg) 
        {
            for (int i = 0; i < 3; ++i) {
                hanoi[i].grab_unity = msg->is_grab[i]; 
                hanoi[i].current_pose = msg->current_pose[i]; 
                hanoi[i].static_pose = msg->static_pose[i]; 
            }
        }

        void robot_states_callback(const flexiv_msgs::msg::RobotStates::SharedPtr msg) 
        {
            robot_states = msg; 
        } 

        void is_manual_callback(const std_msgs::msg::String::SharedPtr msg) 
        {
            if (msg->data == "manual") { is_manual = true; } 
            else if (msg->data == "shared") { is_manual = false; }
        }

        void endeffector_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
        {
            endeffector_pose_ptr = msg; 
        }

        void update() {
            if (is_manual) {
                auto target_pose = *endeffector_pose_ptr; 
                target_pose.header.stamp = this->get_clock()->now();
                target_pose.header.frame_id = target_pose.header.frame_id + "+manual"; 
                // std::cout << target_pose.header.frame_id << std::endl;
                target_pose_pub->publish(target_pose); 
                return; 
            }
            for (auto &hanoi_disk : hanoi) {
                if (hanoi_disk.grab_unity == true and hanoi_disk.grab_robot == false) {
                    // Condition 1: Go and grab the disk
                    auto target_pose = up_in_z(hanoi_disk.static_pose, offset_l + 0.05); 
                    target_pose.header.stamp = this->get_clock()->now();
                    if (fabs(robot_states->flange_pose.position.x - target_pose.pose.position.x) < 0.005 
                     && fabs(robot_states->flange_pose.position.y - target_pose.pose.position.y) < 0.005
                     && fabs(robot_states->flange_pose.position.z - target_pose.pose.position.z) < 0.005) 
                    {
                        condition_1_flag_1 = true; 
                    }

                    if (condition_1_flag_1 == false) {
                        target_pose.header.frame_id = "false+shared"; // Do not suck.
                        target_pose_pub->publish(target_pose);
                        // std::cout << "moving to higher loc" << std::endl;
                        // std::cout << target_pose.header.frame_id << std::endl;
                        break; 
                    }
                    else {
                        auto target_pose = up_in_z(hanoi_disk.static_pose, offset_l + 0.004);
                        target_pose.header.stamp = this->get_clock()->now();
                        target_pose.header.stamp.sec = 0; 
                        if (fabs(robot_states->flange_pose.position.x - target_pose.pose.position.x) < 0.005 
                         && fabs(robot_states->flange_pose.position.y - target_pose.pose.position.y) < 0.005
                         && fabs(robot_states->flange_pose.position.z - target_pose.pose.position.z) < 0.005) 
                        {
                            condition_1_flag_2 = true; 
                            std::this_thread::sleep_for(2s); 
                        }

                        if (condition_1_flag_2 == false) {
                            target_pose.header.frame_id = "false+shared"; // Do not suck.
                            target_pose_pub->publish(target_pose);
                            // std::cout << "moving to lower loc" << std::endl;
                            // std::cout << target_pose.header.frame_id << std::endl;
                            break; 
                        }
                        else {
                            target_pose.header.frame_id = "true+shared"; // Suck 
                            target_pose_pub->publish(target_pose); 
                            hanoi_disk.grab_robot = true;
                            // std::cout << "suck" << std::endl;
                            // std::cout << target_pose.header.frame_id << std::endl;
                        }
                    }
                    condition_1_flag_1 = false; 
                    condition_1_flag_2 = false; 
                }
                else if (hanoi_disk.grab_unity == true and hanoi_disk.grab_robot == true) {
                    // Condition 2: Track 
                    auto target_pose = up_in_z(hanoi_disk.current_pose, offset_l + 0.005); 
                    target_pose.header.stamp = this->get_clock()->now();
                    target_pose.header.frame_id = "true+shared"; // Suck  
                    target_pose_pub->publish(target_pose); 
                    // std::cout << "tracking" << std::endl;
                    // std::cout << target_pose.header.frame_id << std::endl;
                }
                else if (hanoi_disk.grab_unity == false and hanoi_disk.grab_robot == true) {
                    // Condition 3: Release and lift up 
                    if (condition_3_flag_1 == false) {
                        auto target_pose = up_in_z(hanoi_disk.current_pose, offset_l + 0.005); 
                        target_pose.header.frame_id = "false+shared"; 
                        target_pose_pub->publish(target_pose); 
                        condition_3_flag_1 = true; 
                        std::this_thread::sleep_for(2s);
                        // std::cout << "release" << std::endl;
                        // std::cout << target_pose.header.frame_id << std::endl;
                        break; 
                   }
                    else {
                        auto target_pose = up_in_z(hanoi_disk.current_pose, offset_l + 0.05); 
                        target_pose.header.frame_id = "false+shared"; 
                        target_pose_pub->publish(target_pose); 
                        if (fabs(robot_states->flange_pose.position.x - target_pose.pose.position.x) < 0.005 
                         && fabs(robot_states->flange_pose.position.y - target_pose.pose.position.y) < 0.005
                         && fabs(robot_states->flange_pose.position.z - target_pose.pose.position.z) < 0.005) {
                                condition_3_flag_1 = false;
                                hanoi_disk.grab_robot = false; 
                                // std::cout << "moving up" << std::endl;
                                // std::cout << target_pose.header.frame_id << std::endl;
                        }
                    }
                }  
            }
        }

        geometry_msgs::msg::PoseStamped up_in_z(geometry_msgs::msg::Pose pose, double l) {
            
            geometry_msgs::msg::PoseStamped target_pose = geometry_msgs::msg::PoseStamped(); 
            target_pose.pose = pose;
            double qx = pose.orientation.x;
            double qy = pose.orientation.y;
            double qz = pose.orientation.z;
            double qw = pose.orientation.w;

            // Get z axis of the matrix
            double zX = 2.0 * (qx * qz + qw * qy);
            double zY = 2.0 * (qy * qz - qw * qx);
            double zZ = 2.0 * (qw * qw + qz * qz) - 1.0; 
            target_pose.pose.position.x -= zX * l;
            target_pose.pose.position.y -= zY * l;
            target_pose.pose.position.z -= zZ * l;

            return target_pose;
        }

        struct Hanoi
        {
            bool grab_unity = false; 
            bool grab_robot = false; 
            geometry_msgs::msg::Pose current_pose; 
            geometry_msgs::msg::Pose static_pose;
        }; 

        rclcpp::Subscription<flexiv_msgs::msg::HanoiStates>::SharedPtr hanoi_states_sub; 
        rclcpp::Subscription<flexiv_msgs::msg::RobotStates>::SharedPtr robot_states_sub; 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr is_manual_sub; 
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr endeffector_pose_sub; 

        flexiv_msgs::msg::RobotStates::SharedPtr robot_states;
        rclcpp::TimerBase::SharedPtr update_timer;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub;
        std::vector<Hanoi> hanoi;
        bool is_manual; 
        geometry_msgs::msg::PoseStamped::SharedPtr endeffector_pose_ptr; 
        double offset_l = 0.08;
        bool condition_1_flag_1 = false;   // Make sure the robot has moved to the upper location. 
        bool condition_1_flag_2 = false;   // Make sure the robot has moved to the lower location. 
        bool condition_3_flag_1 = false;   
        bool condition_3_flag_2 = false;


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

