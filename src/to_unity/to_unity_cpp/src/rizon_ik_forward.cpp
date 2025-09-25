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

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


using namespace std::chrono_literals;

class RizonIKForward : public rclcpp::Node 
{
    public: 
        RizonIKForward() : Node("rizon_ik_forward") 
        {
            client_cb_group_ = this->create_callback_group(
                                    rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_cb_group_ = this->create_callback_group(
                                    rclcpp::CallbackGroupType::MutuallyExclusive);
            subsrciber_cb_group_ = this->create_callback_group(
                                        rclcpp::CallbackGroupType::Reentrant); 
            auto sub_opt = rclcpp::SubscriptionOptions(); 
            sub_opt.callback_group = subsrciber_cb_group_; 

            mode_controller_pub = this->create_publisher<flexiv_msgs::msg::ModeController>("mode_controller", 10);
            
            //
            // robot_states_pub = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);
            //

            // mode_controller_timer = this->create_wall_timer(1ms, 
            //                         std::bind(&RizonIKForward::mode_controller_callback, this), 
            //                         timer_cb_group_);

            target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "target_pose", 10, 
                std::bind(&RizonIKForward::target_pose_callback, this, std::placeholders::_1), 
                sub_opt);

            mode_client = this->create_client<flexiv_msgs::srv::SetMode>(
                        "set_mode", 
                        rmw_qos_profile_services_default, 
                        client_cb_group_); 

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
            // this->get_parameter("robot_description", urdf_xml);

            std::string filename = "urdf_xml.txt";
            std::ifstream inFile(filename);
            if (inFile.is_open()) {
                // 使用 stringstream 读取整个文件内容到一个字符串
                std::stringstream buffer;
                buffer << inFile.rdbuf();

                // 将内容存储到一个字符串中
                urdf_xml = buffer.str();

                // 关闭文件流
                inFile.close();
            }


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

            KDL::ChainFkSolverPos_recursive fk_solver(chain); 
            KDL::Frame link_pose;
            fk_solver.JntToCart(*nominal, link_pose);
            std::cout << "here"; 
            RCLCPP_INFO(this->get_logger(), "Using %f, %f, %f ", link_pose.p[0], link_pose.p[1], link_pose.p[2]); 
            for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
                KDL::Segment segment = chain.getSegment(i);
                KDL::Frame pose = segment.getFrameToTip();
                RCLCPP_INFO(this->get_logger(), "Link: %s", segment.getName()); 
                RCLCPP_INFO(this->get_logger(), "Position: %f, %f, %f ", pose.p[0], pose.p[1], pose.p[2]); 
    }


            (*nominal)(0) = 0.0;
            (*nominal)(1) = -0.698132;  // -40 degree
            (*nominal)(2) = 0.0;
            (*nominal)(3) = 1.570796;   // 90 degree
            (*nominal)(4) = 0.0;
            (*nominal)(5) = 0.698132;   // 40 degree
            (*nominal)(6) = 0.0; 

            // (*nominal)(0) = 0.0;
            // (*nominal)(1) = 0.0;  // -40 degree
            // (*nominal)(2) = 0.0;
            // (*nominal)(3) = 0.0;   // 90 degree
            // (*nominal)(4) = 0.0;
            // (*nominal)(5) = 0.0;   // 40 degree
            // (*nominal)(6) = 0.0; 

            result = std::make_shared<KDL::JntArray>(chain.getNrOfJoints()); 
            (*result) = (*nominal);

            RCLCPP_INFO(this->get_logger(), "Using %d joints", chain.getNrOfJoints());  
            RCLCPP_INFO(this->get_logger(), "Initial joint state: \n[%lf, %lf, %lf, %lf, %lf, %lf, %lf]", 
                (*nominal)(0), (*nominal)(1), (*nominal)(2), (*nominal)(3), (*nominal)(4), (*nominal)(5), (*nominal)(6)); 
        
            // SetMode at start 
            auto request = std::make_shared<flexiv_msgs::srv::SetMode::Request>();
            request->mode = request->MODE_NRT_JOINT_POSITION; 
            auto result_future = mode_client->async_send_request(request); 
            std::future_status status = result_future.wait_for(1s); 
            if (status == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "Success in setting mode!");
            } 
            std::this_thread::sleep_for(std::chrono::seconds(3));

            flexiv_msgs::msg::ModeController msg = flexiv_msgs::msg::ModeController();  
                msg.mode = msg.MODE_NRT_JOINT_POSITION;
                for (unsigned int i=0; i < nominal->data.size(); i++) 
                {
                    msg.positions[i] = (*nominal)(i);
                    msg.max_vel[i] = 1; 
                    msg.max_acc[i] = 1;
                }
                mode_controller_pub->publish(msg);
                mode_controller_pub->publish(msg);

            //
        
        
        }

    private: 
        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_sub)
        {
            KDL::Frame end_effector_pose;
            double L = 0.0; 
            if (msg_sub->header.frame_id == "controller") {
                L = 0.04;
            }
            else if (msg_sub->header.frame_id == "hanoi") {
                L = 0.0; 
            } 
            else { return; } 
            end_effector_pose = this->ROS2KDLWithShift(msg_sub, L);

            int rc = tracik_solver_ptr->CartToJnt(*nominal, end_effector_pose, *result); 

            if (rc >= 0) {
                *nominal = *result; 
                // 
                flexiv_msgs::msg::ModeController msg = flexiv_msgs::msg::ModeController();  
                msg.mode = msg.MODE_NRT_JOINT_POSITION;
                for (unsigned int i=0; i < result->data.size(); i++) 
                {
                    msg.positions[i] = (*result)(i);
                    msg.max_vel[i] = 3; 
                    msg.max_acc[i] = 2;
                }
                mode_controller_pub->publish(msg);
                //
                // auto msg_pub = flexiv_msgs::msg::RobotStates();
                // for (unsigned int i=0; i < result->data.size(); i++) {
                //     msg_pub.q[i] = (*result)(i);
                // }
                // robot_states_pub->publish(msg_pub);

            }
            else{
                RCLCPP_INFO(this->get_logger(), "No solution!");
            }
        }

        void mode_controller_callback() 
        {
            flexiv_msgs::msg::ModeController msg = flexiv_msgs::msg::ModeController();  
            msg.mode = msg.MODE_NRT_JOINT_POSITION;
            for (unsigned int i=0; i < result->data.size(); i++) 
            {
                msg.positions[i] = (*result)(i);
                msg.max_vel[i] = 3; 
                msg.max_acc[i] = 2;
            }

            mode_controller_pub->publish(msg);
        }

        
        /*inline KDL::Frame coord_unity2ros(const geometry_msgs::msg::PoseStamped::SharedPtr &msg, const double l = 0.0) {
            KDL::Rotation unity_rot = KDL::Rotation::Quaternion(
                msg->pose.orientation.x, msg->pose.orientation.y, 
                msg->pose.orientation.z, msg->pose.orientation.w);   
            KDL::Rotation T(0, 0, 1, -1, 0, 0, 0, 1, 0); 
            KDL::Rotation ros_rot = (T * unity_rot) * T;

            KDL::Vector ros_pos(msg->pose.position.z + l * unity_rot(2, 0), 
                -(msg->pose.position.x + l * unity_rot(0, 0)), msg->pose.position.y + l * unity_rot(1, 0)); 

            return KDL::Frame(ros_rot, ros_pos);
        }*/

        inline KDL::Frame ROS2KDLWithShift(const geometry_msgs::msg::PoseStamped::SharedPtr &msg, double L) {
            KDL::Rotation rot = KDL::Rotation::Quaternion(
                msg->pose.orientation.x, msg->pose.orientation.y, 
                msg->pose.orientation.z, msg->pose.orientation.w);

            KDL::Vector pos = KDL::Vector(msg->pose.position.x + L * rot(0, 2), 
            msg->pose.position.y + L * rot(1, 2), msg->pose.position.z + L * rot(2, 2)); 
            return KDL::Frame(rot, pos);
        }

        rclcpp::Publisher<flexiv_msgs::msg::ModeController>::SharedPtr mode_controller_pub;
        rclcpp::TimerBase::SharedPtr mode_controller_timer; 
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub; 
        std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver_ptr;
        std::shared_ptr<KDL::JntArray> nominal;
        std::shared_ptr<KDL::JntArray> result;
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_; 
        rclcpp::CallbackGroup::SharedPtr subsrciber_cb_group_; 
        rclcpp::Client<flexiv_msgs::srv::SetMode>::SharedPtr mode_client;
        rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr robot_states_pub;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RizonIKForward>());
    rclcpp::shutdown();
    return 0;
}

