#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <cmath> 
#include <algorithm>
#include <thread> 

#include "rclcpp/rclcpp.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"
#include "flexiv_msgs/msg/mode_controller.hpp"
#include "geometry_msgs/msg/pose.hpp" 
#include "flexiv_msgs/srv/set_mode.hpp" 

using namespace std::chrono_literals; 


class RobotTest : public rclcpp::Node 
{
    public: 
        RobotTest(std::string nodeName = "robot_test"): Node(nodeName)
        {
            client_cb_group_ = this->create_callback_group(
                                    rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_cb_group_ = this->create_callback_group(
                                    rclcpp::CallbackGroupType::MutuallyExclusive);
            subsrciber_cb_group_ = this->create_callback_group(
                                        rclcpp::CallbackGroupType::Reentrant); 
            auto sub_opt = rclcpp::SubscriptionOptions(); 
            sub_opt.callback_group = subsrciber_cb_group_; 

            states_subscription_ = this->create_subscription<flexiv_msgs::msg::RobotStates>(
                                            "robot_states", 
                                            10, 
                                            std::bind(&RobotTest::states_callback, this, std::placeholders::_1), 
                                            sub_opt); 

            mode_controller_publisher_ = this->create_publisher<flexiv_msgs::msg::ModeController>(
                                                "mode_controller", 10); 

            mode_controller_timer_ = this->create_wall_timer(1ms, 
                                            std::bind(&RobotTest::mode_controller_callback, this), 
                                            timer_cb_group_);

            mode_client_ = this->create_client<flexiv_msgs::srv::SetMode>(
                                    "set_mode", 
                                    rmw_qos_profile_services_default, 
                                    client_cb_group_); 
        } 

    private: 
        void states_callback(const flexiv_msgs::msg::RobotStates::SharedPtr msg) 
        {
            robot_states_ = msg; 
        }

        void mode_controller_callback() 
        {   
            flexiv_msgs::msg::ModeController msg = flexiv_msgs::msg::ModeController();  

            while (robot_states_->header.stamp.sec < 100) 
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));   
                RCLCPP_INFO(this->get_logger(), "%d", robot_states_->header.stamp.sec); 
            } 

            static std::vector<double> initPos = {robot_states_->q.begin(), robot_states_->q.end()}; 
            static geometry_msgs::msg::Pose initTcpPose(robot_states_->tcp_pose); 
            // std::cout << initPos[0] << " " << initPos[1] << " "
            //     << initPos[2] << " " << initPos[3] << " " << 
            //     initPos[4] << " " << initPos[5] << " "
            //     << initPos[6] << " " << std::endl; 
            // TODO: check initTcpPose 

            const std::vector<double> k_impedanceKp = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0}; 
            const std::vector<double> k_impedanceKd = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};
            const std::vector<double> k_floatingDamping = {100.0, 100.0, 50.0, 50.0, 10.0, 10.0, 10.0}; 

            std::vector<double> currentPos(robot_states_->q.begin(), robot_states_->q.end()); 
            std::vector<double> currentVel(robot_states_->dtheta.begin(), robot_states_->dtheta.end()); 
            // std::cout << currentPos[0] << " " << currentPos[1] << " "
            //     << currentPos[2] << " " << currentPos[3] << " " << 
            //     currentPos[4] << " " << currentPos[5] << " "
            //     << currentPos[6] << " " << std::endl; 
            auto request = std::make_shared<flexiv_msgs::srv::SetMode::Request>();

            // request->mode = request->MODE_RT_JOINT_TORQUE; 
            request->mode = request->MODE_RT_JOINT_POSITION; 
            // request->mode = request->MODE_NRT_JOINT_POSITION; 
            // request->mode = request->MODE_NRT_PLAN_EXECUTION; 
            // request->mode = request->MODE_NRT_PRIMITIVE_EXECUTION; 

            // std::string test_mode = "MODE_RT_CARTESIAN_MOTION_FORCE_BASE"; <== not tested, do NRT version first
            // std::string test_mode = "MODE_NRT_CARTESIAN_MOTION_FORCE_BASE"; <== failed 

            if (robot_states_->mode != request->mode) {
                auto result_future = mode_client_->async_send_request(request); 
                std::future_status status = result_future.wait_for(10s); 
                if (status == std::future_status::ready) {
                    RCLCPP_INFO(this->get_logger(), "Success in setting mode!");
                } 
            }

            static auto start_time = rclcpp::Clock().now().seconds(); 
            // std::cout << rclcpp::Clock().now().seconds() - start_time << std::endl; 

            if (request->mode == flexiv_msgs::msg::ModeController::MODE_RT_JOINT_TORQUE) 
            {
                msg.mode = msg.MODE_RT_JOINT_TORQUE; 
                for (size_t i = 0; i < 7; i++) 
                    msg.torques[i] = -k_floatingDamping[i] * currentVel[i]; 
            }

            else if (request->mode == flexiv_msgs::msg::ModeController::MODE_RT_JOINT_POSITION) 
            {
                msg.mode = msg.MODE_RT_JOINT_POSITION; 
                auto travelling_time = rclcpp::Clock().now().seconds() - start_time;
                 
                for (size_t i = 0; i < 7; i++) 
                    msg.positions[i] = initPos[i] + 0.05 * sin(2 * M_PI * 0.3 * travelling_time);
                // RCLCPP_INFO(this->get_logger(), "travelling_time:'%f", travelling_time );
                //RCLCPP_INFO(this->get_logger(), "positions:'%f'", msg.positions[0]);
            }

            else if (request->mode == flexiv_msgs::msg::ModeController::MODE_NRT_JOINT_POSITION) 
            {
                msg.mode = msg.MODE_NRT_JOINT_POSITION; 
                auto travelling_time = rclcpp::Clock().now().seconds() - start_time; 
                for (size_t i = 0; i < 7; i++) 
                    msg.positions[i] = initPos[i] + 0.05 * sin(2 * M_PI * 0.3 * travelling_time); 
                std::for_each(msg.max_vel.begin(), msg.max_vel.end(), [](double &value){ value = 1.0; }); 
                std::for_each(msg.max_acc.begin(), msg.max_acc.end(), [](double &value){ value = 1.0; }); 
            } 

            else if (request->mode == flexiv_msgs::msg::ModeController::MODE_NRT_PLAN_EXECUTION) 
            {
                msg.mode = msg.MODE_NRT_PLAN_EXECUTION; 
                msg.plan_index = 14; 
            } 
             
            else if (request->mode == flexiv_msgs::msg::ModeController::MODE_NRT_PRIMITIVE_EXECUTION) 
            {
                msg.mode = msg.MODE_NRT_PRIMITIVE_EXECUTION; 
                msg.primitive_name.data = "Home()"; 
                // msg.primitive_name.data = "ZeroFTSensor()"; 
            }

        // //     else if (test_mode == "MODE_RT_CARTESIAN_MOTION_FORCE_BASE") 
        // //     {
        // //         msg.mode = msg.MODE_RT_CARTESIAN_MOTION_FORCE_BASE; 
        // //         auto travelling_time = rclcpp::Clock().now().seconds() - start_time; 
        // //         msg.tcp_pose.position.y = initTcpPose.position.y + 0.1 * sin(2 * M_PI * 0.3 * travelling_time);
        // //     }

        // //     else if (test_mode == "MODE_NRT_CARTESIAN_MOTION_FORCE_BASE") 
        // //     {
        // //         msg.mode = msg.MODE_NRT_CARTESIAN_MOTION_FORCE_BASE; 
        // //         auto travelling_time = rclcpp::Clock().now().seconds() - start_time; 
        // //         msg.tcp_pose.position.y = initTcpPose.position.y + 0.1 * sin(2 * M_PI * 0.3 * travelling_time);
        // //     }

        // //     RCLCPP_INFO(this->get_logger(), "Using " + test_mode + " ..."); 

            mode_controller_publisher_->publish(msg); 

        }

        rclcpp::Subscription<flexiv_msgs::msg::RobotStates>::SharedPtr states_subscription_; 
        rclcpp::Publisher<flexiv_msgs::msg::ModeController>::SharedPtr mode_controller_publisher_; 
        rclcpp::TimerBase::SharedPtr mode_controller_timer_; 
        flexiv_msgs::msg::RobotStates states_ = flexiv_msgs::msg::RobotStates(); 
        flexiv_msgs::msg::RobotStates::SharedPtr robot_states_ = std::make_shared<flexiv_msgs::msg::RobotStates>(states_); 
        rclcpp::Client<flexiv_msgs::srv::SetMode>::SharedPtr mode_client_;
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_; 
        rclcpp::CallbackGroup::SharedPtr subsrciber_cb_group_; 

};

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv); 

    rclcpp::executors::MultiThreadedExecutor executor; 
    auto robot_test = std::make_shared<RobotTest>(); 
    executor.add_node(robot_test); 
    executor.spin();

    // rclcpp::spin(std::make_shared<RobotTest>());
    rclcpp::shutdown();
    return 0;
}
