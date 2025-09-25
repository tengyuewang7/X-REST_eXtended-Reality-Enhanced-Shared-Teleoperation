#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <cmath>
#include <torch/torch.h>

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>
#include <flexiv/Model.hpp>

#include "rclcpp/rclcpp.hpp"
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


#include "../libtorch/src/flexiv_kinematics.hpp" 
#include "/home/rvc/colcon_ws/utils/rizon_nlopt_ik/src/rizon_nlopt_ik.hpp"


using namespace std::chrono_literals;

void start_robot(flexiv::Robot &robot, flexiv::RobotStates &robotStates, flexiv::Log &log);


class FlexivUnityNRT : public rclcpp::Node 
{
    public: 
        // Constructor 
        FlexivUnityNRT(std::string nodeName, flexiv::Robot &new_robot, 
                  flexiv::RobotStates &new_robotStates, flexiv::Log &new_log, RizonNloptIK &new_ik) : 
                  Node(nodeName), robot(new_robot), robotStates(new_robotStates), log(new_log), rizon_nlopt_ik(new_ik)
        {
            timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Publish robot states to ros2 topic "robot_states".
            states_publisher_ = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);
            states_pub_timer_ = this->create_wall_timer(1ms, std::bind(&FlexivUnityNRT::states_callback, this), 
                timer_cb_group_); 

            auto sub_opt = rclcpp::SubscriptionOptions(); 
            sub_opt.callback_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "target_pose", 10, std::bind(&FlexivUnityNRT::target_pose_callback, this, std::placeholders::_1), sub_opt);
            

            control_ik_timer_ = this->create_wall_timer(0.95ms, std::bind(&FlexivUnityNRT::control_loop, this), 
                timer2_cb_group_);

            obstacles.push_back(std::vector<double>{7, 0.0, 4});
            obstacles.push_back(std::vector<double>{5, 0.0, 4});
            rizon_nlopt_ik.kinematics->update_obstacles(obstacles); 
            
            init_robot(); 
        }

        void on_shutdown() {
            ; 
        }

        void init_robot() {
            // Start the robot. 
            if (!robot.isOperational())
                start_robot(robot, robotStates, log);
            else 
                log.info("Robot is already operational. ");
            
            // Move home. 
            robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
            robot.executePrimitive("Home()");
            while (robot.isBusy()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            robot.setMode(flexiv::Mode::NRT_JOINT_POSITION); 

            robot.getRobotStates(robotStates); 
            init_pose = std::vector<double> {robotStates.flangePose[0], robotStates.flangePose[1],
                                             robotStates.flangePose[2], robotStates.flangePose[4], 
                                             robotStates.flangePose[5], robotStates.flangePose[6],
                                             robotStates.flangePose[3]};

            target_pose = init_pose;
            std::array<double, 7> temp;
            std::copy(robotStates.q.begin(), robotStates.q.end(), temp.begin());
            rizon_nlopt_ik.kinematics->update(temp); 

            robot.writeDigitalOutput(0, true); 
            robot.writeDigitalOutput(1, false); 
            gripper_state = false; 
            // startTime = std::chrono::system_clock::now();

            for (int i = 0; i < 300; ++i) {
                rizon_nlopt_ik.kinematics->update(rizon_nlopt_ik.kinematics->current_q); 
                rizon_nlopt_ik.delta_theta = std::vector<double>(7, 0.0); 
                auto desired_pose = std::vector<double>{0.3, 0.2, 0.6, 0, 1, 0, 0}; 
                auto u = rizon_nlopt_ik.get_ik(desired_pose);
            }
        }
 
    
    private: 
        void control_loop() {

            if (is_manual_ptr == nullptr) {
                std::this_thread::sleep_for(std::chrono::seconds(1)); 
                return; 
            }

            auto increment = std::vector<double>(7, 0.0);

            if (*is_manual_ptr) {
                control_mode = "manual"; 
                // increment = rizon_nlopt_ik.get_trac_ik_u(target_pose); 
                increment = rizon_nlopt_ik.kinematics->control_ik(target_pose); 
            }
            else {
                bool large_distance = (torch::norm(rizon_nlopt_ik.kinematics->dpose).item<double>() > 0.02);
                bool dange_zone = (rizon_nlopt_ik.kinematics->min_obstacle_distance < 0.1); 

                if (large_distance && dange_zone) {
                    control_mode = "smooth"; 
                    increment = rizon_nlopt_ik.get_ik(target_pose); 
                    // increment = rizon_nlopt_ik.kinematics->control_ik(target_pose); 
                    // increment = rizon_nlopt_ik.get_trac_ik_u(target_pose); 
                    // std::cout << "nlopt" << std::endl;
                }
                else if (large_distance && !dange_zone)
                {
                    control_mode = "smooth"; 
                    increment = rizon_nlopt_ik.get_ik(target_pose); 
                    // increment = rizon_nlopt_ik.kinematics->control_ik(target_pose); 
                    // increment = rizon_nlopt_ik.get_trac_ik_u(target_pose); 
                    // std::cout << "nlopt" << std::endl;
                }
                else if (!large_distance && dange_zone) {
                    control_mode = "avoiding"; 
                    // increment = rizon_nlopt_ik.get_ik(target_pose); 
                    std::cout << "here" << std::endl; 
                    increment = rizon_nlopt_ik.kinematics->control_ik(target_pose); 
                    // increment = rizon_nlopt_ik.get_trac_ik_u(target_pose); 
                    // std::cout << "control_ik" << std::endl;
                }
                else if (!large_distance && !dange_zone) {
                    std::cout << "tracking" << std::endl; 
                    control_mode = "tracking";
                    // increment = rizon_nlopt_ik.get_ik(target_pose); 
                    increment = rizon_nlopt_ik.kinematics->control_ik(target_pose); 
                    // increment = rizon_nlopt_ik.get_trac_ik_u(target_pose); 
                    // std::cout << "trac_ik" << std::endl;
                }
                std::cout << control_mode << std::endl;
            }

            auto maxIt = std::max_element(increment.begin(), increment.end(), [](double a, double b) {
                return std::abs(a) < std::abs(b);
            });
            double maxAbs = std::abs(*maxIt); 
            if (maxAbs > 0.005) { 
                double scale = 0.005 / maxAbs; 
                for (auto& val : increment) {
                    val *= scale;
                }
            }

            robot.getRobotStates(robotStates); 
            std::vector<double> targetPos(robotStates.q);
            for (size_t i = 0; i < targetPos.size(); i++) { 
                targetPos[i] += increment[i]; 
            }
            std::vector<double> targetVel(7, 0.0);
            std::vector<double> targetAcc(7, 0.0);  
            std::vector<double> maxVel(7, 3.0);  
            std::vector<double> maxAcc(7, 2.0);  
            
            robot.sendJointPosition(targetPos, targetVel, targetAcc, maxVel, maxAcc); 
        }

        void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_sub)
        {
            std::string encoded_msg = msg_sub->header.frame_id;
            size_t plus_sign = encoded_msg.find('+'); 
            std::string gripper_msg = encoded_msg.substr(0, plus_sign); 
            std::string manual_msg = encoded_msg.substr(plus_sign + 1); 

            if (gripper_msg == "true" && gripper_state == false) { 
                // Turn on gripper. 
                // robot.writeDigitalOutput(1, true); 
            }
            else if (gripper_msg == "false" && gripper_state == true) { 
                // Turn off gripper. 
                // robot.writeDigitalOutput(1, false); 
            }
            
            // If the control mode is manual. 
            if (manual_msg == "manual") { is_manual_ptr = std::make_shared<bool>(true); }
            else if (manual_msg == "shared") { is_manual_ptr = std::make_shared<bool>(false); } 

            // Read the target pose, [px, py, pz, rx, ry, rz, rw] 
            target_pose[0] = msg_sub->pose.position.x; 
            target_pose[1] = msg_sub->pose.position.y; 
            target_pose[2] = msg_sub->pose.position.z; 
            target_pose[3] = msg_sub->pose.orientation.x; 
            target_pose[4] = msg_sub->pose.orientation.y; 
            target_pose[5] = msg_sub->pose.orientation.z; 
            target_pose[6] = msg_sub->pose.orientation.w; 
        }

        /** Callback function: read robot states from "robot.getRobotStates(robotStates);", 
            *wrap as flexiv_msgs::msg::RobotStates() and publish. */ 
        inline void states_callback() 
        {
            // Initialize a message.
            // auto states = flexiv_msgs::msg::RobotStates(); 
            // Inquire robot states.
            robot.getRobotStates(robotStates); 
            // Assign state information to message. 
            // See https://rdk.flexiv.com/api/structflexiv_1_1_robot_states.html#aaa3a586fee2204554c7ad2ce591f135e for more details. 
            states.header.stamp = this->get_clock()->now();
            states.header.frame_id = control_mode; // tell the control mode of the controller 
            states.mode = robot.getMode();
            std::copy(robotStates.q.begin(), robotStates.q.end(), states.q.begin());
            std::copy(robotStates.theta.begin(), robotStates.theta.end(), states.theta.begin());
            std::copy(robotStates.dq.begin(), robotStates.dq.end(), states.dq.begin());
            std::copy(robotStates.dtheta.begin(), robotStates.dtheta.end(), states.dtheta.begin());
            std::copy(robotStates.tau.begin(), robotStates.tau.end(), states.tau.begin());
            std::copy(robotStates.tauDes.begin(), robotStates.tauDes.end(), states.tau_des.begin());
            std::copy(robotStates.tauDot.begin(), robotStates.tauDot.end(), states.tau_dot.begin());
            std::copy(robotStates.tauExt.begin(), robotStates.tauExt.end(), states.tau_ext.begin());
            this->copy_pose(robotStates.tcpPose, states.tcp_pose);
            this->copy_pose(robotStates.tcpPoseDes, states.tcp_pose_des);
            this->copy_vel(robotStates.tcpVel, states.tcp_vel);
            this->copy_pose(robotStates.camPose, states.cam_pose); 
            this->copy_pose(robotStates.flangePose, states.flange_pose); 
            this->copy_wrench(robotStates.ftSensorRaw, states.ft_sensor_raw); 
            this->copy_wrench(robotStates.extWrenchInTcp, states.ext_wrench_tcp_frame); 
            this->copy_wrench(robotStates.extWrenchInBase, states.ext_wrench_base_frame); 
            // states.digital_input_0 = (!robot.readDigitalInput(0)); 
            // Publish the message. 
            states_publisher_->publish(states); 
            // Update flexiv kinematics. 
            rizon_nlopt_ik.kinematics->update(states.q);
            // model.updateModel(robotStates.q, robotStates.dq);
            // model.getJacobian("flange");
            // std::cout << states.header.stamp.nanosec << std::endl;
            // std::cout << kinematics.jaco_eff << std::endl;
            // std::cout << model.getJacobian("flange") << std::endl;

        } 
 
        inline void copy_pose(std::vector<double> &s1, decltype(flexiv_msgs::msg::RobotStates().tcp_pose) &s2) 
        {
            s2.position.x = s1[0];
            s2.position.y = s1[1];
            s2.position.z = s1[2];
            s2.orientation.w = s1[3];
            s2.orientation.x = s1[4];
            s2.orientation.y = s1[5];
            s2.orientation.z = s1[6];
        }
        inline void copy_vel(std::vector<double> &s1, decltype(flexiv_msgs::msg::RobotStates().tcp_vel) &s2) 
        {
            s2.linear.x = s1[0]; 
            s2.linear.y = s1[1]; 
            s2.linear.z = s1[2]; 
            s2.angular.x = s1[3]; 
            s2.angular.y = s1[4]; 
            s2.angular.z = s1[5]; 
        }     
        inline void copy_wrench(std::vector<double> &s1, decltype(flexiv_msgs::msg::RobotStates().ft_sensor_raw) &s2) 
        {
            s2.force.x = s1[0]; 
            s2.force.y = s1[1]; 
            s2.force.z = s1[2]; 
            s2.torque.x = s1[3]; 
            s2.torque.y = s1[4]; 
            s2.torque.z = s1[5]; 
        }
        
        flexiv::Robot &robot; 
        flexiv::RobotStates &robotStates;
        flexiv::Log &log;
        RizonNloptIK &rizon_nlopt_ik;
        rclcpp::TimerBase::SharedPtr states_pub_timer_;
        rclcpp::TimerBase::SharedPtr control_ik_timer_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_; 
        rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr states_publisher_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_; 
        rclcpp::CallbackGroup::SharedPtr timer2_cb_group_;
        flexiv_msgs::msg::RobotStates states;

        std::vector<std::vector<double>> obstacles; 

        std::vector<double> init_pose;
        // std::chrono::system_clock::time_point startTime;
        std::map<std::string, std::vector<std::vector<double>>> stored_data;
        std::vector<double> target_pose;    // [px, py, pz, rx, ry, rz, rw]  
        std::string control_mode;    // manual, avoiding, tracking, smooth 
        std::shared_ptr<bool> is_manual_ptr; 
        bool gripper_state; 
};

void start_robot(flexiv::Robot &robot, flexiv::RobotStates &robotStates, flexiv::Log &log) {
    robot.getRobotStates(robotStates);
    log.info(
    "Initial TCP pose set to [position 3x1, rotation (quaternion) "
    "4x1]: "
    + flexiv::utility::vec2Str(robotStates.tcpPose));
    try {
        // RDK Initialization
        //=============================================================================
        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return ;
            }
            log.info("Fault on robot server is cleared");
        }
    
        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");
        
        /*
        robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        while (robot.getMode() != flexiv::Mode::NRT_PRIMITIVE_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.warn(
            "Calibrating force/torque sensors, please don't touch the robot");
        robot.executePrimitive("CaliForceSensor()");
        // Wait for primitive completion
        log.warn("Calibrating force/torque sensors, please don't touch the robot");
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Calibration complete");
        */

    }
    catch (const flexiv::Exception& e) {
        log.error(e.what());
        return ;
    }
}


int main(int argc, char * argv[]) 
{
    flexiv::Robot robot("192.168.2.100", "192.168.2.101"); 
    flexiv::RobotStates robotStates;
    flexiv::Log log;
    // flexiv::Model model(robot);

    rclcpp::init(argc, argv);  
    rclcpp::executors::MultiThreadedExecutor executor; 
    RizonNloptIK rizon_nlopt_ik;
    auto flexiv_unity_nrt = std::make_shared<FlexivUnityNRT>("flexiv_robot", robot, robotStates, log, rizon_nlopt_ik); 
    executor.add_node(flexiv_unity_nrt); 
    executor.spin();
    flexiv_unity_nrt->on_shutdown();
    rclcpp::shutdown(); 

    
    return 0;
}
