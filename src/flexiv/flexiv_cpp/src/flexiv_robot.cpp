#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <fstream>
#include <cmath>

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

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


using namespace std::chrono_literals;

void start_robot(flexiv::Robot &robot, flexiv::RobotStates &robotStates, flexiv::Log &log);
 
class FlexivRobot : public rclcpp::Node 
{
    public: 
        // Constructor 
        FlexivRobot(std::string nodeName, flexiv::Robot &new_robot, 
                  flexiv::RobotStates &new_robotStates, flexiv::Log &new_log): 
                  Node(nodeName), robot(new_robot), robotStates(new_robotStates), log(new_log)
        {
            std::vector<double> init_pos{0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0};
            real_time_msg = std::make_shared<flexiv_msgs::msg::ModeController>(); 
            for (int i =0; i < 7; i++) {
                real_time_msg->positions[i] = init_pos[i]; 
            }

            client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            timer2_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // Publish robot states to ros2 topic "robot_states".
            states_publisher_ = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);
            states_pub_timer_ = this->create_wall_timer(1ms, std::bind(&FlexivRobot::states_callback, this), 
                timer_cb_group_);
            real_time_timer_ = this->create_wall_timer(1ms, std::bind(&FlexivRobot::real_time_callback, this), 
                timer2_cb_group_);
            // Define a subscription option which runs subscription in a new thread.
            auto sub_opt = rclcpp::SubscriptionOptions(); 
            sub_opt.callback_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            auto sub_opt_2 = rclcpp::SubscriptionOptions(); 
            sub_opt_2.callback_group = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive); 
            // Subscribe mode control commands from ros2 topic "mode_controller".
            controller_subscription_ = this->create_subscription<flexiv_msgs::msg::ModeController>(
                "mode_controller", 10, std::bind(&FlexivRobot::mode_callback, this, std::placeholders::_1), 
                sub_opt);
            // Subscribe mode control commands from ros2 topic "motioncapture" 
            motioncapture_subscription_ = this->create_subscription<flexiv_msgs::msg::RigidBody>(
                "motioncapture", 10, std::bind(&FlexivRobot::motioncapture_callback, this, std::placeholders::_1), 
                sub_opt_2);
            // Write to a digital output port on the control box. 
            digital_outpit_subscription_ = this->create_subscription<flexiv_msgs::msg::DigitalOutput>(
                "digital_output", 10, std::bind(&FlexivRobot::digital_output_callback, this, std::placeholders::_1)); 

            mode_service = this->create_service<flexiv_msgs::srv::SetMode>("set_mode", 
                std::bind(&FlexivRobot::set_mode, this, std::placeholders::_1, std::placeholders::_2), 
                rmw_qos_profile_services_default, client_cb_group_); 
            // Start the robot. 
            if (!robot.isOperational())
                start_robot(robot, robotStates, log);
            else 
                log.info("Robot is already operational. ");

        }
        //BasePos: <x,y,z,qw,qx,qy,qz>
        std::vector<std::vector<double>> Get_T(std::vector<double> BasePos_)
        {
            double x = BasePos_[0];
            double y = BasePos_[1];
            double z = BasePos_[2];
            double qw = BasePos_[3];
            double qx = BasePos_[4];
            double qy = BasePos_[5];
            double qz = BasePos_[6];
            std::vector<std::vector<double>> T_ = {{1-2*qy*qy-2*qz*qz, 2*qx*qy+2*qw*qz,2*qx*qz-2*qw*qy,-x }
                                                    ,{2*qx*qy-2*qw*qz,1-2*qx*qx-2*qz*qz, 2*qy*qz+2*qw*qx,-y}
                                                    ,{2*qx*qz+2*qw*qy,2*qy*qz-2*qw*qx,1-2*qx*qx-2*qy*qy,-z}
                                                    ,{0,0,0,1}};
            return T_;
        }
    private: 
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
            states.header.frame_id = "base"; 
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
            states.digital_input_0 = robot.readDigitalInput(0); 
            // Publish the message. 
            states_publisher_->publish(states); 

            /*
            Eigen::Vector3d extForce = {robotStates.extWrenchInBase[0], 
                robotStates.extWrenchInBase[1], robotStates.extWrenchInBase[2]};
            if (extForce.norm() > 25) { robot.stop(); } 
            for (const auto& v : robotStates.tauExt) { if (fabs(v) > 20) { robot.stop(); } }
            */
        } 

        inline void real_time_callback() 
        {
            if (is_real_time_position) 
            {
                robot.getRobotStates(robotStates);
                std::vector<double> targetPos(7, 0.0);
                for (size_t i = 0; i < targetPos.size(); i++) { 
                    
                    targetPos[i] = robotStates.q[i] + real_time_msg->velocities[i];
                }

                std::vector<double> targetVel(real_time_msg->velocities.begin(), real_time_msg->velocities.end());
                std::vector<double> targetAcc(real_time_msg->accelerations.begin(), real_time_msg->accelerations.end());  
                robot.streamJointPosition(targetPos, targetVel, targetAcc); 
            }
        }

        /** Callback function: subscribe and operate control commands. */
        void mode_callback(const flexiv_msgs::msg::ModeController::SharedPtr msg) 
        {
            // Monitor fault on robot server. 
            if (robot.isFault()) {
                throw flexiv::ServerException("Fault occurred on robot server, exiting ...");
            } 
            if (msg->mode != states.mode) {
                robot.setMode(flexiv::Mode(msg->mode));
                std::cout << "changing mode to " << msg->mode << " ... " << std::endl; 
                return; 
            }
            // Operate control commands based on control mode. 
            switch (msg->mode) 
            {
                // Run real-time joint torque control to track continuous commands. 
                case flexiv::Mode::RT_JOINT_TORQUE: 
                {   
                    std::vector<double> targetTorque(msg->torques.begin(), msg->torques.end()); 
                    robot.streamJointTorque(targetTorque, msg->gravity_comp, msg->soft_limits);
                } break; 
                // Run real-time joint position control to track continuous commands. 
                case flexiv::Mode::RT_JOINT_POSITION: 
                {
                    real_time_msg = msg; 
                    // is_real_time_position = true; 
                    std::vector<double> targetPos(msg->positions.begin(), msg->positions.end());
                    std::vector<double> targetVel(msg->velocities.begin(), msg->velocities.end());
                    std::vector<double> targetAcc(msg->accelerations.begin(), msg->accelerations.end());  
                    robot.streamJointPosition(targetPos, targetVel, targetAcc); 

                } break; 
                // Run non-real-time joint position control to track discrete commands (smoothened by internal motion generator). 
                case flexiv::Mode::NRT_JOINT_POSITION: 
                {
                    std::vector<double> targetPos(msg->positions.begin(), msg->positions.end());
                    std::vector<double> targetVel(msg->velocities.begin(), msg->velocities.end());
                    std::vector<double> targetAcc(msg->accelerations.begin(), msg->accelerations.end());
                    std::vector<double> maxVel(msg->max_vel.begin(), msg->max_vel.end()); 
                    std::vector<double> maxAcc(msg->max_acc.begin(), msg->max_acc.end()); 
                    robot.sendJointPosition(targetPos, targetVel, targetAcc, maxVel, maxAcc); 
                } break; 
                // Execute pre-configured robot task plans. 
                case flexiv::Mode::NRT_PLAN_EXECUTION:
                {
                    flexiv::PlanInfo planInfo;
                    if (msg->plan_index == -1) 
                    {
                        auto planList = robot.getPlanNameList(); 
                        for (size_t i = 0; i < planList.size(); i++) 
                            std::cout << "[" << i << "] " << planList[i] << std::endl; 
                        std::cout << std::endl; 
                        break; 
                    }
                    else 
                        robot.executePlan(msg->plan_index); 
                    while (robot.isBusy()) 
                    { 
                        robot.getPlanInfo(planInfo); 
                        log.info("==============================================="); 
                        std::cout << planInfo << std::endl;
                        std::this_thread::sleep_for(1s);  
                    } 
                    std::cout << "Finished executing your plan!" << std::endl; 
                    std::this_thread::sleep_for(10s);  

                } break; 
                // 	Execute robot primitives (unit skills). 
                case flexiv::Mode::NRT_PRIMITIVE_EXECUTION: 
                {
                    robot.executePrimitive(msg->primitive_name.data); 
                    std::this_thread::sleep_for(3s); 
                    std::cout << "Finished executing your primitive!" << std::endl; 

                } break; 
                // // Run real-time Cartesian motion-force control to track continuous commands in base frame. 
                // case flexiv::Mode::RT_CARTESIAN_MOTION_FORCE_BASE: 
                // {
                //     std::vector<double> targetTcpPose = {msg->tcp_pose.position.x, 
                //         msg->tcp_pose.position.y, msg->tcp_pose.position.z, 
                //         msg->tcp_pose.orientation.w, msg->tcp_pose.orientation.x, 
                //         msg->tcp_pose.orientation.y, msg->tcp_pose.orientation.z}; 
                //     robot.streamCartesianMotionForce(targetTcpPose);
                // } break; 
                // // Run non-real-time Cartesian motion-force control to track discrete commands (smoothened by internal motion generator) in base frame. 
                // case flexiv::Mode::NRT_CARTESIAN_MOTION_FORCE_BASE: 
                // {
                //     std::vector<double> targetTcpPose = {msg->tcp_pose.position.x, 
                //         msg->tcp_pose.position.y, msg->tcp_pose.position.z, 
                //         msg->tcp_pose.orientation.w, msg->tcp_pose.orientation.x, 
                //         msg->tcp_pose.orientation.y, msg->tcp_pose.orientation.z}; 
                //     robot.sendCartesianMotionForce(targetTcpPose);
                // } break; 
                // Default: do nothing. 
                default: 
                {
                    ;
                }
            }
        }
        /** Callback function: subscribe and save motioncapture information in MC. */
        void motioncapture_callback(const flexiv_msgs::msg::RigidBody::SharedPtr msg)
        {
            std::copy(msg->center_loc.begin(), msg->center_loc.end(), Rigid_cam.begin());
        }

        void digital_output_callback(const flexiv_msgs::msg::DigitalOutput::SharedPtr msg)
        {
            for (auto digital_state : (msg->digital_output)) 
            {
                robot.writeDigitalOutput(digital_state.pin, digital_state.state); 
            }
            
        }
        /** Callback function: subscribe and write a digital output on the control box. */
        void set_mode(const std::shared_ptr<flexiv_msgs::srv::SetMode::Request> request, 
                        std::shared_ptr<flexiv_msgs::srv::SetMode::Response> response) 
        {
            if (robot.getMode() != request->mode) {
                robot.setMode(flexiv::Mode(request->mode));
            } 
            response->success = true; 
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
        //<x,y,z,qw,qx,qy,qz> bewteen Rigid_cam to cam_on_robot_coordinate.
        std::vector<double> R2cam = {0.030 ,-0.100, 0.040, 0.000 ,0.000, 0.000 ,0.000};
        std::vector<double> Rigid_cam;
        // flexiv_msgs::msg::ModeController mode_controller;
        // flexiv_msgs::msg::ModeController::SharedPtr  mode_controller;

        rclcpp::TimerBase::SharedPtr states_pub_timer_;
        rclcpp::TimerBase::SharedPtr real_time_timer_; 
        rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr states_publisher_;
        rclcpp::Subscription<flexiv_msgs::msg::ModeController>::SharedPtr controller_subscription_;
        rclcpp::Subscription<flexiv_msgs::msg::RigidBody>::SharedPtr motioncapture_subscription_;
        rclcpp::Subscription<flexiv_msgs::msg::DigitalOutput>::SharedPtr digital_outpit_subscription_;
        rclcpp::Service<flexiv_msgs::srv::SetMode>::SharedPtr mode_service; 
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_; 
        rclcpp::CallbackGroup::SharedPtr timer2_cb_group_;
        flexiv_msgs::msg::RobotStates states;
        std::shared_ptr<flexiv_msgs::msg::ModeController> real_time_msg; 
        bool is_real_time_position = false; 
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
 
void start_node(int argc, char * argv[]) 
{
    std::cout << "start" << std::endl;
    flexiv::Robot robot("192.168.2.100", "192.168.2.101"); 
    flexiv::RobotStates robotStates;
    flexiv::Log log;
    rclcpp::init(argc, argv);  
    rclcpp::executors::MultiThreadedExecutor executor; 
    auto flexiv_robot = std::make_shared<FlexivRobot>("flexiv_robot", robot, robotStates, log); 
    executor.add_node(flexiv_robot); 
    executor.spin();

    // rclcpp::spin(std::make_shared<FlexivRobot>("flexiv_robot", robot, robotStates, log)); 

    rclcpp::shutdown();
}

int main(int argc, char * argv[]) 
{
    std::thread my_node(start_node, argc, argv); 
    my_node.join(); 
    return 0;
}
