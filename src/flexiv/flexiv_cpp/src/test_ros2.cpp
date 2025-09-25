#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <cmath>

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


using namespace std::chrono_literals;

class TestROS2 : public rclcpp::Node 
{
public: 
    TestROS2(
        std::string nodeName, flexiv::Robot& new_robot, 
        flexiv::RobotStates& new_robot_states, flexiv::Log& new_log) 
        : Node(nodeName), robot(new_robot), 
            robot_states(new_robot_states), log(new_log)
    {
        // Initialize callback groups 
        timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        timer2_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions(); 
        sub_opt.callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive); 

        // Timer used to update kinematics 
        control_ik_timer_ = this->create_wall_timer(10ms, std::bind(&TestROS2::control_loop, this), timer2_cb_group);

    }


    /** @brief Execute once at the end of this ROS2 node */
    void on_shutdown() {
    }
private: 

    void control_loop() {
        robot.getRobotStates(robot_states); 

        flexiv_msgs::msg::RobotStates states; 

        std::copy(robot_states.q.begin(), robot_states.q.end(), states.q.begin()); 

        states_publisher_->publish(states); 
    }

    // Private member variables 
    //==================================================================================================
    // Reference variables 
    flexiv::Robot& robot; 
    flexiv::RobotStates& robot_states;
    flexiv::Log& log; 

    // ROS2 timer, subscriber, publisher and message 
    rclcpp::TimerBase::SharedPtr control_ik_timer_;
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr states_publisher_;

    // ROS2 timer, subscriber, publisher and message 
    rclcpp::TimerBase::SharedPtr states_pub_timer_;

    flexiv_msgs::msg::RobotStates states;

    // Callback groups 
    rclcpp::CallbackGroup::SharedPtr timer_cb_group;
    rclcpp::CallbackGroup::SharedPtr timer2_cb_group; 
};


class FlexivRobot : public rclcpp::Node 
{
public:
    FlexivRobot(
        std::string nodeName, flexiv::Robot& new_robot, 
        flexiv::RobotStates& new_robot_states, flexiv::Log& new_log, 
        flexiv::Scheduler &new_scheduler)
        : Node(nodeName), robot(new_robot), 
          robot_states(new_robot_states), log(new_log), scheduler(new_scheduler)
    {
        // Initialize robot
        //=====================================================================
        // Start the robot 
        if (!robot.isOperational())
            start_robot();
        else 
            log.info("Robot is already operational. ");
        
        // Move home 
        robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.executePrimitive("Home()");
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Switch to real-time joint position control 
        robot.setMode(flexiv::Mode::RT_JOINT_POSITION); 

        // Initialize suction cup system 
        robot.writeDigitalOutput(1, true); 
        robot.writeDigitalOutput(9, false); 

        // Get the initial position and set the initial values 
        // of pos: position, vel: velocity, acc: acceleration 
        robot.getRobotStates(robot_states);
        target_pos = robot_states.q;  
        target_vel = std::vector<double>(7, 0.0); 
        target_acc = std::vector<double>(7, 0.0); 

        // // Add joint stream task with 1ms interval and highest applicable priority
        // scheduler.addTask(
        //     std::bind(&FlexivRobot::joint_stream_task, this),
        //     "Rizon joint control", 1, scheduler.maxPriority());
    }

private: 
    /** @brief Callback function for real-time joint stream task */ 
    void joint_stream_task() {
        std::cout << "yes" << std::endl; 
    }

    /** @brief Clear fault, enable robot and make sure robot is operational */
    inline void start_robot(){
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
    }

    // Private member variables 
    //=====================================================
    // Reference variables 
    flexiv::Robot& robot; 
    flexiv::RobotStates& robot_states;
    flexiv::Log& log;
    flexiv::Scheduler& scheduler;

    // Target joint position, velocity and acceleration 
    std::vector<double> target_pos; 
    std::vector<double> target_vel; 
    std::vector<double> target_acc; 
};


int main(int argc, char * argv[]) 
{
    // Flexiv variables 
    //=========================================================
    // Instantiate robot interface 
    flexiv::Robot robot("192.168.2.100", "192.168.2.101"); 
    // Create a data struct for storing robot states
    flexiv::RobotStates robot_states; 
    // Logger for printing message with timestamp and coloring 
    flexiv::Log log;
    // Scheduler for running real-time robot tracking
    flexiv::Scheduler scheduler; 

    // ROS2 
    //=========================================================
    // Initialize the ROS2 client library 
    rclcpp::init(argc, argv); 
    // Create a ROS2 executor for running multiple nodes 
    rclcpp::executors::MultiThreadedExecutor executor; 
    auto test_ros2 = std::make_shared<TestROS2>("flexiv_test", 
                                                std::ref(robot), 
                                                std::ref(robot_states), 
                                                std::ref(log)); 
    auto flexiv_robot = std::make_shared<FlexivRobot>("flexiv_robot", 
                                                      std::ref(robot), 
                                                      std::ref(robot_states), 
                                                      std::ref(log), 
                                                      std::ref(scheduler)); 
    
    // // Start to run 
    // //============================================================
    // // Create a thread for running flexiv scheduler 
    // std::thread run_scheduler([&scheduler]() {
    //     scheduler.start(); 
    // }); 

    // Spin ROS2 node 
    executor.add_node(test_ros2); 

    executor.add_node(flexiv_robot); 
    executor.spin(); 

    // // Stop 
    // //=============================================================
    // // Stop flexiv scheduler and 
    // scheduler.stop(); 
    // if (run_scheduler.joinable()) { run_scheduler.join(); }

    // Execute the shutdown function before ending the script 
    test_ros2->on_shutdown();
    // flexiv_robot->on_shutdown(); 

    // Shutdown the ROS2 client library 
    rclcpp::shutdown(); 

    return 0;
}