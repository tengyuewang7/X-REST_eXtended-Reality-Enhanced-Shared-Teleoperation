#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <vector>
#include <filesystem>

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>
#include <flexiv/Model.hpp>
#include <flexiv/Scheduler.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "/home/rvc/colcon_ws/utils/rizon_trac_ik/src/rizon_trac_ik.hpp" 

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "std_msgs/msg/string.hpp" 
#include "flexiv_msgs/msg/robot_states.hpp" 

using namespace std::chrono_literals;


class FlexivTracIK : public rclcpp::Node 
{
public:

    FlexivTracIK(std::string node_name, flexiv::Robot &new_robot, 
        flexiv::RobotStates &new_robot_states, flexiv::Log &new_log) : 
        Node(node_name), robot(new_robot), robot_states(new_robot_states), log(new_log)
    {
        ik = RizonTracIK(); 
        // Robot states publisher 
        states_publisher_ = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);
        states_pub_timer_ = this->create_wall_timer(10ms, std::bind(&FlexivTracIK::states_callback, this)); 
        endeffector_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                        "endeffector_pose", 
                        10, 
                        std::bind(&FlexivTracIK::endeffector_pose_callback, this, std::placeholders::_1));
        robot.getRobotStates(robot_states); 
        target_q = std::make_shared<std::vector<double>>(std::vector<double>
            { 0.0, -40.0/180*3.1415926, 0.0, 90.0/180*3.1415926, 0.0, 40.0/180*3.1415926, 0.0 }); 
    }

    std::shared_ptr<std::vector<double>> target_q = nullptr; 

        /** @brief Execute once at the end of this ROS2 node */
    void on_shutdown() {
        
        // Store recorded data to .txt 


        std::string parent_folder = "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data"; 
        std::string new_folder = std::to_string(std::time(0)); 
        std::filesystem::path new_folder_path = std::filesystem::path(parent_folder) / (new_folder + "trac_ik");
        std::filesystem::create_directory(new_folder_path); 

        for (const auto& pair : stored_data) {
            // std::string file_name = data_file_loc + pair.first + ".txt";
            std::filesystem::path file_name = new_folder_path / (pair.first + ".txt");
            std::ofstream file(file_name);
            if (file.is_open()) {
                for (const auto& vector : pair.second) { 
                    for (const auto& element : vector) {
                        file << element << " ";
                    }
                file << "\n";
                }
                file.close();
            }
        }
        log.info("Shutting down ... ");
    }

private:

    inline void states_callback() 
    {

        robot.getRobotStates(robot_states); 
        auto q = robot_states.q; 
        auto states = flexiv_msgs::msg::RobotStates(); 
        states.digital_input_0 = !(robot.readDigitalInput(1)); 
        std::copy(q.begin(), q.end(), states.q.begin()); 

        // Publish the message. 
        states_publisher_->publish(states); 

        // Store current data
        static std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000000.0;
        stored_data["time_q"].push_back(std::vector<double>(1, duration));
        stored_data["target_pose"].push_back(target_pose); 
        stored_data["target_q"].push_back(*target_q); 
        stored_data["current_q"].push_back(q); 
        auto flange_pose_in_wxyz = robot_states.flangePose; 
        auto flange_pose = std::vector<double>{flange_pose_in_wxyz[0], flange_pose_in_wxyz[1], 
            flange_pose_in_wxyz[2], flange_pose_in_wxyz[4], flange_pose_in_wxyz[5], 
            flange_pose_in_wxyz[6], flange_pose_in_wxyz[3]}; 
        stored_data["current_pose"].push_back(flange_pose); 
    } 

    void endeffector_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        target_pose = std::vector<double>{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w}; 
        bool frame_id = false; 
        if (msg->header.frame_id == "true") { frame_id = true; } 
        else if (msg->header.frame_id == "false") { frame_id = false; } 
        if (suction_state != frame_id) { suction_state = frame_id; robot.writeDigitalOutput(9, frame_id); }
        robot.getRobotStates(robot_states); 
        std::vector<double> q = robot_states.q; 
        std::vector<double> pose = std::vector<double> { msg->pose.position.x, msg->pose.position.y, 
                                                            msg->pose.position.z, msg->pose.orientation.x, 
                                                            msg->pose.orientation.y, msg->pose.orientation.z, 
                                                            msg->pose.orientation.w };                                       
        auto new_q = ik.get_ik(q, pose, 'q').second; 
        for (size_t i = 0; i < new_q.size(); ++i) {
            (*target_q)[i] = new_q[i]; 
        }
    }

    flexiv::Robot& robot;
    flexiv::RobotStates& robot_states;
    flexiv::Log& log;
    RizonTracIK ik; 

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr endeffector_pose_sub; 
    rclcpp::TimerBase::SharedPtr states_pub_timer_;
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr states_publisher_;

    bool suction_state = false; 

    std::vector<double> target_pose = std::vector<double>(7, 0.0); 
    std::map<std::string, std::vector<std::vector<double>>> stored_data; 

};


class FlexivRobot : public rclcpp::Node 
{
public:
    FlexivRobot(
        std::string nodeName, flexiv::Robot& new_robot, 
        flexiv::RobotStates& new_robot_states, flexiv::Log& new_log, flexiv::Scheduler& new_scheduler,std::shared_ptr<std::vector<double>> q)
        : Node(nodeName), robot(new_robot), 
          robot_states(new_robot_states), log(new_log), scheduler(new_scheduler), target_q(q)
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

        // Add joint stream task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(&FlexivRobot::joint_stream_task, this),
            "Rizon joint control", 1, scheduler.maxPriority());
    }

    /** @brief Execute once at the end of this ROS2 node */
    void on_shutdown() {
        // Turn of the suction cup  
        robot.writeDigitalOutput(1, false); 
        
        // Store recorded data to .txt 
        std::ofstream file("/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data/data.txt"); 
        std::string data_file_loc = "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data/";
        for (const auto& pair : stored_data) {
            std::string file_name = data_file_loc + pair.first + ".txt";
            std::ofstream file(file_name);
            if (file.is_open()) {
                for (const auto& vector : pair.second) { 
                    for (const auto& element : vector) {
                        file << element << " ";
                    }
                file << "\n";
                }
                file.close();
            }
        }
        log.info("Shutting down ... ");
    }

private: 
    /** @brief Callback function for real-time joint stream task */ 
    void joint_stream_task() {
        target_vel = std::vector<double>(7, 0.0); 
        target_acc = std::vector<double>(7, 0.0); 

        // log.info(flexiv::utility::vec2Str(*target_q)); 

        // Stream target joint position 
        robot.streamJointPosition(*target_q, target_vel, target_acc);   
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

    // Create a data struct for storing info 
    std::map<std::string, std::vector<std::vector<double>>> stored_data;

    // Target joint position, velocity and acceleration 
    std::vector<double> target_pos; 
    std::vector<double> target_vel; 
    std::vector<double> target_acc; 

    std::shared_ptr<std::vector<double>> target_q; 
};

int main(int argc, char* argv[]) {

    flexiv::Robot robot("192.168.2.100", "192.168.2.101"); 
    flexiv::RobotStates robot_states;
    flexiv::Log log;
    flexiv::Scheduler scheduler; 

    rclcpp::init(argc, argv);  
    rclcpp::executors::MultiThreadedExecutor executor; 

    auto flexiv_trac_ik = std::make_shared<FlexivTracIK>("flexiv_trac_ik", robot, robot_states, log); 

    while (flexiv_trac_ik->target_q == nullptr) {
        std::this_thread::sleep_for(1s); 
    }

    auto flexiv_robot = std::make_shared<FlexivRobot>("flexiv_robot", 
                                                      std::ref(robot), 
                                                      std::ref(robot_states), 
                                                      std::ref(log), 
                                                      std::ref(scheduler), 
                                                      flexiv_trac_ik->target_q); 
    
    // Create a thread for running flexiv scheduler 
    std::thread run_scheduler([&scheduler]() {
        scheduler.start(); 
    }); 

    executor.add_node(flexiv_trac_ik); 
    executor.spin();

    flexiv_trac_ik->on_shutdown(); 

    rclcpp::shutdown(); 



    return 0; 

}

