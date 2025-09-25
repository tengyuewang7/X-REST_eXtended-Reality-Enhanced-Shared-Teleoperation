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
#include "flexiv_msgs/msg/closest_point.hpp" 

#include "/home/rvc/colcon_ws/utils/rizon_ik_solver/src/rizon_ik_solver.hpp"


using namespace std::chrono_literals;

class FlexivTest : public rclcpp::Node 
{
public: 
    FlexivTest(
        std::string nodeName, RizonIKSolver& new_ik, flexiv::Robot& new_robot, 
        flexiv::RobotStates& new_robot_states, flexiv::Log& new_log) 
        : Node(nodeName), rizon_ik_solver(new_ik), robot(new_robot), 
            robot_states(new_robot_states), log(new_log)
    {
        // Initialize callback groups 
        timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 
        timer2_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer3_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_opt = rclcpp::SubscriptionOptions(); 
        sub_opt.callback_group = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Define ROS2 publisher, subscriber and timer
        //===================================================================
        // Robot states publisher 
        states_publisher_ = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);
        states_pub_timer_ = this->create_wall_timer(10ms, std::bind(&FlexivTest::states_callback, this), timer_cb_group); 
        camera_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10); 
        
        // Target pose subscriber 
        target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&FlexivTest::target_pose_callback, this, std::placeholders::_1), sub_opt);
        
        // Timer used to update kinematics 
        control_ik_timer_ = this->create_wall_timer(10ms, std::bind(&FlexivTest::control_loop, this), timer2_cb_group); 

        // Timer 
        closest_point_timer_ = this->create_wall_timer(20ms, std::bind(&FlexivTest::closest_point_loop, this), timer3_cb_group); 
        closest_point_publisher_ = this->create_publisher<flexiv_msgs::msg::ClosestPoint>("closest_point", 10); 

        //======================================================================
        // Initialize variables             
        control_mode_ = "none"; 
        gripper_state_ = false; 
        is_manual_ptr_ = nullptr; 

        // Update obstacles to kinematics 
        std::vector<std::vector<double>> obstacles; 

        // Use scanned obstacles 
        LoadDataFromFile("filtered_point_cloud.txt", obstacles);

        // std::cout << obstacles.size() << std::endl; 
        // obstacles = filter_obstacles(obstacles); 
        // std::cout << obstacles.size() << std::endl; 

        // // Use default obstacles 
        // obstacles.push_back(std::vector<double>{7, 0.0, 4});
        // obstacles.push_back(std::vector<double>{5, 0.0, 4});

        // // Obstacle for linear 
        // obstacles.push_back(std::vector<double>{0.67, 0.0, 0.3});

        // // Obstacle for circular 
        // obstacles.push_back(std::vector<double>{0.4, 0.0, 0.3});

        rizon_ik_solver.kinematics->update_obstacles(obstacles); 

        // Initialize the kinematics 
        std::array<double, 7> init_pos = {0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0}; 
        rizon_ik_solver.kinematics->update(init_pos); 
        target_pose = rizon_ik_solver.kinematics->get_flange_pose(); 

        // Warm up the kinematics for fast running 
        // auto start = std::chrono::high_resolution_clock::now(); 
        for (int i = 0; i < 1; ++i) {
            rizon_ik_solver.kinematics->update(rizon_ik_solver.kinematics->get_current_q_vector()); 
            rizon_ik_solver.delta_theta = std::vector<double>(7, 0.0); 
            auto desired_pose = std::vector<double>{0.3, 0.2, 0.6, 0, 1, 0, 0}; 
            rizon_ik_solver.nlopt_ik(desired_pose);
            rizon_ik_solver.control_ik(desired_pose);
        }
        // auto stop = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        // std::cout << "Function took " << duration.count() / 1e3 << " milliseconds." << std::endl;

        // std::vector<std::vector<double>> current_q; 
        // std::string name = "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data/linear4/"; 
        // LoadDataFromFile(name + "current_q.txt", current_q);
        // std::cout << current_q.size() << std::endl; 
        // std::vector<std::vector<double>> current_pose; 
        // for (auto& q : current_q) {
        //     rizon_ik_solver.kinematics->update(q); 
        //     auto flange_pose = rizon_ik_solver.kinematics->get_flange_pose();
        //     current_pose.push_back(flange_pose); 
        // }
        // std::filesystem::path file_name = name + "current_pose.txt"; 
        // std::ofstream file(file_name);
        // if (file.is_open()) {
        //     for (const auto& vector : current_pose) { 
        //         for (const auto& element : vector) {
        //             file << element << " ";
        //         }
        //     file << "\n";
        //     }
        //     file.close();
        // }
    }

    bool LoadDataFromFile(const std::string& filename, std::vector<std::vector<double>>& data) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return false;
        }

        std::string line;
        while (getline(file, line)) {
            std::vector<double> row;
            std::istringstream iss(line);
            double value;

            // 读取一行中的每个值
            while (iss >> value) {
                row.push_back(value);
                // 跳过逗号，如果数据由逗号分隔
                if (iss.peek() == ',')
                    iss.ignore();
            }

            // 将解析好的行数据添加到结果向量中
            data.push_back(row);
        }

        file.close();
        return true;
    }

    std::vector<std::vector<double>> filter_obstacles(std::vector<std::vector<double>> data) {
        std::vector<std::vector<double>> res;
        for (long unsigned int i = 0; i < data.size(); ++i) {
            if (data[i][2] > 0.2) {
                res.push_back(data[i]);
            }
        }
        return res; 
    }

    /** @brief Execute once at the end of this ROS2 node */
    void on_shutdown() {
        
        // Store recorded data to .txt 

        std::string parent_folder = "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data"; 
        std::string new_folder = std::to_string(std::time(0)); 
        std::filesystem::path new_folder_path = std::filesystem::path(parent_folder) / new_folder;
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
    // Callback functions 
    //===========================================================================
    /** @brief Callback function for local motion planner */
    void control_loop() {
        // std::vector<double> increment = rizon_ik_solver.get_trac_ik_u(target_pose); 

        // Wait until the target pose subscriber is started 
        if (is_manual_ptr_ == nullptr) {
            std::this_thread::sleep_for(std::chrono::seconds(1)); 
            return; 
        }

        // Initialize an increment for this time step 
        auto increment = std::vector<double>(7, 0.0);

        // Check if the control mode is manual 
        if (*is_manual_ptr_) {
            // Manual, use trac_ik to get direct ik solution 
            control_mode_ = "manual"; 
            // FIXME: should use trac_ik 
            // increment = rizon_ik_solver.trac_ik(target_pose).second;
            // increment = rizon_ik_solver.control_ik(target_pose);
            increment = rizon_ik_solver.kinematics->direct_ik(target_pose); 
        
        }
        else { 
            // Shared, determine whether the target pose is distant and whether the robot is within danger zones 
            bool large_distance = (torch::norm(rizon_ik_solver.kinematics->dpose).item<double>() > 0.02);
            bool dange_zone = (rizon_ik_solver.kinematics->min_obstacle_distance < 0.1); 
            // Choose the appropriate control strategy 
            if (large_distance && dange_zone) { 
                // Distance and in danger zones: use smooth 
                control_mode_ = "avoiding"; 
                rizon_ik_solver.kinematics->is_using_meshes = false; 
                increment = rizon_ik_solver.control_ik(target_pose); 
            }
            else if (large_distance && !dange_zone)
            {
                // Distance 
                control_mode_ = "smooth"; 
                rizon_ik_solver.kinematics->is_using_meshes = false; 
                // increment = rizon_ik_solver.nlopt_ik(target_pose); 
                increment = rizon_ik_solver.control_ik(target_pose); 

            }
            else if (!large_distance && dange_zone) { 
                // Proximity and in danger zones: use analytical solution 
                control_mode_ = "avoiding"; 
                rizon_ik_solver.kinematics->is_using_meshes = false; 
                increment = rizon_ik_solver.control_ik(target_pose); 
            }
            else if (!large_distance && !dange_zone) { 
                // Proximity and safe: use trac_ik 
                control_mode_ = "tracking";
                rizon_ik_solver.kinematics->is_using_meshes = false; 
                increment = rizon_ik_solver.control_ik(target_pose); 
            }
            // std::cout << control_mode_ << std::endl;
        }

        // Scale the increment to make it reachable within one time step 
        auto maxIt = std::max_element(increment.begin(), increment.end(), [](double a, double b) {
            return std::abs(a) < std::abs(b);
        });
        double maxAbs = std::abs(*maxIt); 
        double threshold = 0.015; 

        static auto last_control_mode = control_mode_;
        static int tracking_cnt = 0; 
        if (last_control_mode == "avoiding" && control_mode_ == "tracking" && tracking_cnt < 10) {
            tracking_cnt ++; 
            control_mode_ = "avoiding"; 
        } 
        else { tracking_cnt = 0; }

        last_control_mode = control_mode_; 
        

        if (is_slow_) { control_mode_ = "smooth"; threshold /= 5.0; } 

        if (control_mode_ == "smooth") { threshold /= 3.0; }

        if (control_mode_ == "avoiding") { threshold /= 10.0; }

        // std::cout << control_mode_ << std::endl; 

        if (maxAbs > threshold) { 
            double scale = threshold / maxAbs; 
            for (auto& val : increment) {
                val *= scale;
            }
        }

        // Update the kinematics 
        auto temp_q = rizon_ik_solver.kinematics->get_current_q_vector(); 

        for (int i = 0; i < 7; ++i) {
            temp_q[i] += increment[i]; 
        }
        rizon_ik_solver.kinematics->update(temp_q);

        // static std::vector<double> last_target_pose = target_pose; 

        static auto last_q = temp_q; 
        static auto last_target_pose = target_pose; 
        static int cnt_q = 0;
        double change_q = 0.0;
        double change_p = 0.0;
        for (size_t i = 0; i < 7; ++i) { 
            change_q += (temp_q[i] - last_q[i]) * (temp_q[i] - last_q[i]); 
            change_p += (target_pose[i] - last_target_pose[i]) * (target_pose[i] - last_target_pose[i]); 
        }
        if (change_q < 0.0000001 && change_p < 0.0000001) { cnt_q++; }
        else { cnt_q = 0; } 
        if (cnt_q > 200) {
            control_mode_ = "idel"; 
        } 
        last_q = temp_q; 
        last_target_pose = target_pose; 

        static int cnt = 0;
        if (cnt % 1 == 0) {
            // Store current data
            static std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
            std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000000.0;
            stored_data["time"].push_back(std::vector<double>(1, duration));
            stored_data["target_pose"].push_back(target_pose); 
            robot.getRobotStates(robot_states); 
            stored_data["current_q"].push_back(robot_states.q); 
            int mode_id = 0;
            if (control_mode_ == "manual") { mode_id = 4; }
            else if (control_mode_ == "tracking") { mode_id = 1; }
            else if (control_mode_ == "smooth") { mode_id = 2; }
            else if (control_mode_ == "avoiding") { mode_id = 3; }
            else if (control_mode_ == "idel") { mode_id = 0; }
            stored_data["mode"].push_back(std::vector<double>(1, mode_id));
            stored_data["manipulability"].push_back(std::vector<double>(1, rizon_ik_solver.kinematics->get_manipulability())); 
            stored_data["min_obstacle_distance"].push_back(std::vector<double>(1, rizon_ik_solver.kinematics->min_obstacle_distance)); 
        } 
        cnt++; 
    }

    void closest_point_loop() {
        if (rizon_ik_solver.kinematics->is_using_meshes == false) {
            rizon_ik_solver.kinematics->update_meshes(); 
            rizon_ik_solver.kinematics->min_distance_between_obstacle_points_and_point_meshes(); 
        } 
        flexiv_msgs::msg::ClosestPoint closest_point = flexiv_msgs::msg::ClosestPoint(); 
        if (control_mode_ == "avoiding") { closest_point.is_show = true; } 
        else { closest_point.is_show = false; } 
        closest_point.obstacle_point.x = rizon_ik_solver.kinematics->closest_point.first[0].item<double>(); 
        closest_point.obstacle_point.y = rizon_ik_solver.kinematics->closest_point.first[1].item<double>(); 
        closest_point.obstacle_point.z = rizon_ik_solver.kinematics->closest_point.first[2].item<double>(); 

        closest_point.mesh_point.x = rizon_ik_solver.kinematics->closest_point.second[0].item<double>(); 
        closest_point.mesh_point.y = rizon_ik_solver.kinematics->closest_point.second[1].item<double>(); 
        closest_point.mesh_point.z = rizon_ik_solver.kinematics->closest_point.second[2].item<double>(); 

        closest_point_publisher_->publish(closest_point); 
    }

    /** @brief Callback function for receiving target pose from a decision-making ROS2 node */
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_sub) 
    {
        // Resolove the encoded info in message's header.frame_id 
        std::string encoded_msg = msg_sub->header.frame_id;
        size_t plus_sign = encoded_msg.find('+'); 
        // Whether turn on the suction cup 
        std::string gripper_msg = encoded_msg.substr(0, plus_sign); 
        // Whether in manual mode 
        std::string manual_msg = encoded_msg.substr(plus_sign + 1); 

        // Set the is_manual_ptr_ 
        if (manual_msg == "manual") { is_manual_ptr_ = std::make_shared<bool>(true); }
        else if (manual_msg == "shared") { is_manual_ptr_ = std::make_shared<bool>(false); } 

        // Get the subscribed target pose: [px, py, pz, rx, ry, rz, rw] 
        target_pose[0] = msg_sub->pose.position.x; 
        target_pose[1] = msg_sub->pose.position.y; 
        target_pose[2] = msg_sub->pose.position.z; 
        target_pose[3] = msg_sub->pose.orientation.x; 
        target_pose[4] = msg_sub->pose.orientation.y; 
        target_pose[5] = msg_sub->pose.orientation.z; 
        target_pose[6] = msg_sub->pose.orientation.w; 

        // std::cout << msg_sub->header.stamp.sec << std::endl; 
        
        if (msg_sub->header.stamp.sec == 0) { is_slow_ = true; }
        else { is_slow_ = false; }

        if (gripper_msg == "true" && gripper_state_ == false) { 
            // Turn on gripper 
            robot.writeDigitalOutput(9, true); 
            gripper_state_ = true; 
        }
        else if (gripper_msg == "false" && gripper_state_ == true) { 
            // Turn off gripper 

            target_pose[3] = 0;
            target_pose[4] = 1;
            target_pose[5] = 0;
            target_pose[6] = 0;

            robot.writeDigitalOutput(9, false); 
            gripper_state_ = false; 
        }

    }

    /** @brief Callback function for getting robot states from flexiv server, 
     *  warpping as flexiv_msgs::msg::RobotStates() and publishing */
    inline void states_callback() 
    {
        // TODO: Use kinematics or the flexiv? 

        auto q = rizon_ik_solver.kinematics->get_current_q_vector();

        std::copy(q.begin(), q.end(), states.q.begin()); 

        // std::cout << states.q[0] << " " << states.q[1] << " " << std::endl;
        // std::cout << rizon_ik_solver.kinematics->current_q << std::endl;
        // std::cout << *(rizon_ik_solver.kinematics->current_q_ptr) << std::endl;
        // std::cout << &(rizon_ik_solver.kinematics->current_q) << std::endl;
        // std::cout << &(*(rizon_ik_solver.kinematics->current_q_ptr)) << std::endl;

        states.header.frame_id = control_mode_; 

        auto flange_vector = rizon_ik_solver.kinematics->get_flange_pose(); 
        states.flange_pose.position.x = flange_vector[0]; 
        states.flange_pose.position.y = flange_vector[1]; 
        states.flange_pose.position.z = flange_vector[2]; 
        states.flange_pose.orientation.x = flange_vector[3]; 
        states.flange_pose.orientation.y = flange_vector[4]; 
        states.flange_pose.orientation.z = flange_vector[5]; 
        states.flange_pose.orientation.w = flange_vector[6]; 

        // Publish the message. 
        states_publisher_->publish(states); 

        robot.getRobotStates(robot_states); 

        geometry_msgs::msg::PoseStamped temp_pose;
        temp_pose.header.stamp = this->get_clock()->now(); 
        temp_pose.header.frame_id = "camera";
        temp_pose.pose.position.x = robot_states.camPose[0];
        temp_pose.pose.position.y = robot_states.camPose[1];
        temp_pose.pose.position.z = robot_states.camPose[2];
        temp_pose.pose.orientation.x = robot_states.camPose[4];
        temp_pose.pose.orientation.y = robot_states.camPose[5];
        temp_pose.pose.orientation.z = robot_states.camPose[6];
        temp_pose.pose.orientation.w = robot_states.camPose[3];
        camera_pose_pub_->publish(temp_pose);
 
    } 

    // Private member variables 
    //==================================================================================================
    // Reference variables 
    RizonIKSolver& rizon_ik_solver; 
    flexiv::Robot& robot; 
    flexiv::RobotStates& robot_states;
    flexiv::Log& log; 

    // ROS2 timer, subscriber, publisher and message 
    rclcpp::TimerBase::SharedPtr states_pub_timer_;
    rclcpp::TimerBase::SharedPtr control_ik_timer_;
    rclcpp::TimerBase::SharedPtr closest_point_timer_; 

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_; 
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr states_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_; 
    rclcpp::Publisher<flexiv_msgs::msg::ClosestPoint>::SharedPtr closest_point_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_pub_; 


    flexiv_msgs::msg::RobotStates states;

    // Create a data struct for storing info 
    std::map<std::string, std::vector<std::vector<double>>> stored_data; 

    // Variables 
    std::vector<double> target_pose;    // [px, py, pz, rx, ry, rz, rw]  
    std::string control_mode_;    // manual, avoiding, tracking, smooth 
    std::shared_ptr<bool> is_manual_ptr_; 
    bool gripper_state_; 
    bool is_slow_; 

    // Callback groups 
    rclcpp::CallbackGroup::SharedPtr timer_cb_group;
    rclcpp::CallbackGroup::SharedPtr timer2_cb_group; 
    rclcpp::CallbackGroup::SharedPtr timer3_cb_group; 

};


class FlexivRobot : public rclcpp::Node 
{
public:
    FlexivRobot(
        std::string nodeName, RizonIKSolver& new_ik, flexiv::Robot& new_robot, 
        flexiv::RobotStates& new_robot_states, flexiv::Log& new_log, 
        flexiv::Scheduler &new_scheduler)
        : Node(nodeName), rizon_ik_solver(new_ik), robot(new_robot), 
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

    // /** @brief Execute once at the end of this ROS2 node */
    // void on_shutdown() {
    //     // Turn of the suction cup  
    //     robot.writeDigitalOutput(1, false); 
        
    //     // Store recorded data to .txt 
    //     std::ofstream file("/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data/data.txt"); 
    //     std::string data_file_loc = "/home/rvc/colcon_ws/src/flexiv/flexiv_cpp/data/";
    //     for (const auto& pair : stored_data) {
    //         std::string file_name = data_file_loc + pair.first + ".txt";
    //         std::ofstream file(file_name);
    //         if (file.is_open()) {
    //             for (const auto& vector : pair.second) { 
    //                 for (const auto& element : vector) {
    //                     file << element << " ";
    //                 }
    //             file << "\n";
    //             }
    //             file.close();
    //         }
    //     }
    //     log.info("Shutting down ... ");
    // }

private: 
    /** @brief Callback function for real-time joint stream task */ 
    void joint_stream_task() {

        robot.getRobotStates(robot_states);
        auto robot_q = robot_states.q; 

        // Get target joint position, velocity and acceleration 
        // auto qva = rizon_ik_solver.kinematics->get_current_qva_vector();
        target_pos = rizon_ik_solver.kinematics->get_current_q_vector(); 
        target_vel = std::vector<double>(7, 0.0); 
        std::vector<double> target_acc = std::vector<double>(7, 0.0); 
        // Whether use estimated velocity and acceleration 
        // if (false) {
        //     target_vel = qva[1]; 
        //     target_acc = qva[2];
        // }

        // Store current data
        static std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000000.0;
        stored_data["time_q"].push_back(std::vector<double>(1, duration));
        stored_data["target_q"].push_back(target_pos); 
        stored_data["current_q"].push_back(robot_states.q); 

        // Stream target joint position 
        robot.streamJointPosition(target_pos, target_vel, target_acc);  
         
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
    RizonIKSolver& rizon_ik_solver;
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
    // Solvers for IK 
    RizonIKSolver rizon_ik_solver;

    // ROS2 
    //=========================================================
    // Initialize the ROS2 client library 
    rclcpp::init(argc, argv); 
    // Create a ROS2 executor for running multiple nodes 
    rclcpp::executors::MultiThreadedExecutor executor; 
    auto flexiv_test = std::make_shared<FlexivTest>("flexiv_test", 
                                                    std::ref(rizon_ik_solver), 
                                                    std::ref(robot), 
                                                    std::ref(robot_states), 
                                                    std::ref(log)); 
    auto flexiv_robot = std::make_shared<FlexivRobot>("flexiv_robot", 
                                                      std::ref(rizon_ik_solver), 
                                                      std::ref(robot), 
                                                      std::ref(robot_states), 
                                                      std::ref(log), 
                                                      std::ref(scheduler)); 
    
    // Start to run 
    //============================================================
    // Create a thread for running flexiv scheduler 
    std::thread run_scheduler([&scheduler]() {
        scheduler.start(); 
    }); 

    // Spin ROS2 node 
    executor.add_node(flexiv_test); 

    // executor.add_node(flexiv_robot); 
    executor.spin(); 

    // Stop 
    //=============================================================
    // Stop flexiv scheduler and 
    scheduler.stop(); 
    if (run_scheduler.joinable()) { run_scheduler.join(); }

    // Execute the shutdown function before ending the script 
    flexiv_test->on_shutdown();
    // flexiv_robot->on_shutdown(); 

    // Shutdown the ROS2 client library 
    rclcpp::shutdown(); 

    return 0;
}

// int main(int argc, char * argv[]) {
//     // Flexiv variables 
//     //=========================================================
//     // Instantiate robot interface 
//     flexiv::Robot robot("192.168.2.100", "192.168.2.101"); 
//     // Create a data struct for storing robot states
//     flexiv::RobotStates robot_states; 
//     // Logger for printing message with timestamp and coloring 
//     flexiv::Log log;
//     flexiv::Scheduler scheduler; 
//     RizonIKSolver rizon_ik_solver;

//     // ROS2 
//     //=========================================================
//     // Initialize the ROS2 client library 
//     rclcpp::init(argc, argv); 
//     // Create a ROS2 executor for running multiple nodes 
//     rclcpp::executors::MultiThreadedExecutor executor; 
//     auto flexiv_test = std::make_shared<FlexivTest>("flexiv_test", 
//                                                     std::ref(rizon_ik_solver), 
//                                                     std::ref(robot), 
//                                                     std::ref(robot_states), 
//                                                     std::ref(log)); 


//     // Spin ROS2 node 
//     executor.add_node(flexiv_test); 
//     executor.spin(); 

//     // Stop 
//     //=============================================================

//     // Execute the shutdown function before ending the script 
//     flexiv_test->on_shutdown();

//     // Shutdown the ROS2 client library 
//     rclcpp::shutdown(); 

//     return 0;
// }