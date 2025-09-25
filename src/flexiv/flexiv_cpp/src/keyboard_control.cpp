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
#include <unistd.h>
#include <termios.h>
#include <mutex>

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>
#include <flexiv/Model.hpp>
#include <flexiv/Scheduler.hpp>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "flexiv_msgs/msg/robot_states.hpp"
#include "flexiv_msgs/msg/mode_controller.hpp"

#include "/home/rvc/colcon_ws/utils/xbox/src/RizonXbox.hpp"
#include "/home/rvc/colcon_ws/utils/xbox/src/XboxMonitor.hpp"
using namespace std::chrono_literals;

class MyCode : public rclcpp::Node 
{
public:

    MyCode(std::string node_name, flexiv::Robot &new_robot, 
        flexiv::RobotStates &new_robot_states, flexiv::Log &new_log) : 
        Node(node_name), robot(new_robot), robot_states(new_robot_states), log(new_log)
    {
        //timer to control robot.
        // update_timer = this->create_wall_timer(10ms, std::bind(&MyCode::xbox_update, this));
        robot_timer = this->create_wall_timer(1ms, std::bind(&MyCode::robot_loop, this));

        // publish robot states
        states_publisher_ = this->create_publisher<flexiv_msgs::msg::RobotStates>("robot_states", 10);
        states_pub_timer_ = this->create_wall_timer(10ms, std::bind(&MyCode::states_callback, this)); 

        //start keyboard reading thread.
        configure_terminal();
        input_thread_ = std::thread(std::bind(&MyCode::keyboard_thread, this));
        
        start_robot();

        // Read robot states
        robot.getRobotStates(robot_states);
        std::vector<double> initPos = robot_states.q;
        current_q = initPos;
        current_tcpPose_ = std::make_shared<std::vector<double>>();
        (*current_tcpPose_) =  robot_states.tcpPose;
        // std::cout<<(*current_tcpPose_)[0]<<" "<<(*current_tcpPose_)[1]<<" "<<(*current_tcpPose_)[2]<<" "<<(*current_tcpPose_)[3]<<" "<<(*current_tcpPose_)[4]<<" "<<(*current_tcpPose_)[5]<<" "<<(*current_tcpPose_)[6]<<std::endl;
        current_v = std::vector<double>(7, 0.0); 
        current_a = std::vector<double>(7, 0.0);
        maxVel =  std::vector<double>(7, 2.0);
        maxAcc =  std::vector<double>(7, 3.0);
        // robot.writeDigitalOutput(0, true);
        robot.writeDigitalOutput(1, true);
        robot.setMode(flexiv::Mode::NRT_CARTESIAN_MOTION_FORCE_BASE);
        //start xbox control.
        rx.start();
        res_old = rx.get_pose();
        flag_grab = false;
        flag_move = false;
        is_running = false;
        print_help_fuc();
    }
    ~MyCode()
    {
        input_thread_.join();
        restore_terminal();
    }
    void move_to_recorded_joints(){
        robot.stop();
        while (!robot.isStopped()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        robot.setMode(flexiv::Mode::NRT_JOINT_POSITION);
        std::vector<double> temp_joint_position = {2.72998, -1.23841, 0.956538, 0.914583, -0.630053, 0.525956, 2.91486};
        robot.sendJointPosition(temp_joint_position,current_v, current_a, maxVel, maxAcc );
        // Wait for the robot to move
        double error = 0.005;
        robot.getRobotStates(robot_states);
        current_q = robot_states.q;
        double temp_norm = norm(temp_joint_position,current_q);
        while (temp_norm > error) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            robot.getRobotStates(robot_states);
            current_q = robot_states.q;
            temp_norm = norm(temp_joint_position,current_q);
            // std::cout<<temp_norm<<std::endl;
            // std::cout<<(temp_norm > error)<<std::endl;
        }
        
        robot.stop();
        while (!robot.isStopped()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        robot.setMode(flexiv::Mode::NRT_CARTESIAN_MOTION_FORCE_BASE);
        (*current_tcpPose_) =  robot_states.tcpPose;
    }
private:

    void start_robot() {
        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
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
                    "Still waiting for robot to become operational, please check that the robot 1) "
                    "has no fault, 2) is in [Auto (remote)] mode");
            }
        }
        log.info("Robot is now operational");

        // Move robot to home pose
        log.info("Moving to home pose");
        robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.executePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

        /** @brief Callback function for getting robot states from flexiv server, 
     *  warpping as flexiv_msgs::msg::RobotStates() and publishing */
    inline void states_callback() 
    {
        robot.getRobotStates(robot_states); 
        auto states = flexiv_msgs::msg::RobotStates(); 
        states.digital_input_0 = !(robot.readDigitalInput(1));

        std::copy(robot_states.q.begin(), robot_states.q.end(), states.q.begin()); 

        states_publisher_->publish(states); 
 
    } 


    // Function to change terminal settings for immediate key press detection
    void configure_terminal()
    {
        struct termios new_termios;
        tcgetattr(STDIN_FILENO, &old_termios_);
        new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }

    void restore_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
    void keyboard_thread()
    {
        char ch;
        while (rclcpp::ok())
        {
            if (read(STDIN_FILENO, &ch, 1))
            {
                switch(ch)
                {
                    case 'w':
                        (*current_tcpPose_)[0] += 0.01;
                        break;
                    case 's':
                        (*current_tcpPose_)[0] -= 0.01;
                        break;
                    case 'a':
                        (*current_tcpPose_)[1] += 0.01;
                        break;
                    case 'd':
                        (*current_tcpPose_)[1] -= 0.01;
                        break;
                    case 'f':
                        (*current_tcpPose_)[2] += 0.01;
                        break;
                    case 'g':
                        (*current_tcpPose_)[2] -= 0.01;
                        break;
                    //control the grab
                    case 'c':
                        flag_grab = !flag_grab;
                        break;
                    //record the current robot joints.
                    case 'r':
                        robot.getRobotStates(robot_states);
                        (*current_tcpPose_) = robot_states.tcpPose;
                        // std::cout<<"current robot joints:"<<current_q[0]<<" "<<current_q[1]<<" "<<current_q[2]<<" "<<current_q[3]
                        // <<" "<<current_q[4]<<" "<<current_q[5]<<" "<<current_q[6]<<std::endl;
                        std::cout<<(*current_tcpPose_)[0]<<" "<<(*current_tcpPose_)[1]<<" "<<(*current_tcpPose_)[2]<<" "<<(*current_tcpPose_)[3]<<" "<<(*current_tcpPose_)[4]<<" "<<(*current_tcpPose_)[5]<<" "<<(*current_tcpPose_)[6]<<std::endl;
                        break;
                    //move to recorded robot joints.
                    case 'm':
                        flag_move = true;
                        break;
                    default:
                        break;
                }
            }
        }
    }
    // void xbox_update(){
    //     std::cout <<"xbox_update"<< std::endl;
    //     auto res = rx.get_pose();
    //     std::cout <<"xbox_update1"<< std::endl; 
    //     for (auto& i : res) {
    //         (res_old)[i] += (res[i]-res_old[i]);
    //     }
    //     std::cout <<"xbox_update2"<< std::endl;
    // }
    double norm(const std::vector<double>& u, const std::vector<double>& v) {
        if (u.size() != v.size()) {
            throw std::invalid_argument("Vectors must be of the same dimension.");
        }

        double sum = 0.0;
        for (size_t i = 0; i < u.size(); ++i) {
            sum += (u[i] - v[i]) * (u[i] - v[i]);
        }
        return std::sqrt(sum);
    }


    void robot_loop() {
        std::lock_guard<std::mutex> guard(loop_mutex);
        if(is_running){
            log.warn("Previous loop still running.");
            return;            
        }
        is_running = true;
        // rx.input_pose(current_tcpPose);
        auto res = rx.get_pose();
        // std::cout <<"xbox_update1"<< std::endl; 
        for (long unsigned int i = 0; i< res.size(); i++) {
            (*current_tcpPose_)[i] += (res[i]-res_old[i]);
            // std::cout <<(res[i]-res_old[i])<< " ";
            res_old[i] = res[i];
        }

        auto io = rx.get_io(); 
        if (io[0] == true) { robot.writeDigitalOutput(9, true); } 
        else { robot.writeDigitalOutput(9, false); }
        
        // if(flag_grab){
        //     robot.writeDigitalOutput(9, true);
        // }
        // else{
        //     robot.writeDigitalOutput(9, false);
        // }
        
        if(flag_move){
            move_to_recorded_joints();
            flag_move = false;
        }
        //log.info("loop is running.");
        // std::cout << std::endl; 
        // std::cout << robot.readDigitalInput(0)<<std::endl;
        robot.sendCartesianMotionForce(*current_tcpPose_);
        is_running = false; 
    }
    void print_help_fuc(){
        std::cout<<"Using keyborad control the tcp location: w s a d f g!"<<std::endl;
        std::cout<<"Press r to cout the current robot joint position!"<<std::endl;
        std::cout<<"Press m to move to the recorded robot joint position!(Need you to change the joints position in code)"<<std::endl;
        std::cout<<"Using xbox to control the robot tcp location and rotation!(location is good)"<<std::endl;
        std::cout<<"Start control!"<<std::endl;
    }
    flexiv::Robot& robot;
    flexiv::RobotStates& robot_states;
    flexiv::Log& log;
    std::shared_ptr<std::vector<double>> current_tcpPose_; //xyz,qw,qx,qy,qz
    std::vector<double> current_q;
    std::vector<double> current_v;
    std::vector<double> current_a;
    std::vector<double> maxVel;
    std::vector<double> maxAcc;
    rclcpp::TimerBase::SharedPtr update_timer;
    rclcpp::TimerBase::SharedPtr robot_timer; 
    RizonXbox rx;
    std::thread input_thread_;
    std::vector<double> res_old;
    struct termios old_termios_;
    bool flag_grab;
    bool flag_move;
    std::mutex loop_mutex;
    bool is_running;
    rclcpp::Publisher<flexiv_msgs::msg::RobotStates>::SharedPtr states_publisher_;
    rclcpp::TimerBase::SharedPtr states_pub_timer_;
};

int main(int argc, char* argv[]) {

    // flexiv::Scheduler scheduler; 
    
    flexiv::Robot robot("192.168.2.100", "192.168.2.101"); 
    flexiv::RobotStates robot_states;
    flexiv::Log log;
    rclcpp::init(argc, argv);  
    rclcpp::executors::MultiThreadedExecutor executor;
    auto my_code = std::make_shared<MyCode>("keyboard_control", robot, robot_states, log); 
    executor.add_node(my_code); 
    executor.spin();
    // my_code->move_to_recorded_joints();
    // my_code->on_shutdown();
    rclcpp::shutdown();
    // robot.writeDigitalOutput(0, true);

    // robot.writeDigitalOutput(1, true);
    // std::this_thread::sleep_for(1s);
    // robot.writeDigitalOutput(1, false);


    return 0; 

}

