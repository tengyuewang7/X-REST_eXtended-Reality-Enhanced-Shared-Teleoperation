#include "XboxMonitor.hpp" 
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector> 

/**
 * @brief Class to Calculate the new pose based to xbox input.  
*/
class RizonXbox : public XboxMonitor
{
public:
/**
 * @brief Constructor. 
*/
RizonXbox() {
    // Initialize a shared_ptr of pose_ at home 
    pose_ = std::make_shared<std::vector<double>>(
        std::vector<double>(home_pose_));
    // Initialize a shared_ptr of io_ at home //AXBY
    io_ =  std::make_shared<std::vector<bool>>(
    	std::vector<bool>(home_io_));
}

~RizonXbox() {
    stop(); 
}

/**
 * @brief Function overload for starting the xbox input monitoring thread 
 * and pose calculation thread.  
*/
void start() { 
    // Monitoring thread 
    XboxMonitor::start(); 
    // Pose calculation thread 
    running_pose_updating_ = true; 
    pose_updating_thread_ = std::thread(
        &RizonXbox::update_pose, this);
}

/**
 * @brief Function overload for stopping. 
*/
void stop() {
    XboxMonitor::stop(); 
    if (running_pose_updating_) {
        running_pose_updating_ = false; 
        if (pose_updating_thread_.joinable()) {
            pose_updating_thread_.join(); 
        }
    }
}
/**
 * @brief get current_pose and put it in pose_; 
 * @return None. 
*/
void input_pose(std::vector<double> current_pose) {
    std::lock_guard<std::mutex> lock_pose(pose_mutex_);
    *pose_ =  current_pose;
    return; 
}

/**
 * @brief Return the current pose. 
 * @return Size 7 std::vector<double>. 
*/
std::vector<double> get_pose() {
    std::lock_guard<std::mutex> lock_pose(pose_mutex_); 
    return *pose_; 
}

/**
 * @brief Return the current io. 
 * @return Size 4 std::vector<bool>. 
*/
std::vector<bool> get_io() {
    std::lock_guard<std::mutex> lock_pose(pose_mutex_); 
    return *io_; 
}

private: 
/**
 * @brief Keep updating the pose based on the current xbox data. 
 * Running at 10 milliseconds per loop. 
*/
void update_pose() { 
    // Mark the current time 
    auto timer = std::chrono::high_resolution_clock::now();
    // Start running 
    while (running_pose_updating_) { 
        // 10 milliseconds per loop 
        timer += std::chrono::milliseconds(10); 
        // Get the current xbox data 
        xbox_map x_box_data = get_map(); 
        // Update the pose based on xbox data
        { 
            // Mutex lock 
            std::lock_guard<std::mutex> lock_pose(pose_mutex_); 

            // LO stick button: reset translation to home 
            if (x_box_data.lo) {
                for (uint i = 0; i < 3; ++i) {
                    (*pose_)[i] = home_pose_[i]; 
                }
            } 
            // RO stick button: reset rotation to home 
            if (x_box_data.ro) {
                for (uint i = 3; i < 7; ++i) {
                    (*pose_)[i] = home_pose_[i];
                }
            }
            

            static int cnt_a = 0;
            if (x_box_data.a == true && cnt_a == 0) { cnt_a = 1; }
            else if (x_box_data.a == false && cnt_a == 1) { (*io_)[0] = !(*io_)[0]; cnt_a = 0; } 
            static int cnt_x = 0;
            if (x_box_data.x == true && cnt_x == 0) { cnt_x = 1; }
            else if (x_box_data.x == false && cnt_x == 1) { (*io_)[1] = !(*io_)[1]; cnt_x = 0; } 
            static int cnt_b = 0;
            if (x_box_data.b == true && cnt_b == 0) { cnt_b = 1; }
            else if (x_box_data.b == false && cnt_b == 1) { (*io_)[2] = !(*io_)[2]; cnt_b = 0; } 
            static int cnt_y = 0;
            if (x_box_data.y == true && cnt_y == 0) { cnt_y = 1; }
            else if (x_box_data.y == false && cnt_y == 1) { (*io_)[3] = !(*io_)[3]; cnt_y = 0; } 
            // std::cout<< running_pose_updating_<<std::endl;
            // Update the translation and rotation 
            update_trans(std::ref(x_box_data)); 
            update_rot(std::ref(x_box_data)); 
        } 
        // std::cout<< "XIXI"<<std::endl;
        // Sleep to make 10 milliseconds per loop 
        std::this_thread::sleep_until(timer);
    }
}

/**
 * @brief Update translation. 
 * @param x_box_data: reference to the data struct. 
*/
void update_trans(xbox_map& x_box_data) { 
    // Left stick x axis: x position 
    (*pose_)[1] += x_box_data.lx / 12000 * TRANS_SCALE_ * (x_box_data.a + 1); 
    // Left stick y axis: y position 
    (*pose_)[0] += x_box_data.ly / 12000 * TRANS_SCALE_ * (x_box_data.a + 1); 
    // Right trigger and left trigger: z position 
    (*pose_)[2] += (x_box_data.rt - x_box_data.lt) / 2 / 12000 * TRANS_SCALE_ * (x_box_data.a + 1); 
}

/**
 * @brief Update rotation. 
 * @param x_box_data: reference to the data struct. 
*/
void update_rot(xbox_map& x_box_data) {
    // Use Eigen::Quaterniond for quick quaternion calculation 
    Eigen::Quaterniond current_rot((*pose_)[3], (*pose_)[4], (*pose_)[5], (*pose_)[6]); 
    
    // Get rotation angles and their quaternions 
    double x_angle = (x_box_data.rx / 10000) * ROT_SCALE_; 
    Eigen::Quaterniond rot_x(Eigen::AngleAxisd(x_angle, Eigen::Vector3d::UnitX()));
    double y_angle = (x_box_data.ry / 10000) * ROT_SCALE_; 
    Eigen::Quaterniond rot_y(Eigen::AngleAxisd(y_angle, Eigen::Vector3d::UnitY()));
    double z_angle = (x_box_data.rb - x_box_data.lb) * ROT_SCALE_; 
    Eigen::Quaterniond rot_z(Eigen::AngleAxisd(z_angle, Eigen::Vector3d::UnitZ()));
    
    // Rotate the current orientation 
    Eigen::Quaterniond result_rot = rot_z * (rot_y * (rot_x * current_rot));  
	
    // Assign the result quaternion to pose 
    (*pose_)[3] = result_rot.w(); 
    (*pose_)[4] = result_rot.x(); 
    (*pose_)[5] = result_rot.y(); 
    (*pose_)[6] = result_rot.z(); 
}

// Shared_ptr for current pose and home pose 
std::shared_ptr<std::vector<double>> pose_;
std::shared_ptr<std::vector<bool>> io_ ;
std::vector<double> home_pose_{0.688, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0 };
std::vector<bool> home_io_{false,false,false,false}; //AXBY 
// Scales for sensitivity of xbox  
static constexpr double TRANS_SCALE_ = 0.0005; 
static constexpr double ROT_SCALE_ = M_PI  / 1000.0; 
// Thread for pose calculation and its flag 
std::thread pose_updating_thread_; 
std::atomic<double> running_pose_updating_; 
// Mutex
std::mutex pose_mutex_; 
};
