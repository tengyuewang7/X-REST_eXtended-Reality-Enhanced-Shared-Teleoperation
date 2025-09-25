#ifndef ENVIRONMENT_RECONSTRUCTION
#define ENVIRONMENT_RECONSTRUCTION 

#include <flexiv/reconstruction/OnlineReconstruction.hpp>
#include <flexiv/reconstruction/config.h>
#include <iostream>
#include <thread> 
// #include <rclcpp/rclcpp.hpp>
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/msg/point_cloud2_iterator.hpp"



class EnvironmentReconstruction {

public: 
    EnvironmentReconstruction(); 
    ~EnvironmentReconstruction(); 
    void CameraUpdateTask(); 
    void PointCloudTask(); 
    void Run();

private: 
    const bool kAlignFrame = true;
    std::string resources_dir; 
    std::string cam_config_path; 
    std::string device_code; 
    std::shared_ptr<open3d::t::io::RealSenseSensor> rs_cam_ptr; 
    std::shared_ptr<open3d::t::io::RealSenseSensorConfig> rs_config_ptr;
    open3d::camera::PinholeCameraIntrinsic cam_intrinsic;
    std::shared_ptr<flexiv::reconstruction::OnlineReconstruction> online_recon_ptr; 
    static open3d::visualization::gui::Application* app; 
    std::vector<std::thread> threads; 

}; 

#endif