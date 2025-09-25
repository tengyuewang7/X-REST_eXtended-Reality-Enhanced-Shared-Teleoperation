#ifndef ENVIRONMENT_RECONSTRUCTION
#define ENVIRONMENT_RECONSTRUCTION 

#include <flexiv/reconstruction/OnlineReconstruction.hpp>
#include <flexiv/reconstruction/config.h>
#include <iostream>
#include <thread> 
#include <atomic>
#include <opencv2/opencv.hpp>

class EnvironmentReconstruction : public std::enable_shared_from_this<EnvironmentReconstruction>{

public: 
    EnvironmentReconstruction(); 
    ~EnvironmentReconstruction(); 
    void CameraUpdateTask(); 
    void PointCloudTask(); 
    // open3d::t::geometry::PointCloud GetPointCloud(); 
    void Run(std::shared_ptr<std::array<double, 7>> *, 
            std::shared_ptr<open3d::t::geometry::RGBDImage> *, 
            std::shared_ptr<open3d::camera::PinholeCameraIntrinsic> *); 
    void Stop(); 
    std::shared_ptr<open3d::t::geometry::PointCloud> pcd_ptr;

private: 
    const bool kAlignFrame = true;
    // std::string resources_dir; 
    // std::string cam_config_path; 
    // std::string device_code; 
    std::shared_ptr<open3d::t::io::RealSenseSensor> rs_cam_ptr; 
    std::shared_ptr<open3d::t::io::RealSenseSensorConfig> rs_config_ptr;
    open3d::camera::PinholeCameraIntrinsic cam_intrinsic;
    std::shared_ptr<flexiv::reconstruction::OnlineReconstruction> online_recon_ptr; 
    static open3d::visualization::gui::Application* app; 
    std::vector<std::thread> threads; 
    std::shared_ptr<std::array<double, 7>> * camera_pose_ptr; 
    std::shared_ptr<open3d::t::geometry::RGBDImage> * rgbd_image_ptr; 
    std::shared_ptr<std::atomic<bool>> running;

    // Parse Parameters
    //=============================================================================
    std::string resources_dir = flexiv::reconstruction::kResourcesPath + "/Open3D_0.15.1"; 
    std::string cam_config_path = "/home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/resources/camera_configs/RealSense_D435/high_density.json"; 
    std::string device_code = "CPU:0"; 

}; 

#endif