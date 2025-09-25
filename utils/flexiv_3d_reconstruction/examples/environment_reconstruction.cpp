#include "environment_reconstruction.hpp" 

open3d::visualization::gui::Application* EnvironmentReconstruction::app = nullptr;

EnvironmentReconstruction::EnvironmentReconstruction() {

    // Parse Parameters
    //=============================================================================
    resources_dir = flexiv::reconstruction::kResourcesPath + "/Open3D_0.15.1"; 
    cam_config_path = "/home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/resources/camera_configs/RealSense_D435/high_density.json"; 
    device_code = "CPU:0"; 

    // Depth camera
    //=============================================================================
    // Load camera configuration
    rs_cam_ptr = std::make_shared<open3d::t::io::RealSenseSensor>(); 
    rs_config_ptr = std::make_shared<open3d::t::io::RealSenseSensorConfig>(); 
    open3d::io::ReadIJsonConvertible(cam_config_path, *rs_config_ptr); 

    // Initialize camera
    rs_cam_ptr->ListDevices();
    rs_cam_ptr->InitSensor(*rs_config_ptr, 0);
    open3d::utility::LogInfo("Camera metadata parsed from config file: {}",
        rs_cam_ptr->GetMetadata().ToString()); 
    // Load camera intrinsic matrix from camera config
    cam_intrinsic = rs_cam_ptr->GetMetadata().intrinsics_; 

    // Online reconstruction
    //=============================================================================
    // Create open3d GUI application
    app = &open3d::visualization::gui::Application::GetInstance();
    app->Initialize(resources_dir.c_str()); 
    auto font_id = app->AddFont(open3d::visualization::gui::FontDescription(
        open3d::visualization::gui::FontDescription::MONOSPACE));

    // Create online reconstruction instance and add to GUI application
    online_recon_ptr = std::make_shared<flexiv::reconstruction::OnlineReconstruction>(
        device_code, font_id);
    app->AddWindow(online_recon_ptr);

    // Camera capture once and use the result to initialize online
    // reconstruction
    rs_cam_ptr->StartCapture(true);
    open3d::t::geometry::RGBDImage initial_image
        = rs_cam_ptr->CaptureFrame(true, kAlignFrame); 
    online_recon_ptr->Init(initial_image, cam_intrinsic);
} 

EnvironmentReconstruction::~EnvironmentReconstruction() { 
    for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    threads.clear();
}

void EnvironmentReconstruction::Run() { 

    // Start the reconstruction 
    online_recon_ptr->HandleToggle(true); 

    // Create thread to get measured camera pose from robot and update to online reconstruction
    threads.emplace_back(&EnvironmentReconstruction::CameraUpdateTask, this); 

    // Create thread to run online reconstruction tasks
    threads.emplace_back(&flexiv::reconstruction::OnlineReconstruction::Run, 
        online_recon_ptr.get()); 

    // Create thread to obtain point cloud
    threads.emplace_back(&EnvironmentReconstruction::PointCloudTask, this); 

    // Start to run the GUI application, blocking
    app->Run();
}

void EnvironmentReconstruction::CameraUpdateTask() {
    std::cout << "CameraUpdateTask" << std::endl; 

    while (!online_recon_ptr->IsStopped()) {
        // Get new image from camera
        open3d::t::geometry::RGBDImage new_image
            = rs_cam_ptr->CaptureFrame(true, kAlignFrame);
        // A fkae pose 
        std::array<double, 7> current_cam_pose = {0.1, 0.2, 0.3, 1, 0, 0, 0}; 

        // Update online reconstruction with the latest camera captures
        online_recon_ptr->UpdateCameraInput(new_image, current_cam_pose); 
    }
}

void EnvironmentReconstruction::PointCloudTask() {

    open3d::t::geometry::PointCloud pcd; 
    while (!online_recon_ptr->IsStopped()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        online_recon_ptr->GetPointCloud(&pcd);

        if (!pcd.IsEmpty()) {
            std::cout << "Obtained new point cloud: " << std::endl;
            std::cout << pcd.ToString() << std::endl;
        }
        else { std::cout << "is empty" << std::endl; }
    }
}