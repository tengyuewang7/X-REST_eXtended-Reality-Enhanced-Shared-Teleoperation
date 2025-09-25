#include "environment_reconstruction.hpp" 

open3d::visualization::gui::Application* EnvironmentReconstruction::app = nullptr;

EnvironmentReconstruction::EnvironmentReconstruction() {
    pcd_ptr = std::make_shared<open3d::t::geometry::PointCloud>();
    running = std::make_shared<std::atomic<bool>>(true);
} 

EnvironmentReconstruction::~EnvironmentReconstruction() { 
    *running = false;
    for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    threads.clear();
}

void EnvironmentReconstruction::Run(
    std::shared_ptr<std::array<double, 7>> * cp, 
    std::shared_ptr<open3d::t::geometry::RGBDImage> * ori, 
    std::shared_ptr<open3d::camera::PinholeCameraIntrinsic> * cam_intrinsic_ptr) { 
    
    camera_pose_ptr = cp; 
    rgbd_image_ptr = ori; 

    open3d::utility::LogInfo("cam_intrinsic: {}",
        (*cam_intrinsic_ptr)->ToString()); 

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

    // Camera capture once and use the result to initialize online reconstruction
    online_recon_ptr->Init(*(*rgbd_image_ptr), *(*cam_intrinsic_ptr));

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
    while (running->load() && !online_recon_ptr->IsStopped()) {
        // std::cout << "CameraUpdateTask" << std::endl; 
        // Update online reconstruction with the latest camera captures
        online_recon_ptr->UpdateCameraInput(*(*rgbd_image_ptr), *(*camera_pose_ptr)); 

        std::this_thread::sleep_for(std::chrono::milliseconds(25));

    }
}

void EnvironmentReconstruction::PointCloudTask() {
    while (running->load() && !online_recon_ptr->IsStopped()) {
        // std::cout << "PointCloudTask" << std::endl; 
        std::this_thread::sleep_for(std::chrono::seconds(1));
        online_recon_ptr->GetPointCloud(pcd_ptr.get());
    }
}

void EnvironmentReconstruction::Stop() { 
    running->store(false);
    app->PostToMainThread(online_recon_ptr.get(), [this, self = shared_from_this()]() {
        app->RemoveWindow(online_recon_ptr.get());
        std::cout << "GUI closed" << std::endl;
        // Here, shared_from_this() ensures that the object remains alive until this point
    });
}
