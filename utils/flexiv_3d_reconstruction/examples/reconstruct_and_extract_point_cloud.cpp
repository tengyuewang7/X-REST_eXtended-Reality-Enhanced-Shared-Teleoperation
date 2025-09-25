/**
 * @example online_reconstr_measured_pose_rdk.cpp
 * Online 3D reconstruction using streamed depth camera input and measured
 * camera pose from robot.
 * @date Jun 22, 2022
 * @author Peizhang Zhu
 */

#include <flexiv/reconstruction/OnlineReconstruction.hpp>
#include <flexiv/reconstruction/config.h>
#include <iostream>
#include <thread>

namespace {
/** Camera settings */
const bool kAlignFrame = true;
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Invalid program arguments. Usage: [camera config] [device ID]" << std::endl;
    std::cout << "    camera config: path to the depth camera's configuration json file" << std::endl;
    std::cout << "    device ID: CPU:0 or CUDA:0" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

void CameraUpdateTask(
    flexiv::reconstruction::OnlineReconstruction* online_recon,
    open3d::t::io::RealSenseSensor* rs_cam)
{
    while (!online_recon->IsStopped()) {
        // Get new image from camera
        open3d::t::geometry::RGBDImage new_image
            = rs_cam->CaptureFrame(true, kAlignFrame);

        // Get current camera pose from robot
        // NOTE: a fake pose is used here, can replace with RDK's
        // RobotStates::m_camPose
        std::array<double, 7> current_cam_pose = {0.1, 0.2, 0.3, 1, 0, 0, 0};

        // Update online reconstruction with the latest camera captures
        online_recon->UpdateCameraInput(new_image, current_cam_pose);
    }
}

void PointCloudTask(flexiv::reconstruction::OnlineReconstruction* online_recon)
{
    open3d::t::geometry::PointCloud pcd;
    while (!online_recon->IsStopped()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        online_recon->GetPointCloud(&pcd);

        if (!pcd.IsEmpty()) {
            std::cout << "Obtained new point cloud: " << std::endl;
            std::cout << pcd.ToString() << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    // Parse Parameters
    //=============================================================================
    if (argc != 3
        || open3d::utility::ProgramOptionExistsAny(
            argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    PrintOpen3DVersion();

    // Check if file/path specified in program arguments exist
    std::string resources_dir
        = flexiv::reconstruction::kResourcesPath + "/Open3D_0.15.1";
    if (!open3d::utility::filesystem::DirectoryExists(resources_dir)) {
        open3d::utility::LogWarning(
            "Config file {} does not exist.", resources_dir);
        return -1;
    }

    std::string cam_config_path = argv[1];
    if (!open3d::utility::filesystem::FileExists(cam_config_path)) {
        open3d::utility::LogWarning(
            "Config file {} does not exist.", cam_config_path);
        return -1;
    }

    // Check if user has specified device
    std::string device_code = argv[2];
    if (device_code != "CPU:0" && device_code != "CUDA:0") {
        open3d::utility::LogWarning(
            "Unrecognized device {}. Expecting CPU:0 or CUDA:0.", device_code);
        return -1;
    }
    open3d::utility::LogInfo("Using device {}.", device_code);

    // Depth camera
    //=============================================================================
    // Load camera configuration
    open3d::t::io::RealSenseSensor rs_cam;
    open3d::t::io::RealSenseSensorConfig rs_config;
    open3d::io::ReadIJsonConvertible(cam_config_path, rs_config);

    // Initialize camera
    rs_cam.ListDevices();
    rs_cam.InitSensor(rs_config, 0);
    open3d::utility::LogInfo("Camera metadata parsed from config file: {}",
        rs_cam.GetMetadata().ToString());

    // Load camera intrinsic matrix from camera config
    open3d::camera::PinholeCameraIntrinsic cam_intrinsic
        = rs_cam.GetMetadata().intrinsics_;

    // Online reconstruction
    //=============================================================================
    // Create open3d GUI application
    auto& app = open3d::visualization::gui::Application::GetInstance();
    app.Initialize(resources_dir.c_str());
    auto font_id = app.AddFont(open3d::visualization::gui::FontDescription(
        open3d::visualization::gui::FontDescription::MONOSPACE));

    // Create online reconstruction instance and add to GUI application
    auto online_recon
        = std::make_shared<flexiv::reconstruction::OnlineReconstruction>(
            device_code, font_id);
    app.AddWindow(online_recon);

    // Camera capture once and use the result to initialize online
    // reconstruction
    rs_cam.StartCapture(true);
    open3d::t::geometry::RGBDImage initial_image
        = rs_cam.CaptureFrame(true, kAlignFrame);

    // Create thread to get measured camera pose from robot and update to online
    // reconstruction
    std::thread cam_update_thread(
        CameraUpdateTask, online_recon.get(), &rs_cam);

    // Create thread to run online reconstruction tasks
    online_recon->Init(initial_image, cam_intrinsic);
    std::thread online_recon_thread(
        std::bind(&flexiv::reconstruction::OnlineReconstruction::Run,
            online_recon.get()));

    // Create thread to obtain point cloud
    std::thread point_cloud_thread(PointCloudTask, online_recon.get());

    // Start to run the GUI application, blocking
    app.Run();

    // Wait for all threads to properly exit
    cam_update_thread.join();
    online_recon_thread.join();
    point_cloud_thread.join();
    std::cout << ">>>>>>>>>> All threads exited <<<<<<<<<<" << std::endl;
}
