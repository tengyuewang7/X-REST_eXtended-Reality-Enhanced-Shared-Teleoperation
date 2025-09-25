#include <flexiv/reconstruction/OnlineReconstruction.hpp>
#include <chrono>
#include <thread>

namespace flexiv {
namespace reconstruction {

OnlineReconstruction::OnlineReconstruction(
    const std::string& device_id, gui::FontId font_id)
: gui::Window("Open3D - Reconstruction", 1280, 800)
, device_str_(device_id)
, font_(font_id)
{
    ////////////////////////////////////////
    /// General layout
    auto& theme = GetTheme();
    int em = theme.font_size;
    int spacing = int(std::round(0.25f * float(em)));
    int left_margin = em;
    int vspacing = int(std::round(0.5f * float(em)));
    gui::Margins margins(int(std::round(0.5f * float(em))));
    panel_ = std::make_shared<gui::Vert>(spacing, margins);
    widget3d_ = std::make_shared<gui::SceneWidget>();
    fps_panel_ = std::make_shared<gui::Vert>(spacing, margins);

    AddChild(panel_);
    AddChild(widget3d_);
    AddChild(fps_panel_);

    ////////////////////////////////////////
    /// Property panels
    fixed_props_ = std::make_shared<PropertyPanel>(spacing, left_margin);
    fixed_props_->AddIntSlider("Depth scale", &prop_values_.depth_scale, 1000,
        1000, 5000,
        "Scale factor applied to the depth values "
        "from the depth image.");
    fixed_props_->AddFloatSlider("Voxel size", &prop_values_.voxel_size,
        3.0 / 512, 0.004, 0.01, "Voxel size for the TSDF voxel grid.");
    fixed_props_->AddFloatSlider("Trunc multiplier",
        &prop_values_.trunc_voxel_multiplier, 8.0, 1.0, 20.0,
        "Truncate distance multiplier (in voxel size) to control "
        "the volumetric surface thickness.");
    fixed_props_->AddIntSlider("Block count", &prop_values_.bucket_count, 40000,
        10000, 100000,
        "Number of estimated voxel blocks for spatial "
        "hashmap. Will be adapted dynamically, but "
        "may trigger memory issue during rehashing for large scenes.");
    fixed_props_->AddIntSlider("Estimated points",
        &prop_values_.pointcloud_size, 6000000, 500000, 8000000,
        "Estimated number of points in the point cloud; used to speed "
        "extraction of points into the 3D scene.");

    adjustable_props_ = std::make_shared<PropertyPanel>(spacing, left_margin);
    adjustable_props_->AddIntSlider("Update interval",
        &prop_values_.surface_interval, 60, 1, 100,
        "The number of iterations between updating the 3D display.");

    adjustable_props_->AddFloatSlider("Depth max", &prop_values_.depth_max, 3.0,
        1.0, 5.0,
        "Maximum depth before point is "
        "discarded as part of background.");
    adjustable_props_->AddFloatSlider("Depth diff", &prop_values_.depth_diff,
        0.07, 0.03, 0.5,
        "Depth truncation to reject outlier correspondences in "
        "tracking.");
    adjustable_props_->AddBool("Update surface", &prop_values_.update_surface,
        true,
        "Update surface every several frames, "
        "determined by the update interval.");
    adjustable_props_->AddBool("Raycast color", &prop_values_.raycast_color,
        true, "Enable bilinear interpolated color image for visualization.");

    panel_->AddChild(std::make_shared<gui::Label>("Starting settings"));
    panel_->AddChild(fixed_props_);
    panel_->AddFixed(vspacing);
    panel_->AddChild(std::make_shared<gui::Label>("Reconstruction settings"));
    panel_->AddChild(adjustable_props_);

    auto b = std::make_shared<gui::ToggleSwitch>("Resume/Pause");
    b->SetOnClicked([this](bool is_on) {
        HandleToggle(is_on); 
    });
    panel_->AddChild(b);
    panel_->AddFixed(vspacing);

    panel_->AddStretch();

    ////////////////////////////////////////
    /// Tabs
    gui::Margins tab_margins(0, int(std::round(0.5f * float(em))), 0, 0);
    auto tabs = std::make_shared<gui::TabControl>();
    panel_->AddChild(tabs);
    auto tab1 = std::make_shared<gui::Vert>(0, tab_margins);
    input_color_image_ = std::make_shared<gui::ImageWidget>();
    input_depth_image_ = std::make_shared<gui::ImageWidget>();
    tab1->AddChild(input_color_image_);
    tab1->AddFixed(vspacing);
    tab1->AddChild(input_depth_image_);
    tabs->AddTab("Input images", tab1);

    auto tab2 = std::make_shared<gui::Vert>(0, tab_margins);
    raycast_color_image_ = std::make_shared<gui::ImageWidget>();
    raycast_depth_image_ = std::make_shared<gui::ImageWidget>();
    tab2->AddChild(raycast_color_image_);
    tab2->AddFixed(vspacing);
    tab2->AddChild(raycast_depth_image_);
    tabs->AddTab("Raycast images", tab2);

    auto tab3 = std::make_shared<gui::Vert>(0, tab_margins);
    output_info_ = std::make_shared<gui::Label>("");
    output_info_->SetFontId(font_);
    tab3->AddChild(output_info_);
    tabs->AddTab("Info", tab3);

    widget3d_->SetScene(
        std::make_shared<rendering::Open3DScene>(GetRenderer()));

    output_fps_ = std::make_shared<gui::Label>("FPS: 0.0");
    fps_panel_->AddChild(output_fps_);

    is_done_ = false;
    SetOnClose([this]() {
        is_done_ = true;

        if (is_started_) {
            utility::LogInfo("Writing reconstruction to scene.ply...");
            auto pcd
                = model_->ExtractPointCloud(3.0, prop_values_.pointcloud_size);
            auto pcd_legacy
                = std::make_shared<geometry::PointCloud>(pcd.ToLegacy());
            io::WritePointCloud("scene.ply", *pcd_legacy);

            utility::LogInfo("Writing trajectory to trajectory.log...");
            io::WritePinholeCameraTrajectory("trajectory.log", *trajectory_);
        }
        return true; // false would cancel the close
    });
}

void OnlineReconstruction::HandleToggle(bool is_on) {
    if (!this->is_started_) {
    gui::Application::GetInstance().PostToMainThread(this, [this]() {
        int max_points = prop_values_.pointcloud_size;
        t::geometry::PointCloud pcd_placeholder(
            core::Tensor({max_points, 3}, core::Dtype::Float32,
                core::Device("CPU:0")));
        pcd_placeholder.SetPointColors(core::Tensor({max_points, 3},
            core::Dtype::Float32, core::Device("CPU:0")));

        auto mat = rendering::MaterialRecord();
        mat.shader = "defaultUnlit";
        mat.sRGB_vertex_color = true;
        this->widget3d_->GetScene()->GetScene()->AddGeometry(
            "points", pcd_placeholder, mat);

        this->trajectory_
            = std::make_shared<camera::PinholeCameraTrajectory>();

        float voxel_size = prop_values_.voxel_size;
        // The volumetric hash map maps 3D coordinates to
        // 16^3 voxel blocks, to ensure a globally sparse
        // locally dense data structure. This captures the
        // data distribution while maintaining a good memory
        // access pattern.
        this->model_ = std::make_shared<t::pipelines::slam::Model>(
            voxel_size, 16, prop_values_.bucket_count,
            core::Tensor::Eye(
                4, core::Dtype::Float64, core::Device("CPU:0")),
            core::Device(device_str_));
        this->is_started_ = true;
    });
}
this->is_running_ = !(this->is_running_);
this->adjustable_props_->SetEnabled(true);
}

void OnlineReconstruction::Init(
    const open3d::t::geometry::RGBDImage& initial_image,
    const open3d::camera::PinholeCameraIntrinsic& cam_intrinsic)
{
    // Matrix to store current frame pose (cam pose) in world
    T_frame_in_world_mat_ = Eigen::Matrix4d::Identity();

    // Corresponding tensor format
    T_frame_in_world_
        = core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));

    // Create tensor for camera intrinsic matrix
    auto focal_length = cam_intrinsic.GetFocalLength();
    auto principal_point = cam_intrinsic.GetPrincipalPoint();
    cam_intrinsic_t_ = core::Tensor::Init<double>(
        {{focal_length.first, 0, principal_point.first},
            {0, focal_length.second, principal_point.second}, {0, 0, 1}});
    open3d::utility::LogInfo("Camera focal length: X-axis = {}, Y-axis = {}",
        focal_length.first, focal_length.second);
    open3d::utility::LogInfo("Camera principal point: X-axis = {}, Y-axis = {}",
        principal_point.first, principal_point.second);

    // Create a legacy version for use in camera trajectory
    camera::PinholeCameraIntrinsic intrinsic_legacy(-1, -1, focal_length.first,
        focal_length.second, principal_point.first, principal_point.second);

    traj_param_.intrinsic_ = intrinsic_legacy;

    // Save reference depth and color image
    ref_depth_ = initial_image.depth_;
    ref_color_ = initial_image.color_;

    open3d::utility::LogWarning(
        "Setting [Update interval] too small might cause performance issue "
        "that leads to segfault");
}

void OnlineReconstruction::UpdateCameraInput(
    const open3d::t::geometry::RGBDImage& image,
    const std::array<double, 7>& cam_pose)
{
    { // Lock and load camera pose
        std::lock_guard<std::mutex> lock(cam_pose_mutex_);
        for (unsigned int i = 0; i < cam_pose.size(); i++) {
            cam_pose_vec_[i] = cam_pose[i];
        }
    }

    { // Lock and load camera image
        std::lock_guard<std::mutex> lock(cam_image_mutex_);
        cam_image_ = image;
    }
}

void OnlineReconstruction::Run()
{
    core::Device device(device_str_);
    t::pipelines::slam::Frame input_frame(
        ref_depth_.GetRows(), ref_depth_.GetCols(), cam_intrinsic_t_, device);
    t::pipelines::slam::Frame raycast_frame(
        ref_depth_.GetRows(), ref_depth_.GetCols(), cam_intrinsic_t_, device);
    is_scene_updated_ = false;

    // Odometry
    auto traj = std::make_shared<geometry::LineSet>();
    auto frustum = std::make_shared<geometry::LineSet>();
    auto color = std::make_shared<geometry::Image>(ref_color_.ToLegacy());
    auto depth_colored = std::make_shared<geometry::Image>(
        ref_depth_
            .ColorizeDepth(
                prop_values_.depth_scale, 0.3, prop_values_.depth_max)
            .ToLegacy());

    auto raycast_color = std::make_shared<geometry::Image>(t::geometry::Image(
        core::Tensor::Zeros({ref_depth_.GetRows(), ref_depth_.GetCols(), 3},
            core::Dtype::UInt8, core::Device("CPU:0")))
                                                               .ToLegacy());
    auto raycast_depth_colored
        = std::make_shared<geometry::Image>(t::geometry::Image(
            core::Tensor::Zeros({ref_depth_.GetRows(), ref_depth_.GetCols(), 3},
                core::Dtype::UInt8, core::Device("CPU:0")))
                                                .ToLegacy());

    // Add placeholder in case color raycast is disabled in the beginning.
    raycast_frame.SetData("color",
        core::Tensor::Zeros({ref_depth_.GetRows(), ref_depth_.GetCols(), 3},
            core::Dtype::UInt8, core::Device("CPU:0")));

    // Render once to refresh
    gui::Application::GetInstance().PostToMainThread(this,
        [this, color, depth_colored, raycast_color, raycast_depth_colored]() {
            this->input_color_image_->UpdateImage(color);
            this->input_depth_image_->UpdateImage(depth_colored);
            this->raycast_color_image_->UpdateImage(color);
            this->raycast_depth_image_->UpdateImage(depth_colored);
            this->SetNeedsLayout(); // size of image changed

            geometry::AxisAlignedBoundingBox bbox(
                Eigen::Vector3d(-5, -5, -5), Eigen::Vector3d(5, 5, 5));
            auto center = bbox.GetCenter().cast<float>();
            this->widget3d_->SetupCamera(60, bbox, center);
            this->widget3d_->LookAt(center, center + Eigen::Vector3f {0, 1, 3},
                {0.0f, -1.0f, 0.0f});
        });

    Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");

    const int fps_interval_len = 30;
    double time_interval = 0;
    size_t idx = 0;

    utility::Timer timer;
    timer.Start();
    while (!is_done_) {
        if (!is_started_ || !is_running_) {
            // If we aren't running, sleep a little bit so that we don't
            // use 100% of the CPU just checking if we need to run.
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // Form camera pose matrix using measured camera pose vector in world
        // frame
        {
            std::lock_guard<std::mutex> lock(cam_pose_mutex_);
            T_frame_in_world_mat_.block(0, 0, 3, 3)
                = Eigen::Quaterniond(cam_pose_vec_[3], cam_pose_vec_[4],
                    cam_pose_vec_[5], cam_pose_vec_[6])
                      .toRotationMatrix();
            T_frame_in_world_mat_(0, 3) = cam_pose_vec_[0];
            T_frame_in_world_mat_(1, 3) = cam_pose_vec_[1];
            T_frame_in_world_mat_(2, 3) = cam_pose_vec_[2];
        }

        // Convert matrix to tensor format
        T_frame_in_world_
            = core::eigen_converter::EigenMatrixToTensor(T_frame_in_world_mat_);

        // Get new camera input
        t::geometry::Image input_depth = cam_image_.depth_;
        t::geometry::Image input_color = cam_image_.color_;

        input_frame.SetDataFromImage("depth", input_depth);
        input_frame.SetDataFromImage("color", input_color);

        // Integrate model
        model_->UpdateFramePose(idx, T_frame_in_world_);
        model_->Integrate(input_frame, prop_values_.depth_scale,
            prop_values_.depth_max, prop_values_.trunc_voxel_multiplier);
        model_->SynthesizeModelFrame(raycast_frame, prop_values_.depth_scale,
            0.1, prop_values_.depth_max, prop_values_.trunc_voxel_multiplier,
            prop_values_.raycast_color);

        auto K_eigen
            = core::eigen_converter::TensorToEigenMatrixXd(cam_intrinsic_t_);
        auto T_eigen
            = core::eigen_converter::TensorToEigenMatrixXd(T_frame_in_world_);
        traj_param_.extrinsic_ = T_eigen;
        trajectory_->parameters_.push_back(traj_param_);

        std::stringstream info, fps;
        info.setf(std::ios::fixed, std::ios::floatfield);
        info.precision(4);
        info << fmt::format("Frame {}\n\n", idx);

        info << "Transformation:\n";
        info << T_eigen.format(CleanFmt) << "\n\n";

        info << fmt::format("Active voxel blocks: {}/{}\n",
            model_->GetHashMap().Size(), model_->GetHashMap().GetCapacity());
        {
            std::lock_guard<std::mutex> locker(surface_.lock);
            int64_t len = surface_.pcd.HasPointPositions()
                              ? surface_.pcd.GetPointPositions().GetLength()
                              : 0;
            info << fmt::format(
                "Surface points: {}/{}\n", len, prop_values_.pointcloud_size)
                 << "\n";
        }

        if (idx % fps_interval_len == 0) {
            timer.Stop();
            time_interval = timer.GetDuration();
            timer.Start();
        }
        std::string fps_str = fmt::format(
            "FPS: {:.3f}\n", 1000.0 / (time_interval / fps_interval_len));
        info << fps_str;
        fps << fps_str;

        traj->points_.push_back(T_eigen.block<3, 1>(0, 3));
        if (traj->points_.size() > 1) {
            int n = traj->points_.size();
            traj->lines_.push_back({n - 1, n - 2});
            traj->colors_.push_back(kTangoSkyBlueDark);
        }

        frustum = geometry::LineSet::CreateCameraVisualization(
            color->width_, color->height_, K_eigen, T_eigen.inverse(), 0.2);
        frustum->PaintUniformColor(kTangoOrange);

        // TODO: update support for timages-image conversion
        color = std::make_shared<geometry::Image>(
            input_frame.GetDataAsImage("color").ToLegacy());
        depth_colored = std::make_shared<geometry::Image>(
            input_frame.GetDataAsImage("depth")
                .ColorizeDepth(
                    prop_values_.depth_scale, 0.3, prop_values_.depth_max)
                .ToLegacy());

        if (prop_values_.raycast_color) {
            raycast_color = std::make_shared<geometry::Image>(
                raycast_frame.GetDataAsImage("color")
                    .To(core::Dtype::UInt8, false, 255.0f)
                    .ToLegacy());
        }

        raycast_depth_colored = std::make_shared<geometry::Image>(
            raycast_frame.GetDataAsImage("depth")
                .ColorizeDepth(
                    prop_values_.depth_scale, 0.3, prop_values_.depth_max)
                .ToLegacy());

        // Extract surface on demand (do before we increment idx, so that
        // we see something immediately, on interation 0)
        if (prop_values_.update_surface
            && idx % static_cast<int>(prop_values_.surface_interval) == 0) {
            std::lock_guard<std::mutex> locker(surface_.lock);
            surface_.pcd = model_
                               ->ExtractPointCloud(std::min<float>(idx, 3.0f),
                                   prop_values_.pointcloud_size)
                               .To(core::Device("CPU:0"));
            is_scene_updated_ = true;
        }

        gui::Application::GetInstance().PostToMainThread(this,
            [this, color, depth_colored, raycast_color, raycast_depth_colored,
                traj, frustum, info = info.str(), fps = fps.str()]() {
                // Disable depth_scale and pcd buffer size change
                this->fixed_props_->SetEnabled(false);

                this->raycast_color_image_->SetVisible(
                    this->prop_values_.raycast_color);

                this->SetInfo(info);
                this->SetFPS(fps);
                this->input_color_image_->UpdateImage(color);
                this->input_depth_image_->UpdateImage(depth_colored);

                if (prop_values_.raycast_color) {
                    this->raycast_color_image_->UpdateImage(raycast_color);
                }
                this->raycast_depth_image_->UpdateImage(raycast_depth_colored);

                this->widget3d_->GetScene()->RemoveGeometry("frustum");
                auto mat = rendering::MaterialRecord();
                mat.shader = "unlitLine";
                mat.line_width = 5.0f;
                this->widget3d_->GetScene()->AddGeometry(
                    "frustum", frustum.get(), mat);

                if (traj->points_.size() > 1) {
                    // 1) Add geometry once w/ max size
                    // 2) Update geometry
                    // TPointCloud
                    this->widget3d_->GetScene()->RemoveGeometry("trajectory");
                    auto mat = rendering::MaterialRecord();
                    mat.shader = "unlitLine";
                    mat.line_width = 5.0f;
                    // TODO peizhang: bug here, trajectory disabled now
                    // this->widget3d_->GetScene()->AddGeometry(
                    //     "trajectory", traj.get(), mat);
                }

                if (is_scene_updated_) {
                    using namespace rendering;
                    std::lock_guard<std::mutex> locker(surface_.lock);
                    if (surface_.pcd.HasPointPositions()
                        && surface_.pcd.HasPointColors()) {
                        auto* scene = this->widget3d_->GetScene()->GetScene();

                        scene->UpdateGeometry("points", surface_.pcd,
                            Scene::kUpdatePointsFlag
                                | Scene::kUpdateColorsFlag);
                    }
                    is_scene_updated_ = false;
                }
            });

        // Note that the user might have closed the window, in which case we
        // want to maintain a value of true.
        idx++;
    }
}

void OnlineReconstruction::GetPointCloud(
    open3d::t::geometry::PointCloud* output)
{
    std::lock_guard<std::mutex> locker(surface_.lock);
    *output = surface_.pcd;
}

void OnlineReconstruction::Layout(const gui::LayoutContext& context)
{
    int em = context.theme.font_size;
    int panel_width = 20 * em;
    // The usable part of the window may not be the full size if there
    // is a menu.
    auto content_rect = GetContentRect();
    panel_->SetFrame(gui::Rect(
        content_rect.x, content_rect.y, panel_width, content_rect.height));
    int x = panel_->GetFrame().GetRight();
    widget3d_->SetFrame(gui::Rect(
        x, content_rect.y, content_rect.GetRight() - x, content_rect.height));

    int fps_panel_width = 7 * em;
    int fps_panel_height = 2 * em;
    fps_panel_->SetFrame(gui::Rect(content_rect.GetRight() - fps_panel_width,
        content_rect.y, fps_panel_width, fps_panel_height));

    // Now that all the children are sized correctly, we can super to
    // layout all their children.
    gui::Window::Layout(context);
}
void OnlineReconstruction::StartOrPause()
{
    this->is_running_ = !(this->is_running_);
}
} /* namespace reconstruction */
} /* namespace flexiv */