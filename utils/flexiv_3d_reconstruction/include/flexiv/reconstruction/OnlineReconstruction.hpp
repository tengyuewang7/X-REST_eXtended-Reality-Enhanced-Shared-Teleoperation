// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "PropertyPanel.hpp"
#include <open3d/Open3D.h>
#include <memory>
#include <string>
#include <atomic>

#ifndef FLEXIV_RECONSTRUCTION_ONLINE_RECONSTRUCTION_HPP_
#define FLEXIV_RECONSTRUCTION_ONLINE_RECONSTRUCTION_HPP_

using namespace open3d;
using namespace open3d::visualization;

namespace flexiv {
namespace reconstruction {

/**
 * @class OnlineReconstruction
 * @brief Run online 3D reconstruction using Open3D with GUI enabled. CPU or
 * CUDA device can be used.
 */
class OnlineReconstruction : public gui::Window
{

public:
    /**
     * @brief Initialize modules used by online reconstruction.
     * @param[in] device_id Device ID string, e.g. "CPU:0" or "CUDA:0".
     * @param[in] font_id Font used by the visualizer.
     */
    OnlineReconstruction(const std::string& device_id, gui::FontId font_id);
    virtual ~OnlineReconstruction() = default;

    /**
     * @brief Initialize online reconstruction.
     * @param[in] initial_image One capture from the depth camera used for
     * initialization.
     * @param[in] cam_intrinsic Depth camera intrinsic information loaded from
     * camera config file.
     */
    void Init(const open3d::t::geometry::RGBDImage& initial_image,
        const open3d::camera::PinholeCameraIntrinsic& cam_intrinsic);

    /**
     * @brief Run the online reconstruction, including camera capture,
     * pose estimation, model tracking, etc.
     * @warning Blocking, run in a dedicated thread if needed.
     */
    void Run();

    /**
     * @brief Set new camera input, including current camera pose and images.
     * @param[in] image The latest RGBD image captured from depth camera.
     * @param[in] cam_pose Measured camera pose in world frame. 3 positions and
     * 4 quaternions [x, y, z, q_w, q_x, q_y, q_z].
     */
    void UpdateCameraInput(const open3d::t::geometry::RGBDImage& image,
        const std::array<double, 7>& cam_pose);

    /**
     * @brief Obtain the latest point cloud of the constructed surfaces in the
     * scene.
     * @param[out] output Output pointer to store the point cloud data.
     */
    void GetPointCloud(open3d::t::geometry::PointCloud* output);

    /**
     * @brief Set layout for the visualization window.
     */
    void Layout(const gui::LayoutContext& context) override;

    /**
     * @brief Check if online reconstruction is stopped.
     */
    bool IsStopped() { return is_done_; }

    /**
     * @brief Update surface
     */
    void StartOrPause();

    void HandleToggle(bool is_on); 

protected:
    /** Helper function to set the info display */
    void SetInfo(const std::string& output)
    {
        output_info_->SetText(output.c_str());
    }

    /** Helper function to set the FPS display */
    void SetFPS(const std::string& output)
    {
        output_fps_->SetText(output.c_str());
    }

protected:
    /** Measured camera pose in world frame. 3 positions and 4
     * quaternions [x, y, z, q_w, q_x, q_y, q_z]. */
    Eigen::Matrix<double, 7, 1> cam_pose_vec_;
    std::mutex cam_pose_mutex_;

    /** The latest camera input image */
    open3d::t::geometry::RGBDImage cam_image_;
    std::mutex cam_image_mutex_;

    /** Tanglo colorscheme */
    const Eigen::Vector3d kTangoOrange = {0.961, 0.475, 0.000};
    const Eigen::Vector3d kTangoSkyBlueDark = {0.125, 0.290, 0.529};

    /** Device ID string, CPU:0 or CUDA:0 */
    std::string device_str_;

    /** General logic */
    std::atomic<bool> is_running_ = {false};
    std::atomic<bool> is_started_ = {false};
    std::atomic<bool> is_done_ = {false};

    /** GUI panels and controls */
    gui::FontId font_;
    std::shared_ptr<gui::Vert> panel_;
    std::shared_ptr<gui::Label> output_info_;
    std::shared_ptr<PropertyPanel> fixed_props_;
    std::shared_ptr<PropertyPanel> adjustable_props_;
    std::shared_ptr<gui::SceneWidget> widget3d_;
    std::shared_ptr<gui::Vert> fps_panel_;
    std::shared_ptr<gui::Label> output_fps_;

    /** Images */
    std::shared_ptr<gui::ImageWidget> input_color_image_;
    std::shared_ptr<gui::ImageWidget> input_depth_image_;
    std::shared_ptr<gui::ImageWidget> raycast_color_image_;
    std::shared_ptr<gui::ImageWidget> raycast_depth_image_;

    /** Parameters of online reconstruction operation that can be tuned in GUI
     */
    struct OnlineReconstructionParams
    {
        std::atomic<int> surface_interval;
        std::atomic<int> pointcloud_size;
        std::atomic<int> depth_scale;
        std::atomic<int> bucket_count;
        std::atomic<double> voxel_size;
        std::atomic<double> trunc_voxel_multiplier;
        std::atomic<double> depth_max;
        std::atomic<double> depth_diff;
        std::atomic<bool> raycast_color;
        std::atomic<bool> update_surface;
    } prop_values_;

    /** Data describing the constructed surfaces */
    struct SurfaceData
    {
        std::mutex lock;
        t::geometry::PointCloud pcd;
    } surface_;

    /** Flag to indicate that the scene is updated once */
    std::atomic<bool> is_scene_updated_;

    /** Open3d SLAM model that describes the constructed scene */
    std::shared_ptr<t::pipelines::slam::Model> model_;

    /** Complete trajectory of the camera */
    std::shared_ptr<camera::PinholeCameraTrajectory> trajectory_;

    /** Matrix to store current frame pose (cam pose) in world frame */
    Eigen::Matrix4d T_frame_in_world_mat_ = {};

    /** Tensor format of T_frame_in_world_mat_ */
    open3d::core::Tensor T_frame_in_world_ = {};

    /** Reference depth and color image from first camera capture */
    open3d::t::geometry::Image ref_depth_ = {};
    open3d::t::geometry::Image ref_color_ = {};

    /** Tensor format of the camera intrinsic matrix */
    open3d::core::Tensor cam_intrinsic_t_;

    /** Camera trajectory to display in GUI */
    open3d::camera::PinholeCameraParameters traj_param_;

    /** The switch in GUI */
    bool isCollectPcd = false;
};

} /* namespace reconstruction */
} /* namespace flexiv */

#endif /* FLEXIV_RECONSTRUCTION_ONLINE_RECONSTRUCTION_HPP_ */