#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <cmath> 
#include <algorithm>
#include <thread> 
#include <typeinfo>

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "flexiv_msgs/msg/robot_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "/home/rvc/colcon_ws/utils/flexiv_3d_reconstruction/scripts/environment_reconstruction.hpp" 

using namespace std::chrono_literals;
using namespace message_filters;
using namespace std::placeholders;

typedef sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped> MySyncPolicy;

class Flexiv3dReconstruction : public rclcpp::Node
{
public:
    Flexiv3dReconstruction(std::shared_ptr<EnvironmentReconstruction> er)
        : Node("flexiv_3d_reconstruction"), env_recon_ptr(er)
    {
        // Initialize image subscribers 
        rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/local_camera/color/image_raw");
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/local_camera/aligned_depth_to_color/image_raw");
        
        // Initialize camera pose subscribers 
        camera_pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, "/camera_pose");

        // Initialize sync
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *rgb_sub_, *depth_sub_, *camera_pose_sub_);
        sync_->registerCallback(std::bind(&Flexiv3dReconstruction::image_callback, this, _1, _2, _3));
    
        // Initialize camera info subscriber
        one_time_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/local_camera/color/camera_info", 10,
            std::bind(&Flexiv3dReconstruction::camera_info_callback, this, std::placeholders::_1));

        // Initialize point cloud publisher and its timer 
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
        timer_ = this->create_wall_timer(
            1s, std::bind(&Flexiv3dReconstruction::timer_callback, this));

        // // a fake pose 
        // camera_pose_ptr = std::make_shared<std::array<double, 7>>(std::array<double, 7>{0.7645, -0.1122, 0.3375, 0.0, -0.707, 0.707, 0.0});

    }

    // Public shared_ptr, share data with Flexiv3dReconstruction
    std::shared_ptr<std::array<double, 7>> camera_pose_ptr; // in flexiv pose format 
    std::shared_ptr<open3d::t::geometry::RGBDImage> open3d_rgbd_image_ptr; 
    std::shared_ptr<open3d::camera::PinholeCameraIntrinsic> camera_intrinsic_ptr;
    std::shared_ptr<EnvironmentReconstruction> env_recon_ptr; 

private:
    // Callback functions 
    //===============================================================================
    // Point cloud publish callback function: Publish reconstructed point cloud if it is not empty 
    void timer_callback() { 
        if (env_recon_ptr->pcd_ptr->IsEmpty()) {
            return; 
        }

        std::cout << env_recon_ptr->pcd_ptr->ToString() << std::endl; 

        // Get point cloud info 
        const open3d::core::Tensor points = env_recon_ptr->pcd_ptr->GetPointPositions(); 
        const open3d::core::Tensor colors = env_recon_ptr->pcd_ptr->GetPointColors(); 

        saved_points = points; 

        // Initialize a point cloud message 
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "pointcloud"; 
        message.height = 1; 
        message.width = points.GetLength(); // number of points 
        message.fields.resize(6);
        // Include position (xyz) and color (rgb)
        message.fields[0].name = "x";
        message.fields[1].name = "y";
        message.fields[2].name = "z";
        message.fields[3].name = "r";
        message.fields[4].name = "g";
        message.fields[5].name = "b";
        for (size_t i = 0; i < 3; ++i) {
            message.fields[i].offset = i * 4;
            message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            message.fields[i].count = 1;
        }
        for (size_t i = 3; i < 6; ++i) {
            message.fields[i].offset = i * 4;
            message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            message.fields[i].count = 1;
        }
        // Params
        message.is_bigendian = false;
        message.point_step = 24; // 3 * 4 (x, y, z) + 3 * 4 (r, g, b)
        message.row_step = message.point_step * message.width;
        message.data.resize(message.row_step * message.height);
        message.is_dense = true;
        
        // Assign data into the message 
        sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_r(message, "r");
        sensor_msgs::PointCloud2Iterator<float> iter_g(message, "g");
        sensor_msgs::PointCloud2Iterator<float> iter_b(message, "b");

        for (int64_t i = 0; i < points.GetLength(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            *iter_x = points[i][0].Item<float>();
            *iter_y = points[i][1].Item<float>();
            *iter_z = points[i][2].Item<float>();
            *iter_r = colors[i][0].Item<float>();
            *iter_g = colors[i][1].Item<float>();
            *iter_b = colors[i][2].Item<float>();
        }

        // Publish the point cloud 
        pointcloud_pub_->publish(message); 

    }

    // Sync image subscribe callback function: Receive the color images and depth images from the same frame 
    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg, 
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, 
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &camera_pose_msg)
    {
        // // Print header times of RGB image and depth image 
        // RCLCPP_INFO(this->get_logger(), "RGB Image Header Time: %u.%u", rgb_msg->header.stamp.sec, rgb_msg->header.stamp.nanosec);
        // RCLCPP_INFO(this->get_logger(), "Depth Image Header Time: %u.%u", depth_msg->header.stamp.sec, depth_msg->header.stamp.nanosec);
        
        // Read the color image
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        auto rgb_image = cv_ptr->image; 

        auto rgb_tensor = open3d::core::Tensor(
            rgb_image.data, 
            {rgb_image.rows, rgb_image.cols, 3}, 
            open3d::core::Dtype::UInt8, open3d::core::Device("CPU:0"));
        
        // Read the depth image 
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        std::vector<uint16_t> depth_data; 
        depth_data.resize(depth_msg->data.size() / 2);
        for (size_t i = 0; i < depth_data.size(); ++i) {
            uint16_t value = (static_cast<uint16_t>(depth_msg->data[2*i + 1]) << 8) |
                            static_cast<uint16_t>(depth_msg->data[2*i]);
            depth_data[i] = value;
        }

        auto depth_tensor = open3d::core::Tensor(
            depth_data, 
            {cv_ptr->image.rows, cv_ptr->image.cols, 1}, 
            open3d::core::Dtype::UInt16);
        
        // Get Open3D RGBDImage 
        open3d_rgbd_image_ptr = std::make_shared<open3d::t::geometry::RGBDImage>(rgb_tensor, depth_tensor);
        camera_pose_ptr = std::make_shared<std::array<double, 7>>(
            std::array<double, 7>{camera_pose_msg->pose.position.x, camera_pose_msg->pose.position.y, 
            camera_pose_msg->pose.position.z, camera_pose_msg->pose.orientation.w, camera_pose_msg->pose.orientation.x, 
            camera_pose_msg->pose.orientation.y, camera_pose_msg->pose.orientation.z}); 
    }

    // Camera info subscribe callback function: receive the camera info; once received, destroy this subscriber 
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // Transfer the camera info into Open3D PinholeCameraIntrinsic
        open3d::camera::PinholeCameraIntrinsic intrinsic(
            msg->width,
            msg->height,
            msg->k[0],  // fx
            msg->k[4],  // fy
            msg->k[2],  // cx
            msg->k[5]   // cy
        );
        camera_intrinsic_ptr = std::make_shared<open3d::camera::PinholeCameraIntrinsic>(intrinsic);

        std::cout << "camera_intrinsic_ptr: " << std::endl << camera_intrinsic_ptr->ToString() << std::endl; 

        // Stop subscribing the camera info 
        one_time_camera_info_sub.reset();
    }

public:
    void SaveTensorToText() {
    
        std::string filename = "my_point_cloud.txt"; 

        // Tensor转换为2D向量
        std::vector<std::vector<float>> data;
        open3d::core::SizeVector shape = saved_points.GetShape();
        for (int i = 0; i < shape[0]; ++i) {
            // 获取Tensor的每一行，假设每个点是3D坐标
            auto point = saved_points[i].ToFlatVector<float>();
            data.push_back(point);
        }

        // 打开文件进行写入
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open the file for writing: " << filename << std::endl;
            return;
        }

        // 写入每个点的数据
        for (const auto& point : data) {
            for (size_t j = 0; j < point.size(); ++j) {
                file << point[j];
                if (j != point.size() - 1) file << ", ";
            }
            file << std::endl;
        }

        file.close();
        std::cout << "Saved point cloud" << std::endl; 
    }

private: 
    // Member Variable 
    //================================================================================
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> camera_pose_sub_; 

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr one_time_camera_info_sub; 

    // std::shared_ptr<EnvironmentReconstruction> env_recon_ptr; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_; 

    open3d::core::Tensor saved_points; 
};



// Close the 3d reconstruction 
void close(std::shared_ptr<EnvironmentReconstruction> env_recon_ptr, std::shared_ptr<Flexiv3dReconstruction> flexiv_3d_reconstruction_ptr) {
    // Run 10 seconds 
    std::this_thread::sleep_for(std::chrono::seconds(1000));
    // Stop open3d GUI 
    env_recon_ptr->Stop(); 
    // Destroy the 3d reconstruction 
    env_recon_ptr.reset();
    flexiv_3d_reconstruction_ptr->env_recon_ptr.reset(); 
    // Destroy this ROS2 node 
    flexiv_3d_reconstruction_ptr.reset(); 
    std::cout << "Node closed" << std::endl; 
}

int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv); 

    auto env_recon_ptr = std::make_shared<EnvironmentReconstruction>(); 

    auto flexiv_3d_reconstruction_ptr = std::make_shared<Flexiv3dReconstruction>(env_recon_ptr); 

    // Spin ROS2 node in a thread
    std::thread node_thread([flexiv_3d_reconstruction_ptr]() {
        rclcpp::spin(flexiv_3d_reconstruction_ptr); 
    });

    // A close example that closes the whole node, including the open3d GUI, the 3d reconstruction and this ROS2 node 
    std::thread close_thread(close, env_recon_ptr, flexiv_3d_reconstruction_ptr); 

    // Make sure the ROS2 node is running
    while (flexiv_3d_reconstruction_ptr->camera_pose_ptr == nullptr || 
        flexiv_3d_reconstruction_ptr->open3d_rgbd_image_ptr == nullptr || 
        flexiv_3d_reconstruction_ptr->camera_intrinsic_ptr ==nullptr) { 
        // Wait ... 
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
        std::cout << "Waiting for receiving images and robot info ... " << std::endl; 
        std::cout << (flexiv_3d_reconstruction_ptr->camera_pose_ptr == nullptr) << std::endl; 
        std::cout << (flexiv_3d_reconstruction_ptr->open3d_rgbd_image_ptr == nullptr) << std::endl; 
        std::cout << (flexiv_3d_reconstruction_ptr->camera_intrinsic_ptr ==nullptr) << std::endl; 

    }

    // Run the reconstruction, blocking 
    env_recon_ptr->Run(
        &(flexiv_3d_reconstruction_ptr->camera_pose_ptr), 
        &(flexiv_3d_reconstruction_ptr->open3d_rgbd_image_ptr), 
        &(flexiv_3d_reconstruction_ptr->camera_intrinsic_ptr)); 
    
    flexiv_3d_reconstruction_ptr->SaveTensorToText(); 
    node_thread.join();
    close_thread.join(); 
    rclcpp::shutdown();

    return 0;
}
