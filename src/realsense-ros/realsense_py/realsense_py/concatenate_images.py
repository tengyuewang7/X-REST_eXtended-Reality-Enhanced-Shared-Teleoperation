"""
This code documention is designed to support a workflow with multiple realsense cameras.
(Single realsense camera operation is also supported.)

Author: Tengyue Wang
Email: tengyuewang7@gmail.com
Date: 2023-05-30
"""

import pyrealsense2 as rs
import numpy as np
# import open3d as o3d
import cv2
import multiprocessing as mp
import threading 
from cv_bridge import CvBridge
import copy 

import rclpy 
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

class RealsenseCamera: 
    """
    Process specified camera device"""
    def __init__(self, device=None, camera_width=640, camera_height=480, camera_fps=30):
        # Create a RealSense pipeline
        self.camera_name = device.get_info(rs.camera_info.name)
        self.camera_serial_number = device.get_info(rs.camera_info.serial_number)
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.camera_serial_number)
        self.config.enable_stream(rs.stream.depth, self.camera_width, 
                                  self.camera_height, rs.format.z16, 
                                  self.camera_fps)
        self.config.enable_stream(rs.stream.color, self.camera_width, 
                                  self.camera_height, rs.format.bgr8, 
                                  self.camera_fps)
        self.pipeline.start(self.config)

        # Get stream profile and camera intrinsics
        profile = self.pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        intrinsics = color_profile.get_intrinsics()
        self.camera_color_info_msg = self.create_camera_info(intrinsics)

        # Get the depth sensor's depth scale 
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("    name:          ", self.camera_name)
        print("    serial number: ", self.camera_serial_number)
        print("    Depth Scale:   " , self.depth_scale)


        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        # Set placeholder for image information
        self.depth_frame = None
        self.color_frame = None
        self.depth_image = None
        self.color_image = None
        self.depth_colormap = None
        # Set a colorizer
        self.colorizer = rs.colorizer()
        # Set a pointcloud 
        self.pc = rs.pointcloud()

    def create_camera_info(self, intrinsics):
        """Create CameraInfo message from intrinsics"""
        camera_info = CameraInfo()
        camera_info.width = intrinsics.width
        camera_info.height = intrinsics.height
        camera_info.k = [0.0] * 9
        camera_info.k[0] = intrinsics.fx
        camera_info.k[2] = intrinsics.ppx
        camera_info.k[4] = intrinsics.fy
        camera_info.k[5] = intrinsics.ppy
        camera_info.k[8] = 1.0
        camera_info.d = list(intrinsics.coeffs)
        # camera_info.d = [intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]]
        camera_info.distortion_model = 'plumb_bob'
        return camera_info

    def update_frames(self): 
        """Update depth frame and color frame (run every time when requiring new images )"""
        frames = self.pipeline.wait_for_frames()
        # Align the depth frame to color frame
        frames = self.align.process(frames)
        # Get the depth and color frames
        self.depth_frame = frames.get_depth_frame()
        self.color_frame = frames.get_color_frame()
        if not self.depth_frame or not self.color_frame: 
            print("Frames are missing!")
    
    def update_images(self): 
        """Update images after update frames"""
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
        # generates color images based on input depth frame
        self.depth_colormap = np.asanyarray(self.colorizer.colorize(self.depth_frame).get_data())
    
    def stop(self):
        """Close camera pipeline when it is terminated"""
        self.pipeline.stop()

    def view_images(self): 
        """Visualize color image and depth colormap"""
        images = np.hstack((self.color_image, self.depth_colormap))
        cv2.namedWindow(self.camera_name, cv2.WINDOW_AUTOSIZE)
        cv2.imshow(self.camera_name, images)
        cv2.waitKey(1)

def processing(i, camera):
    while True:
        camera.update_frames()       
        camera.update_images()
        camera.view_images()

class ImagePublisher(Node): 
    def __init__(self):
        super().__init__('image_publisher')

        self.concatenate_image_publisher = self.create_publisher(Image, '/camera_images', 10)
        self.global_depth_image_publisher = self.create_publisher(Image, '/global_camera/aligned_depth_to_color/image_raw', 10)
        self.global_color_image_publisher = self.create_publisher(Image, '/global_camera/color/image_raw', 10)
        self.local_depth_image_publisher = self.create_publisher(Image, '/local_camera/aligned_depth_to_color/image_raw', 10)
        self.local_color_image_publisher = self.create_publisher(Image, '/local_camera/color/image_raw', 10)
        self.global_camera_info_publisher = self.create_publisher(CameraInfo, '/global_camera/color/camera_info', 10) 
        self.local_camera_info_publisher = self.create_publisher(CameraInfo, '/local_camera/color/camera_info', 10) 


        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bridge = CvBridge()

        devices = rs.context().devices
        self.camera_handlers = {}
        for i, device in enumerate(devices):
            print("Camera %d: " %i)

            if (device.get_info(rs.camera_info.serial_number)=="233722070710"):
                self.camera_handlers["global"] = RealsenseCamera(device, 1280, 720, 15)   
            elif (device.get_info(rs.camera_info.serial_number)=="242322077348"):
                self.camera_handlers["local"] = RealsenseCamera(device, 1280, 720, 15)  

        self.cnt = 0

    def timer_callback(self):

        self.cnt += 1

        self.camera_handlers["local"].update_frames()
        self.camera_handlers["local"].update_images()       
        self.camera_handlers["global"].update_frames()  
        self.camera_handlers["global"].update_images()   

        green_color = (0, 255, 0)
        thickness = 10

        local_color_image = copy.deepcopy(self.camera_handlers["local"].color_image)

        local_color_image = cv2.resize(local_color_image, (640, 480))
        
        # if (self.cnt == 300): 
        #     cv2.imwrite('local_image.jpg', local_color_image)

        local_color_image[:5, :] = green_color
        local_color_image[-10:, :] = green_color
        local_color_image[:, :10] = green_color
        local_color_image[:, -5:] = green_color

        local_depth_image = copy.deepcopy(self.camera_handlers["local"].depth_colormap)
        local_depth_image = cv2.resize(local_depth_image, (640, 480))
        local_depth_image[:5, :] = green_color
        local_depth_image[-thickness:, :] = green_color
        local_depth_image[:, :5] = green_color
        local_depth_image[:, -thickness:] = green_color 

        global_color_image = copy.deepcopy(self.camera_handlers["global"].color_image)

        # if (self.cnt == 300): 
        #     cv2.imwrite('gloabl_image.jpg', global_color_image)

        # global_color_image = cv2.flip(cv2.flip(global_color_image, 0), 1)

        global_color_image[:thickness, :] = green_color
        global_color_image[-5:, :] = green_color
        global_color_image[:, :10] = green_color
        global_color_image[:, -10:] = green_color 

        images = np.hstack((local_color_image, local_depth_image))
        images = np.vstack((global_color_image, images)) 

        concatenate_msg = self.bridge.cv2_to_imgmsg(images, encoding='bgr8')
        self.concatenate_image_publisher.publish(concatenate_msg)

        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", images)
        cv2.waitKey(1) 

        global_depth_msg = self.bridge.cv2_to_imgmsg(self.camera_handlers["global"].depth_image, encoding='16UC1')
        self.global_depth_image_publisher.publish(global_depth_msg) 

        global_color_msg = self.bridge.cv2_to_imgmsg(self.camera_handlers["global"].color_image, encoding='bgr8')
        self.global_depth_image_publisher.publish(global_color_msg)  

        local_depth_msg = self.bridge.cv2_to_imgmsg(self.camera_handlers["local"].depth_image, encoding='16UC1')
        local_depth_msg.header.stamp = self.get_clock().now().to_msg()
        self.local_depth_image_publisher.publish(local_depth_msg) 

        local_color_msg = self.bridge.cv2_to_imgmsg(self.camera_handlers["local"].color_image, encoding='bgr8')
        local_color_msg.header.stamp = self.get_clock().now().to_msg()
        self.local_color_image_publisher.publish(local_color_msg) 

        self.global_camera_info_publisher.publish(self.camera_handlers["global"].camera_color_info_msg) 
        self.local_camera_info_publisher.publish(self.camera_handlers["local"].camera_color_info_msg) 

        # cv2.namedWindow("con", cv2.WINDOW_AUTOSIZE)
        # cv2.imshow("con", images)
        # cv2.waitKey(1)



def main(): 
    rclpy.init(args=None)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
