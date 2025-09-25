#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import PoseStamped 
import numpy as np      
import cv2 
from cv_bridge import CvBridge 
from apriltag import apriltag
from scipy.spatial.transform import Rotation
import time

class AprilTagDetection(Node): 
    def __init__(self, name):
        super().__init__(name)   
        self.image = Image()   
        self.cv_bridge = CvBridge()
        self.camera_matrix = None 
        self.dist_coeffs = None 
        # option = apriltag.DetectorOptions(
        #     families='tag36h11',
        #     border=1,
        #     nthreads=4,
        #     quad_decimate=1.0, 
        #     quad_blur=0.0, 
        #     refine_edges=True,
        #     refine_decode=False,
        #     refine_pose=True,
        #     debug=False,
        #     quad_contours=True
        # )
        self.detector = apriltag("tag36h11")
        # self.detector = apriltag.Detector()
        self.tag_size = 0.107 
        # ob_pt1 = [-self.tag_size/2, -self.tag_size/2, 0.0]
        # ob_pt2 = [ self.tag_size/2, -self.tag_size/2, 0.0]
        # ob_pt3 = [ self.tag_size/2,  self.tag_size/2, 0.0]
        # ob_pt4 = [-self.tag_size/2,  self.tag_size/2, 0.0]
        # ob_pts = ob_pt1 + ob_pt2 + ob_pt3 + ob_pt4
        # self.object_pts = np.array(ob_pts).reshape(4,3)
        self.tag_points = np.array([[-self.tag_size/2, -self.tag_size/2, 0], 
                                    [self.tag_size/2, -self.tag_size/2, 0], 
                                    [self.tag_size/2, self.tag_size/2, 0], 
                                    [-self.tag_size/2, self.tag_size/2, 0]]) 
        self.corners = None
        # self.opoints = np.array([
        #     -1, -1, 0,
        #     1, -1, 0,
        #     1,  1, 0,
        #     -1,  1, 0,
        #     -1, -1, -2*1,
        #     1, -1, -2*1,
        #     1,  1, -2*1,
        #     -1,  1, -2*1,
        #     ]).reshape(-1, 1, 3) * 0.5 * self.tag_size
        self.color_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
            ) 
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        ) 
        self.apriltag_pose_pub = self.create_publisher(
            PoseStamped, '/apriltag_pose', 10
        )
        # self.pub_timer = self.create_timer(0.1, self.estimate_pose)
        self.start_time = time.time()
        
    def image_callback(self, msg):  
        # self.get_logger().info('Receiving video frame') 
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 
        self.estimate_pose()

        # cv2.line(self.image, self.start_point[0], end_point, line_color, thickness=2)


        cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("image", self.image)
        cv2.waitKey(1) 

    def camera_info_callback(self, msg): 
        self.dist_coeffs = np.array(msg.d)
        self.camera_matrix = np.array(msg.k).reshape(3, 3) 
    
    def estimate_pose(self): 
        if (time.time() - self.start_time) < 1: 
            self.get_logger().info("Holding pose estimation ...")
            return 
        # Convert color image to gray image 
        gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY) 
        # Detect AprilTag within the gray image 
        detection = self.detector.detect(gray_image)
        if len(detection) > 1:        # If more than one AprilTag is detected 
            self.tag_id = -1
            self.get_logger().info("Please use only one apriltag!!!") 
        elif len(detection) == 0:     # If no AprilTag is detected 
            self.tag_id = -1
            self.get_logger().info("No apriltag is detected!!!") 
            self.corners = None 
        else: 
            detection = detection[0]  # Only one AprilTag
            print(detection)
            self.get_logger().info("Detect AprilTag ID: %s" %detection['id']) 
            # Four corners are detected 
            self.corners = detection['lb-rb-rt-lt']

            # Solve the rotation vectors and translation vectors between the AprilTag and camera 

            # good, prvecs, ptvecs = cv2.solvePnP(self.object_pts, 
            #                                     corners, 
            #                                     self.camera_matrix, 
            #                                     self.dist_coeffs, 
            #                                     flags=cv2.SOLVEPNP_ITERATIVE) 

            apriltag


            _ ,rvec, tvec = cv2.solvePnP(self.tag_points, 
                                         self.corners, 
                                         self.camera_matrix, 
                                         self.dist_coeffs,
                                         flags=cv2.SOLVEPNP_ITERATIVE) 
            
            rotation, _ = cv2.Rodrigues(rvec)
            translation = tvec.squeeze() 

            print("==========")
            print(translation)
            print(rotation)


            q = Rotation.from_matrix(rotation).as_quat()
            T = np.eye(4)
            T[0:3, 0:3] = rotation 
            T[0:3, 3] = translation 

            apriltag_pose = PoseStamped()
            apriltag_pose.header.stamp = self.get_clock().now().to_msg()

            apriltag_pose.pose.position.x = tvec[0][0]
            apriltag_pose.pose.position.y = tvec[1][0] 
            apriltag_pose.pose.position.z = tvec[2][0] 
            apriltag_pose.pose.orientation.w = q[0]
            apriltag_pose.pose.orientation.x = q[1]
            apriltag_pose.pose.orientation.y = q[2]
            apriltag_pose.pose.orientation.z = q[3]
            self.apriltag_pose_pub.publish(apriltag_pose)




def main(args=None): 
    rclpy.init(args=args) 
    node = AprilTagDetection("AprilTagDetection") 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 