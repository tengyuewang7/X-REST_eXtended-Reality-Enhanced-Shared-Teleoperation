#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo 
import numpy as np      
import cv2 
from cv_bridge import CvBridge 
import time 
from scipy.spatial.transform import Rotation
import yaml
import sys
from std_msgs.msg import String
from threading import Thread
sys.path.append('/home/rvc/colcon_ws/src/flexiv/flexiv_py/flexiv_py')
from robot_operation import RobotOperation
sys.path.insert(0, "/home/rvc/flexiv/flexiv_rdk/lib_py")
import flexivrdk
from flexiv_msg.msg import RigidBody
from flexiv_msg.msg import Marker

class Camera2CamerCalibration(Node): 
    def __init__(self, args):
        super().__init__(args["node_name"])   

        self.log = flexivrdk.Log() 
        self.mode = flexivrdk.Mode 
        self.plan_info = flexivrdk.PlanInfo()
        self.robot_states = flexivrdk.RobotStates()
        # Instantiate robot interface
        self.robot = flexivrdk.Robot(args["robot_ip"], args["local_ip"]) 
        # Enable robot
        RobotOperation.enable_robot(self) 
        # Wait for the robot to become operational
        RobotOperation.wait_for_robot(self) 
        # Start a thread for free drive the robot arm 
        self.target_pose_movement = np.zeros(3)
        self.cv_bridge = CvBridge()
        self.camera1 = "D435"
        self.camera2 = "D435I"
        # Store images, camera matrix and distortion coefficients as dicts 
        self.images = {self.camera1: Image(), self.camera2: Image()}
        self.camera_matrix = {self.camera1: np.zeros(5), self.camera2: np.zeros(5)}
        self.dist_coeffs = {self.camera1: np.zeros((3, 3)), self.camera2: np.zeros((3, 3))}
        # Store the Marker information: x,y,z
        self.Marker = np.zeros(3)
        # Store the transformation matrix of Motion Capture to robot base
        self.MC2Base = np.eye(4)
        # Store the fixed transformation of marker2cam:x,y,z
        self.marker2cam = np.array([0.000 ,-0.000, 0.008])
        # Define coefficients of AprilTag
        self.tag_size = 0.107 
        self.tag_points = np.array([[-self.tag_size/2, -self.tag_size/2, 0], 
                                    [self.tag_size/2, -self.tag_size/2, 0], 
                                    [self.tag_size/2, self.tag_size/2, 0], 
                                    [-self.tag_size/2, self.tag_size/2, 0]])
        # Subscribe image from D435 
        self.image_sub_1 = self.create_subscription(
            Image, '/D435/color/image_raw', self.image_callback_1, 10
            ) 
        # Subscribe camera information of D435 
        self.camera_info_sub_1 = self.create_subscription(
            CameraInfo, '/D435/color/camera_info', self.camera_info_callback_1, 10
        ) 
        # Subscribe image from D435I 
        self.image_sub_2 = self.create_subscription(
            Image, '/D435I/color/image_raw', self.image_callback_2, 10
            ) 
        # Subscribe camera information of D435I
        self.camera_info_sub_2 = self.create_subscription(
            CameraInfo, '/D435I/color/camera_info', self.camera_info_callback_2, 10
        )
        # Subscribe marker information of MC
        self.marker_sub = self.create_subscription(
            Marker, 'Natnet/Marker', self.marker_callback, 10
            )
        self.start_time = time.time()  
        # Store transformation 
        self.T = np.empty((1, 4, 4)) 
        # Start a timer which conduct one calibration every 0.1 seconds
        self.timer = self.create_timer(0.1, lambda:self.calibration(False))
        self.Thread_MC = Thread(target=self.calibration_MC)
        

    def image_callback_1(self, msg):  
        self.images[self.camera1] = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 
        cv2.namedWindow("D435", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("D435", self.images[self.camera1])
        cv2.waitKey(1) 
    
    def image_callback_2(self, msg):  
        self.images[self.camera2] = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 
        cv2.namedWindow("D435I", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("D435I", self.images[self.camera2])
        cv2.waitKey(1) 
    
    def camera_info_callback_1(self, msg): 
        self.dist_coeffs[self.camera1] = np.array(msg.d)
        self.camera_matrix[self.camera1] = np.array(msg.k).reshape(3, 3) 
    
    def camera_info_callback_2(self, msg): 
        self.dist_coeffs[self.camera2] = np.array(msg.d)
        self.camera_matrix[self.camera2] = np.array(msg.k).reshape(3, 3)

    def marker_callback(self, msg):
        
        self.Marker[0] = msg.x
        self.Marker[1] = msg.y
        self.Marker[2] = msg.z
        #print("geting: ", self.Marker)



    def get_camera1_to_base(self): 
        # Update the robot state 
        self.robot.getRobotStates(self.robot_states) 
        # Get camera pose --> [px,py,pz,qw,qx,qy,qz]
        cam_pose = np.array(self.robot_states.camPose)
        # Switch the pose representation: [px,py,pz,qw,qx,qy,qz] -> [px,py,pz,qx,qy,qz,qw]
        cam_pose[3], cam_pose[6] = cam_pose[6], cam_pose[3]
        T_cam = np.eye(4)
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
        T_cam[0:3, 0:3] = Rotation.from_quat(np.array(cam_pose[3:])).as_matrix()
        T_cam[0:3, 3] = cam_pose[0:3]

        # Calibrate the camera1 from robot to optical: ROS2(Robot) vs Optical(Camera) Coordination Systems
        T = np.array([[0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [1, 0, 0, 0],
                      [0, 0, 0, 1]])
        T_cam2base = np.matmul(T_cam, T)
        return T_cam2base,T_cam
    def calibration_MC(self):
        T_camera1ToBase, T_cam = self.get_camera1_to_base()
        # Move robot to home pose
        self.get_logger().info("Moving to home pose")
        self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.executePrimitive("Home()")
        # Wait for the primitive to finish
        while (self.robot.isBusy()):
            time.sleep(1)

        Origin_temp = self.Marker.copy()
        self.get_logger().info("Origin_temp is \n%s\n" %str(Origin_temp))
        if(sum(self.Marker)!=0):
            
            
            #Moving in +x
            self.target_pose_movement = np.array([0.1,0,0])
            RobotOperation.Move_target(self,self.target_pose_movement)
            time.sleep(1)
            X_temp = self.Marker.copy()
            self.get_logger().info("X_temp is \n%s\n" %str(X_temp))
            #Moving in +y
            self.target_pose_movement = np.array([0,0.1,0])
            RobotOperation.Move_target(self,self.target_pose_movement)
            time.sleep(1)
            Y_temp = self.Marker.copy()
            self.get_logger().info("Y_temp is \n%s\n" %str(Y_temp))
            #Moving in +z
            self.target_pose_movement = np.array([0,0,0.1])
            RobotOperation.Move_target(self,self.target_pose_movement)
            time.sleep(1)
            Z_temp = self.Marker.copy()
            self.get_logger().info("Z_temp is \n%s\n" %str(Z_temp))
            self.MC2Base[0:3,0] = (X_temp-Origin_temp)/np.linalg.norm(X_temp-Origin_temp)
            self.MC2Base[0:3,1] = (Y_temp-Origin_temp)/np.linalg.norm(Y_temp-Origin_temp)
            self.MC2Base[0:3,2] = (Z_temp-Origin_temp)/np.linalg.norm(Z_temp-Origin_temp)
        self.MC2Base[0:3,3] = -np.matmul(self.MC2Base[0:3,0:3],(T_cam[0:3, 3] - self.marker2cam)) + Origin_temp
        self.get_logger().info("MC2Base's  is \n%s \n %s \n %s \n%s \n" %(str(self.MC2Base[0:3,0]),str(self.MC2Base[0:3,1]),str(self.MC2Base[0:3,2]),str(self.MC2Base[0:3,3])))
        self.MC2Base = np.linalg.inv(self.MC2Base)

    def calibration(self, cam2camflag ):
        # Make sure images and camera information have been subscribed 
        while ((time.time() - self.start_time) < 1): 
            print("Holding calibration ...")
            time.sleep(0.1)
        # Pose estimation of AprilTag 
        T_camera1ToBase, T_cam = self.get_camera1_to_base()
        if(cam2camflag):
            T_tagToCamera1, tag_id_1 = \
                RobotOperation.detect_apriltag_pose(self.images[self.camera1], 
                                                    self.tag_points, 
                                                    self.camera_matrix[self.camera1], 
                                                    self.dist_coeffs[self.camera1])
            
            T_tagToCamera2, tag_id_2 = \
                RobotOperation.detect_apriltag_pose(self.images[self.camera2], 
                                                    self.tag_points, 
                                                    self.camera_matrix[self.camera2], 
                                                    self.dist_coeffs[self.camera2])
            
            if tag_id_1 != tag_id_2 and tag_id_1 != -1 and tag_id_2 != -1: 
                self.get_logger().warn("Cameras are detecting different apriltags!!!") 
                time.sleep(1)
                return
            T_inv = np.linalg.inv(T_tagToCamera2)
            T_camera2ToCamera1 = np.matmul(T_tagToCamera1, T_inv) 
            
            T_camera2ToBase = np.matmul(T_camera1ToBase, T_camera2ToCamera1)
            T_tagToBase_1 = np.matmul(T_camera1ToBase, T_tagToCamera1)
            T_tagToBase_2 = np.matmul(T_camera2ToBase, T_tagToCamera2)     
            if tag_id_1 == tag_id_2 and tag_id_1 != -1: 
                self.T = np.append(self.T, np.expand_dims(T_camera2ToBase, 0), axis=0)
            self.get_logger().info("\n  AprilTag at Base according to D435 is: \n%s\
                    AprilTag at Base according to D435I is: \n%s"%(str(T_tagToBase_1), str(T_tagToBase_2))) 
        

    
    def dump_yaml(self):
        self.Thread_MC.join() 
        T_average = np.sum(self.T, axis=0) / self.T.shape[0]
        q = Rotation.from_matrix(T_average[0:3, 0:3]).as_quat() # xyzw
        yaml_output = {"TFPublisher": {"ros__parameters": 
            { "camera2": {
            "rotation": q.tolist(), "translation": T_average[0:3, 3].tolist(),
            "desc": "The transformation of camera2 based on robot base.(xyzqxqyqzqw)"},
            "MC":{
            "rotation": Rotation.from_matrix(self.MC2Base[0:3, 0:3]).as_quat().tolist(), "translation": self.MC2Base[0:3, 3].tolist(),
            "desc": "The transformation of Motion Capture based on robot base.(xyzqxqyqzqw) That is, P_in_MC can be transfer to P_in_base."
            }}
        }}

        # Dump calibration information 
        with open('/home/rvc/colcon_ws/src/flexiv/flexiv_py/config/camera2_tf.yaml', 'w') as outfile:
            yaml.dump(yaml_output, outfile, default_flow_style=False)
        self.get_logger().info("Finish dumping transformation information to /config/my_data.yaml")



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Marker, 'Natnet/Marker', self.marker_callback,
            10)
        self.subscription  # prevent unused variable warning

    def marker_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.x)

def test(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()



def main(): 
    args = {"robot_ip": "192.168.2.100",
            "local_ip": "192.168.2.101",
            "node_name": "Camera2CamerCalibration"}

    rclpy.init() 
    node_1 = Camera2CamerCalibration(args)
    one_time = True 
    rclpy.get_default_context().on_shutdown(node_1.dump_yaml)
    while rclpy.ok() and (time.time() - node_1.start_time) < 30:
        rclpy.spin_once(node_1)
        if(one_time):
            node_1.Thread_MC.start()
            one_time = False
    # rclpy.spin(node_1)
     
    node_1.destroy_node() 
    rclpy.shutdown() 
    # test()