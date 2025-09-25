# https://pupil-apriltags.readthedocs.io/en/stable/api.html
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import PoseStamped 
from flexiv_msgs.msg import RobotStates
import numpy as np      
import cv2 
from cv_bridge import CvBridge 
from apriltag import apriltag
from scipy.spatial.transform import Rotation
import time
from pupil_apriltags import Detector


class AprilTagDetection(Node): 
    def __init__(self, name):
        super().__init__(name)   
        self.image = Image()   
        self.cv_bridge = CvBridge()
        self.camera_matrix = None 
        self.dist_coeffs = None 
        self.camera_params = None 

        self.detector = Detector(
            families="tagStandard52h13",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)

        self.tag_size = 0.1125

        self.T_opecv2ros = np.array([[ 0,  0,  1,  0], 
                                     [-1,  0,  0,  0],
                                     [ 0, -1,  0,  0],
                                     [ 0,  0,  0,  1]])
        
        self.camera_pose = None

        self.color_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
            ) 
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        ) 
        self.apriltag_pose_pub = self.create_publisher(
            PoseStamped, '/apriltag_pose', 10
        )

        self.robot_states_sub = self.create_subscription(
            RobotStates, '/robot_states', self.robot_states_callback, 10
        )
        
    def robot_states_callback(self, msg):
        self.robot_states = msg
        # print(self.robot_states.cam_pose)
        translation = np.array([self.robot_states.cam_pose.position.x, 
                                self.robot_states.cam_pose.position.y, 
                                self.robot_states.cam_pose.position.z])
        rotation = np.array([self.robot_states.cam_pose.orientation.x, 
                             self.robot_states.cam_pose.orientation.y,
                             self.robot_states.cam_pose.orientation.z,
                             self.robot_states.cam_pose.orientation.w])
        self.camera_pose = np.eye(4)
        self.camera_pose[0:3, 0:3] = Rotation.from_quat(rotation).as_matrix() 
        self.camera_pose[0:3, 3] = translation
        # print("===========")
        # print(self.camera_pose) 

        # translation = np.array([self.robot_states.flange_pose.position.x, 
        #                     self.robot_states.flange_pose.position.y, 
        #                     self.robot_states.flange_pose.position.z])
        # rotation = np.array([self.robot_states.flange_pose.orientation.x, 
        #                      self.robot_states.flange_pose.orientation.y,
        #                      self.robot_states.flange_pose.orientation.z,
        #                      self.robot_states.flange_pose.orientation.w])
        # self.flange_pose = np.eye(4)
        # self.flange_pose[0:3, 0:3] = Rotation.from_quat(rotation).as_matrix() 
        # self.flange_pose[0:3, 3] = translation
        # print(self.flange_pose)




    def image_callback(self, msg):  
        if ((self.camera_params is None) or (self.camera_pose is None)):
            return 
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 
        self.estimate_pose()

        cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("image", self.image)
        cv2.waitKey(1) 

    def camera_info_callback(self, msg): 
        self.dist_coeffs = np.array(msg.d)
        self.camera_matrix = np.array(msg.k).reshape(3, 3) 
        self.camera_params = ([self.camera_matrix[0, 0], self.camera_matrix[1, 1], 
                              self.camera_matrix[0, 2], self.camera_matrix[1, 2]])

    def estimate_pose(self): 
        # Convert color image to gray image 
        gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY) 
        # Detect AprilTag within the gray image 
        detections = self.detector.detect(gray_image, True, self.camera_params, self.tag_size)
        self.get_logger().info("Number of AprilTag detected: %d" %len(detections)) 
        if len(detections) == 0:     # If no AprilTag is detected 
            self.tag_id = -1
            self.corners = None 
        else: 
            for detection in detections:
                # print(detection) 

                cv2.drawContours(self.image, [np.int32(detection.corners)], -1, (0, 255, 0), 2) 

                T_opencv = np.eye(4)
                T_opencv[0:3, 0:3] = detection.pose_R
                T_opencv[0:3, 3] = detection.pose_t[0:3, 0]
                # T_ros = np.matmul(self.T_opecv2ros, T_opencv)
                # print(T_ros)
                print(np.matmul(self.camera_pose, T_opencv))

            # apriltag_pose = PoseStamped()
            # apriltag_pose.header.stamp = self.get_clock().now().to_msg()

            # apriltag_pose.pose.position.x = tvec[0][0]
            # apriltag_pose.pose.position.y = tvec[1][0] 
            # apriltag_pose.pose.position.z = tvec[2][0] 
            # apriltag_pose.pose.orientation.w = q[0]
            # apriltag_pose.pose.orientation.x = q[1]
            # apriltag_pose.pose.orientation.y = q[2]
            # apriltag_pose.pose.orientation.z = q[3]
            # self.apriltag_pose_pub.publish(apriltag_pose)


def main(args=None): 

    rclpy.init(args=args) 
    node = AprilTagDetection("AprilTagDetection") 
    rclpy.spin(node) 
    node.destroy_node() 
    rclpy.shutdown() 