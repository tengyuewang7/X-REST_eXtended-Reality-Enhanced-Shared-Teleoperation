import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point 
import numpy as np      
import cv2 
from cv_bridge import CvBridge 
import time
from scipy.spatial.transform import Rotation

class ArucoTagDetection(Node): 
    def __init__(self, name):
        super().__init__(name)   
        self.color_image = Image()
        self.depth_image = Image()   
        self.cv_bridge = CvBridge()
        self.color_camera_matrix = None 
        self.color_dist_coeffs = None 
        self.color_camera_params = None
        self.depth_camera_matrix = None 
        self.depth_dist_coeffs = None 
        self.depth_camera_params = None  
        self.corners = None
        self.ids = None
        self.tag_size = 0.145

        self.color_window_name = 'Color Image Window'
        cv2.namedWindow(self.color_window_name)
        cv2.setMouseCallback(self.color_window_name, self.mouse_callback)
        self.depth_window_name = 'Depth Image Window'
        cv2.namedWindow(self.depth_window_name)
        self.T_opecv2ros = np.array([[ 0,  0,  1,  0], 
                                     [-1,  0,  0,  0],
                                     [ 0, -1,  0,  0],
                                     [ 0,  0,  0,  1]])
        
        self.camera_pose = None

        self.color_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_image_callback, 10
            ) 
        self.color_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.color_camera_info_callback, 10
        )
        self.depth_image_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_image_callback, 10
            ) 
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.depth_camera_info_callback, 10
        )  
        self.arucotag_pose_pub = self.create_publisher(
            PoseStamped, '/arucotag_pose', 10
        )
        self.cam_point_pub = self.create_publisher(
            Point, '/cam_point', 10
        )
    def detect_markers(self):

        # 检测图像中的aruco marker, 并且计算每个marker作为世界坐标系对应的外参矩阵

        # 通道交换：RGB到RBG
        # color_image = color_image[:, :, [1, 2, 0]]
        # gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)


        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # 子像素级别优化
        corners, ids, _ = cv2.aruco.detectMarkers(self.color_image, aruco_dict, parameters=parameters)
        # print("corners: ",corners)
        self.corners = corners
        self.ids = ids
        arucotag_pose_msg = PoseStamped()
        arucotag_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.rvecs = []
        self.tvecs = []
        
        if len(corners) > 0:
            # print(corners)
            ids = ids.flatten()	 # Flatten the ArUCo IDs list
            for (markerCorner, markerID) in zip(corners, ids):
                # if 'Corner' not in tag_pose:
                #     tag_pose['Corner'] = {}
                # if 'ProjectionMatrix' not in tag_pose:
                #     tag_pose['ProjectionMatrix'] = {}
                # if 'Rvecs' not in tag_pose:
                #     tag_pose['Rvecs'] = {}
                # if 'Tvecs' not in tag_pose:
                #     tag_pose['Tvecs'] = {}
                # Estimate pose of marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.tag_size,
                                                                        self.color_camera_matrix, self.color_dist_coeffs)
                # get the projection matrix
                self.rvecs.append(rvec)
                self.tvecs.append(tvec)
                for rvec, tvec in zip(rvec, tvec):
                    # Convert rotation vector to quaternion
                    rmat, _ = cv2.Rodrigues(rvec)
                    quat = Rotation.from_matrix(rmat).as_quat()
                    # Create PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = 'camera_frame'
                    pose_msg.pose.position.x = tvec[0][0]
                    pose_msg.pose.position.y = tvec[0][1]
                    pose_msg.pose.position.z = tvec[0][2]
                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]
                    # print(quat)
                    # Publish the PoseStamped message
                    self.arucotag_pose_pub.publish(pose_msg)
                # tag_pose['Corner'][f'{markerID}'] = markerCorner
                # tag_pose['ProjectionMatrix'][f'{markerID}'] = p
                # tag_pose['Rvecs'][f'{markerID}'] = rvecs
                # tag_pose['Tvecs'][f'{markerID}'] = tvecs
                # print(pose)

        else:
            print('no tag detected!')
        # print(tag_pose)
    def detect_ball(self):
        # Define the color range for the silver ball in BGR format
        lower_bound = np.array([90, 90, 90])   # BGR lower threshold
        upper_bound = np.array([130, 130, 130])  # BGR upper threshold

        # Create a mask based on the color range
        mask = cv2.inRange(self.color_image, lower_bound, upper_bound)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(mask, (11, 11), 0)

        # Find contours on the mask
        contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours and detect the largest contour, which is likely the ball
        for contour in contours:
            # Calculate area and filter out small contours
            area = cv2.contourArea(contour)
            if area > 300:  # Filter out very small areas (noise)
                # Calculate the perimeter of the contour
                perimeter = cv2.arcLength(contour, True)
                
                # Avoid dividing by zero
                if perimeter == 0:
                    continue
                
                # Calculate circularity: (4 * pi * area) / (perimeter^2)
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                # Filter based on circularity (perfect circle has circularity = 1)
                if 0.7 < circularity < 1.2:  # Adjust thresholds based on how round the ball is
                    # Draw a bounding circle around the detected ball
                    ((x, y), radius) = cv2.minEnclosingCircle(contour)
                    if 10 < radius < 100:  # Adjust based on the expected size of the ball
                        # Draw the circle and the center
                        cv2.circle(self.color_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                        cv2.circle(self.color_image, (int(x), int(y)), 5, (255, 0, 0), -1)
    # Define the mouse callback function
    def mouse_callback(self, event, x, y, flags, param):
        tag_point_3d = Point()
        if event == cv2.EVENT_LBUTTONDOWN:  # Left button click event
            print(f"Mouse clicked at position: x={x}, y={y}")
            image_point = np.array([[x, y, 1]], dtype=np.float32)
            depth = self.depth_image[y, x] * 0.001
            image_point_3d = np.dot(np.linalg.inv(self.color_camera_matrix), image_point.T) * depth
            # Compute the tag's rotation matrix
            if self.ids is not None:
                for i in range(len(self.ids)):
                    # print(i)
                    R, _ = cv2.Rodrigues(self.rvecs[i])
                    # Transform the 3D point from camera to tag coordinates
                    point_3d = np.dot(R.T, image_point_3d - self.tvecs[i].reshape((3,1)))
                    tag_point_3d.x = float(point_3d[0])
                    tag_point_3d.y = float(point_3d[1])
                    tag_point_3d.z = float(point_3d[2])
                    self.cam_point_pub.publish(tag_point_3d)

            cv2.circle(self.color_image, (x, y), 5, (255, 0, 0), -1)
            cv2.imshow(self.color_window_name, self.color_image)

    def color_image_callback(self, msg):  
        if ((self.color_camera_params is None)):
            return 
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        # self.color_image = cv2.imread("/home/rvc/colcon_ws/local_image.jpg")
        # cv2.imshow("Origin", self.color_image) 
        self.detect_markers()
        # self.detect_ball()
        # print(len(self.ids))
        # print(len(self.rvecs))
        # print(len(self.tvecs))   
        if self.ids is not None:
            # self.color_image = cv2.aruco.drawDetectedMarkers(self.color_image, self.corners, self.ids)

            # Optionally, draw the ID of the marker
            for i in range(len(self.ids)):
                corner = self.corners[i].reshape((4, 2)).astype(int)
                cv2.polylines(self.color_image, [corner], isClosed=True, color=(0, 255, 0), thickness=3)
                cv2.drawFrameAxes(self.color_image, self.color_camera_matrix, self.color_dist_coeffs, self.rvecs[i], self.tvecs[i], self.tag_size*0.3 ,2)
        # cv2.imwrite('image.jpg', self.color_image)
        cv2.imshow(self.color_window_name, self.color_image)
        cv2.waitKey(1)
            
        
    def color_camera_info_callback(self, msg): 
        self.color_dist_coeffs = np.array(msg.d)
        self.color_camera_matrix = np.array(msg.k).reshape(3, 3) 
        self.color_camera_params = ([self.color_camera_matrix[0, 0], self.color_camera_matrix[1, 1], 
                              self.color_camera_matrix[0, 2], self.color_camera_matrix[1, 2]])
    def depth_image_callback(self, msg):  
        if ((self.depth_camera_params is None)):
            return 
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, '16UC1')
        cv2.imshow(self.depth_window_name, self.depth_image)
        cv2.waitKey(1) 
            
        
    def depth_camera_info_callback(self, msg): 
        self.depth_dist_coeffs = np.array(msg.d)
        self.depth_camera_matrix = np.array(msg.k).reshape(3, 3) 
        self.depth_camera_params = ([self.depth_camera_matrix[0, 0], self.depth_camera_matrix[1, 1], 
                              self.depth_camera_matrix[0, 2], self.depth_camera_matrix[1, 2]])

def main(args=None): 

    rclpy.init(args=args) 
    node = ArucoTagDetection("ArucoTagDetection") 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

