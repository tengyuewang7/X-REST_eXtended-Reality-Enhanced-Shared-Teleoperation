#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import rclpy 
from rclpy.node import Node   
import cv2 

class ViewImages(Node): 
    def __init__(self, name):
        super().__init__(name)   
        self.image1 = Image()   

        self.image1_sub = self.create_subscription(
            Image, '/camera/depth/color/points', self.image1_callback, 10
            ) 

    def image1_callback(self, msg):  
        # self.get_logger().info('Receiving video frame') 
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        center_idx = np.array(depth_array.shape) / 2
        # print ('center depth:', depth_array[center_idx[0], center_idx[1]])
        print(1)
        print ('center depth:', depth_array[10, 10])


    
def main(args=None): 
    rclpy.init(args=args) 
    node = ViewImages("aaaa") 
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()


# def convert_depth_image(ros_image):
#     bridge = CvBridge()
#      # Use cv_bridge() to convert the ROS image to OpenCV format
#     try:
#      #Convert the depth image using the default passthrough encoding
#         depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
#         depth_array = np.array(depth_image, dtype=np.float32)
#         center_idx = np.array(depth_array.shape) / 2
#         print ('center depth:', depth_array[center_idx[0], center_idx[1]])

#     except CvBridgeError, e:
#         print e
#      #Convert the depth image to a Numpy array


# def pixel2depth():
# 	rospy.init_node('pixel2depth',anonymous=True)
# 	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
# 	rospy.spin()

# if __name__ == '__main__':
# 	pixel2depth()
    