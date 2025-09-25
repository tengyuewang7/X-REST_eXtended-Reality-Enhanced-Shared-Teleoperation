import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image  
import cv2 
from cv_bridge import CvBridge 

class ViewImages(Node): 
    def __init__(self, name):
        super().__init__(name)   
        self.image1 = Image()   
        self.image2 = Image()   
        self.cv_bridge = CvBridge()

        self.image1_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image1_callback, 10
            ) 
        self.image2_sub = self.create_subscription(
            Image, '/camera/depth/color/points', self.image2_callback, 10
            ) 

    def image1_callback(self, msg):  
        # self.get_logger().info('Receiving video frame') 
        self.image1 = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 

        cv2.namedWindow("image1", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("image1", self.image1)
        cv2.waitKey(1) 

    def image2_callback(self, msg):  
        # self.get_logger().info('Receiving video frame') 
        self.image2 = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 

        cv2.namedWindow("image2", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("image2", self.image2)
        cv2.waitKey(1) 
    

def main(args=None): 
    rclpy.init(args=args) 
    node = ViewImages("ViewImages") 
    import time
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 10:
        rclpy.spin_once(node)
    node.destroy_node() 
    rclpy.shutdown() 
