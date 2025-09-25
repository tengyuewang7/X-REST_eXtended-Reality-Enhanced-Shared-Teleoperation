import rclpy
from rclpy.node import Node

from flexiv_msgs.msg import RobotStates, FramePose, RizonFrame 
from geometry_msgs.msg import Pose
import math
import time

import numpy as np 
import copy
from scipy.spatial.transform import Rotation as R

# import sys
# sys.path.append('/home/rvc/colcon_ws/src/to_unity/to_unity_py/to_unity_py')
from .forward_kinematics import ForwardKinematics 


class TestTCP(Node):

    def __init__(self):
        super().__init__('forward_joint_state')
        self.fk = ForwardKinematics()
        self.publisher_ = self.create_publisher(RizonFrame, 'key_frames', 10)
        self.subscription = self.create_subscription(
            RobotStates,
            'robot_states',
            self.pulish_link_position,
            10)


    def pulish_link_position(self, rs): 
        # rs = RobotStates()
        # rs.q = [np.deg2rad(10.92), np.deg2rad(-57.69), np.deg2rad(-0.73), 
        #         np.deg2rad(100.49), np.deg2rad(1.28), np.deg2rad(68.24),
        #         np.deg2rad(9.33)]
        # # rs.q = [0., 0., 0., 0., 0., 0., 0.]
        self.fk.update_link_transformations(rs.q)
        frame_info = RizonFrame()

        for i in range(1, 9): 
            frame = FramePose()
            frame.name = "link" + str(i)

            pos = self.fk.T_link[i][0:3, 3]
            quat = R.from_matrix(self.fk.T_link[i][0:3, 0:3]).as_quat()  # x, y, z, w
            frame.data = [pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]]
            frame_info.frames.append(frame)
        self.publisher_.publish(frame_info)




def main(args=None):
    rclpy.init(args=args)

    test_tcp = TestTCP()

    rclpy.spin(test_tcp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_tcp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()