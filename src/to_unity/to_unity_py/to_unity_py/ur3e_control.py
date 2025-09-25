import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState 

import sys
sys.path.append('/home/rvc/ur_ikfast')
from ur_ikfast import ur_kinematics
import copy
from scipy.spatial.transform import Rotation as R
import numpy as np 


class UR3eControl(Node):

    def __init__(self):
        super().__init__('ur3e_control')
        self.publisher_ = self.create_publisher(JointTrajectory, 
                                                '/joint_trajectory_controller/joint_trajectory', 
                                                10)

        self.joint_state_publisher_ = self.create_publisher(JointState, 
                                                            'ik_joint_state', 
                                                            10)

        # self.timer = self.create_timer(10, self.timer_callback) 
        self.subscription = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.listener_callback,
            10)



        self.ur3e_kine = ur_kinematics.URKinematics('ur3e') 

        # joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
        # pose_matrix = self.ur3e_kine.forward(joint_angles) # [x, y, z, w. qx, qy, qz]
        # print(pose_matrix)

        self.L = 0.00
        self.T = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]]) 

    def listener_callback(self, msg):
        pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                msg.pose.orientation.w, msg.pose.orientation.x, 
                msg.pose.orientation.y, msg.pose.orientation.z]
        # pose = self.coord_trans(pose)
        # new_pose[3] = pose[6]
        # new_pose[4] = pose[3]
        # new_pose[5] = pose[4]
        # new_pose[6] = pose[5]
        print("===========")
        print(pose)

        pose = np.array([-0.287935048, -0.140407637, 0.306456357, 1.0000000002255947, 
                         1.0000000000001356638, -4.29355962e-04, -2.06427328e-02])


        # new_pose = copy.deepcopy(pose)
        # new_pose[3] = 0.7
        # new_pose[4] = 0.7
        # new_pose[5] = 0.0
        # new_pose[6] = 0.0


        # joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
        # pose = self.ur3e_kine.forward(joint_angles) # [x, y, z, w. qx, qy, qz]
        # print(pose)
        target_q = self.ur3e_kine.inverse(pose, False) 
        print(target_q)

        joint_msg = JointState()
        joint_msg.position = list(target_q)
        self.joint_state_publisher_.publish(joint_msg)

        # msg.pose.position.x = target_pose[0]
        # msg.pose.orientation.w = target_pose[6]

    def coord_trans(self, unity_pose): 
        pose = copy.deepcopy(unity_pose)
        r = R.from_quat(unity_pose[3:]).as_matrix()

        T = np.array([[-1.37139403e-04, 4.18887194e-03, 9.99991238e-01], 
                      [-9.99998748e-01, 1.56427803e-03, 4.18865215e-03], 
                      [1.56486616e-03, 9.99989986e-01,  1.43693062e-04]])
        
        trans = np.matmul(np.matmul(T, r), T)
        quat = R.from_matrix(trans).as_quat()
        pose[3:] = quat

        L = 0.00

        pose[2] = unity_pose[1]
        pose[1] = -unity_pose[0]
        pose[0] = unity_pose[2]
        pose[0] -= L * trans[0, 2]
        pose[1] -= L * trans[1, 2]
        pose[2] -= L * trans[2, 2]

        return pose

    def timer_callback(self):
        msg = JointTrajectory() 
        msg.joint_names = ["elbow_joint", "shoulder_lift_joint", 
                           "shoulder_pan_joint", "wrist_1_joint", 
                           "wrist_2_joint", "wrist_3_joint"] 
        new_point = JointTrajectoryPoint()
        new_point.positions = [0.5] * 6
        new_point.time_from_start.sec = 1
        msg.points.append(new_point)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ur3e_control = UR3eControl()

    rclpy.spin(ur3e_control)

    ur3e_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()