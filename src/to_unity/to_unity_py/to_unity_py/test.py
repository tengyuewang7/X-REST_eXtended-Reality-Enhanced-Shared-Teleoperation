import numpy as np 
import matplotlib.pyplot as plt

# v1 = [1, 2, 3]
# a = 1
# b = 2
# c = 3

# v2 = [0, 0, 1]
# m = 4
# n = 6
# l = -2

# k = np.linalg.norm(v1) 

# x = np.linspace(-k, k, 10000) 
# y = np.linspace(-k, k, 10000) 
# max_value = 0.0
# for xx in x:
#     for yy in y:
#         zz = -(a * xx + b * yy) / c 
#         if (xx ** 2 + yy ** 2 + zz ** 2 - k ** 2) < 0.0001:
#             if (m * xx + n * yy + l * zz) > max_value:
#                 max_value = zz
#                 print(xx, yy, zz) 
#                 print()



# n = v1 / np.linalg.norm(v1) 
# v3 = v2 / np.linalg.norm(v2)
# proj = (v3 -  np.dot(n, v3) * n)
# print(proj / np.linalg.norm(proj) * k) 


# x = np.linspace(0.03, 0.2, 10000)
# y = x * (1.0 / x - 1.0 / 0.2) ** 2 / 50.0

# xx = np.linspace(0.03, 0.1, 10000)
# yy = xx * (1.0 / xx - 1.0 / 0.1) ** 1.5 / 7.0

# plt.plot(x, y) 
# plt.plot(xx, yy) 
# plt.show() 

# x = np.linspace(0.01, 0.2, 10000)
# y = 0.9 * np.exp(-100 * x) + 0.1
# plt.plot(x, y) 

# yy = 0.98 * np.exp(-30 * x) + 0.02
# plt.plot(x, yy) 


# t = np.linspace(0, 10, 10000)
# y = (0.1 + 0.01 * np.cos(6*t)) * np.sin(1.0 * t) + (-0.11) 
# x = (0.1 + 0.01 * np.cos(6*t)) * np.cos(1.0 * t) + (0.6888) 

# plt.plot(x, y) 


# plt.show() 


# if (maxAbsU > 0.03):
#     u = u / torch.max(torch.abs(u)) * 0.03
#     # u = torch.clamp(u, max=0.03)
# self.th = 0.03


# self.th = 0.025 * np.sin(5 * np.pi * maxDis) + 0.005
# self.th = 0.04 * np.cos(5.0 * np.pi * (maxDis - 0.01)) + 0.005
# if (self.th < 0.005):
#     self.th = 0.005

# if (maxAbsU > self.th):
#     u = u / torch.max(torch.abs(u)) * self.th 
#     print("limited")
# else:
#     print("free") 


# # Method 1, slow response, smooth 
# (maxDis - self.last_dis)
# if (maxDis > self.last_dis): 
#     self.th += 0.002
# else:
#     self.th -= 0.001
# if (self.th > 0.03):
#     self.th = 0.03
# if (self.th < 0.005):
#     self.th = 0.005 

# if (maxAbsU > self.th): 
#     u = u / torch.max(torch.abs(u)) * self.th 
#     print("limited")
# else:
#     print("free")
# self.last_dis = maxDis

# Method2, quick response, damping 
# threshold = 0
# if (maxDis < 0.05): 
#     u = torch.clamp(u, max=0.03)
#     if (maxAbsU > 0.03):
#         threshold = 0.03
#         u = u / torch.max(torch.abs(u)) * 0.03
#         # u = torch.clamp(u, max=0.03)
#         print("maxDis<0.05")
#     print("free u: ", u)
# elif (maxDis > 0.1): 
#     if (maxAbsU > 0.015): 
#         threshold = 0.015
#         u = u / torch.max(torch.abs(u)) * 0.015
#         print("maxDis > 0.1")
# else: 
#     threshold = -(0.015/0.05) * maxDis + 0.045
#     if (maxAbsU > threshold):
#         u = u / torch.max(torch.abs(u)) * threshold 
#         print("middle")
# self.th = threshold

# threshold = 0.0075 * torch.tanh(-(maxDis-0.075)*10) + 0.0225 
# threshold = 0.0125 * np.tanh(-(maxDis-0.03)*30) + 0.035 / 2.0
# self.th = threshold

# if (is_slow):
#      u = 0.1 * (j_eef_T @ torch.inverse(self.j_eef @ j_eef_T + lmbda) @ dpose) 
#      threshold = 0.005

# if (maxAbsU > threshold): 
#     u = u / torch.max(torch.abs(u)) * threshold 

# u = torch.clamp(u, max=threshold)
# delta_time = time.time() - self.start_time
# self.get_logger().info(f'time: {delta_time}')


# import torch
# import torch.nn.functional as F

# def find_closest_points(tensor1, tensor2):
#     # 计算两个张量中每对点之间的距离
#     pairwise_distances = F.pairwise_distance(tensor1.unsqueeze(1), tensor2.unsqueeze(0), p=2)

#     # 找到最小距离的索引
#     min_distance_index = torch.argmin(pairwise_distances)

#     # 将索引转换为在tensor1和tensor2中的对应点的索引
#     tensor1_index = min_distance_index // tensor2.size(0)
#     tensor2_index = min_distance_index % tensor2.size(0)

#     # 获取最近的两个点
#     closest_points = tensor1[tensor1_index], tensor2[tensor2_index]

#     return closest_points 

# tensor1 = torch.tensor([[1, 2], [3, 4], [5, 6]])
# tensor2 = torch.tensor([[7, 8], [9, 10]])

# closest_points = find_closest_points(tensor1, tensor2)
# print(closest_points)

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from flexiv_msgs.msg import RobotStates

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            RobotStates,
            'robot_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        # flange = array_to_matrix(ros_pose_to_array(msg.flange_pose))
        # camera = array_to_matrix(ros_pose_to_array(msg.cam_pose))
        
        # alpha = 0.0
        # a = -0.07717
        # theta = np.deg2rad(-90.0)
        # d = -0.0445
        # T_flange2camera = np.array([[np.cos(theta),                 -np.sin(theta),                0,              a                  ], 
        #             [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d ], 
        #             [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),  np.cos(alpha),  np.cos(alpha) * d ],
        #             [0,                             0,                              0,              1                 ]]) 

        # TT = np.array([[0, 1, 0, -0.07717], 
        #                [-1, 0, 0, 0], 
        #                [0, 0, 1, -0.0445],
        #                [0, 0, 0, 1]]) 
        
        flange = ros_pose_to_array(msg.flange_pose)
        camera = ros_pose_to_array(msg.cam_pose)

        # flange: [px, py, pz, qx, qy, qz, qw]
        # camera: [px, py, pz, qx, qy, qz, qw]
        
        TT = np.array([-0.07717, 0.0, -0.0445, 0.0, 0.0, 0.70710678, -0.70710678])

        extened_T = np.array([TT[0], TT[1], TT[2], 0]) 
        con = np.array([-flange[3], -flange[4], -flange[5], flange[6]])
        offest = quat_mul(quat_mul(flange[3: ], extened_T), con)
        new_cam = np.array([0.0] * 7)
        new_cam[0:3] = offest[:3] + flange[0:3]  # position
        new_cam[3:] = -quat_mul(flange[3:], TT[3:])  # rotation 

        print(camera) 
        print(new_cam)
        print()

def quat_mul(a, b):
    # a, b: [x, y, z, w]
    x1, y1, z1, w1 = a[0], a[1], a[2], a[3]
    x2, y2, z2, w2 = b[0], b[1], b[2], b[3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    return np.array([x, y, z, w])

def array_to_matrix(pose):

    T = np.eye(4)
    T[0:3, 3] = pose[0:3] 
    T[0:3, 0:3] = R.from_quat(pose[3:]).as_matrix()
    return T

def ros_pose_to_array(pose): 
    # [px, py, pz, qx, qy, qz, qw]
    return np.array([pose.position.x, pose.position.y, pose.position.z, 
                         pose.orientation.x, pose.orientation.y, pose.orientation.z, 
                         pose.orientation.w]) 


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()