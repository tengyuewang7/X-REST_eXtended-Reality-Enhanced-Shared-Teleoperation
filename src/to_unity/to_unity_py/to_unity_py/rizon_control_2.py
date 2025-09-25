import numpy as np
import torch
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from flexiv_msgs.msg import HanoiGrab, HanoiPose, RobotStates, ModeController ,DigitalOutput, DigitalState 
from flexiv_msgs.srv import SetMode 
from geometry_msgs.msg import PoseStamped, Pose
import time
import copy
import threading 
from .forward_kinematics import ForwardKinematics 
from .hanoi_state import Hanoi
from .utils import * 

class RizonControl(Node): 
    def __init__(self):
        super().__init__('rizon_control') 
        cb_group_1 = MutuallyExclusiveCallbackGroup()
        cb_group_2 = ReentrantCallbackGroup()
        cb_group_3 = MutuallyExclusiveCallbackGroup()
        cb_group_4 = MutuallyExclusiveCallbackGroup() 
        cb_group_5 = MutuallyExclusiveCallbackGroup() 
        cb_group_6 = ReentrantCallbackGroup() 
        
        self.robot_states_sub = self.create_subscription(RobotStates, 
                                                         "robot_states", 
                                                         self.robot_states_callback, 
                                                         qos_profile=1, 
                                                         callback_group=cb_group_2) 
        
        
        self.mode_controller_pub = self.create_publisher(ModeController, "mode_controller", 10) 
        self.update_timer = self.create_timer(0.001, self.update, callback_group=cb_group_4)
        # self.update_timer_2 = self.create_timer(0.001, self.update_2, callback_group=cb_group_3)

        self.mode_client = self.create_client(SetMode, "set_mode", callback_group=cb_group_5)

        self.L = 0.08
        self.hanoi_at_start = True 
        self.digital_input_0 = True 

        self.damping  = 0.05 

        self.current_q = None
        self.current_pose = None 
        self.current_dq = None
        self.target_pose = None 
        self.fk = ForwardKinematics()
        self.j_eef = None 
        self.device = 'cuda:0'
   

        # Move robot to home 
        request = SetMode.Request()
        request.mode = request.MODE_NRT_JOINT_POSITION
        _ = self.mode_client.call_async(request) 
        time.sleep(1)
        msg = ModeController()
        msg.mode = msg.MODE_NRT_JOINT_POSITION 
        msg.positions = [0.0, -0.698132, 0.0, 1.570796, 0.0, 0.698132, 0.0]
        # msg.positions = [-5.4173e-10, -1.1241e+00,  1.0339e-09,  1.5603e+00, -9.6400e-11, 2.6843e+00, -8.8768e-10]

        self.obstacle = torch.tensor([[0.5, 0.2, 0.3]], dtype=torch.float)

        # self.obstacle = torch.tensor([[0.6888, -0.01, 0.291], [0.7, 0.01, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], 
        #                               [0.7, 0.02, 0.25], [0.67, -0.03, 0.3], ], dtype=torch.float)

        msg.max_vel = [1.] * 7
        msg.max_acc = [1.] * 7
        self.mode_controller_pub.publish(msg)
        self.mode_controller_pub.publish(msg)
        time.sleep(3) 
        self.flag = False
        request.mode = request.MODE_RT_JOINT_POSITION
        _ = self.mode_client.call_async(request) 
        time.sleep(2)
        self.flag = True


        self.start_time = time.time()
        self.target_pose = None 

        self.saved_data = torch.empty((0, 13))
        self.vel_data = torch.empty((0, 7))
        self.u_data = torch.empty((0, 8))

        self.th = 0.0
        self.last_dis = 0.0
        self.min_dis = 10.0
        self.loop_count = 0
        self.last_time = 0


    def on_shutdown(self):
        request = SetMode.Request()
        request.mode = request.MODE_NRT_JOINT_POSITION
        # _ = self.mode_client.call_async(request) 
        msg = ModeController()
        msg.mode = msg.MODE_RT_JOINT_POSITION 
        msg.positions = self.current_q
        msg.velocities = [0.0] * 7
        self.mode_controller_pub.publish(msg)
        torch.save(self.saved_data, 'pos_data.pt')
        torch.save(self.vel_data, 'vel_data.pt')
        torch.save(self.u_data, 'u_data.pt')
        print("On shutdown ... ")



    def control_ik(self, dpose, is_slow=False, good_rot=False): 
        # solve damped least squares

        j_eef_T = torch.transpose(self.j_eef, 0, 1)
        lmbda = torch.eye(6) * (self.damping ** 2)

        value = torch.exp(-10 * torch.norm(dpose[0:3]))
    
        if (good_rot):
            value = 1.0

        dpose[3:] = value * dpose[3:]

        maxDis = torch.norm(dpose)
        # scale = 0.3 * np.cos(5 * np.pi * (maxDis - 0.1)) + 0.2
        scale = 0.98 * torch.exp(-30 * maxDis) + 0.02
        # scale = 0.95 * torch.exp(-30 * maxDis) + 0.05


        if (maxDis < 0.05):
            self.loop_count += 1 
            scale = 0.238
        else:
            self.loop_count = 0
        scale += 0.005 * np.max([self.loop_count - 0, 0]) 

        u3 = 0

        # Two modes: when obstacle is close, obstacle avoidance first; when obstacle is far, use nullspace for obstacle avoidance


        if (scale > 0.3): 
            scale = 0.3 

        u = (j_eef_T @ torch.inverse(self.j_eef @ j_eef_T + lmbda) @ dpose) 

        u = scale * (u + u3)

        maxAbsU = torch.max(torch.abs(u))  
        if (is_slow):
            self.th = 0.01
        else: 
            self.th = 0.03
        if (maxAbsU > self.th):
            u = u / torch.max(torch.abs(u)) * self.th 

        return u 
    
    def move_arm(self, is_slow=False, is_stopped=False, good_rot=False): 
        if self.current_q is None: 
            return 
        
        print("here")
        print("time: ", time.time() - self.last_time)
        self.last_time = time.time()

        self.fk.update(self.current_q)
        self.fk.closest_to_links()


        self.target_pose = torch.tensor([0.5888, -0.11, 0.291, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)
        # # target_pose = torch.tensor([0.62, 0.3, 0.2905, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)
        t = time.time()- self.start_time
        self.target_pose[1] = (0.1) * np.sin(1.0 * t) + (-0.11) 
        self.target_pose[0] = (0.1) * np.cos(1.0 * t) + (0.6) 
        # self.target_pose[1] = (0.08 + 0.005 * np.cos(6*t)) * np.sin(1.0 * t) + (-0.11) 
        # self.target_pose[0] = (0.08 + 0.005 * np.cos(6*t)) * np.cos(1.0 * t) + (0.6) 

        flange_pose_trans = self.fk.T_link[-1]  # last one is flange 
        self.j_eef = self.fk.jaco_eff
        pos_err = self.target_pose[0:3] - flange_pose_trans[0:3, 3]
        rot = torch.tensor(R.from_matrix(flange_pose_trans[0:3, 0:3]).as_quat(), dtype=torch.float) 
        quat_err = orientation_error(self.target_pose[3:7], rot)
        dpose = torch.cat([pos_err, quat_err], -1)


        # save data
        new_tensor = torch.cat((torch.tensor([time.time() - self.start_time], dtype=torch.float), dpose))
        new_tensor = torch.cat((new_tensor, self.target_pose[0].unsqueeze(0)))
        new_tensor = torch.cat((new_tensor, flange_pose_trans[0, 3].unsqueeze(0))) 
        new_tensor = torch.cat((new_tensor, self.target_pose[1].unsqueeze(0)))
        new_tensor = torch.cat((new_tensor, flange_pose_trans[1, 3].unsqueeze(0))) 
        new_tensor = torch.cat((new_tensor, self.target_pose[2].unsqueeze(0)))
        new_tensor = torch.cat((new_tensor, flange_pose_trans[2, 3].unsqueeze(0))) 
        self.saved_data = torch.cat((self.saved_data, new_tensor.unsqueeze(0)), 0)
        vel = torch.tensor(self.current_dq, dtype=torch.float).unsqueeze(0)
        self.vel_data = torch.cat((self.vel_data, vel), 0)

        u = self.control_ik(dpose, is_slow=is_slow, good_rot=good_rot) 
        if (is_stopped):
            u = torch.zeros(u.shape)

        u_data = torch.empty((1, 8))
        u_data[0, :7] = u
        u_data[0, 7] = self.th
        self.u_data = torch.cat((self.u_data, u_data), 0)

        target_q = torch.tensor(self.current_q, dtype=torch.float) + u 

        msg = ModeController()
        msg.mode = msg.MODE_RT_JOINT_POSITION 
        msg.positions = target_q.tolist()
        msg.velocities = u.tolist()
        msg.accelerations = [0.0] * 7
        self.mode_controller_pub.publish(msg)

    def update(self): 
        # if (not self.flag) or (self.current_pose is None):
        #     return 
        
        self.move_arm() 
        # for _ in range(1000):
        # self.fk.update([0.23] * 7)
        # self.fk.closest_to_links()

        # print("time: ", time.time() - self.last_time)
        # self.last_time = time.time()

    def update_2(self):
        self.fk.update([0.23] * 7)
        self.fk.closest_to_links()
        # print("here")

    def robot_states_callback(self, msg): 
        self.current_q = msg.q
        self.current_pose = ros_pose_to_tensor(msg.flange_pose)
        self.current_dq = msg.dq 


def main(args=None):
    rclpy.init(args=args)

    test_tcp = RizonControl()

    executor = MultiThreadedExecutor(num_threads=64)
    # executor = MultiThreadedExecutor()

    executor.add_node(test_tcp)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        test_tcp.on_shutdown()
        test_tcp.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()