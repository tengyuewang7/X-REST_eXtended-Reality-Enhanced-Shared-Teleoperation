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
        cb_group_2 = MutuallyExclusiveCallbackGroup()
        cb_group_3 = MutuallyExclusiveCallbackGroup()
        cb_group_4 = MutuallyExclusiveCallbackGroup() 
        cb_group_5 = MutuallyExclusiveCallbackGroup() 
        cb_group_6 = ReentrantCallbackGroup() 


        self.hanoi = [Hanoi(), Hanoi(), Hanoi()] 

        self.hanoi_grab_sub = self.create_subscription(HanoiGrab, 
                                                       'hanoi_grab', 
                                                       self.hanoi_grab_callback, 
                                                       qos_profile=1, 
                                                       callback_group=cb_group_1) 
        
        self.hanoi_pose_sub = self.create_subscription(HanoiPose, 
                                                       'hanoi_pose', 
                                                       self.hanoi_pose_callback, 
                                                       qos_profile=1, 
                                                       callback_group=cb_group_1)
        
        self.robot_states_sub = self.create_subscription(RobotStates, 
                                                         "robot_states", 
                                                         self.robot_states_callback, 
                                                         qos_profile=1, 
                                                         callback_group=cb_group_2) 
        
        self.monitor_moved_timer = self.create_timer(0.01, 
                                                     self.monitor_moved_callback, 
                                                     callback_group=cb_group_3) 
        
        self.digital_output_pub = self.create_publisher(DigitalOutput, 'digital_output', 10)
        self.mode_controller_pub = self.create_publisher(ModeController, "mode_controller", 10) 
        self.update_timer = self.create_timer(0.005, self.update, callback_group=cb_group_4)
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
        self.device = 'cpu'

        # Turn on digital dispaly 
        msg = DigitalOutput() 
        pin_state = DigitalState()
        pin_state.pin = 1 
        pin_state.state = True
        msg.digital_output.append(pin_state)
        self.digital_output_pub.publish(msg) 

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


        print("time: ", time.time() - self.last_time)
        self.last_time = time.time()

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

        self.target_pose = torch.tensor([0.5888, -0.11, 0.291, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)
        # # target_pose = torch.tensor([0.62, 0.3, 0.2905, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)
        t = time.time()- self.start_time
        self.target_pose[1] = (0.1) * np.sin(1.0 * t) + (-0.11) 
        self.target_pose[0] = (0.1) * np.cos(1.0 * t) + (0.6) 
        # self.target_pose[1] = (0.08 + 0.005 * np.cos(6*t)) * np.sin(1.0 * t) + (-0.11) 
        # self.target_pose[0] = (0.08 + 0.005 * np.cos(6*t)) * np.cos(1.0 * t) + (0.6) 

        self.fk.update(self.current_q) 
        flange_pose_trans = self.fk.T_link[-1].cpu()  # last one is flange 
        self.j_eef = self.fk.jaco_eff.cpu()
        pos_err = self.target_pose[0:3] - flange_pose_trans[0:3, 3]
        rot = torch.tensor(R.from_matrix(flange_pose_trans[0:3, 0:3]).as_quat(), dtype=torch.float) 
        quat_err = orientation_error(self.target_pose[3:7], rot)
        dpose = torch.cat([pos_err, quat_err], -1)

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
        # u_data = torch.cat((u.unsqueeze(0), torch.tensor(self.th, dtype=torch.float)).unsqueeze(0), -1)
        self.u_data = torch.cat((self.u_data, u_data), 0)

        target_q = torch.tensor(self.current_q, dtype=torch.float) + u 

        msg = ModeController()
        msg.mode = msg.MODE_RT_JOINT_POSITION 
        msg.positions = target_q.tolist()
        msg.velocities = u.tolist()
        msg.accelerations = [0.0] * 7

        
        self.mode_controller_pub.publish(msg)

    def update(self): 
        if (not self.flag) or (self.current_pose is None):
            return 
        # target = torch.tensor([0.6888, -0.11, 0.291, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)
        # self.target = torch.tensor([0.6339, -0.0019, 0.1382 + 0.05, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)

        # self.target_pose = torch.tensor([0.5888, -0.11, 0.291, 0.0, 1.0, 0.0, 0.0], dtype=torch.float)


        # while (torch.norm(self.target[0:3] - self.current_pose[0:3]) > 0.003): 
        self.move_arm() 
        

        # while (np.linalg.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.005):


        # for hanoi_disk in self.hanoi: 
        #     if hanoi_disk.grab_unity == True and hanoi_disk.grab_robot == False:
        #         # print("task1")
        #         self.target_pose = up_in_z(hanoi_disk.pose, self.L + 0.05)
        #         while (torch.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.003):
        #             self.move_arm() 
        #             # time.sleep(0.01)
        #         self.target_pose = up_in_z(hanoi_disk.init_pose, self.L + 0.005)
        #         self.gripper(True) 
        #         while (torch.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.003):
        #             self.move_arm(is_slow=True, good_rot=True) 
        #             # time.sleep(0.01)
        #         # count = 0
        #         # while (self.digital_input_0):
        #         #     time.sleep(0.01) 
        #         #     count += 1 
        #         #     if (count > 300): 
        #         #         print("Fail to grab the disk")
        #         #         # break 
                
        #         hanoi_disk.grab_robot = True 
        #         break
            
        #     elif hanoi_disk.is_moved == True and hanoi_disk.grab_unity == True and hanoi_disk.grab_robot == True: 
        #         # print("task2")
        #         self.target_pose = up_in_z(hanoi_disk.pose, self.L + 0.005)
        #         self.move_arm() 
        #         break

        #     elif hanoi_disk.grab_unity == False and hanoi_disk.grab_robot == True: 
        #         # print("task3")
        #         # time.sleep(1)
        #         hanoi_disk.pose[3:] = torch.tensor([0, 1, 0, 0], dtype=torch.float)
        #         self.target_pose = up_in_z(hanoi_disk.pose, self.L + 0.005)
        #         while (torch.norm(self.target_pose - self.current_pose) > 0.003):
        #             self.move_arm(is_slow=True, good_rot=True) 
        #             # time.sleep(0.01) 
        #         self.gripper(False) 
        #         time.sleep(1)
        #         # while not self.digital_input_0: 
        #         #     time.sleep(0.01)
        #         #     print("here")
        #         hanoi_disk.grab_robot = False 
        #         hanoi_disk.init_pose = hanoi_disk.pose 
        #         self.target_pose = up_in_z(hanoi_disk.pose, self.L + 0.05)

        #         self.move_arm() 
        #         while (torch.norm(self.target_pose - self.current_pose) > 0.003):
        #             self.move_arm(is_slow=True, good_rot=True) 
        #             # time.sleep(0.01) 
        #         self.move_arm(is_stopped=True)
        #         break 

    def gripper(self, state):
        msg = DigitalOutput() 
        pin_state = DigitalState()
        pin_state.pin = 0 
        pin_state.state = state
        msg.digital_output.append(pin_state)
        self.digital_output_pub.publish(msg) 

    def hanoi_grab_callback(self, msg): 
        for i in range(len(self.hanoi)): 
            self.hanoi[i].grab_unity = msg.is_grab[i] 

    def hanoi_pose_callback(self, msg): 
        for i in range(len(self.hanoi)): 
            self.hanoi[i].pose = ros_pose_to_tensor(msg.pose_array[i])

        if self.hanoi_at_start: 
            for i in range(len(self.hanoi)):
                self.hanoi[i].init_pose = self.hanoi[i].pose.clone()
            self.hanoi_at_start = False 

    def robot_states_callback(self, msg): 
        self.current_q = msg.q
        self.current_pose = ros_pose_to_tensor(msg.flange_pose)
        self.current_dq = msg.dq 
        self.digital_input_0 = msg.digital_input_0  
        # print(msg.flange_pose.position.x)
        # new_tensor = torch.cat((torch.tensor([time.time() - self.start_time], dtype=torch.float), ros_pose_to_tensor(msg.flange_pose)))
        # self.saved_data = torch.cat((self.saved_data, new_tensor.unsqueeze(0)), 0)


    def monitor_moved_callback(self):
        if self.hanoi[0].init_pose is None:
            return
        for i in range(len(self.hanoi)): 
            if torch.norm(self.hanoi[i].init_pose - self.hanoi[i].pose) > 0.01: 
                self.hanoi[i].is_moved = True


def main(args=None):
    rclpy.init(args=args)

    test_tcp = RizonControl()

    executor = MultiThreadedExecutor(num_threads=16)
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