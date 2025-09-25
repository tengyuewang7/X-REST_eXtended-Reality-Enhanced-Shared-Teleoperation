import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from flexiv_msgs.msg import HanoiGrab, HanoiPose, RobotStates, DigitalOutput, DigitalState
from geometry_msgs.msg import PoseStamped, Pose
import time
import copy
import threading 

class TowerOfHanoi(Node): 
    def __init__(self):
        super().__init__('test') 
        cb_group_1 = ReentrantCallbackGroup()
        cb_group_2 = ReentrantCallbackGroup()
        cb_group_3 = ReentrantCallbackGroup()
        cb_group_4 = MutuallyExclusiveCallbackGroup()

        self.hanoi = [Hanoi(), Hanoi(), Hanoi()] 
        self.hanoi_grab_sub = self.create_subscription(HanoiGrab, 'hanoi_grab', self.hanoi_grab_callback, qos_profile=1, callback_group=cb_group_1)
        self.hanoi_pose_sub = self.create_subscription(HanoiPose, 'hanoi_pose', self.hanoi_pose_callback, qos_profile=1, callback_group=cb_group_1)
        self.robot_states_sub = self.create_subscription(RobotStates, "robot_states", self.robot_states_callback, qos_profile=1, callback_group=cb_group_2)
        
        self.monitor_moved_timer = self.create_timer(0.01, self.monitor_moved_callback, callback_group=cb_group_3) 
        self.taget_pose_pub = self.create_publisher(PoseStamped, 'target_pose', 10) 
        self.digital_output_pub = self.create_publisher(DigitalOutput, 'digital_output', 10)
        self.update_timer = self.create_timer(0.01, self.update, callback_group=cb_group_4)

        self.robot_states = None 
        self.L = 0.08
        self.T = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]]) 

        self.target_pose = [0.688755, -0.11023, 0.291, 0.0, 1.0, 0.0, 0.0]
        self.at_start = True 
        self.digital_input_0 = True

        # turn on digital dispaly 
        msg = DigitalOutput() 
        pin_state = DigitalState()
        pin_state.pin = 1 
        pin_state.state = True
        msg.digital_output.append(pin_state)
        self.digital_output_pub.publish(msg)


    def update(self): 

        for hanoi_disk in self.hanoi:

            if hanoi_disk.grab_unity == True and hanoi_disk.grab_robot == False:
                # print("task1")
                # self.task1(hanoi_disk) 
                self.target_pose = self.up_in_z(hanoi_disk.pose, self.L + 0.05)
                self.move_arm(self.target_pose) 
                while (np.linalg.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.005):
                    time.sleep(0.01)
                self.target_pose = self.up_in_z(hanoi_disk.init_pose, self.L + 0.005)
                self.move_arm(self.target_pose) 
                while (np.linalg.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.005):
                    time.sleep(0.01)
                self.gripper(True) 
                count = 0
                while (self.digital_input_0):
                    time.sleep(0.01) 
                    count += 1 
                    if (count > 300): 
                        print("Fail to grab the disk")
                        # break 
                
                hanoi_disk.grab_robot = True 
                break
            
            elif hanoi_disk.is_moved == True and hanoi_disk.grab_unity == True and hanoi_disk.grab_robot == True: 
                # print("task2")
                # self.task2(hanoi_disk) 
                self.target_pose = self.up_in_z(hanoi_disk.pose, self.L)
                self.move_arm(self.target_pose) 
                break
                
            elif hanoi_disk.grab_unity == False and hanoi_disk.grab_robot == True: 
                # print("task3")
                # self.task3(hanoi_disk)
                time.sleep(1)
                self.target_pose = self.up_in_z(hanoi_disk.pose, self.L)
                self.move_arm(self.target_pose) 
                while (np.linalg.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.005):
                    time.sleep(0.01) 
                self.gripper(False) 
                while not self.digital_input_0: 
                    time.sleep(0.01)
                    print("here")
                hanoi_disk.grab_robot = False 
                hanoi_disk.init_pose = hanoi_disk.pose 
                self.target_pose = self.up_in_z(hanoi_disk.pose, self.L + 0.05)

                self.move_arm(self.target_pose) 
                while (np.linalg.norm(self.target_pose[0:3] - self.current_pose[0:3]) > 0.005):
                    time.sleep(0.01) 
                break


    def move_arm(self, target_pose): 
        msg = PoseStamped() 
        msg.header.frame_id = "hanoi"
        msg.pose.position.x = target_pose[0]
        msg.pose.position.y = target_pose[1]
        msg.pose.position.z = target_pose[2]
        msg.pose.orientation.x = target_pose[3]
        msg.pose.orientation.y = target_pose[4]
        msg.pose.orientation.z = target_pose[5]
        msg.pose.orientation.w = target_pose[6]
        self.taget_pose_pub.publish(msg) 
    
    def ros_pose_to_array(self, pose):
        return np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, 
                         pose.orientation.y, pose.orientation.z, pose.orientation.w]) 

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
        # [px, py, pz, qx, qy, qz, qw]
        for i in range(len(self.hanoi)): 
            self.hanoi[i].pose = np.array([msg.pose_array[i].position.x, msg.pose_array[i].position.y, msg.pose_array[i].position.z, 
                                           msg.pose_array[i].orientation.x, msg.pose_array[i].orientation.y, 
                                           msg.pose_array[i].orientation.z, msg.pose_array[i].orientation.w])
        if self.at_start: 
            self.hanoi[0].init_pose = copy.deepcopy(self.hanoi[0].pose)
            self.hanoi[1].init_pose = copy.deepcopy(self.hanoi[1].pose)
            self.hanoi[2].init_pose = copy.deepcopy(self.hanoi[2].pose)
            self.at_start = False 


    def robot_states_callback(self, msg): 
        self.current_pose = self.ros_pose_to_array(msg.flange_pose) 
        self.digital_input_0 = msg.digital_input_0 

    def monitor_moved_callback(self):
        if self.hanoi[0].init_pose is None:
            for i in range(len(self.hanoi)): 
                self.hanoi[i].init_pose = copy.deepcopy(self.hanoi[i].pose)
            return
        for i in range(len(self.hanoi)): 
            if (np.sum(np.abs(self.hanoi[i].init_pose - self.hanoi[i].pose)) > 0.01): 
                self.hanoi[i].is_moved = True
    
    def up_in_z(self, origianl_pose, l_z):
        pose = copy.deepcopy(origianl_pose) 
        rot = R.from_quat(origianl_pose[3:]).as_matrix() 
        pose[0] -= l_z * rot[0, 2] 
        pose[1] -= l_z * rot[1, 2] 
        pose[2] -= l_z * rot[2, 2] 
        return pose


class Hanoi:
    def __init__(self): 
        self.grab_unity = False 
        self.pose = None
        self.grab_robot = False 
        self.init_pose = None 
        self.is_moved = False

def main(args=None):
    rclpy.init(args=args)

    test_tcp = TowerOfHanoi()

    executor = MultiThreadedExecutor()
    executor.add_node(test_tcp)

    executor.spin()

    test_tcp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
