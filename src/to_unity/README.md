# Hanoi Demo
## Step 1: 
### Establish connection between Unity and ROS2 
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.5.102 
```

## Step 2:
### Start Unity 
### Subscribe: (1) "robot_states": receive the current robot state of real robot  
### Publish: (1) "hanoi_grab": if hanoi is grabbed in Unity; (2) "hanoi_pose": the current pose of each hanoi disk   

## Step3: 
### Bring up robot arm 
```bash
ros2 run flexiv_cpp flexiv_robot
```
### Service: (1) "set_mode": transit operation mode to required mode 
### Subscribe: (1) "mode_controller": Receive the control command and operate the robot arm  
### Publish: (1) "robot_states": Send the robot states

## Step4: 
### Start IK solver: solve the flange pose to 7 joint states 
```bash
ros2 launch to_unity_cpp rizon_ik_forward.launch.py 
```
### Client: (1) "set_mode": transit operation mode to [NRT_JOINT_POSITION] 
### Subscribe: (1) "target_pose": Receive the target pose of flange 
### Publishe: (1) "mode_controller": Send the IK solution to control the robot arm 


## Step 5 
### Start controller which subsrcibes the interaction information from Unity and publishes the command to manipulate robot arm 
```bash
ros2 run to_unity_py decision_maker 
```
### Subscribe: (1) "hanoi_grab": whether the hanoi disk is grabbed in Unity; (2) monitor the poses of hanoi disk 
### Publish: (1) "target_pose": Send the decided target pose of flange 
