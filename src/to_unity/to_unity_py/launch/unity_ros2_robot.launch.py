from launch import LaunchContext, LaunchDescription, actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart
from launch.actions import RegisterEventHandler


def generate_launch_description():

    default_server_endpoint = Node(package='ros_tcp_endpoint', 
                                   executable='default_server_endpoint',
                                   parameters=[{"ROS_IP": "192.168.4.101"}, {"ROS_TCP_PORT": 10000}], 
                                   emulate_tty=True,
                                   output="screen")
    
    flexiv_robot = Node(package='flexiv_cpp', 
                        executable='flexiv_robot', 
                        emulate_tty=True,
                        output="screen")
    
    rizon_ik_forward = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("to_unity_cpp"),
                "launch",
                "rizon_ik_forward.launch.py"
            ])
        ]),
    )

    def start_rizon_ik_forward(event: ProcessStarted, context: LaunchContext): 
        return actions.TimerAction(period=5.0, actions=[rizon_ik_forward]),


    return LaunchDescription(
        [
            RegisterEventHandler(event_handler=OnProcessStart(
                target_action=flexiv_robot, on_start=start_rizon_ik_forward)),

            default_server_endpoint, 
            flexiv_robot, 
        ]
    )