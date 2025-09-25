import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from flexiv_msgs.srv import SpawnRizonVisual
from flexiv_msgs.msg import RobotStates
import time



class CallbackGroupDemo(Node):
    def __init__(self):
        super().__init__('client_node')

        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(SpawnRizonVisual, 'spawn_rizon_visual', callback_group=client_cb_group)
        self.call_timer = self.create_timer(0.01, self._timer_cb, callback_group=timer_cb_group)
        self.request = SpawnRizonVisual.Request()
        self.is_spawn_visual = True

        self.publisher_ = self.create_publisher(RobotStates, '/rizon_states', 10)

        self.start_pos = [0] * 7
        self.target_pos = [1.6] * 7
        self.start_time = time.time()




    def _timer_cb(self):
        if (self.is_spawn_visual):
            self.spawn()
            self.is_spawn_visual = False

        msg = RobotStates()
        msg.q = [self.start_pos[0] + 0.1 * (time.time() - self.start_time)] * 7
        if (msg.q[0] < 1.6):
            self.publisher_.publish(msg) 

    def spawn(self):
        for i in range(4):

            self.get_logger().info('Sending request')
            self.request.joint_state = [0.4 * (i+1)] * 7

            _ = self.client.call(self.request)
            self.get_logger().info('Received response')


def main():
    rclpy.init()
    node = CallbackGroupDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()