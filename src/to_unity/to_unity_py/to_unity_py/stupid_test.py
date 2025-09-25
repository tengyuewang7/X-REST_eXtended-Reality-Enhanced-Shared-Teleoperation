import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from flexiv_msgs.srv import SetMode


class CallbackGroupDemo(Node):
    def __init__(self):
        super().__init__('client_node')

        cb_group_1 = ReentrantCallbackGroup()
        cb_group_2 = ReentrantCallbackGroup()
        cb_group_3 = ReentrantCallbackGroup()
        cb_group_4 = MutuallyExclusiveCallbackGroup() 
        cb_group_5 = MutuallyExclusiveCallbackGroup() 

        self.client = self.create_client(SetMode, 'set_mode', callback_group=cb_group_5)
        # self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)

        request = SetMode.Request()
        request.mode = request.MODE_NRT_JOINT_POSITION
        _ = self.client.call_async(request)
        print("here")

    def _timer_cb(self):
        self.get_logger().info('Sending request')
        request = SetMode.Request()
        request.mode = request.MODE_NRT_JOINT_POSITION
        _ = self.client.call(request)
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

if __name__ == '__main__':
    main()