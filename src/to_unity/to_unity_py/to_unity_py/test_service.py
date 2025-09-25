import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from flexiv_msgs.srv import SetMode

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(SetMode, 'test_service', callback=self.service_callback)

    def service_callback(self, request, result):
        self.get_logger().info('Received request, responding...')
        return result

def main():
    rclpy.init()
    node = ServiceNode()
    try:
        node.get_logger().info("Starting server node, shut down with CTRL-C")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()