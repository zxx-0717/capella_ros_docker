import rclpy
import sys
from rclpy.node import Node
from subprocess import Popen
from capella_ros_service_interfaces.srv import CartographerMode


class CartographerClient(Node):
    def __init__(self, name):
        super().__init__(name)
        self.carto_client = self.create_client(CartographerMode, '/mission_server/cartographer_mode')
        while not self.carto_client.wait_for_service(timeout_sec = 5.0):
            self.get_logger().info('Keep waitting !')
        self.request = CartographerMode.Request()

    def send_mode(self):
        self.future = self.carto_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    carto_cli = CartographerClient('carto_cli')
    response = carto_cli.send_mode()
    if response.cart_status:
        carto_cli.get_logger().info('Cartographer successfully activated !')
    else:
        carto_cli.get_logger().info('Cartographer start failed !')
    carto_cli.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
