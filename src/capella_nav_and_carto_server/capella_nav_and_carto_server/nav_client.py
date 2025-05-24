import rclpy
import numpy as np
from rclpy.node import Node
from capella_ros_service_interfaces.msg import MissionStatus
from capella_ros_service_interfaces.srv import CartographerMode
from capella_ros_service_interfaces.srv import NavLaunchMode
from capella_ros_service_interfaces.srv import StopNavAndCarto



class NavClient(Node):
        def __init__(self, name):
                super().__init__(name)
                self.nav_client = self.create_client(NavLaunchMode, '/mission_server/nav_launch_mode')
                self.request = NavLaunchMode.Request()
                # self.request.data = open(r'/workspaces/capella_ros_docker/src/capella_nav_and_carto_server/capella_nav_and_carto_server/map4.png','rb').readlines()
                self.request.image = [255]
                self.request.free_thresh = 0.25
                self.request.occupied_thresh = 0.75
                self.request.negate = 0
                self.request.resolution = 0.05
                self.request.x = 0.0
                self.request.y = 0.0
                self.request.z = 0.0
                

                while not self.nav_client.wait_for_service(timeout_sec = 5.0):
                        self.get_logger().info('Keep waiting !')
                        
                        

        def send_mode(self):
                self.future = self.nav_client.call_async(self.request)
                rclpy.spin_until_future_complete(self, self.future)
                return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    carto_cli = NavClient('nav_cli')
    response = carto_cli.send_mode()
    if response.nav_status:
        carto_cli.get_logger().info('navigation successfully activated !')
    else:
        carto_cli.get_logger().info('navigation start failed !')
    rclpy.shutdown()

if __name__ == '__main__':
    main()