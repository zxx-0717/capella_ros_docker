import os
import psutil
import signal
import rclpy
import numpy as np
from rclpy.node import Node
from capella_ros_service_interfaces.srv import StopNavAndCarto

class StopClient(Node):
        def __init__(self, node_name):
                super().__init__(node_name)
                self.stop = self.create_client(StopNavAndCarto,'/mission_server/stop_nav_and_carto')
                self.request = StopNavAndCarto.Request()
                while not self.stop.wait_for_service(5.0):
                        self.get_logger().info("waitting server !")


        def stop_(self):
                self.future = self.stop.call_async(self.request)
                rclpy.spin_until_future_complete(self, self.future)
                return self.future.result()

def main(args = None):
        rclpy.init(args=args)
        node = StopClient('stopclient')
        node.stop_()
        rclpy.shutdown()
if __name__ == '__main__':
        main()