import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapMonitor(Node):
        def __init__(self, node_name):
                super().__init__(node_name)
                self.get_logger().info("启动地图监听！")
                self.monitor = self.create_subscription(OccupancyGrid,'map',self.monitor_callback,100)


        def monitor_callback(self,response):
                self.get_logger().info("地图已发布！")
                print('data unique:',np.unique(np.array(response.data).flatten()))
                print('data shape:',np.array(response.data).shape)
                print('map height and width:',response.info.height,'  ',response.info.width)
                print('map origin:',response.info.origin)
                print('map header.frame_id:',response.header.frame_id)
                print('map header.stamp:',response.header.stamp)

def main(args=None):
        rclpy.init(args=args)
        node = MapMonitor('mapmonitor')
        rclpy.spin(node)
        rclpy.shutdown()