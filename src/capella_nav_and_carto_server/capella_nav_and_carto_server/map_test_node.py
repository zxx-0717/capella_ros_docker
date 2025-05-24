from ast import arg
import cv2
import rclpy
import sys
import numpy as np
from PIL import Image
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from map_msgs.msg import OccupancyGridUpdate


class MapTest(Node):
        def __init__(self, name):
                super().__init__(name)
                self.get_logger().info("map node !")
                self.map_pulish = self.create_publisher(OccupancyGrid,'map',1000)
                self.time = self.create_timer(0.5,self.manual_pub)

                self.map = OccupancyGrid()
                self.data = cv2.imread(r'src/capella_nav_server/map/map1.png',0)
                
                self.map.header.frame_id = 'map'
                self.map.info.resolution = 0.05
                self.map.info.origin.position.x = -15.0
                self.map.info.origin.position.y = -15.0
                self.map.info.origin.position.z = 0.0
                self.map.info.origin.orientation.x = 0.0
                self.map.info.origin.orientation.y = 0.0
                self.map.info.origin.orientation.z = 0.0
                self.map.info.origin.orientation.w = 1.0
                self.map.info.width = self.data.shape[1]
                self.map.info.height = self.data.shape[0]
                self.data1 = []
                for i in range(self.map.info.height):
                        for j in range(self.map.info.width):
                                if self.data[i][j] > 240:
                                        self.data1.append(0)
                                elif self.data[i][j] < 15:
                                        self.data1.append(100)
                                else:
                                        self.data1.append(-1)

                self.map.data = self.data1


        def manual_pub(self):
                self.map_pulish.publish(self.map)
                self.get_logger().info("publish map !")


def main(gras=None):
        rclpy.init(args=gras)
        node = MapTest("maptest")
        rclpy.spin(node)
        rclpy.shutdown()