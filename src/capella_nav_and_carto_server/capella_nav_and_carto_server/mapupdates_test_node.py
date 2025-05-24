from ast import arg
import rclpy
import sys
import numpy as np
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from map_msgs.msg import OccupancyGridUpdate


class MapUpdatesTest(Node):
        def __init__(self, name):
                super().__init__(name)
                self.get_logger().info("mapupdates node !")
                # self.pub = self.create_publisher(String,'test',10)
                # self.time = self.create_timer(1,self.time_callback)
                # self.data = String()

                self.mapupdates_pulish = self.create_publisher(OccupancyGridUpdate,'map_updates',100)
                self.time = self.create_timer(5,self.manual_pub)

                self.map = OccupancyGridUpdate()

                self.map.header.frame_id = "map"
                # self.map.header.stamp = self.timers().now()

                self.data = cv2.imread(r'src/capella_nav_server/map/map5.png',0)        
                self.map.x = 0
                self.map.y = 0
                # self.map.height = 200
                # self.map.width = 200
                # self.data1 = [101]*200*200
                self.map.height = self.data.shape[0]
                self.map.width = self.data.shape[1]

                self.data1 = [] 
                # for i in range(self.map.height-1,-1,-1):
                #         for j in range(self.map.width):
                #                 if self.data[i][j] > 200:
                #                         self.data1.append(0)
                #                 elif self.data[i][j] < 50:
                #                         self.data1.append(100)
                #                 else:
                #                         self.data1.append(-1)
                
         
                for i in range(self.map.height):
                        for j in range(self.map.width):
                                if self.data[i][j] > 200:
                                        self.data1.append(0)
                                elif self.data[i][j] < 50:
                                        self.data1.append(100)
                                else:
                                        self.data1.append(-1)

                self.map.data = self.data1


        def manual_pub(self):
                self.mapupdates_pulish.publish(self.map)
                self.get_logger().info("publish mapupdates !")
                # self.get_logger().info('map height:{}'.format(self.map.height))
                # self.get_logger().info('map width:{}'.format(self.map.width))
        

def main(gras=None):
        rclpy.init(args=gras)
        node = MapUpdatesTest("map_updates_test")
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == '__main__':
        main()