import os
from signal import SIGINT

import psutil
import rclpy
import subprocess

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from capella_ros_service_interfaces.srv import *
from capella_ros_service_interfaces.msg import *
from capella_ros_msg.msg import *
from nav2_msgs.srv import *

from ament_index_python.packages import get_package_share_directory

class NodeManagerService(Node):

    def __init__(self):
        super().__init__('mission_server')

        self._headless = self.declare_parameter("headless", False).value
        self._logger.info('Running in {} mode.'.format('HEADLESS' if self._headless else 'DESKTOP'))

        self.current_proc = None

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.status_publisher = self.create_publisher(MissionStatus, 
                'mission_server/mission_status', qos_profile = latching_qos)
        
        self.cartographer_srv = self.create_service(
            CartographerMode, 'mission_server/cartographer_mode', self.start_cartographer)
        self.nav_srv = self.create_service(
            NavLaunchMode, 'mission_server/nav_launch_mode', self.start_nav)
        self.idle_srv = self.create_service(
            StopNavAndCarto, 'mission_server/stop_nav_and_carto', self.stop)

        self.set_status(MissionStatus.MISSION_STATUS_IDLE)

        self._load_map_client = self.create_client(LoadMap, '/map_server/load_map')

    def set_status(self, newStatus):
        self.mode = newStatus

        msg = MissionStatus()
        msg.mission_status = self.mode
        self.status_publisher.publish(msg)

        self._logger.info('Mission server transitioned to state %d' % newStatus)

    def start_cartographer(self, request, response):
        response.cart_status = 1

        if self.mode == MissionStatus.MISSION_STATUS_CARTO:
            return response

        self.set_status(MissionStatus.MISSION_STATUS_CARTO)
        self.stop_current()
        
        self.current_proc = subprocess.Popen(
            ["ros2", "launch", 
                "capella_ros_launcher",
                "capella_wr_lidar_cartographer.launch.py",
                "headless:={}".format(self._headless)])

        return response

    def start_nav(self, request:NavLaunchMode.Request, response:NavLaunchMode.Response):
        response.nav_status = 1

        map_file = self._write_map(request)

        if self.mode == MissionStatus.MISSION_STATUS_NAV:
            self._change_map(map_file)
            return response

        self.set_status(MissionStatus.MISSION_STATUS_NAV)
        self.stop_current()

        
        self.current_proc = subprocess.Popen(
            ["ros2", "launch", "capella_ros_launcher",
               "nav2_opt.launch.py", "map:={}".format(map_file),
               "no_navigator:=True", "headless:={}".format(self._headless)])

        return response

    def stop(self, request, response):
        response.status = 0

        self.stop_current()
        self.set_status(MissionStatus.MISSION_STATUS_IDLE)

        return response

    def terminate(self, proc: subprocess.Popen):
        parent_pid = proc.pid   # my example
        parent = psutil.Process(parent_pid)
        for child in parent.children(recursive=True):  # or parent.children() for recursive=False
            child.send_signal(SIGINT)
        parent.send_signal(SIGINT)

    def stop_current(self):
        if self.current_proc != None:
            self.terminate(self.current_proc)
            self.current_proc = None

    def _write_map(self, request:NavLaunchMode.Request):
        yaml_path = '/tmp/capella_map.yaml'
        png_path = '/tmp/capella_map.png'

        with open(yaml_path, "w") as file:
            file.writelines([
                "image: capella_map.png\n",
                "resolution: {}\n".format(request.resolution),
                "origin: [{x}, {y}, 0.0]\n".format(x=request.x, y=request.y),
                "negate: {}\n".format(request.negate),
                "occupied_thresh: {}\n".format(request.occupied_thresh),
                "free_thresh: {}\n".format(request.free_thresh)
            ])
        
        with open(png_path, "wb") as file:
            file.write(request.image.tobytes())

        return yaml_path

    def _change_map(self, map_filepath):
        """Change the current static map in the map server."""
        while not self._load_map_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath

        self._load_map_client.call_async(req)
        self._logger.info('Change map request was successful!')
        

def main():
    rclpy.init()

    service = NodeManagerService()
    rclpy.spin(service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()