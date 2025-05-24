import os
from signal import SIGINT

import psutil
import rclpy
import subprocess
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from capella_ros_service_interfaces.srv import *
from capella_ros_service_interfaces.msg import *
class IrdetectionService(Node):

    def __init__(self):
        super().__init__('irdetection_server')

        self._headless = self.declare_parameter("headless", False).value
        self._logger.info('Running in {} mode.'.format('HEADLESS' if self._headless else 'DESKTOP'))
        self.infrared_srv = self.create_service(
            InfraredFaceDetection, "mission_server/infrared_detection_mode", self.infrared_callback)
        self.mode = 0
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.irdetection_status_publisher = self.create_publisher(IrdetectionStatus, 
                'irdetection_status', qos_profile = latching_qos)
        self.set_status(self.mode)
        self.current_proc = None

    def terminate(self, proc: subprocess.Popen):
        parent_pid = proc.pid   # my example
        parent = psutil.Process(parent_pid)
        for child in parent.children(recursive=True):  # or parent.children() for recursive=False
            child.send_signal(SIGINT)
        parent.send_signal(SIGINT)

    def stop(self):
        if self.current_proc != None:
            self.terminate(self.current_proc)
            self.current_proc = None

    def set_status(self, mode):
        msg = IrdetectionStatus()
        msg.irdetection_status = self.mode
        self.irdetection_status_publisher.publish(msg)
        self._logger.info('irdetection state is %d' % self.mode)

    def infrared_callback(self, request:InfraredFaceDetection.Request, response:InfraredFaceDetection.Response):
        if request.infrared_face_detection == 1:
            if self.mode == 1:
                response.face_detection_status = 1
                return response
            else:
                self.current_proc = subprocess.Popen(
                args=["ros2", "launch", 
                    "capella_infrared_detection",
                    "infrared_detection.launch.py",
                    "headless:={}".format(self._headless)], cwd='/workspaces/capella_ros_docker/install/lib/capella_irimage_sdk')
                response.face_detection_status = 1
                self.set_status(self.mode)
                return response
        elif request.infrared_face_detection == 0:
            self.mode = 0
            self.stop()
            self.set_status(self.mode)
            response.face_detection_status = 0
            return response
def main():
    rclpy.init()
    service = IrdetectionService()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()