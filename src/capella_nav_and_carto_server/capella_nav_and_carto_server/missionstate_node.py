import rclpy
from rclpy.node import Node
from capella_ros_service_interfaces.msg import MissionStatus
from capella_ros_service_interfaces.srv import CartographerMode
from capella_ros_service_interfaces.srv import NavLaunchMode
from capella_ros_service_interfaces.srv import StopNavAndCarto



class Missionstate(Node):
        def __init__(self, name):
                super().__init__(name)
                self.missionstate = self.create_subscription(MissionStatus, '/mission_server/mission_status',self.receive_callback,1)


        def receive_callback(self,response):
                self.get_logger().info(f'missionstate is {response.mission_status} !')
                


def main(args=None):
    rclpy.init(args=args)
    node = Missionstate('missionstate')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()