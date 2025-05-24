import multiprocessing
from re import A
import tempfile
from setuptools import find_namespace_packages
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import os
import sys
import time
import shlex
import psutil
import signal
import subprocess
import numpy as np
from ament_index_python.packages import get_package_share_directory
from capella_ros_service_interfaces.msg import MissionStatus
from capella_ros_service_interfaces.srv import CartographerMode
from capella_ros_service_interfaces.srv import NavLaunchMode
from capella_ros_service_interfaces.srv import StopNavAndCarto
import platform
import warnings


signal.signal(signal.SIGCHLD,signal.SIG_IGN)

class NavAndCartoService(Node):
        def __init__(self, name):
                super().__init__(name)
                print("system info :",platform.system())
                latching_qos = QoSProfile(depth=1,
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                self.space_time = 0
                self.nav_state = 0
                self.carto_state = 0
                self.missionstate = MissionStatus()
                self.missionstate.mission_status = 0
                self.nav_srv = self.create_service(NavLaunchMode, '/mission_server/nav_launch_mode', self.nav_callback)
                self.carto_srv = self.create_service(CartographerMode, '/mission_server/cartographer_mode', self.carto_callback)
                self.stop_srv = self.create_service(StopNavAndCarto,'/mission_server/stop_nav_and_carto',self.stop_callback)
                self.mission_server = self.create_publisher(MissionStatus, "/mission_server/mission_status", qos_profile = latching_qos)
                self.timer_period = 1.0
                self.timer = self.create_timer(self.timer_period, self.missionstate_callback)
                self.nav_state_set = ['map_server','controller_server','planner_server','recoveries_server','bt_navigator','waypoint_follower']
                self.carto_state_set = ['cartographer_node', 'occupancy_grid_node']



        def missionstate_callback(self):
                self.get_logger().info('start update missionstate !')
                pids = psutil.pids()
                for pid in pids:
                        try:
                                p = psutil.Process(pid)
                        except:
                                continue
                        if p.name() in self.carto_state_set and p.ppid() != 1:
                                self.carto_state = 1
                                break
                        else:
                                self.carto_state = 0
                for pid in pids:
                        try:
                                p = psutil.Process(pid)
                        except:
                                continue
                        if p.name() in self.nav_state_set and p.ppid() != 1:
                                self.nav_state = 1
                                break
                        else:
                                self.nav_state = 0
                if self.nav_state == 0 and self.carto_state == 0:
                        self.get_logger().info("navigation and cartographer is off !")
                        self.missionstate.mission_status = 0
                elif self.nav_state == 0 and self.carto_state == 1:
                        self.get_logger().info("cartographer is on !")
                        self.missionstate.mission_status = 1
                elif self.nav_state == 1 and self.carto_state == 0:
                        self.get_logger().info("navigation is on !")
                        self.missionstate.mission_status = 2
                else:
                        self.get_logger().info("please restart service,cartographer and navigation is on !")
                        self.missionstate.mission_status = -1
                self.mission_server.publish(self.missionstate)
                self.get_logger().info(f"publish missionstate:{self.missionstate.mission_status} !")


        def nav_callback(self,request, response):
                self.get_logger().info("\n**********************************************************************\n**********************************************************************\n*********************receive navigation request !*********************\n**********************************************************************\n**********************************************************************")
                try:
                        if self.missionstate.mission_status == 0:
                                self.save_img_and_yaml(request)
                                self.nav_start()
                                self.get_logger().info("start navigation !")
                                if self.missionstate.mission_status != 2:
                                        response.nav_status = 0
                                        self.get_logger().info('navigation start fail !')
                                else:
                                        response.nav_status = 1
                                        self.get_logger().info('navigation start success !')
                        elif self.missionstate.mission_status == 1:
                                self.carto_stop()
                                self.save_img_and_yaml(request)
                                self.nav_start()
                                self.get_logger().info("stop cartographer,start navigation !")
                                if self.missionstate.mission_status != 2:
                                        response.nav_status = 0
                                        self.get_logger().info('navigation start fail !')
                                else:
                                        response.nav_status = 1
                                        self.get_logger().info('navigation start success !')
                        elif self.missionstate.mission_status == 2:
                                self.get_logger().info("navigation is running already,will restart navigation !")
                                self.nav_stop()
                                self.save_img_and_yaml(request)
                                self.nav_start()
                                if self.missionstate.mission_status != 2:
                                        response.nav_status = 0
                                        self.get_logger().info('navigation start fail !')
                                else:
                                        response.nav_status = 1
                                        self.get_logger().info('navigation start success !')
                        else:
                                self.get_logger().info("please restart nav_and_carto_service !")
                                response.nav_status = 0
                except:
                        self.get_logger().info('navigation start fail !')
                        response.nav_status = 0

                self.get_logger().info("return nav response !")
                return response

        def carto_callback(self,request, response):
                self.get_logger().info("\n**********************************************************************\n**********************************************************************\n********************receive cartographer request !********************\n**********************************************************************\n**********************************************************************")
                try:
                        if self.missionstate.mission_status == 0:
                                self.carto_start()
                                self.get_logger().info("start cartographer !")
                                if self.missionstate.mission_status != 1:
                                        response.cart_status = 0
                                        self.get_logger().info('cartographer start fail !')
                                else:
                                        response.cart_status = 1
                                        self.get_logger().info('cartographer start success !')
                        elif self.missionstate.mission_status == 1:
                                self.get_logger().info("cartographer is running already !")
                                if self.missionstate.mission_status != 1:
                                        response.cart_status = 0
                                        self.get_logger().info('cartographer start fail !')
                                else:
                                        response.cart_status = 1
                                        self.get_logger().info('cartographer start success !')
                        elif self.missionstate.mission_status == 2:
                                self.nav_stop()
                                self.get_logger().info("stop nav successful ,properation start carto!")
                                self.carto_start()
                                self.get_logger().info("stop navigation,start cartographer !")
                                if self.missionstate.mission_status != 1:
                                        response.cart_status = 0
                                        self.get_logger().info('cartographer start fail !')
                                else:
                                        response.cart_status = 1
                                        self.get_logger().info('cartographer start success !')
                        else:
                                self.get_logger().info("please restart nav_and_carto_service !")
                                response.cart_status = 0
                except:
                        self.get_logger().info('cartographer start fail !')
                        response.cart_status = 0
                
                
                self.get_logger().info("return carto response !")
                return response

        def stop_callback(self, request=StopNavAndCarto.Request(), response=StopNavAndCarto.Response()):
                self.get_logger().info("\n**********************************************************************\n**********************************************************************\n************************receive stop request !************************\n**********************************************************************\n**********************************************************************")
                if self.missionstate.mission_status == 0:
                        self.get_logger().info("navigation and cartographer is off !")
                        response.status = 0
                elif self.missionstate.mission_status == 1:
                        self.carto_stop()
                        self.get_logger().info("stop cartographer !")
                        response.status = 0
                elif self.missionstate.mission_status == 2:
                        self.nav_stop()
                        self.get_logger().info("stop navigation !")
                        response.status = 0
                
                return response


        def nav_start(self):
                space_list = []
                while time.time() - self.space_time <= 10:
                        wait_second = int(time.time() - self.space_time)
                        if wait_second not in space_list:
                                self.get_logger().info(f"waitting 10s：{wait_second} s")
                        else:
                                continue
                        space_list.append(wait_second)

                second_list = []
                start_time = time.time()
                self.nav_srv_start = subprocess.Popen(shlex.split('ros2 launch capella_ros_launcher nav2.launch.py'))
                # self.nav_srv_start = multiprocessing.Process(target=os.system,args=['ros2 launch capella_ros_launcher nav2.launch.py'])
                # self.nav_srv_start.start()
                while time.time() - start_time <= 10:
                        wait_second = int(time.time() - start_time)
                        if wait_second not in second_list:
                                self.get_logger().info(f"defend 10s for nav strat：{wait_second} s")
                        else:
                                continue
                        second_list.append(wait_second)
                self.missionstate_callback()
                if self.missionstate.mission_status == 2:
                        self.get_logger().info('Navigation is on !')
                else:
                        self.get_logger().info('Navigation start fail !')
                self.space_time =time.time()

        def nav_stop(self):
                # pids = psutil.pids()
                # for pid in pids:
                #         try:
                #                 p = psutil.Process(pid)
                #         except:
                #                 continue
                #         if p.name() == "ros2" and psutil.Process(p.ppid()).name() =='nav_and_carto_s':
                #                 for children in p.children():
                #                         self.kill_process_and_children(children.pid)
                space_list = []
                while time.time() - self.space_time <= 10:
                        wait_second = int(time.time() - self.space_time)
                        if wait_second not in space_list:
                                self.get_logger().info(f"waitting 10s：{wait_second} s")
                        else:
                                continue
                        space_list.append(wait_second)

                self.get_logger().info("stop nav !")
                pid = os.getpid()
                p = psutil.Process(pid)
                for children in p.children():
                        self.kill_process_and_children(children.pid)
        
                self.missionstate_callback()
                self.space_time =time.time()
                


        def carto_start(self):
                # self.nav_srv_start = multiprocessing.Process(target=subprocess.call,args=(['ros2','launch','capella_cartographer_launcher','bainan_1_cartographer.launch.py'],99999999999))
                # self.nav_srv_start.start()
                # self.carto_srv_start = multiprocessing.Process(target=os.system,args=(['ros2 launch capella_cartographer_launcher capella_wj_lidar_cartographer.launch.py']))
                # self.carto_srv_start.start()
                # self.carto_srv_start = subprocess.Popen(['ros2','launch','capella_cartographer_launcher','bainan_1_cartographer.launch.py'],bufsize=999999999,stdout=subprocess.PIPE,stderr=subprocess.PIPE,stdin=subprocess.PIPE)
                space_list = []
                while time.time() - self.space_time <= 10:
                        wait_second = int(time.time() - self.space_time)
                        if wait_second not in space_list:
                                self.get_logger().info(f"waitting 10s：{wait_second} s")
                        else:
                                continue
                        space_list.append(wait_second)

                second_list = []
                start_time = time.time()
                self.carto_srv_start = subprocess.Popen(shlex.split('ros2 launch capella_cartographer_launcher capella_wj_lidar_cartographer.launch.py'))
                # self.carto_srv_start = multiprocessing.Process(target=os.system,args=['ros2 launch capella_cartographer_launcher capella_wj_lidar_cartographer.launch.py'])
                # self.carto_srv_start.start()
                while time.time() - start_time <= 10:
                        wait_second = int(time.time() - start_time)
                        if wait_second not in second_list:
                                self.get_logger().info(f"defend 10s for nav strat：{wait_second} s")
                        else:
                                continue
                        second_list.append(wait_second)
                self.missionstate_callback()
                if self.missionstate.mission_status == 1:
                        self.get_logger().info('Cartographer is on !')
                else:
                        self.get_logger().info('Cartographer start fail !')
                self.space_time =time.time()

        def carto_stop(self):            
                # pids = psutil.pids()
                # for pid in pids:
                #         try:
                #                 p = psutil.Process(pid)
                #         except:
                #                 continue
                        
                #         if p.name() == "ros2" and psutil.Process(p.ppid()).name() =='nav_and_carto_s':
                #                 for children in p.children():
                #                         self.kill_process_and_children(children.pid)
                space_list = []
                while time.time() - self.space_time <= 10:
                        wait_second = int(time.time() - self.space_time)
                        if wait_second not in space_list:
                                self.get_logger().info(f"waitting 10s：{wait_second} s")
                        else:
                                continue
                        space_list.append(wait_second)
                
                self.get_logger().info("stop carto")
                pid = os.getpid()
                p = psutil.Process(pid)
                for children in p.children():
                        self.kill_process_and_children(children.pid)
                        
                self.missionstate_callback()
                self.space_time =time.time()

               
        def save_img_and_yaml(self,request:NavLaunchMode.Request):
                space_list = []
                while time.time() - self.space_time <= 5:
                        wait_second = int(time.time() - self.space_time)
                        if wait_second not in space_list:
                                self.get_logger().info(f"waitting 5s：{wait_second} s")
                        else:
                                continue
                        space_list.append(wait_second)
                try:
                        img_name = "sub_picture1.png"
                        yaml_name = "sub_picture1.yaml"
                        save_path = get_package_share_directory('capella_nav_and_carto_server')
                        img_save_path = os.path.join(save_path,'map', img_name)
                        yaml_save_path = os.path.join(save_path,'map', yaml_name)
                        img_file = open(img_save_path,'wb')
                        img_file.write(request.image)
                        img_file.close()
                        yaml_file = open(yaml_save_path, 'w')
                        yaml_file.write(f"name: {img_name}\n")
                        yaml_file.write(f"resolution: {request.resolution}\n")
                        yaml_file.write(f"negate: {request.negate}\n")
                        yaml_file.write(f"origin: {[request.x , request.y , request.z]}\n")
                        yaml_file.write(f"occupied_thresh: {request.occupied_thresh}\n")
                        yaml_file.write(f"free_thresh: {request.free_thresh}\n")
                        yaml_file.close()
                        self.get_logger().info("successful save map image and yaml file !")
                        self.get_logger().info("path is : {} !".format(os.path.join(save_path,'map')))
                except:
                        self.get_logger().info("save image and yaml file fail,will use default file !")
                self.space_time = time.time()

        def kill_process(self,pid):
                if psutil.pid_exists(pid):
                        p = psutil.Process(pid)
                        p_name = p.name()
                        start_time = time.time()
                        # os.kill(pid,signal.SIGINT)
                        os.kill(pid,signal.SIGKILL)
                        while psutil.pid_exists(pid):
                                if time.time() - start_time < 10:
                                        continue

                                else:
                                        self.get_logger().info('kill pid {}:{} timeout!'.format(pid,p_name))
                                        break
                        self.get_logger().info('successful kill pid {}:{} !'.format(pid,p_name))
                else:
                        self.get_logger().info('pid {} not exist !'.format(pid))
        def kill_process_and_children(self,pid):
                if psutil.pid_exists(pid):
                        p = psutil.Process(pid)
                        if hasattr(p,'children') and len(p.children())>0:
                                for child in p.children():
                                        self.kill_process_and_children(child.pid)

                                self.kill_process(pid)
                        else:
                                self.kill_process(pid)
                else:
                        self.get_logger().info('pid not exist !')
                

def main(args=None):
        rclpy.init(args=args)
        node = NavAndCartoService('mission_server')
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
         main()

