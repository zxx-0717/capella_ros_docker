from imaplib import Commands
import shlex
import threading
import rclpy
import os
import psutil
from rclpy.node import Node
import tempfile
import subprocess
import signal
import time

# print(len(threading.enumerate()))
# pids = psutil.pids()

# for pid in pids:
#         try:
#                 p = psutil.Process(pid)
#         except:
#                 continue
#         print('name : %-30s'%p.name(),'    pid : %-8d'%pid,'    ppid : %-8d'%p.ppid(),'    gpid : ',os.getpgid(pid))


# def kill_process(pid):
#         if psutil.pid_exists(pid):
#                 p = psutil.Process(pid)
#                 p_name = p.name()
#                 start_time = time.time()
#                 os.kill(pid,signal.SIG_IGN)
#                 while psutil.pid_exists(pid):
#                         if time.time() - start_time < 10:
#                                 pass

#                         else:
#                                 break
#                 print('successful kill pid {}:{} !'.format(pid,p_name))
#         else:
#                 print('pid {} not exist !'.format(pid))
#         print(time.time() - start_time)
# def kill_process_and_children(pid):
#         if psutil.pid_exists(pid):
#                 p = psutil.Process(pid)
#                 if hasattr(p,'children') and len(p.children())>0:
#                         for child in p.children():
#                                 kill_process_and_children(child.pid)

#                         kill_process(pid)
#                 else:
#                         kill_process(pid)
#         else:
#                 print('pid not exist !')

# kill_process_and_children(16791)


# def kill_process(pid):
#                 if psutil.pid_exists(pid):
#                         p = psutil.Process(pid)
#                         p_name = p.name()
#                         start_time = time.time()
#                         os.kill(pid,signal.SIG_IGN)
#                         while psutil.pid_exists(pid):
#                                 if time.time() - start_time < 10:
#                                         pass

#                                 else:
#                                         print('kill pid {}:{} timeout!'.format(pid,p_name))
#                                         break
#                         print('successful kill pid {}:{} !'.format(pid,p_name))
#                 else:
#                         print('pid {} not exist !'.format(pid))
# def kill_process_and_children(pid):
#         if psutil.pid_exists(pid):
#                 p = psutil.Process(pid)
#                 if hasattr(p,'children') and len(p.children())>0:
#                         for child in p.children():
#                                 kill_process_and_children(child.pid)

#                         kill_process(pid)
#                 else:
#                         kill_process(pid)
#         else:
#                 print('pid not exist !')

for i in range(5000):
        print(f'正在进行{i}轮测试！')

        print('-------------------------------------------------------------------------------------------------------------------------')
        print('-------------------------------------------------------------------------------------------------------------------------')
        print('--------------------------------------------------启动cartographer！------------------------------------------------------')
        print('-------------------------------------------------------------------------------------------------------------------------')
        print('-------------------------------------------------------------------------------------------------------------------------')
        start_time = time.time()
        second_list = []
        cart = subprocess.Popen(shlex.split(r'ros2 service call /mission_server/cartographer_mode capella_ros_service_interfaces/srv/CartographerMode "{}"'))
        a = cart.wait()
        print(a)
        
        print('-------------------------------------------------------------------------------------------------------------------------')
        print('-------------------------------------------------------------------------------------------------------------------------')
        print('--------------------------------------------------启动navigation！--------------------------------------------------------')
        print('-------------------------------------------------------------------------------------------------------------------------')
        print('-------------------------------------------------------------------------------------------------------------------------')
        start_time = time.time() 
        second_list = []
        nav = subprocess.Popen(shlex.split(r'ros2 service call /mission_server/nav_launch_mode capella_ros_service_interfaces/srv/NavLaunchMode "{}"'))
        a = nav.wait()
        print(a)

        # print('-------------------------------------------------------------------------------------------------------------------------')
        # print('-------------------------------------------------------------------------------------------------------------------------')
        # print('-----------------------------------------------------启动stop！-----------------------------------------------------------')
        # print('-------------------------------------------------------------------------------------------------------------------------')
        # print('-------------------------------------------------------------------------------------------------------------------------')
        # start_time = time.time() 
        # second_list = []
        # nav = subprocess.Popen(shlex.split(r'ros2 service call /mission_server/stop_nav_and_carto capella_ros_service_interfaces/srv/StopNavAndCarto "{}"'))
        # a = nav.wait()
        # print(a)

print("完成1000次测试！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！")