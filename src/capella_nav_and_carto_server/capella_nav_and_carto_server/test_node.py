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

rclpy

pids = psutil.pids()
print(time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(time.time()+28800)))
for pid in pids:
        try:
                p = psutil.Process(pid)
        except:
                continue
        print('name : %-30s'%p.name(),'    pid : %-8d'%pid,'    ppid : %-8d'%p.ppid(),'    gpid : ',os.getpgid(pid))

