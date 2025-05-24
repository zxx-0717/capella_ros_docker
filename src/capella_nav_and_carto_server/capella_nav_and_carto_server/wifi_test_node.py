import rclpy
from rclpy.node import Node
import sys
import time
from capella_ros_msg.srv import ChargePileWifi
from wpa_supplicant.core import WpaSupplicantDriver
from twisted.internet.selectreactor import SelectReactor
import threading


reactor = SelectReactor()
threading.Thread(target=reactor.run, kwargs={'installSignalHandlers':0},daemon=True).start()
time.sleep(1)
driver = WpaSupplicantDriver(reactor)
supplicant = driver.connect()
try:
        iface = supplicant.get_interface('wlp2s0')
except:
        iface = supplicant.create_interface('wlp2s0')
scan_results = iface.scan(block=True)
for bss in scan_results:
        print(bss.get_ssid())
        print(bss.get_bssid())
        if bss.get_bssid() == 'CA:C9:A3:98:DE:18':
                network_cfg = {}
                network_cfg['psk'] = '1234567890'
                network_cfg['ssid'] = bss.get_ssid()
                network_cfg['bssid'] = bss.get_bssid()
                network_cfg['key_mgmt'] = 'WPA-PSK'
                net = network_cfg
                break
if net == None:
        print('想要连接的wifi（bssid）不存在。')
        sys.exit()
net = iface.add_network(net)
iface.select_network(net.get_path())
time.sleep(1)
print(iface.get_state())

print(iface.get_current_bss())
iface.disconnect_network()
print(iface.get_current_bss())
reactor.stop()                
