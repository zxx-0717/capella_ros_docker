# 导入库
import os
from pickle import TRUE

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# 定义函数名称为：generate_launch_description
def generate_launch_description():

    dsfbot_bringup_dir = get_package_share_directory('capella_ros_launcher')
    # urdf_dir = get_package_share_directory('capella_cartographer_launcher')

    # serial communication between MCU and pc via usb 
    serial_port_node = Node(
        package = "capella_ros_serial",
        executable = "serial_port_node",
        output = 'screen',  #四个可选项 
        parameters = [{'port': "/dev/ttyUSB0"},
                     {'baud': 4103}]
        )

    #IMU relay from clbrobot_msgs/Imu to sensor_msgs/Imu
    apply_calib_node =  Node(
        package = "capella_ros_imu_calib",
        executable = "apply_calib",
        output = 'screen',  #四个可选项 
        parameters = [{'calibrate_gyros': True}]
        )
    
    imu_ekf_node =  Node(
        package = "capella_ros_imu_ekf",
        executable = "imu_ekf_node",
        output = 'screen',  #四个可选项 
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'imu_ekf_node.yaml')]
        )

    urdf_name = "bainan_1_description.urdf"
    urdf_model_path = os.path.join(dsfbot_bringup_dir, f'urdf/{urdf_name}')

    # Publish urdf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf_model_path])
    
    # laser_scan_wr_10 = Node(
    #     package="wr_ls_udp",
    #     executable="wr_ls_udp",
    #     # respawn="True",
    #     output="screen",
    #     parameters=parameters_wr_10)

    # laser_scan_wr_20 = Node(
    #     package="wr_ls_udp_high",
    #     executable="wr_ls_udp_high",
    #     respawn="True",
    #     output="screen",
    #     parameters=parameters_wr_20)

    # laser_scan_wj_20 = Node(
    #     package="wj_716n_lidar",
    #     executable="wj_716n_lidar",
    #     # respawn="True",
    #     output="screen")

    # Filter and fuse raw imu data
    imu_filter_node =  Node(
        package = "capella_ros_imu_filter",
        executable = "imu_filter_node",
        output = 'screen',  #四个可选项 
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'imu_filter_node.yaml')]
        )

    # Publish static transform from base_footprint to imu_link
    # base_footprint_to_imu_link = Node( 
    #     package = "tf2_ros",
    #     executable = "static_transform_publisher" ,
    #     name = "base_footprint_to_imu_link" ,
    #     arguments = ['0.156', '0', '0.3745', '0', '0', '0', 'base_footprint', 'imu_link']
    #     )

    # Publish Dsfrobot odometry
    dsf_base_node =  Node(
        package = "capella_ros_launcher",
        executable = "dsf_base_node",
        output = 'screen',  #四个可选项 
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'dsf_node.yaml')]
        )

    # Publish static transform from base_footprint to base_link
    # base_footprint_to_base_link = Node( 
    #     package = "tf2_ros",
    #     executable = "static_transform_publisher" ,
    #     name = "base_footprint_to_base_link" ,
    #     arguments = ['0', '0', '0.4125', '0', '0', '0', 'base_footprint', 'base_link']
    #     )

    # Odom-IMU Extended Kalman Filter
    ekf_filter_node = Node( 
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_filter_node",
        output = 'screen',
        remappings = [('odometry/filtered', 'odom')],
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'robot_localization.yaml')],
        )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    # launch_description = LaunchDescription([serial_port_node, apply_calib_node, imu_filter_node, base_footprint_to_imu_link, dsf_base_node,
    #                                         base_footprint_to_base_link, ekf_localization])
    launch_description = LaunchDescription([serial_port_node, apply_calib_node, imu_filter_node, robot_state_publisher_node, dsf_base_node,
                                            joint_state_publisher_node, ekf_filter_node, imu_ekf_node])
    
    # laser_scan_name = os.getenv("LASER_SCAN")
    # try:
    #     if (laser_scan_name == "laser_scan_wr_10"):
    #         launch_description.add_action(laser_scan_wr_10)
    #     elif (laser_scan_name == "laser_scan_wr_20"):
    #         launch_description.add_action(laser_scan_wr_20)
    #     elif (laser_scan_name == "laser_scan_wj_20"):
    #         launch_description.add_action(laser_scan_wj_20)
    #     else:
    #         raise
    # except Exception:
    #     print("请从以下激光雷达名称中输入正确的激光雷达型号：[laser_scan_wr_10; laser_scan_wr_20; laser_scan_wr_high_20, laser_scan_wj_20]")
    # # 返回让ROS2根据launch描述执行节点
    return launch_description

# parameters_wr_10 = [
#     {"frame_id": "laser_link"},
#     {"range_min": 0.01},
#     {"range_max": 10.0},
#     {"hostname": "192.168.0.10"},
#     {"port": "2112" },
#     {"timelimit": 5},
#     {"checkframe": True},
#     {"min_ang": -2.357},
#     {"max_ang": 2.357},
#     {"intensity": 1},
#     {"skip": 0},
#     {"time_offset": -0.001},
#     {"auto_reboot": False},
#     {"debug_mode": False},
#     {"time_increment": 0.000061722},
#     ]

# parameters_wr_20 = [
#     {"frame_id": "laser_link"},
#     {"range_min": 0.01},
#     {"range_max": 20.0},
#     {"hostname": "192.168.0.10"},
#     {"port": "2112" },
#     {"timelimit": 5},
#     {"checkframe": True},
#     {"min_ang": -2.357},
#     {"max_ang": 2.357},
#     {"intensity": True},
#     {"skip": 0},
#     {"time_offset": -0.001},
#     {"auto_reboot": False},
#     {"debug_mode": False},
#     {"time_increment": 0.000061722},
#     {"angle_resolution": 0.25},
#     ]