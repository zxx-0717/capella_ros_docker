# 导入库
import os
from pickle import TRUE

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

# 定义函数名称为：generate_launch_description
def generate_launch_description():

    #=============================1.定位到包的地址=============================================================
    dsfbot_bringup_dir = get_package_share_directory('capella_ros_launcher')
    scan_odom_dir = get_package_share_directory('capella_ros_scan_odom')

     #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    # if 'ROBOT_VERSION' in os.environ:
    #     if os.environ.get('ROBOT_VERSION') == 'bainan_1':
    #         urdf_name = "bainan_1_description.urdf"
    #     elif os.environ.get('ROBOT_VERSION') == 'bainan_2':
    #         urdf_name = "bainan_2_description.urdf"
    # else:
    urdf_name = "bainan_2_description_for_charge.urdf"
    urdf_model_path = os.path.join(dsfbot_bringup_dir, f'urdf/{urdf_name}')

    # serial communication between MCU and pc via usb 
    serial_port_node = Node(
        package = "capella_ros_serial",
        executable = "serial_port_node",
        output = 'screen',  #四个可选项 
        parameters = [{'port': "/dev/ttyUSB0"},
                     {'baud': 4098}]
        )

    # Publish urdf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf_model_path])

    #IMU relay from clbrobot_msgs/Imu to sensor_msgs/Imu
    apply_calib_node =  Node(
        package = "capella_ros_imu_calib",
        executable = "apply_calib",
        output = 'screen',  #四个可选项 
        parameters = [{'calibrate_gyros': True}]
        )
    #lidar node
    laser_scan_wr_10 = Node(
        package="wr_ls_udp",
        executable="wr_ls_udp",
        # respawn="true",
        output="screen",
        parameters=parameters_wr_10)

    laser_scan_wr_20 = Node(
        package="wr_ls_udp",
        executable="wr_ls_udp",
        # respawn="true",
        output="screen",
        parameters=parameters_wr_20)

    laser_scan_wr_high_20 = Node(
        package="wr_ls_udp_high",
        executable="wr_ls_udp_high",
        # respawn="T",
        output="screen",
        parameters=parameters_wr_high_20)

    laser_scan_wj_20 = Node(
        package="wj_716n_lidar",
        executable="wj_716n_lidar",
        # respawn="true",
        output="screen")

    #lidar odom node
    plicp_odom_node = Node(
       package="capella_ros_scan_odom",
       executable="plicp_odom_node",
       name="plicp_odom_node",
       parameters = [os.path.join(scan_odom_dir, 'config', 'plicp_odom.yaml')],)

    # Filter and fuse raw imu data
    imu_filter_node =  Node(
        package = "capella_ros_imu_filter",
        executable = "imu_filter_node",
        output = 'screen',  #四个可选项 
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'imu_filter_node.yaml')]
        )

    # Publish Dsfrobot odometry
    dsf_base_node =  Node(
        package = "capella_ros_launcher",
        executable = "dsf_base_node",
        output = 'screen',  #四个可选项 
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'dsf_node.yaml')]
        )

    # Odom-IMU Extended Kalman Filter
    ekf_filter_node = Node( 
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_filter_node",
        output = 'screen',
        remappings = [('odometry/filtered', 'odom')],
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'robot_localization_opt.yaml')],
        )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([serial_port_node, robot_state_publisher_node, joint_state_publisher_node, 
                                            apply_calib_node, imu_filter_node, dsf_base_node,
                                            plicp_odom_node, ekf_filter_node])
    # launch_description = LaunchDescription([robot_state_publisher_node, joint_state_publisher_node,
    #                                          plicp_odom_node, ekf_filter_node])

    if 'LASER_SCAN' in os.environ:
        laser_scan_name = os.environ.get("LASER_SCAN")
        try:
            if (laser_scan_name == "laser_scan_wr_10"):
                launch_description.add_action(laser_scan_wr_10)
            elif (laser_scan_name == "laser_scan_wr_20"):
                launch_description.add_action(laser_scan_wr_20)
            elif (laser_scan_name == "laser_scan_wr_high_20"):
                launch_description.add_action(laser_scan_wr_high_20)
            elif (laser_scan_name == "laser_scan_wj_20"):
                launch_description.add_action(laser_scan_wj_20)
            else:
                raise 
        except Exception:
            print("请从以下激光雷达名称中输入正确的激光雷达型号：[laser_scan_wr_10; laser_scan_wr_20; laser_scan_wr_high_20; laser_scan_wj_20]")
    else:
        launch_description.add_action(laser_scan_wr_10)
    return launch_description

parameters_wr_10 = [
    {"frame_id": "laser_link"},
    {"range_min": 0.25},
    {"range_max": 10.0},
    {"hostname": "192.168.0.10"},
    {"port": "2112" },
    {"timelimit": 5},
    {"checkframe": True},
    {"min_ang": -2.357},
    {"max_ang": 2.357},
    {"intensity": 1},
    {"skip": 0},
    {"time_offset": -0.001},
    {"auto_reboot": False},
    {"debug_mode": False},
    {"time_increment": 0.000061722},
    ]

parameters_wr_20 = [
    {"frame_id": "laser_link"},
    {"range_min": 0.25},
    {"range_max": 20.0},
    {"hostname": "192.168.0.10"},
    {"port": "2112" },
    {"timelimit": 5},
    {"checkframe": True},
    {"min_ang": -2.357},
    {"max_ang": 2.357},
    {"intensity": 1},
    {"skip": 0},
    {"time_offset": -0.001},
    {"auto_reboot": False},
    {"debug_mode": False},
    {"time_increment": 0.000061722},
    ]

parameters_wr_high_20 = [
    {"frame_id": "laser_link"},
    {"range_min": 0.5},
    {"range_max": 30.0},
    {"hostname": "192.168.0.10"},
    {"port": "2112" },
    {"timelimit": 5},
    {"checkframe": True},
    {"min_ang": -2.357},
    {"max_ang": 2.357},
    {"intensity": True},
    {"skip": 0},
    {"time_offset": -0.001},
    {"auto_reboot": False},
    {"debug_mode": False},
    {"time_increment": 0.000061722},
    {"angle_resolution": 0.25},
    ]