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
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'False')

    urdf_name = "bainan_1_description.urdf"
    urdf_model_path = os.path.join(dsfbot_bringup_dir, f'urdf/{urdf_name}')

    # serial communication between MCU and pc via usb 
    serial_port_node = Node(
        package = "capella_ros_serial",
        executable = "serial_port_node",
        output = 'screen',  #四个可选项 
        parameters = [{'port': "/dev/ttyUSB0"},
                     {'baud': 4103}]
        )

    # Publish urdf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_model_path])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
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
    
    imu_ekf_node =  Node(
        package = "capella_ros_imu_ekf",
        executable = "imu_ekf_node",
        output = 'screen',  #四个可选项 
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'imu_ekf_node.yaml')]
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
    
     # 检测发出激光雷达探测到的障碍物
    lidar_detect_person_node =  Node(
        package = "lidar_detect_person",
        executable = "lidar_nanodet_detect_person_node",      
        output = 'screen', 
        )
    
    # 评估机器人前方是否有台阶
    drop_proof_node =  Node(
        package="capella_robot_drop_proof",
        executable="drop_proof",
        parameters=[{"min_range": 0.2},
                    {"max_range": 1.0},
                    {"min_height": 0.1},
                    {"min_ground_point_num": 30000}]
    )

    # Sensor_status check node
    sensor_status_check_node = Node( 
        package = "capella_ros2_node_supervise",
        executable = "ros2_node_supervise_node",
        name = "ros2_node_supervise_node",
        output = 'screen',
        parameters = [os.path.join(dsfbot_bringup_dir, 'param', 'ros2_node_supervise_parameters.yaml')],
        )

    launch_description = LaunchDescription([serial_port_node, robot_state_publisher_node, joint_state_publisher_node, 
                                            apply_calib_node, imu_filter_node, dsf_base_node, imu_ekf_node, plicp_odom_node,
                                            ekf_filter_node, lidar_detect_person_node, drop_proof_node, sensor_status_check_node])

    launch_description.add_action(laser_scan_wr_20)
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
    {"angle_resolution": 0.25}
    ]

parameters_wr_20 = [
    {"frame_id": "laser_link"},
    {"range_min": 0.25},
    {"range_max": 20.0},
    {"hostname": "192.168.0.10"},
    {"port": "2112" },
    {"timelimit": 5},
    {"checkframe": True},
    {"min_ang": -1.57},
    {"max_ang": 1.57},
    {"intensity": 1},
    {"skip": 0},
    {"time_offset": -0.001},
    {"auto_reboot": False},
    {"debug_mode": False},
    {"time_increment": 0.000061722},
    {"angle_resolution": 0.25}
    ]