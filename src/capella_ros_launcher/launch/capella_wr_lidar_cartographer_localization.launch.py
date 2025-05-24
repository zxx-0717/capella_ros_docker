import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

# os.environ.setdefault("laser_scan", "laser_scan_wr_20")

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default="false")
    dsfbot_nav2_dir = get_package_share_directory('capella_ros_launcher')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(dsfbot_nav2_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='bainan_1_2d_localization.lua')
    load_state_filename = LaunchConfiguration('load_state_filename',default=os.path.join(dsfbot_nav2_dir, 'config/yilou0807.pbstream'))
    declare_pure_localization = DeclareLaunchArgument(
        'pure_localization', default_value='False',
        description='Whether to only localizition')
    
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                   '-load_state_filename', load_state_filename],
        remappings=[('imu', 'imu/data')])

    # 评估定位置信度
    localization_check_node = Node(
        package="capella_robot_localization_check",
        executable="localization_check",
        parameters=[{"min_score": 0.35}]
    )

    # 重定位使机器人原地旋转动作服务器
    relocalization_action_server = Node(
        package = "capella_relocalization_spin_action_server",
        executable = "relocalization_action_server",
        parameters=[{"min_score": 0.6}]
    )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(declare_pure_localization)
    ld.add_action(localization_check_node)
    ld.add_action(relocalization_action_server)
    return ld


