import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default="false")
    resolution = LaunchConfiguration('resolution', default="0.05")
    publish_period_sec = LaunchConfiguration('publish_period_sec', default="1.0")
    dsfbot_nav2_dir = get_package_share_directory('capella_ros_launcher')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(dsfbot_nav2_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='bainan_1_2d.lua')
    rviz_config_dir = LaunchConfiguration('configuration_directory',default= os.path.join(dsfbot_nav2_dir, 'rviz/rviz_config.rviz'))

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dsfbot_nav2_dir, '/launch','/bringup_opt.launch.py']),
    )
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        remappings=[('imu', 'imu/data')])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    ld = LaunchDescription()


    ld.add_action(bringup_launch)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    return ld


