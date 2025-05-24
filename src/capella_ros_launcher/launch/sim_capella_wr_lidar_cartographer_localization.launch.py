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
    use_sim_time = LaunchConfiguration('use_sim_time', default="True")
    dsfbot_nav2_dir = get_package_share_directory('capella_ros_launcher')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(dsfbot_nav2_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='bainan_1_2d_localization.lua')
    load_state_filename = LaunchConfiguration('load_state_filename',default=os.path.join(dsfbot_nav2_dir, 'config/first_floor_0731.pbstream'))
    declare_pure_localization = DeclareLaunchArgument(
        'pure_localization', default_value='True',
        description='Whether to only localizition')
    carto_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    rviz_config_dir = os.path.join(carto_share, 'configuration_files/demo_2d.rviz')

    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                   '-load_state_filename', load_state_filename],
        remappings=[('imu', 'imu/data')])

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', rviz_config_dir],
        parameters = [{'use_sim_time': True}],
    )

    ld = LaunchDescription()

    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)
    return ld


