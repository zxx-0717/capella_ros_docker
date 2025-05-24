import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import *
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
     #=============================1.定位到包的地址=============================================================

    dsfbot_nav2_dir = get_package_share_directory('capella_ros_launcher')
    urdf_dir = get_package_share_directory('capella_cartographer_launcher')
    #nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    no_navigator = LaunchConfiguration('no_navigator', default = 'False')
    headless = LaunchConfiguration('headless', default = 'False')

    map_yaml_path = LaunchConfiguration('map', default = os.path.join(dsfbot_nav2_dir, 'map', 'first0728.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default = os.path.join(dsfbot_nav2_dir, 'param', 'dsf_nav2.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file', default = os.path.join(dsfbot_nav2_dir, 'param', 'nav2_smac.yaml'))

    urdf_name = "bainan_1_description.urdf"
    urdf_model_path = os.path.join(urdf_dir, f'urdf/{urdf_name}')

    rviz_config_dir = os.path.join(dsfbot_nav2_dir,'rviz','nav2_default_view.rviz')

     #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time, description = 'Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('headless', default_value = headless, description = 'Run in headless mode (no window).'),
        DeclareLaunchArgument('no_navigator', default_value = no_navigator, description = 'Do not start multi navigator node.'),
        DeclareLaunchArgument('map', default_value = map_yaml_path, description = 'Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value = nav2_param_path, description = 'Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dsfbot_nav2_dir, '/launch','/bringup.launch.py']),
        ),

        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]),

        Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[urdf_model_path]),


        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir,
        #                                                'localization_launch.py')),
        #     launch_arguments={'map': map_yaml_path,
        #                       'use_sim_time': use_sim_time,
        #                       'use_lifecycle_mgr': 'false'}.items()),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        #     launch_arguments={'use_sim_time': use_sim_time,
        #                       'params_file': nav2_param_path,
        #                       'use_lifecycle_mgr': 'false',
        #                       'map_subscribe_transient_local': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),),
        
        Node(
            condition=IfCondition(PythonExpression(['not ', headless])),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        Node(
            condition=IfCondition(PythonExpression(['not ', no_navigator])),
            package='nav2_simple_commander',
            executable='multi_navigator',
            name='multi_navigator_node',
            output='screen'),
    ])
