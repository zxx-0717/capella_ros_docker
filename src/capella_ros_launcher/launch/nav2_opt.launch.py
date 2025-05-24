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
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'False')
    no_navigator = LaunchConfiguration('no_navigator', default = 'False')
    headless = LaunchConfiguration('headless', default = 'False')

    param_file_name = 'dsf_nav2.yaml'
    map_yaml_path = LaunchConfiguration('map', default = os.path.join(dsfbot_nav2_dir, 'map', 'yanshi0707.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default = os.path.join(dsfbot_nav2_dir, 'param', param_file_name))
    # nav2_param_path = LaunchConfiguration('params_file', default = os.path.join(dsfbot_nav2_dir, 'param', 'nav2_smac.yaml'))

    rviz_config_dir = os.path.join(dsfbot_nav2_dir,'rviz','nav2_default_view.rviz')

     #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time, description = 'Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('headless', default_value = headless, description = 'Run in headless mode (no window).'),
        DeclareLaunchArgument('no_navigator', default_value = no_navigator, description = 'Do not start multi navigator node.'),
        DeclareLaunchArgument('map', default_value = map_yaml_path, description = 'Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value = nav2_param_path, description = 'Full path to param file to load'),

        # 启动节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dsfbot_nav2_dir, '/launch','/bringup_opt.launch.py']),
        ),
        #启动dabai_dcw摄像头
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([astra_camera_dir, '/launch','/multi_dabai_dcw.launch.py'])
            # condition=IfCondition(PythonExpression([use_depth_camera]))
        ),
        # cartographer定位程序
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dsfbot_nav2_dir, '/launch','/capella_wr_lidar_cartographer_localization.launch.py'])
            # condition=IfCondition(PythonExpression(['not ', str(amcl_localization)]))
        ),
        
        #将深度相机的点云数据转为2D雷达数据格式
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pointcloud2laserscan_dir, '/launch','/sample_pointcloud_to_laserscan_launch.py']),
        ),
        #执行自动回充任务程序
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([capella_ros_dock, '/launch','/test_dock.launch.py']),
        ),

        #启动nav2自带节点
        IncludeLaunchDescription(PythonLaunchDescriptionSource([bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'use_composition': 'False',
                'use_respawn': 'True'}.items(),),
        
        Node(
            condition=IfCondition(PythonExpression(['not ', headless])),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
