import os
import launch_ros
import launch
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='capella_ros_launcher').find('capella_ros_launcher') 
    urdf_model_path = os.path.join(pkg_share, 'urdf/bainan_1_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        arguments=[urdf_model_path]
        )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        )

    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz2_node
    ])
