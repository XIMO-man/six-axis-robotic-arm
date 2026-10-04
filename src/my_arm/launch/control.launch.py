"""One command for the manual control panel, RViz and Gazebo physics."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('my_arm01'))
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/moveit.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gazebo_gui'),
                              'moveit_rviz': LaunchConfiguration('rviz')}.items()),
        Node(package='my_arm01', executable='control_panel.py',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
