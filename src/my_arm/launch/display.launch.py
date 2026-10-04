"""Display the arm and control its joints with sliders."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('my_arm01'))
    description = xacro.process_file(
        str(share / 'urdf' / 'six_axis_arm.urdf.xacro')
    ).toxml()
    # Start bent, with the gripper open, to make the joints easy to see.
    initial = {'zeros.joint_2': -0.45, 'zeros.joint_3': 0.9,
               'zeros.joint_5': 0.45, 'zeros.finger_joint': 0.03}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui', parameters=[initial],
             condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='joint_state_publisher',
             executable='joint_state_publisher', parameters=[initial],
             condition=UnlessCondition(LaunchConfiguration('gui')),
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
