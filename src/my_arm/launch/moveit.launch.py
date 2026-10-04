"""Gazebo with MoveIt planning, execution and an optional RViz planning panel."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('my_arm01'))

    def config(name):
        return yaml.safe_load((share / 'config' / name).read_text())

    model = {
        'robot_description': xacro.process_file(str(share / 'urdf/six_axis_arm.urdf.xacro')).toxml(),
        'robot_description_semantic': (share / 'config/arm.srdf').read_text(),
        'robot_description_kinematics': config('kinematics.yaml'),
        'robot_description_planning': config('joint_limits.yaml'),
        'use_sim_time': True,
    }
    planning = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': config('ompl.yaml'),
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('moveit_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/gazebo.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false'}.items()),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[model, planning, config('moveit_controllers.yaml')], output='screen'),
        Node(package='my_arm01', executable='motion_plan.py', arguments=['scene'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/moveit.rviz')],
             parameters=[model, planning], condition=IfCondition(LaunchConfiguration('moveit_rviz'))),
    ])
