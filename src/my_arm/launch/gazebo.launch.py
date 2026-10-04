"""Gazebo physics, real joint feedback, and arm/gripper controllers."""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    share = Path(get_package_share_directory('my_arm01'))
    description = xacro.process_file(
        str(share / 'urdf/six_axis_arm.urdf.xacro'),
        mappings={'sim': 'true', 'controllers_file': str(share / 'config/controllers.yaml')}
    ).toxml()
    headless = LaunchConfiguration('gui').perform(context).lower() == 'false'
    gz_args = ['gz', 'sim', '-r', '-s', str(share / 'worlds/grasp.sdf')]
    nodes = [
        ExecuteProcess(cmd=gz_args, output='screen', on_exit=Shutdown(),
                       additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
                           os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                           os.environ.get('LD_LIBRARY_PATH', '')])}),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen',
                       condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'grasp_world', '-name', 'six_axis_arm',
                        '-topic', 'robot_description'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml')}]),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_fingers_controller',
                        '--controller-manager-timeout', '120'], output='screen'),
        Node(package='my_arm01', executable='gripper_action.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/display.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
