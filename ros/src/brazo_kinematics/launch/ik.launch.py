import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('brazo_kinematics'), 'config', 'arm_ik.yaml')
    return LaunchDescription([
        # El firmware (main.c) crea su nodo en el namespace "prometeo" y se suscribe a "brazo/cmd"
        DeclareLaunchArgument('namespace', default_value='prometeo'),
        Node(package='brazo_kinematics', executable='ik_node', name='brazo_ik',
             namespace=LaunchConfiguration('namespace'),
             parameters=[cfg], output='screen'),
    ])
