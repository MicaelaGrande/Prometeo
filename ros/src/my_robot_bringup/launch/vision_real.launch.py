from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg_path = get_package_share_directory('my_robot_bringup')

    # Detection on the real robot: no Gazebo, no TF. The camera images come
    # from the ESP32-CAM as /camera/image_raw and /camera/camera_info.
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_params',
            default_value=PathJoinSubstitution(
                [bringup_pkg_path, 'config', 'vision.yaml']),
            description='parameter file for the vision node'),
        Node(
            package='huggin_vision',
            executable='object_recognition',
            name='vision_node',
            parameters=[LaunchConfiguration('vision_params')],
            output='screen'),
    ])
