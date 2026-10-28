from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    bringup_pkg_path = get_package_share_directory('my_robot_bringup')

    arguments = [
        DeclareLaunchArgument(
            "xacro_file",
            default_value='urdf/my_robot.urdf.xacro',
            description='Path to robot robot xacro file'
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="vision_testground",
            description="world name",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="gazebo.rviz",
            description="rviz config file to use"
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value="odom",
            description="fixed frame the cube markers are published in"
        ),
    ]

    # Gazebo + bridge + robot model/TF + RViz + teleop
    gazebo_stack = IncludeLaunchDescription(
        PathJoinSubstitution([bringup_pkg_path, 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'xacro_file': LaunchConfiguration('xacro_file'),
            'world_name': LaunchConfiguration('world_name'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    # Cube detection: /vision/cubes/*, /vision/cube_markers, /vision/annotated
    vision_node = Node(
        package='huggin_vision',
        executable='object_recognition',
        name='vision_node',
        parameters=[{
            'use_sim_time': True,
            'world_frame': LaunchConfiguration('world_frame'),
        }],
        output='screen',
    )

    return LaunchDescription(
        arguments
        + [
            gazebo_stack,
            vision_node,
        ]
    )
