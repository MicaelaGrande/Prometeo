from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    description_pkg_path = get_package_share_directory('robot_description')

    arguments = [
        DeclareLaunchArgument(
            'xacro_file',
            default_value='urdf/my_robot.urdf.xacro',
            description='Path to robot robot xacro file'
        ),
    ]

    # --- RViz2: visualiza robot, pointcloud y odometría ---

    # Robot Model and TF publisher
    xacro_path = PathJoinSubstitution([description_pkg_path, LaunchConfiguration('xacro_file')])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen',
    )

    nodes = [
            description_node,
        ]

    return LaunchDescription(
        [] + arguments
           + nodes
    )
