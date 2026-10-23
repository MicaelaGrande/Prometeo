
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():

    bringup_pkg_path = get_package_share_directory('my_robot_bringup')
    description_pkg_path = get_package_share_directory('robot_description')

    arguments = [
        DeclareLaunchArgument(
            "xacro_file",
            default_value='urdf/my_robot.urdf.xacro',
            description='Path to robot robot xacro file'
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="square_room",
            description="world name",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="gazebo.rviz",
            description="rviz config file to use"
        )
    ]

    # --- RViz2: visualiza robot, pointcloud y odometría ---

    rviz_config_file = PathJoinSubstitution([bringup_pkg_path, 'config', LaunchConfiguration('rviz_config')])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Robot Model and TF publisher

    tf_node = IncludeLaunchDescription(
        PathJoinSubstitution([ description_pkg_path, 'launch', 'model.launch.py' ]),
        launch_arguments={'xacro_file': LaunchConfiguration('xacro_file')}.items(),
    )

    ros_control = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                get_package_share_directory('teleop_twist_joy'),
                'launch',
                'teleop-launch.py'
            ]
        ),
        launch_arguments={
            'joy_config': 'xbox',  # Replace with your own world file
        }.items(),
    )

    gazebo_nodes = IncludeLaunchDescription(
        PathJoinSubstitution([bringup_pkg_path, 'launch', 'gz.launch.py']),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),  # Replace with your own world file
        }.items(),
    )

    return LaunchDescription(
        arguments
        + [
            rviz_node,
            tf_node,
            gazebo_nodes,
            ros_control,
        ]
    )
