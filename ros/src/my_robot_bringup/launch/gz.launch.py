import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def parse_start_marker_pose(world_file):
    """
    Busca el modelo 'start_marker' dentro del archivo world y devuelve:

        x, y, z, roll, pitch, yaw
    """

    tree = ET.parse(world_file)
    root = tree.getroot()

    for model in root.findall('.//model'):
        if model.get('name') != 'start_marker':
            continue

        pose = model.find('pose')

        if pose is None or not pose.text:
            raise RuntimeError(
                f"El modelo 'start_marker' no tiene una pose en: {world_file}"
            )

        values = [float(value) for value in pose.text.split()]

        if len(values) != 6:
            raise RuntimeError(
                "La pose de 'start_marker' debe contener "
                "6 valores: x y z roll pitch yaw"
            )

        return values

    raise RuntimeError(
        f"No se encontró el modelo 'start_marker' en: {world_file}"
    )


def launch_setup(context):
    """
    Esta función se ejecuta cuando ROS 2 ya resolvió los argumentos
    del launch, incluido world_name.
    """

    world_name = LaunchConfiguration('world_name').perform(context)

    gazebo_worlds_share = get_package_share_directory('gazebo_worlds')
    bringup_share = get_package_share_directory('my_robot_bringup')

    world_path = os.path.join(
        gazebo_worlds_share,
        'worlds',
        world_name,
        f'{world_name}.world'
    )

    if not os.path.isfile(world_path):
        raise RuntimeError(
            '\nNo se encontró el archivo del mundo:\n'
            f'  {world_path}\n\n'
            'La estructura esperada es:\n'
            '  gazebo_worlds/worlds/<world_name>/<world_name>.world\n'
        )

    x, y, z, roll, pitch, yaw = parse_start_marker_pose(world_path)

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'{world_path} -r',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge_config = os.path.join(
        bringup_share,
        'config',
        'bridge.yaml'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_my_robot',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',

            '-name',
            'my_robot',

            '-x',
            str(x),

            '-y',
            str(y),

            '-z',
            str(z),

            '-R',
            str(roll),

            '-P',
            str(pitch),

            '-Y',
            str(yaw),
        ],
    )

    delayed_spawn = TimerAction(
        period=4.0,
        actions=[
            spawn_node
        ],
    )

    return [
        gz_sim_launch,
        bridge_node,
        delayed_spawn,
    ]


def generate_launch_description():
    description_share = get_package_share_directory(
        'robot_description'
    )

    bringup_share = get_package_share_directory(
        'my_robot_bringup'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='slam_test_world',
            description=(
                'Nombre del mundo, sin extensión. '
                'Debe existir en gazebo_worlds/worlds/'
                '<world_name>/<world_name>.world'
            ),
        ),

        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(description_share, '..')
        ),

        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(bringup_share, 'models')
        ),

        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            os.path.join(description_share, '..')
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])
