# Abre Gazebo con el laberinto y pone el robot adentro.
#
# Solo en la PC principal: la PC liviana no puede correr Gazebo. Se usa asi:
#   ros2 launch prometeo_gazebo simulacion.launch.py

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    # Carpetas donde quedan instalados los paquetes despues de compilar
    paquete = get_package_share_directory('prometeo_gazebo')
    descripcion = get_package_share_directory('prometeo_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    archivo_urdf = os.path.join(descripcion, 'urdf', 'prometeo.urdf.xacro')
    archivo_puente = os.path.join(paquete, 'config', 'puente.yaml')

    # Gazebo no entiende xacro: convertimos el mundo a SDF y lo guardamos en un
    # archivo temporal, que es el que le pasamos
    mundo_xacro = os.path.join(paquete, 'worlds', 'laberinto.sdf.xacro')
    archivo_mundo = os.path.join(tempfile.gettempdir(), 'prometeo_laberinto.sdf')
    with open(archivo_mundo, 'w') as archivo:
        archivo.write(xacro.process_file(mundo_xacro).toxml())

    # Le dice a Gazebo donde buscar los modelos 3D de los package://
    ruta_modelos = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(descripcion))

    # Abre Gazebo. -r hace que la simulacion arranque sin apretar play.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {archivo_mundo}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Corre el comando xacro y guarda el URDF final como texto
    descripcion_robot = ParameterValue(Command(['xacro ', archivo_urdf]), value_type=str)

    # Publica en /tf donde esta cada pieza. use_sim_time le indica que use el
    # reloj de Gazebo, igual que todos los nodos que trabajen con la simulacion.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': descripcion_robot, 'use_sim_time': True}],
    )

    # Lee el robot del topic /robot_description y lo agrega al mundo.
    # z es la altura inicial: un centimetro, para que no nazca atravesando el piso.
    agregar_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'prometeo', '-z', '0.01'],
    )

    # Traduce los topics entre Gazebo y ROS 2 (ver config/puente.yaml)
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': archivo_puente, 'use_sim_time': True}],
    )

    return LaunchDescription([
        ruta_modelos,
        gazebo,
        robot_state_publisher,
        agregar_robot,
        puente,
    ])
