# Muestra el robot en RViz, sin simulador.
#
# Un launch arranca varios nodos con un solo comando. Se usa asi:
#   ros2 launch prometeo_description display.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Carpeta donde queda instalado el paquete despues de compilar
    paquete = get_package_share_directory('prometeo_description')
    archivo_urdf = os.path.join(paquete, 'urdf', 'prometeo.urdf.xacro')
    archivo_rviz = os.path.join(paquete, 'rviz', 'prometeo.rviz')

    # Corre el comando xacro y guarda el URDF final como texto
    descripcion_robot = ParameterValue(Command(['xacro ', archivo_urdf]), value_type=str)

    # Lee el URDF y publica en /tf donde esta cada pieza del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': descripcion_robot}],
    )

    # Abre una ventana con un deslizador por cada rueda y publica su angulo.
    # Reemplaza a los encoders mientras no hay robot ni simulador.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Abre RViz con la configuracion guardada (-d indica cual archivo cargar)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', archivo_rviz],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
