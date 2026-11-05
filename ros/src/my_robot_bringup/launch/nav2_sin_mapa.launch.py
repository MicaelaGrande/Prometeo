"""Nav2 sin mapa (T2, primera tarea).

Levanta solo las piezas de Nav2 que hacen falta para ir a un punto. No abre Gazebo:
eso se hace antes con gazebo.launch.py, en otra terminal.

Piezas (cada una es un nodo):
  - planner_server:    calcula la ruta ("el GPS").
  - controller_server: sigue la ruta y publica /cmd_vel ("el conductor").
  - behavior_server:   girar, retroceder o esperar si se traba ("los planes B").
  - bt_navigator:      coordina a los otros ("el jefe").
  - lifecycle_manager: los prende en orden. En Nav2 cada nodo tiene un "ciclo de
    vida": nace apagado, se configura y despues se activa, como una maquina que
    primero se enchufa y despues se pone en marcha. Este nodo hace eso por nosotros.

No hay map_server ni AMCL: todo pasa en el frame odom (ver nav2_sin_mapa.yaml).
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    parametros = PathJoinSubstitution([
        get_package_share_directory('my_robot_bringup'), 'config', 'nav2_sin_mapa.yaml'])

    # Nodo de Nav2 -> paquete donde viene instalado (verificado en /opt/ros/jazzy/lib/).
    # El orden importa: lifecycle_manager los prende en este orden, y el jefe
    # (bt_navigator) va ultimo porque necesita a los demas listos.
    paquete_de = {
        'controller_server': 'nav2_controller',
        'planner_server': 'nav2_planner',
        'behavior_server': 'nav2_behaviors',     # con "s" al final
        'bt_navigator': 'nav2_bt_navigator',
    }
    nodos_nav2 = list(paquete_de)   # solo los nombres, en el mismo orden

    nodos = [
        Node(
            package=paquete_de[nombre],
            executable=nombre,       # en Nav2, el ejecutable se llama igual que el nodo
            name=nombre,             # tiene que coincidir con la seccion del YAML
            parameters=[parametros],
            output='screen',
        )
        for nombre in nodos_nav2
    ]

    gestor = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,          # prenderlos apenas arrancan
            'node_names': nodos_nav2,
        }],
        output='screen',
    )

    return LaunchDescription(nodos + [gestor])
