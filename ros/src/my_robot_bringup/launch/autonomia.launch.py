from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Categoria A: todo lo que hace que el robot ande solo, en un comando.

    Arranca:
      - detector_lugares: avisa curvas, bifurcaciones y callejones (/navegacion/lugar)
      - mover:            atiende /mover (avanzar, girar) y publica cmd_vel_auto
      - mapeo:            la libreta, y Exploracion cuando no hay flecha
      - nodo_decision:    elige la salida con las flechas (Natasha)

    NO arranca el selector de modo, el panel ni vision: esos los arranca panel.launch.py,
    que tiene que estar corriendo (el selector es quien deja pasar cmd_vel_auto en
    automatico). El robot no se mueve hasta poner el panel en automatico.

    Robot real (topics en /prometeo):
        ros2 launch my_robot_bringup panel.launch.py
        ros2 launch my_robot_bringup autonomia.launch.py
    Gazebo (topics sin namespace):
        ros2 launch my_robot_bringup gazebo.launch.py world_name:=laberinto joystick:=false
        ros2 launch my_robot_bringup panel.launch.py ns:=''
        ros2 launch my_robot_bringup autonomia.launch.py ns:=''
    """
    ns = LaunchConfiguration('ns')
    # "/prometeo" o "" segun el namespace, igual que en panel.launch.py
    prefijo = PythonExpression(["'/' + '", ns, "' if '", ns, "' else ''"])
    # En Gazebo (sin namespace) el reloj es el de la simulacion
    sim = ParameterValue(PythonExpression(["'", ns, "' == ''"]), value_type=bool)
    reloj = {'use_sim_time': sim}

    arguments = [
        DeclareLaunchArgument('ns', default_value='prometeo',
                              description="namespace del robot: 'prometeo' (real) o '' (Gazebo)"),
        DeclareLaunchArgument('sin_brazo', default_value='true',
                              description='true: Decision hace de cuenta que agarro el cubo'),
    ]

    nodos = [
        # El detector anota los lugares con la posicion de laberinto de /mover, no con
        # /odom: las ruedas patinan (y el robot real no publica /odom).
        Node(package='prometeo_navigation', executable='detector_lugares', namespace=ns,
             parameters=[reloj], remappings=[('odom', 'odom_laberinto')], output='screen'),
        Node(package='prometeo_navigation', executable='mover', namespace=ns,
             parameters=[reloj], output='screen'),
        Node(package='prometeo_navigation', executable='mapeo', namespace=ns,
             parameters=[reloj], output='screen'),
        # Decision escucha /prometeo/modo_actual y /prometeo/freno_operador escritos fijos:
        # en Gazebo se remapean a los del selector sin namespace.
        Node(package='prometeo_decision', executable='nodo_decision',
             parameters=[reloj, {
                 'sin_brazo': ParameterValue(LaunchConfiguration('sin_brazo'), value_type=bool),
                 'topic_seguridad': [prefijo, '/seguridad_habilitada'],
             }],
             remappings=[
                 ('/prometeo/modo_actual', [prefijo, '/modo_actual']),
                 ('/prometeo/freno_operador', [prefijo, '/freno_operador']),
             ],
             output='screen'),
    ]
    return LaunchDescription(arguments + nodos)
